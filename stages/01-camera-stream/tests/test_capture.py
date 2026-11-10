from dataclasses import replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import multiprocessing as mp
import threading
import time

import cv2
import numpy as np
import pytest

from drone_camera import capture
from drone_camera.capture import CameraSource, State
from drone_camera.config import defaults
from conftest import wait_for


def wait_frame(source, sequence=0):
    source.poll()
    return source.latest_frame(sequence)


def test_two_real_video_sources_latest_frame_fps_and_independent_disconnect(video):
    configs = [replace(c, input_type="file", source=str(video), loop_file=True) for c in defaults()]
    sources = [CameraSource(c) for c in configs]
    try:
        for source in sources:
            source.connect()
        frames = [wait_for(lambda s=s: wait_frame(s)) for s in sources]
        assert all(f.resolution == (320, 180) for f in frames)
        assert frames[0].camera_id != frames[1].camera_id
        assert abs(frames[0].received_at - time.time()) < 10
        before = frames[0].sequence
        time.sleep(0.5)  # Deliberately slow consumer: fetch latest, not FIFO.
        latest = sources[0].latest_frame(before)
        assert latest.sequence > before + 3
        assert time.monotonic() - latest.received_monotonic < 0.6
        latest.image[:] = 0
        assert sources[0].latest_frame().image.any()  # Returned image is an owned copy.
        assert sources[0].latest_frame(10**9) is None
        assert 10 < sources[0].status().fps < 60
        sources[0].disconnect()
        wait_for(lambda: (sources[0].poll(), not sources[0].active)[1])
        assert sources[0].status().state == State.DISCONNECTED
        sequence = sources[1].latest_frame().sequence
        assert wait_for(lambda: wait_frame(sources[1], sequence))
        sources[0].connect()
        assert wait_for(lambda: wait_frame(sources[0]))
    finally:
        for source in sources:
            source.close()
    assert all(not s.active for s in sources)


def test_file_end_is_not_network_reconnect(video):
    source = CameraSource(replace(defaults()[0], input_type="file", source=str(video)))
    try:
        source.connect()
        wait_for(lambda: (source.poll(), source.status().state == State.ENDED)[1])
        wait_for(lambda: (source.poll(), not source.active)[1])
        assert source.latest_frame().sequence == 90
    finally:
        source.close()


def test_missing_file_does_not_prevent_other_camera(video, tmp_path):
    bad = CameraSource(replace(defaults()[0], input_type="file", source=str(tmp_path / "missing.avi")))
    good = CameraSource(replace(defaults()[1], input_type="file", source=str(video), loop_file=True))
    try:
        bad.connect()
        good.connect()
        wait_for(lambda: (bad.poll(), bad.status().state == State.ERROR)[1])
        assert "does not exist" in bad.status().message
        assert wait_for(lambda: wait_frame(good))
    finally:
        bad.close()
        good.close()


@pytest.mark.parametrize("kind,url", [("mjpeg", "http://user:secret@127.0.0.1:1/video"),
                                       ("rtsp", "rtsp://user:secret@127.0.0.1:1/live")])
def test_network_backend_timeout_parameters(monkeypatch, kind, url):
    calls = []
    monkeypatch.setattr(cv2, "VideoCapture", lambda *args: calls.append(args))
    capture._open_capture(replace(defaults()[0], input_type=kind, source=url, timeout_seconds=1.5))
    assert calls == [(url, cv2.CAP_FFMPEG, [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 1500,
                                         cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1500])]


def test_bounded_automatic_retries_and_release(monkeypatch):
    attempts, releases = [], []
    class FailedCapture:
        def isOpened(self):
            return False
        def release(self):
            releases.append(True)
    def opener(_):
        attempts.append(time.monotonic())
        return FailedCapture()
    monkeypatch.setattr(capture, "_open_capture", opener)
    ctx = mp.get_context("spawn")
    slot = capture._Slot(ctx)
    config = replace(defaults()[0], auto_reconnect=True, max_reconnect_attempts=2,
                     reconnect_delay_seconds=0.1)
    capture._capture_worker(config, slot, ctx.Event())
    assert len(attempts) == len(releases) == 3
    assert attempts[1] - attempts[0] >= 0.09
    assert slot.state.value == State.ERROR


def test_automatic_recovery_after_initial_failure(monkeypatch):
    ctx = mp.get_context("spawn")
    slot = capture._Slot(ctx)
    stop = ctx.Event()
    attempts = []
    releases = []
    class RecoveringCapture:
        def isOpened(self):
            return len(attempts) > 1
        def get(self, _):
            return 30
        def read(self):
            stop.set()  # One successful frame is enough to exercise recovery.
            return True, np.full((120, 160, 3), 80, np.uint8)
        def release(self):
            releases.append(True)
    def opener(_):
        attempts.append(True)
        return RecoveringCapture()
    monkeypatch.setattr(capture, "_open_capture", opener)
    config = replace(defaults()[0], max_reconnect_attempts=2, reconnect_delay_seconds=0.1)
    capture._capture_worker(config, slot, stop)
    assert len(attempts) == len(releases) == 2
    assert slot.meta[0] == 1
    assert slot.meta[1:3] == [160, 120]
    assert slot.error.value == 0


def test_usb_uses_integer_device_backend(monkeypatch):
    calls = []
    monkeypatch.setattr(cv2, "VideoCapture", lambda *args: calls.append(args))
    capture._open_capture(replace(defaults()[0], source=2))
    assert calls == [(2,)]


def test_equal_clock_ticks_do_not_break_stream(monkeypatch):
    ctx = mp.get_context("spawn")
    slot = capture._Slot(ctx)
    stop = ctx.Event()
    reads = []
    class BurstCapture:
        def isOpened(self):
            return True
        def get(self, _):
            return 30
        def read(self):
            reads.append(True)
            if len(reads) == 3:
                stop.set()
            return True, np.full((120, 160, 3), 80, np.uint8)
        def release(self):
            pass
    monkeypatch.setattr(capture, "_open_capture", lambda _: BurstCapture())
    monkeypatch.setattr(capture.time, "perf_counter", lambda: 1.0)
    capture._capture_worker(defaults()[0], slot, stop)
    assert len(reads) == 3
    assert slot.meta[0] == 3
    assert slot.error.value == 0
    assert slot.meta[5] == 0  # FPS is initially unknown, not a division error.


def _hung_worker(config, slot, stop):
    # Simulates a native backend ignoring timeouts and the stop event.
    time.sleep(120)


def test_stuck_backend_shutdown_is_bounded(monkeypatch):
    monkeypatch.setattr(capture, "_capture_worker", _hung_worker)
    source = CameraSource(defaults()[0])
    source.connect()
    started = time.monotonic()
    source.close()
    assert time.monotonic() - started < 5
    assert not source.active


def test_watchdog_reports_timeout_after_process_cleanup(monkeypatch):
    monkeypatch.setattr(capture, "_capture_worker", _hung_worker)
    source = CameraSource(defaults()[0])
    try:
        source.connect()
        source._slot.heartbeat.value = time.monotonic() - 100
        source.poll()
        wait_for(lambda: (source.poll(), not source.active)[1])
        assert source.status().state == State.ERROR
        assert "timeout" in source.status().message
    finally:
        source.close()


def test_real_http_mjpeg_stream():
    ok, jpeg = cv2.imencode(".jpg", np.full((120, 160, 3), 140, np.uint8))
    assert ok
    payload = jpeg.tobytes()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                for _ in range(300):
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                     str(len(payload)).encode() + b"\r\n\r\n" + payload + b"\r\n")
                    self.wfile.flush()
                    time.sleep(1 / 30)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    source = CameraSource(replace(defaults()[0], input_type="mjpeg",
        source=f"http://127.0.0.1:{server.server_port}/video", timeout_seconds=2))
    try:
        source.connect()
        frame = wait_for(lambda: wait_frame(source))
        assert frame.resolution == (160, 120)
        assert source.status().state == State.CONNECTED
    finally:
        source.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
