from dataclasses import replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import multiprocessing as mp
import socket
import threading
import time

import pytest

from drone_camera import probe
from drone_camera.capabilities import CapabilityError, StreamMode
from drone_camera.config import defaults
from drone_camera.gui import SettingsDialog
from conftest import wait_for


def _hung_probe(*args):
    time.sleep(120)  # Simulate an uninterruptible native camera read.


def test_slow_probe_keeps_gui_responsive_and_cancel_reaps_worker(qapp, monkeypatch):
    from PySide6.QtCore import QTimer
    monkeypatch.setattr(probe, "_probe_worker", _hung_probe)
    dialog = SettingsDialog(defaults())
    ticks = []
    timer = QTimer()
    timer.timeout.connect(lambda: ticks.append(True))
    timer.start(10)
    try:
        dialog.probe_capabilities(0)
        process = dialog.probes[0]._process
        assert not dialog.probe_buttons[0].isEnabled()
        wait_for(lambda: len(ticks) >= 5, timeout=2, app=qapp)
        assert process.is_alive()
        started = time.monotonic()
        dialog.reject()
        assert time.monotonic() - started < 2
        assert process not in mp.active_children()
        assert dialog.probes == [None, None]
        assert not dialog.probe_timer.isActive()
    finally:
        timer.stop()
        dialog._stop_probes()


def test_probe_deadline_stops_stuck_native_process(monkeypatch):
    monkeypatch.setattr(probe, "_probe_worker", _hung_probe)
    job = probe.CapabilityProbe("usb", "0")
    process = job._process
    try:
        job._deadline = time.monotonic() - 1
        with pytest.raises(CapabilityError, match="초과"):
            job.poll()
        assert process not in mp.active_children()
    finally:
        job.close()


def test_source_change_discards_probe_and_old_capabilities(qapp, monkeypatch):
    monkeypatch.setattr(probe, "_probe_worker", _hung_probe)
    dialog = SettingsDialog(defaults())
    try:
        dialog.device_modes[0] = (StreamMode(1280, 720, 30),)
        dialog.probe_capabilities(0)
        process = dialog.probes[0]._process
        dialog.fields[0][2].setText("1")
        assert dialog.probes[0] is None
        assert dialog.device_modes[0] is None
        assert dialog.probe_buttons[0].isEnabled()
        assert process not in mp.active_children()
    finally:
        dialog.reject()


@pytest.mark.parametrize("modes", [(), (StreamMode(7680, 4320, 30),)])
def test_empty_or_filtered_modes_show_error_without_gui_exception(qapp, monkeypatch, modes):
    class EmptyProbe:
        def __init__(self, *args, **kwargs): pass
        def poll(self): return modes
        def close(self): pass
    monkeypatch.setattr("drone_camera.gui.CapabilityProbe", EmptyProbe)
    dialog = SettingsDialog(defaults())
    try:
        dialog.apply_best(0, False)
        dialog._poll_probes()
        assert "조합이 없습니다" in dialog.supported_labels[0].text()
        assert dialog.device_modes[0] is None
        assert dialog.probes[0] is None
    finally:
        dialog.reject()


def test_max_fps_action_applies_after_async_probe_completes(qapp, monkeypatch):
    class DelayedProbe:
        def __init__(self, *args, **kwargs): self.ready = False
        def poll(self):
            return (StreamMode(3840, 2160, 60), StreamMode(1280, 720, 120)) if self.ready else None
        def close(self): pass
    monkeypatch.setattr("drone_camera.gui.CapabilityProbe", DelayedProbe)
    dialog = SettingsDialog(defaults())
    try:
        dialog.apply_best(0, True)
        dialog._poll_probes()
        assert dialog.probes[0] is not None
        dialog.probes[0].ready = True
        dialog._poll_probes()
        assert dialog.fields[0][8].currentData() == (1280, 720)
        assert dialog.fields[0][9].currentData() == 120
        assert dialog.probe_buttons[0].isEnabled()
    finally:
        dialog.reject()


def test_real_file_probe_resolves_relative_path_and_releases_process(video):
    job = probe.CapabilityProbe("file", video.name, directory=video.parent)
    process = job._process
    try:
        modes = wait_for(job.poll)
        assert modes == (StreamMode(320, 180, 30),)
        assert process not in mp.active_children()
    finally:
        job.close()


@pytest.mark.parametrize("platform", ["android", "ios"])
def test_phone_capabilities_use_authenticated_usb_relay_without_wifi(qapp, monkeypatch, platform):
    token = "a" * 32
    requests = []
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def do_GET(self):
            requests.append(self.path)
            body = json.dumps({"modes": [{"width": 1280, "height": 720, "max_fps": 60}]}).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    # Substitute only physical USB discovery. HTTP and TCP forwarding are real;
    # the persisted phone URL deliberately has no reachable Wi-Fi host.
    connections = []
    def connect(tunnel):
        connections.append((tunnel.platform, tunnel.serial, tunnel.remote_port))
        return socket.create_connection(("127.0.0.1", server.server_port), timeout=3)
    monkeypatch.setattr(probe.UsbTunnel, "_connect", connect)
    configs = defaults()
    configs[0] = replace(configs[0], input_type="mjpeg", usb_platform=platform,
                         usb_serial="physical-device", source=f"http://192.0.2.1:8080/stream.mjpg?token={token}")
    dialog = SettingsDialog(configs)
    try:
        dialog.probe_capabilities(0)
        job = dialog.probes[0]
        wait_for(lambda: dialog.device_modes[0], app=qapp)
        assert dialog.device_modes[0] == (StreamMode(1280, 720, 60),)
        assert requests == [f"/capabilities?token={token}"]
        assert connections == [(platform, "physical-device", 8080)]
        assert not job._tunnel._thread.is_alive()
    finally:
        dialog.reject()
        server.shutdown(); server.server_close(); thread.join(timeout=2)


def test_probe_hides_unexpected_backend_errors(monkeypatch):
    def broken(*args, **kwargs): raise RuntimeError("secret-address?token=private")
    monkeypatch.setattr(probe, "fetch_capabilities", broken)
    ctx = mp.get_context("spawn")
    result, length = ctx.RawArray("B", 65536), ctx.Value("i", 0)
    probe._probe_worker("mjpeg", "unused", 3, result, length)
    payload = json.loads(bytes(result[:length.value]))
    assert "error" in payload
    assert "secret" not in payload["error"] and "private" not in payload["error"]
