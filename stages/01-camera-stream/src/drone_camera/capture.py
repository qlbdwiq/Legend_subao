"""Independent capture processes with one shared latest-frame slot per camera.

OpenCV backends can block inside native open/read calls (notably USB). Processes
allow bounded cancellation without unsafe thread termination or GUI blocking.
No frames or URLs pass through an unbounded event/queue backlog.
"""

from collections import deque
from dataclasses import dataclass
from enum import IntEnum
import multiprocessing as mp
import os
from pathlib import Path
import time

import numpy as np

from .config import CameraConfig

MAX_FRAME_BYTES = 3840 * 2160 * 3


class State(IntEnum):
    DISCONNECTED = 0
    CONNECTING = 1
    CONNECTED = 2
    RECONNECTING = 3
    ERROR = 4
    ENDED = 5
    STOPPING = 6


ERRORS = {
    0: "",
    1: "Cannot open source. Check address, permissions, device and network.",
    2: "Frame reception stopped. Check the camera or network.",
    3: "Video exceeds the supported 3840×2160 frame buffer.",
    4: "Camera worker stopped unexpectedly.",
    5: "Connection/frame timeout. Check source availability.",
    6: "Video file does not exist.",
}


@dataclass(frozen=True)
class Frame:
    camera_id: str
    sequence: int
    received_at: float  # Local wall-clock reception time, NOT camera exposure time.
    received_monotonic: float
    image: np.ndarray  # Owned BGR copy; caller can modify it without affecting capture.

    @property
    def resolution(self) -> tuple[int, int]:
        return self.image.shape[1], self.image.shape[0]


@dataclass(frozen=True)
class Status:
    state: State
    fps: float
    width: int
    height: int
    last_received: float
    message: str


class _Slot:
    def __init__(self, ctx):
        self.lock = ctx.Lock()
        self.pixels = ctx.RawArray("B", MAX_FRAME_BYTES)
        # sequence, width, height, wall time, monotonic time, fps
        self.meta = ctx.RawArray("d", 6)
        self.state = ctx.RawValue("i", State.CONNECTING)
        self.error = ctx.RawValue("i", 0)
        self.heartbeat = ctx.RawValue("d", time.monotonic())


def _open_capture(config):
    import cv2
    # Never let backend diagnostics print URLs containing credentials.
    cv2.setLogLevel(0)
    if config.input_type in ("mjpeg", "rtsp"):
        milliseconds = round(config.timeout_seconds * 1000)
        return cv2.VideoCapture(config.source, cv2.CAP_FFMPEG, [
            cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, milliseconds,
            cv2.CAP_PROP_READ_TIMEOUT_MSEC, milliseconds,
        ])
    return cv2.VideoCapture(config.source)


def _capture_worker(config: CameraConfig, slot: _Slot, stop):
    os.environ["OPENCV_FFMPEG_LOGLEVEL"] = "-8"  # AV_LOG_QUIET; URLs may contain passwords.
    import cv2
    cv2.setLogLevel(0)
    retries = 0
    sequence = 0
    while not stop.is_set():
        capture = None
        error = 1
        slot.state.value = State.CONNECTING if retries == 0 else State.RECONNECTING
        slot.heartbeat.value = time.monotonic()
        try:
            if config.input_type == "file" and not Path(str(config.source)).is_file():
                slot.error.value = 6
                slot.state.value = State.ERROR
                return
            capture = _open_capture(config)
            if capture.isOpened():
                samples = deque(maxlen=120)
                fps = capture.get(cv2.CAP_PROP_FPS)
                interval = 1.0 / fps if np.isfinite(fps) and 0 < fps <= 240 else 1.0 / 30
                deadline = time.monotonic()
                while not stop.is_set():
                    slot.heartbeat.value = time.monotonic()
                    ok, image = capture.read()
                    if not ok or image is None:
                        if config.input_type == "file":
                            if config.loop_file and samples and capture.set(cv2.CAP_PROP_POS_FRAMES, 0):
                                deadline = time.monotonic()
                                continue
                            slot.state.value = State.ENDED if samples else State.ERROR
                            slot.error.value = 0 if samples else 2
                            return
                        error = 2
                        break
                    now = time.monotonic()
                    height, width = image.shape[:2]
                    if (width > 3840 or height > 2160 or image.nbytes > MAX_FRAME_BYTES
                            or image.ndim != 3 or image.shape[2] != 3):
                        slot.error.value = 3
                        slot.state.value = State.ERROR
                        return
                    samples.append(now)
                    actual_fps = (len(samples) - 1) / (samples[-1] - samples[0]) if len(samples) > 1 else 0
                    sequence += 1
                    # Drop a frame instead of waiting for a slow consumer.
                    if slot.lock.acquire(timeout=0.01):
                        try:
                            np.frombuffer(slot.pixels, dtype=np.uint8)[:image.size] = image.reshape(-1)
                            slot.meta[:] = (sequence, width, height, time.time(), now, actual_fps)
                        finally:
                            slot.lock.release()
                    slot.error.value = 0
                    slot.state.value = State.CONNECTED
                    if config.input_type == "file":
                        deadline += interval
                        stop.wait(max(0, deadline - time.monotonic()))
        except Exception:
            # Do not forward backend exceptions that may contain credentials.
            error = 2
        finally:
            if capture is not None:
                capture.release()
        if stop.is_set():
            break
        slot.error.value = error
        if not config.auto_reconnect or retries >= config.max_reconnect_attempts:
            slot.state.value = State.ERROR
            return
        retries += 1
        slot.state.value = State.RECONNECTING
        stop.wait(config.reconnect_delay_seconds)
    slot.state.value = State.DISCONNECTED


class CameraSource:
    """Main-process API reusable by later stages; does not depend on Qt.

    Call poll() regularly (the GUI does so at 30 Hz). Manual connect starts a
    fresh bounded reconnect budget. Auto retries count per connection session.
    """

    def __init__(self, config: CameraConfig):
        self.config = config
        self._ctx = mp.get_context("spawn")
        self._slot = None
        self._process = None
        self._stop = None
        self._stopping_at = None
        self._last_meta = (0, 0, 0, 0, 0, 0)
        self._override = None

    @property
    def active(self):
        return self._process is not None

    def connect(self):
        if self.active:
            return
        self._slot = _Slot(self._ctx)
        self._stop = self._ctx.Event()
        self._override = None
        self._stopping_at = None
        self._last_meta = (0, 0, 0, 0, 0, 0)
        self._process = self._ctx.Process(target=_capture_worker,
            args=(self.config, self._slot, self._stop), daemon=True, name=self.config.id)
        try:
            self._process.start()
        except Exception:
            self._process = None
            self._override = (State.ERROR, 4)

    def disconnect(self):
        if self.active and self._stopping_at is None:
            self._stop.set()
            self._stopping_at = time.monotonic()
            self._override = (State.STOPPING, 0)
        elif not self.active:
            self._override = (State.DISCONNECTED, 0)

    def _reap(self):
        self._process.join(timeout=0)
        self._process.close()
        self._process = None

    def poll(self):
        if not self.active:
            return
        if not self._process.is_alive():
            if self._stopping_at is not None and self._override == (State.STOPPING, 0):
                self._override = (State.DISCONNECTED, 0)
            elif self._override is None and self._slot.state.value not in (State.ERROR, State.ENDED):
                self._override = (State.ERROR, 4)
            self._reap()
            return
        now = time.monotonic()
        if self._stopping_at is not None:
            if now - self._stopping_at > 1:
                self._process.terminate()
            if now - self._stopping_at > 2:
                self._process.kill()
        elif (self._slot.state.value in (State.CONNECTING, State.CONNECTED, State.RECONNECTING)
              and now - self._slot.heartbeat.value > self.config.timeout_seconds + 3 + (
                  self.config.reconnect_delay_seconds if self._slot.state.value == State.RECONNECTING else 0)):
            # Native USB/file backends may ignore timeouts. Recover safely by
            # ending this process; the user can explicitly reconnect.
            self.disconnect()
            self._override = (State.ERROR, 5)

    def status(self) -> Status:
        if self._slot is None:
            return Status(State.DISCONNECTED, 0, 0, 0, 0, "")
        if self._slot.lock.acquire(timeout=0.005):
            try:
                self._last_meta = tuple(self._slot.meta)
            finally:
                self._slot.lock.release()
        _, w, h, _, received, fps = self._last_meta
        state, error = self._override or (State(self._slot.state.value), self._slot.error.value)
        if state != State.CONNECTED or time.monotonic() - received > 2:
            fps = 0
        return Status(state, fps, int(w), int(h), received, ERRORS[error])

    def latest_frame(self, after_sequence: int = 0) -> Frame | None:
        slot = self._slot
        if slot is None or not slot.lock.acquire(timeout=0.005):
            return None
        try:
            sequence, w, h, wall, received, _ = slot.meta
            if sequence <= after_sequence or w == 0 or h == 0:
                return None
            image = np.frombuffer(slot.pixels, dtype=np.uint8, count=int(w * h * 3)).reshape(
                int(h), int(w), 3).copy()
            return Frame(self.config.id, int(sequence), wall, received, image)
        finally:
            slot.lock.release()

    def close(self):
        """Bounded final cleanup, including a backend stuck in native I/O."""
        if self.active:
            self._stop.set()
            self._process.join(timeout=1)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=1)
            if self._process.is_alive():
                self._process.kill()
                self._process.join(timeout=1)
            if self._process.is_alive():
                raise RuntimeError("Camera process could not be stopped.")
            self._reap()
        self._slot = None
        self._override = None
