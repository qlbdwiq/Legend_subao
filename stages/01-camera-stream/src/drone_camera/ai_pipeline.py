"""Background AI inference for both cameras. Qt independent and reusable.

Design rules (see docs):
* Heavy work (importing torch, loading the model, inference) runs on ONE
  worker thread, never on the GUI thread, and never inside the capture processes.
* Each camera has a depth-1 mailbox holding only the NEWEST frame. A slow model
  replaces waiting frames instead of building a backlog or lagging further.
* Each camera owns its own engine (own ByteTrack state), TrackBook and result, so
  tracking states of camera-01 and camera-02 can never mix. The single thread
  serves the cameras round-robin so two models never fight over CPU/GPU threads.
* ``reset_camera`` (reconnect, AI toggle) invalidates in-flight results through a
  generation counter, so an old frame can never publish into a new session.
"""

from collections import deque
from dataclasses import dataclass
import threading
import time

from .detections import FrameDetections, TrackBook, build_observations
from .engine import ModelError, ModelInfo, YoloTrackerEngine

MAX_CONSECUTIVE_ERRORS = 3
STATS_WINDOW = 20


@dataclass(frozen=True)
class InferenceParams:
    confidence: float = 0.25
    iou: float = 0.5
    imgsz: int = 960
    safety_factor: float = 1.2


@dataclass(frozen=True)
class ModelStatus:
    state: str = "none"          # none / loading / ready / error
    message: str = ""
    info: ModelInfo | None = None


@dataclass(frozen=True)
class CameraAiStats:
    processed: int = 0           # Frames analysed since the last reset.
    skipped: int = 0             # Frames replaced in the mailbox before analysis.
    fps: float = 0.0             # AI frames per second (NOT the video display FPS).
    inference_ms: float = 0.0    # Mean model call duration.
    latency_ms: float = 0.0      # Frame reception to result (includes queue wait).
    error: str = ""


def default_engine_factory(config_dir):
    def factory(path, device):
        return YoloTrackerEngine(path, device, config_dir)
    return factory


class _CameraState:
    def __init__(self):
        self.frame = None
        self.result = None
        self.engine = None
        self.book = TrackBook()
        self.generation = 0
        self.needs_reset = True
        self.processed = 0
        self.skipped = 0
        self.finished = deque(maxlen=STATS_WINDOW)      # perf_counter at completion
        self.durations = deque(maxlen=STATS_WINDOW)     # inference ms
        self.latencies = deque(maxlen=STATS_WINDOW)
        self.errors = 0
        self.error = ""

    def clear(self):
        self.frame = None
        self.result = None
        self.book = TrackBook()
        self.generation += 1
        self.needs_reset = True
        self.processed = self.skipped = self.errors = 0
        self.finished.clear()
        self.durations.clear()
        self.latencies.clear()
        self.error = ""


class AiPipeline:
    def __init__(self, camera_ids=("camera-01", "camera-02"), engine_factory=None, config_dir=None):
        self.camera_ids = tuple(camera_ids)
        self._factory = engine_factory or default_engine_factory(config_dir)
        self._cond = threading.Condition()
        self._thread = None
        self._stop = False
        self._cameras = [_CameraState() for _ in self.camera_ids]
        self._params = InferenceParams()
        self._enabled = False
        self._status = ModelStatus()
        self._pending_load = None
        self._cursor = 0

    # ------------------------------------------------------------ control
    def start(self):
        with self._cond:
            if self._thread is not None:
                return
            self._stop = False
            self._thread = threading.Thread(target=self._run, name="ai-inference", daemon=True)
            self._thread.start()

    def stop(self, timeout: float = 10.0) -> bool:
        with self._cond:
            self._stop = True
            thread = self._thread
            self._cond.notify_all()
        if thread is None:
            return True
        thread.join(timeout)
        stopped = not thread.is_alive()
        if stopped:
            self._thread = None
        return stopped

    def load_model(self, path: str, device: str = "auto"):
        """Asynchronously (re)load the model for both cameras."""
        self.start()
        with self._cond:
            self._pending_load = (path, device)
            self._status = ModelStatus("loading", "모델을 불러오는 중…")
            for camera in self._cameras:
                camera.engine = None
                camera.clear()
            self._cond.notify_all()

    def set_enabled(self, enabled: bool):
        with self._cond:
            if enabled == self._enabled:
                return
            self._enabled = enabled
            for camera in self._cameras:
                camera.clear()          # Toggling must never resume stale tracks.
            self._cond.notify_all()

    def configure(self, params: InferenceParams):
        with self._cond:
            self._params = params

    def reset_camera(self, index: int):
        with self._cond:
            self._cameras[index].clear()

    # --------------------------------------------------------------- data
    def submit(self, index: int, frame) -> bool:
        """Offer the newest frame; never blocks and never queues more than one."""
        with self._cond:
            if not self._enabled or self._status.state != "ready":
                return False
            camera = self._cameras[index]
            if camera.frame is not None:
                camera.skipped += 1
            camera.frame = frame
            self._cond.notify()
            return True

    def latest(self, index: int) -> FrameDetections | None:
        with self._cond:
            return self._cameras[index].result

    def model_status(self) -> ModelStatus:
        with self._cond:
            return self._status

    @property
    def enabled(self) -> bool:
        return self._enabled

    def stats(self, index: int) -> CameraAiStats:
        with self._cond:
            camera = self._cameras[index]
            finished = list(camera.finished)
            fps = 0.0
            if len(finished) >= 2 and time.perf_counter() - finished[-1] < 2 and finished[-1] > finished[0]:
                fps = (len(finished) - 1) / (finished[-1] - finished[0])
            mean = lambda values: sum(values) / len(values) if values else 0.0
            return CameraAiStats(camera.processed, camera.skipped, fps, mean(camera.durations),
                                 mean(camera.latencies), camera.error)

    # ------------------------------------------------------------- worker
    def _next_camera(self):
        count = len(self._cameras)
        for offset in range(count):
            index = (self._cursor + offset) % count
            if self._cameras[index].frame is not None:
                self._cursor = (index + 1) % count
                return index
        return None

    def _run(self):
        while True:
            with self._cond:
                index = None
                while not self._stop:
                    if self._pending_load is not None:
                        break
                    if self._enabled and self._status.state == "ready":
                        index = self._next_camera()
                        if index is not None:
                            break
                    self._cond.wait(0.5)
                if self._stop:
                    return
                load, self._pending_load = self._pending_load, None
                if load is None:
                    camera = self._cameras[index]
                    frame, camera.frame = camera.frame, None
                    generation = camera.generation
                    params = self._params
                    engine = camera.engine
                    reset = camera.needs_reset
                    camera.needs_reset = False
                    book = camera.book
            if load is not None:
                self._load(load)
            else:
                self._process(index, frame, generation, params, engine, reset, book)

    def _load(self, load):
        path, device = load
        engines, info = [], None
        try:
            for _ in self._cameras:
                engine = self._factory(path, device)
                info = engine.load()
                engines.append(engine)
            status = ModelStatus("ready", info.device_note or info.class_warning, info)
        except ModelError as error:
            engines, status = None, ModelStatus("error", str(error))
        except Exception as error:
            engines, status = None, ModelStatus("error", f"모델 로드 중 오류가 발생했습니다 ({type(error).__name__}).")
        with self._cond:
            if self._pending_load is not None:
                return          # A newer request superseded this one.
            for camera, engine in zip(self._cameras, engines or [None] * len(self._cameras)):
                camera.engine = engine
                camera.clear()
            self._status = status

    def _process(self, index, frame, generation, params, engine, reset, book):
        camera = self._cameras[index]
        try:
            if reset:
                engine.reset()
            started = time.perf_counter()
            tracks = engine.track(frame.image, params.confidence, params.iou, params.imgsz)
            elapsed = (time.perf_counter() - started) * 1000
            observations = build_observations(
                frame.camera_id, frame.sequence, frame.received_at, frame.received_monotonic,
                tracks, params.safety_factor, book)
            completed = time.monotonic()
            height, width = frame.image.shape[:2]
            result = FrameDetections(frame.camera_id, frame.sequence, frame.received_at,
                                     frame.received_monotonic, completed, elapsed,
                                     (width, height), observations)
        except Exception as error:
            with self._cond:
                if camera.generation != generation:
                    return
                camera.errors += 1
                camera.error = f"추론 오류 ({type(error).__name__})"
                if camera.errors >= MAX_CONSECUTIVE_ERRORS:
                    self._status = ModelStatus("error", f"추론이 반복해서 실패했습니다 ({type(error).__name__}). 모델을 다시 불러오세요.")
                    for other in self._cameras:
                        other.engine = None
                        other.result = None
            return
        with self._cond:
            if camera.generation != generation:
                return              # Camera was reset/toggled while this frame ran.
            camera.errors = 0
            camera.error = ""
            camera.result = result
            camera.processed += 1
            camera.finished.append(time.perf_counter())
            camera.durations.append(elapsed)
            camera.latencies.append(result.latency_ms)
