"""Mock inference engines. Tests using these exercise our threading/state logic
only; they say nothing about YOLO or ByteTrack. Real-model tests are marked
``real_model``."""
import threading
import time

from drone_camera.detections import RawTrack
from drone_camera.engine import ModelError, ModelInfo


class FakeEngine:
    instances = []

    def __init__(self, path="fake.pt", device="cpu", *, delay=0.0, tracks=None, fail_load=None,
                 fail_track=False):
        self.path, self.device = path, device
        self.delay, self.fail_load, self.fail_track = delay, fail_load, fail_track
        self.script = tracks if tracks is not None else [RawTrack(1, 10, 20, 50, 60, 0.9)]
        self.calls = []            # (thread name, image shape, conf, iou, imgsz)
        self.resets = 0
        self.images = []
        FakeEngine.instances.append(self)

    def load(self):
        if self.fail_load:
            raise self.fail_load
        return ModelInfo(self.path, {0: "drone"}, self.device, "", "test", "test")

    def track(self, image, conf, iou, imgsz):
        self.calls.append((threading.current_thread().name, image.shape, conf, iou, imgsz))
        self.images.append(int(image[0, 0, 0]))
        if self.fail_track:
            raise RuntimeError("boom")
        if self.delay:
            time.sleep(self.delay)
        return list(self.script)

    def reset(self):
        self.resets += 1


def factory(**kwargs):
    FakeEngine.instances = []
    return lambda path, device: FakeEngine(path, device, **kwargs)


def make_frame(camera_id="camera-01", sequence=1, value=0, size=(64, 48)):
    import numpy as np
    from drone_camera.capture import Frame
    image = np.full((size[1], size[0], 3), value, np.uint8)
    return Frame(camera_id, sequence, time.time(), time.monotonic(), image)
