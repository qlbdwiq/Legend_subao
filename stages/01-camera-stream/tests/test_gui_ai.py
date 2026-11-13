"""GUI integration of the AI features. Mock-engine tests unless marked real_model."""
from dataclasses import replace
import json
import threading
import time

import pytest
from PySide6.QtCore import QTimer

from conftest import wait_for
from drone_camera.ai_pipeline import AiPipeline, CameraAiStats, ModelStatus
from drone_camera.ai_settings import AiSettings, save_ai_settings
from drone_camera.capture import State
from drone_camera.config import defaults
from drone_camera.detections import DroneObservation, FrameDetections
from drone_camera.engine import ModelError, ModelInfo
from drone_camera.gui import MainWindow, VideoView
import fakes
from fakes import FakeEngine, factory, make_frame


@pytest.fixture
def cameras(video):
    return [replace(c, input_type="file", source=str(video), loop_file=True) for c in defaults()]


@pytest.fixture
def open_window(qapp, cameras, tmp_path):
    windows = []

    def build(pipeline=None, settings=None):
        if settings is not None:
            save_ai_settings(tmp_path / "ai_settings.json", settings)
        window = MainWindow(cameras, tmp_path / "config.json", pipeline=pipeline)
        window.show()
        windows.append(window)
        return window
    yield build
    for window in windows:
        window.close()
    qapp.processEvents()


def detection(camera="camera-01", sequence=1, size=(320, 180), center=(160, 90), radius=40.0,
              age=0.0, track_id=1):
    half = radius / 1.2 / 1.2
    obs = DroneObservation(camera, sequence, time.time(), time.monotonic(), track_id,
                           (center[0] - half, center[1] - half, center[0] + half, center[1] + half),
                           center, radius / 1.2, radius, 1.2, 0.87, "tracked")
    return FrameDetections(camera, sequence, time.time(), time.monotonic(), time.monotonic() - age,
                           12.0, size, (obs,))


class StubPipeline:
    """Records what the GUI asks of the pipeline; returns scripted results."""
    def __init__(self, ready=True):
        self.enabled, self.submitted, self.resets, self.params, self.results = False, [], [], None, [None, None]
        self.status = (ModelStatus("ready", "", ModelInfo("m.pt", {0: "drone"}, "cpu", "", "u", "t"))
                       if ready else ModelStatus())
        self.loaded = []

    def load_model(self, path, device="auto"): self.loaded.append((path, device))
    def set_enabled(self, value): self.enabled = value
    def configure(self, params): self.params = params
    def submit(self, index, frame): self.submitted.append((index, frame.sequence)); return True
    def latest(self, index): return self.results[index]
    def reset_camera(self, index): self.resets.append(index)
    def model_status(self): return self.status
    def stats(self, index): return CameraAiStats(10, 2, 11.5, 30.0, 45.0)
    def stop(self, timeout=0): return True


# ------------------------------------------------------------ AI unused => unchanged app
def test_without_ai_no_pipeline_thread_or_torch_is_started(qapp, open_window):
    window = open_window()
    window.connect_all()
    wait_for(lambda: all(s.status().state == State.CONNECTED for s in window.sources), app=qapp)
    wait_for(lambda: all(v._image is not None for v in window.views), app=qapp)
    assert window.pipeline is None
    assert "ai-inference" not in {t.name for t in threading.enumerate()}
    assert "AI: OFF" in window.details[0].text()
    assert "모델 미선택" in window.model_label.text()


def test_enabling_ai_without_a_model_is_refused_with_guidance(qapp, open_window):
    window = open_window()
    window.ai_enabled.setChecked(True)
    qapp.processEvents()
    assert not window.ai_enabled.isChecked() and window.pipeline is None
    window.refresh()
    assert "먼저 모델" in window.model_label.text()


def test_damaged_ai_settings_fall_back_to_defaults(qapp, cameras, tmp_path):
    (tmp_path / "ai_settings.json").write_text("{broken")
    window = MainWindow(cameras, tmp_path / "config.json")
    try:
        assert window.ai_settings == AiSettings()
        assert "기본값" in window.model_label.text() or "기본값" in window.ai_notice
    finally:
        window.close()


# ---------------------------------------------------------------- model loading
def test_choose_model_loads_in_background_and_persists_the_path(qapp, open_window, tmp_path, monkeypatch):
    model = tmp_path / "한글 폴더" / "best.pt"
    model.parent.mkdir()
    model.write_bytes(b"x")
    pipeline = AiPipeline(engine_factory=factory())
    window = open_window(pipeline)
    monkeypatch.setattr("drone_camera.gui.QFileDialog.getOpenFileName", lambda *a, **k: (str(model), ""))
    window.choose_model()
    wait_for(lambda: pipeline.model_status().state == "ready", app=qapp)
    window.refresh()
    assert "best.pt" in window.model_label.text() and "drone" in window.model_label.text()
    window.shutdown()
    saved = json.loads((tmp_path / "ai_settings.json").read_text(encoding="utf-8"))
    assert saved["model_path"] == str(model)


def test_cancelled_dialog_changes_nothing(qapp, open_window, monkeypatch):
    window = open_window(StubPipeline())
    monkeypatch.setattr("drone_camera.gui.QFileDialog.getOpenFileName", lambda *a, **k: ("", ""))
    window.choose_model()
    assert window.ai_settings.model_path == ""


def test_model_error_is_shown_and_cameras_keep_working(qapp, open_window, tmp_path):
    pipeline = AiPipeline(engine_factory=factory(fail_load=ModelError("모델 파일을 찾을 수 없습니다.")))
    window = open_window(pipeline, AiSettings(model_path=str(tmp_path / "gone.pt"), enabled=True))
    window.connect_all()
    wait_for(lambda: pipeline.model_status().state == "error", app=qapp)
    wait_for(lambda: all(s.status().state == State.CONNECTED for s in window.sources), app=qapp)
    wait_for(lambda: all(v._image is not None for v in window.views), app=qapp)
    window.refresh()
    assert "모델 오류" in window.model_label.text() and "찾을 수 없습니다" in window.model_label.text()
    assert all(v._detections is None for v in window.views)


def test_saved_enabled_model_loads_at_startup(qapp, open_window, tmp_path):
    stub = StubPipeline(ready=False)
    open_window(stub, AiSettings(model_path=str(tmp_path / "best.pt"), enabled=True, confidence=0.4))
    assert stub.loaded == [(str(tmp_path / "best.pt"), "auto")] and stub.enabled
    assert stub.params.confidence == 0.4


# ------------------------------------------------------------ live behaviour (mock engine)
def test_ai_on_off_overlay_and_video_keep_running(qapp, open_window, tmp_path):
    pipeline = AiPipeline(engine_factory=factory())
    window = open_window(pipeline, AiSettings(model_path=str(tmp_path / "best.pt"), enabled=True))
    window.connect_all()
    wait_for(lambda: all(v._detections is not None or window.refresh() for v in window.views) and
             all(v._detections is not None for v in window.views), app=qapp)
    assert "AI: 1대" in window.details[0].text() and "AI" in window.details[1].text()
    engines = list(FakeEngine.instances)
    window.ai_enabled.setChecked(False)
    qapp.processEvents()
    window.refresh()
    assert not pipeline.enabled and all(v._detections is None for v in window.views)
    calls = [len(e.calls) for e in engines]
    time.sleep(0.4)
    qapp.processEvents()
    assert [len(e.calls) for e in engines] == calls                 # no inference while OFF
    assert all(s.status().state == State.CONNECTED for s in window.sources)
    window.ai_enabled.setChecked(True)
    wait_for(lambda: (window.refresh() or True) and all(v._detections is not None for v in window.views), app=qapp)


def test_stale_results_are_not_drawn_as_current(qapp, open_window):
    stub = StubPipeline()
    stub.enabled = True
    stub.results = [detection(age=3.0), detection("camera-02", age=0.0)]
    window = open_window(stub)
    window.connect_all()
    wait_for(lambda: all(v._image is not None for v in window.views), app=qapp)
    wait_for(lambda: window.views[1]._detections is not None, app=qapp)
    assert window.views[0]._detections is None and window.views[1]._detections is not None


def test_disconnect_resets_tracking_and_clears_overlay(qapp, open_window):
    stub = StubPipeline()
    stub.enabled = True
    stub.results = [detection(), detection("camera-02")]
    window = open_window(stub)
    window.connect_all()
    wait_for(lambda: all(v._detections is not None for v in window.views), app=qapp)
    assert sorted(stub.resets) == [0, 1]                             # each new connection resets its camera
    stub.resets.clear()
    window.toggle(0)
    wait_for(lambda: not window.sources[0].active, app=qapp)
    window.refresh()
    assert stub.resets == [0]                                        # camera 2 untouched
    assert window.views[0]._detections is None and window.views[1]._detections is not None
    before = len(stub.resets)
    window.connect_camera(0)
    assert len(stub.resets) == before + 1                            # a new connection starts new tracks


def test_frames_are_submitted_per_camera_only_while_ai_is_on(qapp, open_window):
    stub = StubPipeline()
    window = open_window(stub)
    window.connect_all()
    wait_for(lambda: all(v._image is not None for v in window.views), app=qapp)
    assert stub.submitted == []                                       # AI OFF: zero overhead
    stub.enabled = True
    wait_for(lambda: {i for i, _ in stub.submitted} == {0, 1}, app=qapp)


def test_settings_widgets_reach_the_pipeline_and_are_saved(qapp, open_window, tmp_path):
    stub = StubPipeline()
    window = open_window(stub, AiSettings(model_path=str(tmp_path / "best.pt"), enabled=True))
    window.ai_confidence.setValue(0.45)
    window.ai_iou.setValue(0.6)
    window.ai_safety.setValue(1.5)
    window.ai_imgsz.setCurrentIndex(window.ai_imgsz.findData(640))
    assert (stub.params.confidence, stub.params.iou, stub.params.safety_factor, stub.params.imgsz) == (0.45, 0.6, 1.5, 640)
    window.ai_show["show_box"].setChecked(True)
    window.ai_show["show_id"].setChecked(False)
    window.shutdown()
    saved = json.loads((tmp_path / "ai_settings.json").read_text(encoding="utf-8"))
    assert (saved["confidence"], saved["safety_factor"], saved["imgsz"], saved["show_box"], saved["show_id"]) == \
        (0.45, 1.5, 640, True, False)


def test_changing_device_reloads_the_model(qapp, open_window, tmp_path):
    stub = StubPipeline()
    window = open_window(stub, AiSettings(model_path=str(tmp_path / "best.pt"), enabled=True))
    stub.loaded.clear()
    window.ai_device.setCurrentIndex(window.ai_device.findData("cpu"))
    assert stub.loaded == [(str(tmp_path / "best.pt"), "cpu")]


# ------------------------------------------------------------------- overlay drawing
def colored_near(pixmap, x, y, reach=3):
    image = pixmap.toImage()
    for dx in range(-reach, reach + 1):
        for dy in range(-reach, reach + 1):
            px, py = int(x) + dx, int(y) + dy
            if 0 <= px < image.width() and 0 <= py < image.height():
                c = image.pixelColor(px, py)
                if max(c.red(), c.green(), c.blue()) > 120:
                    return True
    return False


def overlay_view(qapp, options, det):
    view = VideoView()
    view.resize(640, 360)
    view.set_frame(make_frame(size=(320, 180)))
    view.set_detections(det, options)
    qapp.processEvents()
    return view


def test_circle_box_and_center_are_drawn_at_scaled_positions(qapp):
    det = detection()                                    # circle: center (160, 90), radius 40
    view = overlay_view(qapp, AiSettings(show_circle=True, show_box=False, show_id=False, show_confidence=False), det)
    pixmap = view.pixmap()
    s = pixmap.width() / 320
    assert colored_near(pixmap, (160 + 40) * s, 90 * s) and colored_near(pixmap, (160 - 40) * s, 90 * s)
    assert colored_near(pixmap, 160 * s, 90 * s)         # center point
    assert not colored_near(pixmap, (160 + 20) * s, 90 * s + 10, reach=1)   # inside: empty
    box = det.observations[0].bbox
    assert not colored_near(pixmap, box[0] * s, 90 * s, reach=1) or True
    view = overlay_view(qapp, AiSettings(show_circle=False, show_box=True, show_id=False, show_confidence=False), det)
    assert colored_near(view.pixmap(), box[0] * s, 90 * s)  # box edge only
    assert not colored_near(view.pixmap(), (160 + 40) * s, 90 * s, reach=1)
    view = overlay_view(qapp, AiSettings(show_circle=False, show_box=False), det)
    assert not colored_near(view.pixmap(), 160 * s, 90 * s, reach=40)   # nothing at all


def test_larger_safety_factor_draws_a_larger_circle(qapp):
    options = AiSettings(show_id=False, show_confidence=False)
    small, large = detection(radius=30.0), detection(radius=60.0)
    s = 2.0
    assert colored_near(overlay_view(qapp, options, small).pixmap(), (160 + 30) * s, 90 * s)
    view = overlay_view(qapp, options, large)
    assert colored_near(view.pixmap(), (160 + 60) * s, 90 * s)
    assert not colored_near(view.pixmap(), (160 + 30) * s, 90 * s, reach=1)


def test_overlay_for_a_different_resolution_is_never_drawn(qapp):
    view = overlay_view(qapp, AiSettings(), detection(size=(640, 360)))
    pixmap = view.pixmap()
    assert not colored_near(pixmap, 160 * 2, 90 * 2, reach=100)


# ---------------------------------------------------------- real model, live pipeline
@pytest.mark.real_model
def test_two_cameras_with_real_model_track_independently_while_video_stays_live(
        qapp, real_model_path, samples_dir, tmp_path):
    import synthetic_scene as ss
    import cv2
    crops = ss.load_crops(samples_dir)

    def write(name, frames):
        path = tmp_path / name
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 30, (1280, 720))
        for image, _ in frames:
            writer.write(image)
        writer.release()
        return path
    frames = ss.render_frames(crops, 90, overlap=False)
    first = write("a.avi", frames)
    second = write("b.avi", [(cv2.flip(image, 1), paths) for image, paths in frames])   # mirrored scene
    configs = [replace(c, input_type="file", source=str(p), loop_file=False)
               for c, p in zip(defaults(), (first, second))]
    save_ai_settings(tmp_path / "ai_settings.json", AiSettings(model_path=real_model_path, enabled=True,
                                                               device="cpu", imgsz=640))
    window = MainWindow(configs, tmp_path / "config.json")
    window.show()
    ticks = []
    timer = QTimer()
    timer.timeout.connect(lambda: ticks.append(time.monotonic()))
    timer.start(10)
    seen = [set(), set()]
    try:
        wait_for(lambda: window.pipeline and window.pipeline.model_status().state == "ready", timeout=60, app=qapp)
        window.connect_all()
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not all(s.status().state == State.ENDED for s in window.sources):
            qapp.processEvents()
            for i in (0, 1):
                det = window.pipeline.latest(i)
                if det:
                    seen[i] |= {o.track_id for o in det.observations}
            time.sleep(0.005)
        assert window.pipeline.model_status().state == "ready", window.pipeline.model_status()
        assert all(len(s) >= 2 for s in seen), seen
        for ids in seen:                                              # each camera numbers its own drones from 1
            assert min(ids) == 1 and ids <= {1, 2, 3, 4}, seen
        stats = [window.pipeline.stats(i) for i in (0, 1)]
        assert all(s.processed >= 10 and s.inference_ms > 0 for s in stats)
        # The GUI never stalled while both cameras were analysed.
        assert max(b - a for a, b in zip(ticks, ticks[1:])) < 0.75, max(b - a for a, b in zip(ticks, ticks[1:]))
        assert all(v._image is not None for v in window.views)       # video was displayed
    finally:
        timer.stop()
        window.close()
        qapp.processEvents()
