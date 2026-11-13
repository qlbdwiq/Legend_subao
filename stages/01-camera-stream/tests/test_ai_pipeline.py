"""AiPipeline logic with mock engines (no torch, no YOLO)."""
import threading
import time

import pytest

from conftest import wait_for
from drone_camera.ai_pipeline import AiPipeline, InferenceParams
from drone_camera.detections import RawTrack
from drone_camera.engine import ModelError
import fakes
from fakes import FakeEngine, factory, make_frame


@pytest.fixture
def make_pipeline():
    created = []

    def build(**kwargs):
        pipeline = AiPipeline(engine_factory=factory(**kwargs))
        created.append(pipeline)
        return pipeline
    yield build
    for pipeline in created:
        assert pipeline.stop(5)


def ready(pipeline, **load):
    pipeline.load_model("model.pt", "cpu")
    wait_for(lambda: pipeline.model_status().state == "ready")
    pipeline.set_enabled(True)
    return pipeline


def test_loading_and_inference_run_off_the_calling_thread(make_pipeline):
    pipeline = make_pipeline()
    pipeline.load_model("model.pt", "cpu")
    assert pipeline.model_status().state in ("loading", "ready")
    wait_for(lambda: pipeline.model_status().state == "ready")
    pipeline.set_enabled(True)
    assert pipeline.submit(0, make_frame("camera-01", 1))
    wait_for(lambda: pipeline.latest(0))
    names = {call[0] for engine in FakeEngine.instances for call in engine.calls}
    assert names == {"ai-inference"} and threading.current_thread().name not in names


def test_model_errors_are_reported_and_do_not_raise(make_pipeline):
    pipeline = make_pipeline(fail_load=ModelError("모델 파일을 찾을 수 없습니다."))
    pipeline.load_model("missing.pt", "cpu")
    status = wait_for(lambda: pipeline.model_status().state == "error" and pipeline.model_status())
    assert "찾을 수 없습니다" in status.message
    pipeline.set_enabled(True)
    assert pipeline.submit(0, make_frame()) is False        # nothing is queued without a model


def test_unexpected_load_exception_is_contained(make_pipeline):
    pipeline = make_pipeline(fail_load=RuntimeError("secret://token@host"))
    pipeline.load_model("m.pt", "cpu")
    status = wait_for(lambda: pipeline.model_status().state == "error" and pipeline.model_status())
    assert "token" not in status.message and "RuntimeError" in status.message


def test_only_the_newest_frame_is_processed_and_backlog_never_builds(make_pipeline):
    pipeline = ready(make_pipeline(delay=0.05))
    started = time.perf_counter()
    for sequence in range(1, 41):
        pipeline.submit(0, make_frame("camera-01", sequence, value=sequence))
    assert time.perf_counter() - started < 0.5              # submit never waits for inference
    wait_for(lambda: pipeline.latest(0) and pipeline.latest(0).frame_sequence == 40)
    stats = pipeline.stats(0)
    assert stats.processed < 40 and stats.skipped >= 30 and stats.processed + stats.skipped >= 39
    engine = FakeEngine.instances[0]
    assert engine.images == sorted(engine.images)           # never an older frame after a newer one


def test_cameras_keep_independent_engines_results_and_track_state(make_pipeline):
    pipeline = ready(make_pipeline())
    first, second = FakeEngine.instances
    assert first is not second
    pipeline.submit(0, make_frame("camera-01", 5, value=11))
    pipeline.submit(1, make_frame("camera-02", 9, value=22))
    wait_for(lambda: pipeline.latest(0) and pipeline.latest(1))
    assert first.images == [11] and second.images == [22]   # each camera feeds only its own engine
    a, b = pipeline.latest(0), pipeline.latest(1)
    assert (a.camera_id, a.frame_sequence) == ("camera-01", 5)
    assert (b.camera_id, b.frame_sequence) == ("camera-02", 9)
    # Both engines emit "ID 1": the same number in two cameras is not the same drone.
    assert a.observations[0].track_id == b.observations[0].track_id == 1
    pipeline.submit(0, make_frame("camera-01", 6))
    pipeline.submit(1, make_frame("camera-02", 10))
    wait_for(lambda: pipeline.latest(0).frame_sequence == 6 and pipeline.latest(1).frame_sequence == 10)
    assert pipeline.latest(0).observations[0].state == "tracked"
    pipeline.reset_camera(0)                                  # reconnecting camera 1 only
    assert pipeline.latest(0) is None and pipeline.latest(1).frame_sequence == 10
    pipeline.submit(0, make_frame("camera-01", 1))
    wait_for(lambda: pipeline.latest(0))
    assert pipeline.latest(0).observations[0].state == "new"   # fresh track lifecycle
    assert first.resets >= 1


def test_reset_discards_an_inflight_result(make_pipeline):
    pipeline = ready(make_pipeline(delay=0.3))
    pipeline.submit(0, make_frame("camera-01", 1))
    wait_for(lambda: FakeEngine.instances[0].calls)          # inference of the old session started
    pipeline.reset_camera(0)
    time.sleep(0.5)
    assert pipeline.latest(0) is None and pipeline.stats(0).processed == 0


def test_toggle_off_stops_processing_and_clears_results(make_pipeline):
    pipeline = ready(make_pipeline())
    pipeline.submit(0, make_frame())
    wait_for(lambda: pipeline.latest(0))
    pipeline.set_enabled(False)
    assert pipeline.latest(0) is None
    assert pipeline.submit(0, make_frame()) is False
    time.sleep(0.2)
    assert pipeline.latest(0) is None
    pipeline.set_enabled(True)                               # never resumes the old tracks
    pipeline.submit(0, make_frame())
    wait_for(lambda: pipeline.latest(0))
    assert pipeline.latest(0).observations[0].state == "new"


def test_parameters_and_safety_factor_are_applied_to_later_frames(make_pipeline):
    pipeline = ready(make_pipeline(tracks=[RawTrack(1, 0, 0, 30, 40, 0.9)]))
    pipeline.configure(InferenceParams(0.4, 0.6, 640, 1.5))
    pipeline.submit(0, make_frame("camera-01", 1))
    wait_for(lambda: pipeline.latest(0))
    assert FakeEngine.instances[0].calls[-1][2:] == (0.4, 0.6, 640)
    obs = pipeline.latest(0).observations[0]
    assert obs.base_radius == 25 and obs.safe_radius == 37.5 and obs.safety_factor == 1.5
    pipeline.configure(InferenceParams(0.4, 0.6, 640, 1.0))
    pipeline.submit(0, make_frame("camera-01", 2))
    wait_for(lambda: pipeline.latest(0).frame_sequence == 2)
    assert pipeline.latest(0).observations[0].safe_radius == 25


def test_repeated_inference_failures_disable_ai_without_crashing(make_pipeline):
    pipeline = ready(make_pipeline(fail_track=True))
    for sequence in range(1, 8):
        pipeline.submit(0, make_frame("camera-01", sequence))
        time.sleep(0.05)
    status = wait_for(lambda: pipeline.model_status().state == "error" and pipeline.model_status())
    assert "RuntimeError" in status.message and "boom" not in status.message
    assert pipeline.latest(0) is None


def test_model_reload_replaces_both_engines_and_clears_state(make_pipeline):
    pipeline = ready(make_pipeline())
    pipeline.submit(0, make_frame())
    wait_for(lambda: pipeline.latest(0))
    old = list(FakeEngine.instances)
    pipeline.load_model("other.pt", "cpu")
    wait_for(lambda: pipeline.model_status().state == "ready" and pipeline.model_status().info.path == "other.pt")
    assert pipeline.latest(0) is None
    assert not set(old) & {e for e in FakeEngine.instances[-2:]}


def test_stats_report_ai_fps_and_latency_separately_from_video(make_pipeline):
    pipeline = ready(make_pipeline(delay=0.02))
    for sequence in range(1, 60):
        pipeline.submit(0, make_frame("camera-01", sequence))
        time.sleep(0.01)
    wait_for(lambda: pipeline.stats(0).processed >= 5)
    stats = pipeline.stats(0)
    assert 5 < stats.fps < 100 and stats.inference_ms >= 15 and stats.latency_ms >= stats.inference_ms - 1
    assert pipeline.stats(1).processed == 0 and pipeline.stats(1).fps == 0


def test_stop_is_idempotent_and_joins_the_thread(make_pipeline):
    pipeline = ready(make_pipeline())
    assert pipeline.stop(5) and pipeline.stop(5)
