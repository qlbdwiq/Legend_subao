"""Automated tests for Phase 05 experiment data recording, CSV export, and playback analysis."""

import csv

import numpy as np
import pytest

from drone_camera.collision import (
    RISK_CAUTION,
    RISK_DANGER,
    RISK_SAFE,
    RISK_UNKNOWN,
    CollisionAnalyzer,
    CollisionSettings,
)
from drone_camera.recorder import (
    CSV_COLUMNS,
    EVENT_CSV_COLUMNS,
    ExperimentRecorder,
    RecorderError,
    export_events_csv,
    export_ticks_csv,
    load_session,
    summarize_session,
)
from drone_camera.stereo_3d import (
    STEREO_STATUS_OK,
    Drone3DPose,
    StereoFrameResult,
    SyncStatus,
)


def _sample_stereo(t: float, x1: float, x2: float) -> StereoFrameResult:
    sync = SyncStatus(True, 0.004, 0.004, 0.004, 0.0, 0.10, "동기화 정상")
    d1 = Drone3DPose(
        1, 1, 2, (600.0, 360.0), (550.0, 360.0), 25.0, 25.0,
        (x1, 0.0, 8.0), 0.7, 0.4, 0.01, 12.0, 0.05, 0.92, 0.90, t, 1700000000.0 + t,
    )
    d2 = Drone3DPose(
        2, 2, 1, (700.0, 360.0), (650.0, 360.0), 25.0, 25.0,
        (x2, 0.0, 8.0), 0.9, 0.5, 0.01, 12.0, 0.05, 0.90, 0.88, t, 1700000000.0 + t,
    )
    return StereoFrameResult(
        status=STEREO_STATUS_OK,
        status_message="2대 3D 확정",
        is_calibrated=True,
        calibration_type="synthetic",
        sync=sync,
        timestamp=t,
        wall_time=1700000000.0 + t,
        drones=(d1, d2),
    )


def test_experiment_recorder_jsonl_video_csv_and_playback_summary(tmp_path):
    recorder = ExperimentRecorder()
    analyzer = CollisionAnalyzer(
        CollisionSettings(
            warning_distance_m=5.0,
            danger_distance_m=2.5,
            min_closing_speed_mps=45.0,
            uncertainty_margin_sigma=0.0,
        )
    )
    jsonl_path = recorder.start(tmp_path / "한글 기록 폴더", session_name="test_run", record_video=True)
    assert recorder.active

    img1 = np.full((180, 320, 3), (40, 80, 120), dtype=np.uint8)
    img2 = np.full((180, 320, 3), (80, 40, 120), dtype=np.uint8)

    # Tick 1: distance 8.0 m -> SAFE
    # Tick 2: distance 4.0 m -> CAUTION
    # Tick 3: distance 1.8 m -> DANGER
    positions = [(-4.0, 4.0), (-2.0, 2.0), (-0.9, 0.9)]
    for idx, (x1, x2) in enumerate(positions):
        t = 10.0 + idx * 0.2
        stereo = _sample_stereo(t, x1, x2)
        collision = analyzer.analyze(stereo)
        recorder.record_tick(None, None, stereo, collision, frame_01=img1, frame_02=img2)

    stopped_path = recorder.stop()
    assert not recorder.active
    assert stopped_path == jsonl_path and jsonl_path.is_file()
    for vpath in recorder.video_paths:
        assert vpath is not None and vpath.is_file() and vpath.stat().st_size > 0

    # Load session back and summarize
    loaded_ticks = load_session(jsonl_path)
    assert len(loaded_ticks) == 3
    summary = summarize_session(loaded_ticks)
    assert summary.total_ticks == 3
    assert summary.valid_pair_ticks == 3
    assert summary.duration_s == pytest.approx(0.4)
    assert summary.calibration_type == "synthetic"
    assert summary.min_distance_m == pytest.approx(1.8)
    assert summary.risk_counts[RISK_SAFE] == 1
    assert summary.risk_counts[RISK_CAUTION] == 1
    assert summary.risk_counts[RISK_DANGER] == 1
    assert summary.mean_reprojection_error_px == pytest.approx(0.8)

    # Export frame telemetry CSV
    csv_file = export_ticks_csv(tmp_path / "telemetry.csv", loaded_ticks)
    with csv_file.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 3
    assert tuple(rows[0].keys()) == CSV_COLUMNS
    assert float(rows[0]["distance_m"]) == pytest.approx(8.0)
    assert float(rows[2]["distance_m"]) == pytest.approx(1.8)
    assert rows[2]["risk_level"] == RISK_DANGER

    # Export risk events CSV
    events_csv = export_events_csv(tmp_path / "events.csv", analyzer.events)
    with events_csv.open("r", encoding="utf-8") as f:
        event_rows = list(csv.DictReader(f))
    assert len(event_rows) == 2  # CAUTION and DANGER transitions
    assert tuple(event_rows[0].keys()) == EVENT_CSV_COLUMNS
    assert [r["risk_level"] for r in event_rows] == [RISK_CAUTION, RISK_DANGER]


def test_load_session_rejects_missing_or_corrupt_files(tmp_path):
    with pytest.raises(RecorderError):
        load_session(tmp_path / "nonexistent.jsonl")

    empty = tmp_path / "empty.jsonl"
    empty.write_text("\n", encoding="utf-8")
    with pytest.raises(RecorderError):
        load_session(empty)

    corrupt = tmp_path / "corrupt.jsonl"
    corrupt.write_text("{not valid json\n", encoding="utf-8")
    with pytest.raises(RecorderError):
        load_session(corrupt)
