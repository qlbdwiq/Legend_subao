"""End-to-end GUI integration tests for Phase 03~05: 3D stereo tracking, collision warnings,
calibration wizard, experiment recording, CSV export, and session playback."""

from dataclasses import replace
import json
import math
import time

import numpy as np
import pytest

from conftest import wait_for
from drone_camera.ai_pipeline import CameraAiStats, ModelStatus
from drone_camera.calibration import (
    create_synthetic_calibration,
    project_points_3d,
    render_synthetic_chessboard_image,
)
from drone_camera.capture import Frame, State
from drone_camera.collision import RISK_CAUTION, RISK_DANGER, RISK_SAFE, RISK_UNKNOWN
from drone_camera.config import defaults
from drone_camera.dashboard_widgets import CalibrationWizardDialog, PlaybackDialog
from drone_camera.detections import DroneObservation, FrameDetections
from drone_camera.engine import ModelInfo
from drone_camera.gui import MainWindow


class ScriptedStereoPipeline:
    """Stub AI pipeline returning scripted 2D observations for Camera 1 and Camera 2."""

    def __init__(self):
        self.enabled = True
        self.results = [None, None]
        self.status = ModelStatus("ready", "", ModelInfo("best.pt", {0: "drone"}, "cpu", "", "8.4", "2.15"))

    def load_model(self, path, device="auto"):
        pass

    def set_enabled(self, value):
        self.enabled = value

    def configure(self, params):
        pass

    def submit(self, index, frame):
        return True

    def latest(self, index):
        return self.results[index]

    def reset_camera(self, index):
        pass

    def model_status(self):
        return self.status

    def stats(self, index):
        return CameraAiStats(25, 0, 24.0, 14.0, 22.0)

    def stop(self, timeout=0):
        return True


def _build_stereo_detections(
    calib,
    xyz_list: list[tuple[float, float, float]],
    t1: float,
    t2: float | None = None,
    cam1_ids: list[int] | None = None,
    cam2_ids: list[int] | None = None,
):
    if t2 is None:
        t2 = t1
    pts = np.array(xyz_list, dtype=np.float64)
    px1 = project_points_3d(pts, calib.camera_01) if len(xyz_list) else np.zeros((0, 2))
    px2 = project_points_3d(pts, calib.camera_02, calib.R, calib.T) if len(xyz_list) else np.zeros((0, 2))
    ids1 = cam1_ids or [i + 1 for i in range(len(xyz_list))]
    ids2 = cam2_ids or [i + 10 for i in range(len(xyz_list))]

    obs1 = []
    for tid, (cx, cy) in zip(ids1, px1):
        obs1.append(
            DroneObservation(
                "camera-01", 1, 1700000000.0 + t1, t1, tid,
                (float(cx - 15), float(cy - 15), float(cx + 15), float(cy + 15)),
                (float(cx), float(cy)), 21.2, 25.4, 1.2, 0.92, "tracked",
            )
        )
    obs2 = []
    for tid, (cx, cy) in zip(ids2, px2):
        obs2.append(
            DroneObservation(
                "camera-02", 1, 1700000000.0 + t2, t2, tid,
                (float(cx - 15), float(cy - 15), float(cx + 15), float(cy + 15)),
                (float(cx), float(cy)), 21.2, 25.4, 1.2, 0.91, "tracked",
            )
        )
    now_mono = time.monotonic()
    det1 = FrameDetections("camera-01", 1, 1700000000.0 + t1, t1, now_mono, 12.0, calib.camera_01.image_size, tuple(obs1))
    det2 = FrameDetections("camera-02", 1, 1700000000.0 + t2, t2, now_mono, 12.0, calib.camera_02.image_size, tuple(obs2))
    return det1, det2


def test_dashboard_uncalibrated_to_calibrated_3d_collision_and_recording_workflow(
    qapp, video, tmp_path, monkeypatch
):
    configs = [replace(c, input_type="file", source=str(video), loop_file=True) for c in defaults()]
    pipeline = ScriptedStereoPipeline()
    window = MainWindow(configs, tmp_path / "config.json", pipeline=pipeline)
    window.show()
    try:
        window.connect_all()
        wait_for(lambda: all(s.status().state == State.CONNECTED for s in window.sources), app=qapp)

        # 1. Uncalibrated state: must be UNKNOWN ("판단 불가") and never output fake 3D coordinates
        calib = create_synthetic_calibration(baseline_m=2.0, convergence_deg=5.0)
        t_now = time.monotonic()
        pipeline.results = list(
            _build_stereo_detections(calib, [(-3.5, -0.6, 8.0), (3.5, 0.6, 8.0)], t_now)
        )
        window.refresh()
        assert window.calibration is None
        assert window.latest_collision_result.risk_level == RISK_UNKNOWN
        assert "판단 불가" in window.risk_banner.text()
        assert "미보정" in window.status_overview_label.text()

        # 2. Apply synthetic calibration and start experiment recording
        window.set_calibration(calib, persist=True)
        assert (tmp_path / "stereo_calibration.json").is_file()
        window.toggle_recording()
        assert window.recorder.active

        # Feed SAFE pair (distance ~ 7.10 m > warning 5.0 m), with non-matching ByteTrack IDs (1,2 vs 20,10)
        t_now += 2.0
        pipeline.results = list(
            _build_stereo_detections(
                calib, [(-3.5, -0.6, 8.0), (3.5, 0.6, 8.0)], t_now, cam1_ids=[1, 2], cam2_ids=[20, 10]
            )
        )
        window.refresh()
        assert window.latest_collision_result.risk_level == RISK_SAFE
        assert "안전 / SAFE" in window.risk_banner.text()
        assert "7.10 m" in window.status_overview_label.text()
        assert "Cam1 ID:1 ↔ Cam2 ID:20" in window.drone_telemetry_label.text()
        assert "Cam1 ID:2 ↔ Cam2 ID:10" in window.drone_telemetry_label.text()

        # Exercise 3D visualization view modes
        for mode in ("top", "front", "3d"):
            window.space_3d_widget.set_view_mode(mode)
            window.space_3d_widget.grab()

        # 3. Feed CAUTION pair (distance ~ 3.93 m <= warning 5.0 m)
        t_now += 2.0
        pipeline.results = list(
            _build_stereo_detections(calib, [(-1.9, -0.5, 8.0), (1.9, 0.5, 8.0)], t_now)
        )
        window.refresh()
        assert window.latest_collision_result.risk_level == RISK_CAUTION
        assert "주의 / CAUTION" in window.risk_banner.text()

        # 4. Feed DANGER pair (distance ~ 1.79 m <= danger 2.5 m)
        t_now += 2.0
        pipeline.results = list(
            _build_stereo_detections(calib, [(-0.8, -0.4, 8.0), (0.8, 0.4, 8.0)], t_now)
        )
        window.refresh()
        assert window.latest_collision_result.risk_level == RISK_DANGER
        assert "위험 / DANGER" in window.risk_banner.text()
        assert window.audio_beep_count >= 1
        assert "DANGER" in window.risk_events_label.text()

        # 5. Feed unsynchronized frames (delta = 0.45 s > 0.10 s) -> must become UNKNOWN ("판단 불가")
        t_now += 2.0
        pipeline.results = list(
            _build_stereo_detections(
                calib, [(-3.5, -0.6, 8.0), (3.5, 0.6, 8.0)], t_now, t2=t_now - 0.45
            )
        )
        window.refresh()
        assert window.latest_collision_result.risk_level == RISK_UNKNOWN
        assert "판단 불가" in window.risk_banner.text()

        # 6. Stop recording and export CSVs
        window.toggle_recording()
        assert not window.recorder.active
        assert len(window.recorder.ticks) >= 4

        csv_out = tmp_path / "exported_telemetry.csv"
        events_out = tmp_path / "exported_events.csv"
        monkeypatch.setattr(
            "drone_camera.gui.QFileDialog.getSaveFileName", lambda *a, **k: (str(csv_out), "")
        )
        window.export_recorded_csv()
        assert csv_out.is_file()

        monkeypatch.setattr(
            "drone_camera.gui.QFileDialog.getSaveFileName", lambda *a, **k: (str(events_out), "")
        )
        window.export_risk_events_csv()
        assert events_out.is_file()

        # 7. Test PlaybackDialog on the recorded session
        player = PlaybackDialog(jsonl_path=window.recorder.jsonl_path, parent=window)
        assert len(player.ticks) == len(window.recorder.ticks)
        player.seek_to(1)
        assert "주의" in player.tick_detail_label.text() or "CAUTION" in player.tick_detail_label.text()
        player.seek_to(2)
        assert "위험" in player.tick_detail_label.text() or "DANGER" in player.tick_detail_label.text()
        player.close()
    finally:
        window.close()
        qapp.processEvents()


def test_threshold_controls_persist_and_calibration_wizard_works(qapp, tmp_path, monkeypatch):
    monkeypatch.setattr("drone_camera.dashboard_widgets.QMessageBox.warning", lambda *a, **k: None)
    monkeypatch.setattr("drone_camera.dashboard_widgets.QMessageBox.critical", lambda *a, **k: None)
    calib = create_synthetic_calibration(image_size=(640, 480), focal_px=550.0, baseline_m=0.10, convergence_deg=0.0)
    cols, rows, sq_m = 6, 4, 0.06
    pose_rvec, pose_tvec = (0.04, -0.05, 0.02), (-0.12, -0.10, 1.5)
    img1 = render_synthetic_chessboard_image(calib.camera_01, cols, rows, sq_m, pose_rvec, pose_tvec)
    img2 = render_synthetic_chessboard_image(
        calib.camera_02, cols, rows, sq_m, pose_rvec, pose_tvec, calib.R, calib.T
    )
    f1 = Frame("camera-01", 1, time.time(), time.monotonic(), img1)
    f2 = Frame("camera-02", 1, time.time(), time.monotonic(), img2)

    wizard = CalibrationWizardDialog(None, frame_provider=lambda: (f1, f2))
    wizard.cols_spin.setValue(cols)
    wizard.rows_spin.setValue(rows)
    wizard.square_mm_spin.setValue(sq_m * 1000.0)
    wizard.capture_from_live_cameras()
    assert len(wizard.captured_corners_01) == 1

    wizard.generate_and_calibrate_synthetic_board()
    assert wizard.result_calibration is not None
    assert wizard.result_calibration.calibration_type == "synthetic"
    assert wizard.result_calibration.rms_error < 0.1

    wizard.apply_quick_layout_preset()
    assert wizard.result_calibration.calibration_type == "auto"
    wizard.close()

    # Check threshold spinboxes in MainWindow save to collision_settings.json
    window = MainWindow(defaults(), tmp_path / "config.json")
    try:
        window.apply_quick_layout_calibration()
        assert window.calibration is not None
        assert window.calibration.calibration_type == "auto"
        window.monocular_fallback_check.setChecked(True)
        assert window.stereo_tracker.monocular_fallback is True

        window.warn_dist_spin.setValue(7.5)
        window.danger_dist_spin.setValue(3.2)
        window.horizon_spin.setValue(6.0)
        assert window.collision_settings.warning_distance_m == 7.5
        assert window.collision_settings.danger_distance_m == 3.2
        saved = json.loads((tmp_path / "collision_settings.json").read_text(encoding="utf-8"))
        assert saved["warning_distance_m"] == 7.5 and saved["danger_distance_m"] == 3.2
    finally:
        window.close()
        qapp.processEvents()
