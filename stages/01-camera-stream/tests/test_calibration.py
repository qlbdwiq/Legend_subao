"""Automated tests for Phase 03 camera calibration, lens distortion, and persistence."""

import json
import math

import numpy as np
import pytest

from drone_camera.calibration import (
    CalibrationError,
    CameraIntrinsics,
    StereoCalibration,
    calibrate_single_camera,
    calibrate_stereo_cameras,
    calibration_path_for,
    chessboard_object_points,
    create_synthetic_calibration,
    detect_chessboard_corners,
    draw_chessboard_preview,
    load_calibration,
    project_chessboard_corners_pair,
    project_points_3d,
    render_synthetic_chessboard_image,
    save_calibration,
    undistort_frame,
    undistort_point_px,
    undistort_points_normalized,
)


def _sample_board_poses():
    return [
        ((0.05, -0.08, 0.02), (-0.35, -0.25, 2.2)),
        ((-0.12, 0.10, -0.05), (0.10, -0.20, 2.5)),
        ((0.18, 0.05, 0.12), (-0.20, 0.05, 2.8)),
        ((-0.08, -0.18, -0.10), (0.20, 0.10, 2.4)),
        ((0.15, -0.14, 0.20), (-0.10, -0.15, 2.0)),
        ((-0.20, 0.16, -0.15), (0.05, -0.05, 3.1)),
        ((0.02, 0.22, -0.08), (-0.30, -0.10, 2.6)),
        ((-0.16, -0.06, 0.18), (0.15, -0.22, 2.3)),
    ]


def test_uncalibrated_load_returns_none_and_never_invents_parameters(tmp_path):
    path = calibration_path_for(tmp_path / "config.json")
    assert path.name == "stereo_calibration.json"
    assert load_calibration(path) is None


def test_calibration_save_and_load_roundtrip_preserves_all_fields(tmp_path):
    calib = create_synthetic_calibration(
        image_size=(1280, 720),
        focal_px=920.0,
        baseline_m=2.5,
        convergence_deg=8.0,
        dist_coeffs_01=(-0.12, 0.04, 0.001, -0.001, 0.0),
        dist_coeffs_02=(-0.10, 0.03, -0.001, 0.001, 0.0),
        time_offset_s=0.025,
        max_sync_delta_s=0.08,
        notes="단위 테스트 보정",
    )
    path = tmp_path / "한글 보정 폴더" / "stereo_calibration.json"
    save_calibration(path, calib)
    loaded = load_calibration(path)
    assert loaded == calib
    assert loaded.calibration_type == "synthetic"
    assert not loaded.is_field_calibrated
    assert [p.name for p in path.parent.iterdir()] == ["stereo_calibration.json"]


def test_baseline_length_override_scales_translation_vector():
    calib = create_synthetic_calibration(baseline_m=2.0, convergence_deg=10.0)
    updated = calib.with_baseline(3.75)
    assert updated.baseline_m == pytest.approx(3.75)
    assert np.linalg.norm(updated.T) == pytest.approx(3.75, rel=1e-6)
    cam2_center = updated.camera_2_center_in_cam1()
    assert np.linalg.norm(cam2_center) == pytest.approx(3.75, rel=1e-6)


def test_lens_distortion_and_undistortion_roundtrip():
    calib = create_synthetic_calibration(
        image_size=(1280, 720),
        focal_px=900.0,
        dist_coeffs_01=(-0.18, 0.06, 0.0015, -0.001, 0.0),
    )
    intr = calib.camera_01
    pts_3d = np.array([
        [-0.8, -0.4, 4.0],
        [0.6, 0.35, 3.5],
        [0.0, 0.0, 5.0],
    ], dtype=np.float64)
    distorted_px = project_points_3d(pts_3d, intr)
    norm_recovered = undistort_points_normalized(distorted_px, intr)
    norm_truth = pts_3d[:, :2] / pts_3d[:, 2:3]
    np.testing.assert_allclose(norm_recovered, norm_truth, atol=1e-5)

    # Ideal pixel coordinate comparison
    u_ideal, v_ideal = undistort_point_px(distorted_px[0, 0], distorted_px[0, 1], intr)
    assert u_ideal == pytest.approx(intr.fx * norm_truth[0, 0] + intr.cx, abs=1e-3)
    assert v_ideal == pytest.approx(intr.fy * norm_truth[0, 1] + intr.cy, abs=1e-3)

    # Full frame undistortion keeps shape and dtype
    raw_img = np.zeros((720, 1280, 3), dtype=np.uint8)
    undist_img = undistort_frame(raw_img, intr)
    assert undist_img.shape == raw_img.shape and undist_img.dtype == np.uint8


def test_single_and_stereo_calibration_numerical_accuracy_from_projections():
    true_calib = create_synthetic_calibration(
        image_size=(1280, 720),
        focal_px=880.0,
        baseline_m=1.20,
        convergence_deg=6.0,
        dist_coeffs_01=(-0.08, 0.02, 0.0005, -0.0005, 0.0),
        dist_coeffs_02=(-0.06, 0.015, -0.0004, 0.0006, 0.0),
    )
    cols, rows, sq_m = 7, 5, 0.05
    corners_1, corners_2 = [], []
    for rvec, tvec in _sample_board_poses():
        c1, c2 = project_chessboard_corners_pair(true_calib, cols, rows, sq_m, rvec, tvec)
        corners_1.append(c1)
        corners_2.append(c2)

    estimated = calibrate_stereo_cameras(
        corners_1,
        corners_2,
        (1280, 720),
        (1280, 720),
        cols,
        rows,
        sq_m,
        calibration_type="field",
    )
    assert estimated.is_field_calibrated
    assert estimated.rms_error < 0.05
    assert estimated.camera_01.rms_error < 0.05
    assert estimated.camera_02.rms_error < 0.05
    assert estimated.camera_01.fx == pytest.approx(880.0, rel=2e-3)
    assert estimated.camera_02.fx == pytest.approx(880.0, rel=2e-3)
    assert estimated.baseline_m == pytest.approx(1.20, abs=0.005)
    np.testing.assert_allclose(estimated.R, true_calib.R, atol=2e-3)
    np.testing.assert_allclose(estimated.T, true_calib.T, atol=5e-3)


def test_rendered_chessboard_image_corner_detection_and_preview():
    calib = create_synthetic_calibration(image_size=(640, 480), focal_px=550.0, baseline_m=0.8)
    cols, rows, sq_m = 6, 4, 0.06
    image = render_synthetic_chessboard_image(
        calib.camera_01, cols, rows, sq_m, (0.04, -0.05, 0.02), (-0.15, -0.10, 1.4)
    )
    detected = detect_chessboard_corners(image, cols, rows)
    assert detected is not None
    assert detected.shape == (cols * rows, 1, 2)
    preview = draw_chessboard_preview(image, cols, rows, detected)
    assert preview.shape == image.shape
    assert not np.array_equal(preview, image)

    # Blank image returns None without raising
    blank = np.full((480, 640, 3), 128, dtype=np.uint8)
    assert detect_chessboard_corners(blank, cols, rows) is None


@pytest.mark.parametrize("bad_Updater", [
    lambda d: d.update(version=2),
    lambda d: d.update(calibration_type="unknown"),
    lambda d: d.update(baseline_m=-1.0),
    lambda d: d.update(baseline_m=10.0),  # Mismatches translation vector norm
    lambda d: d.update(rotation=[[1, 0, 0], [0, 1, 0], [0, 0, -1]]),  # Reflection det = -1
    lambda d: d.update(max_sync_delta_s=0.0),
    lambda d: d["camera_01"].update(camera_id="wrong"),
    lambda d: d["camera_01"].update(camera_matrix=[[0, 0, 320], [0, 500, 240], [0, 0, 1]]),
])
def test_invalid_calibration_data_is_rejected(bad_Updater, tmp_path):
    valid = create_synthetic_calibration().to_dict()
    bad_Updater(valid)
    path = tmp_path / "bad_calib.json"
    path.write_text(json.dumps(valid), encoding="utf-8")
    with pytest.raises(CalibrationError):
        load_calibration(path)
