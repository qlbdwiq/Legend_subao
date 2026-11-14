"""Automated tests for Phase 03 stereo triangulation, time sync, and cross-camera matching."""

import math

import numpy as np
import pytest

from drone_camera.calibration import create_synthetic_calibration, project_points_3d
from drone_camera.detections import DroneObservation, FrameDetections
from drone_camera.stereo_3d import (
    STEREO_STATUS_AMBIGUOUS,
    STEREO_STATUS_NO_MATCH,
    STEREO_STATUS_OK,
    STEREO_STATUS_UNCALIBRATED,
    STEREO_STATUS_UNSYNCHRONIZED,
    StereoTracker3D,
    estimate_time_offset_s,
    evaluate_frame_sync,
    symmetric_epipolar_distance_px,
    triangulate_points,
)


def _make_obs(camera_id: str, track_id: int | None, center: tuple[float, float],
              t_mono: float = 100.0, conf: float = 0.9, radius: float = 24.0) -> DroneObservation:
    cx, cy = float(center[0]), float(center[1])
    half = radius / 1.2 / math.sqrt(2)
    return DroneObservation(
        camera_id=camera_id,
        frame_sequence=1,
        received_at=1700000000.0 + t_mono,
        received_monotonic=t_mono,
        track_id=track_id,
        bbox=(cx - half, cy - half, cx + half, cy + half),
        center=(cx, cy),
        base_radius=radius / 1.2,
        safe_radius=radius,
        safety_factor=1.2,
        confidence=conf,
        state="tracked",
    )


def _make_frame_det(camera_id: str, observations: list[DroneObservation], t_mono: float = 100.0,
                    size: tuple[int, int] = (1280, 720)) -> FrameDetections:
    return FrameDetections(
        camera_id=camera_id,
        frame_sequence=1,
        received_at=1700000000.0 + t_mono,
        received_monotonic=t_mono,
        completed_monotonic=t_mono + 0.015,
        inference_ms=12.0,
        image_size=size,
        observations=tuple(observations),
    )


def test_uncalibrated_tracker_never_outputs_fake_3d_coordinates():
    tracker = StereoTracker3D()
    det1 = _make_frame_det("camera-01", [_make_obs("camera-01", 1, (640.0, 360.0))])
    det2 = _make_frame_det("camera-02", [_make_obs("camera-02", 1, (520.0, 360.0))])
    result = tracker.process(det1, det2, calibration=None)
    assert result.status == STEREO_STATUS_UNCALIBRATED
    assert not result.is_calibrated
    assert result.drones == ()
    assert "보정 데이터가 없습니다" in result.status_message


@pytest.mark.parametrize("true_xyz", [
    (0.0, 0.0, 5.0),
    (1.2, -0.6, 8.0),
    (-1.5, 0.9, 12.5),
    (2.5, 1.1, 20.0),
    (0.4, -0.3, 3.2),
])
def test_synthetic_triangulation_recovers_exact_3d_coordinates_with_distortion(true_xyz):
    calib = create_synthetic_calibration(
        image_size=(1280, 720),
        focal_px=950.0,
        baseline_m=2.0,
        convergence_deg=8.0,
        dist_coeffs_01=(-0.14, 0.04, 0.001, -0.001, 0.0),
        dist_coeffs_02=(-0.11, 0.03, -0.001, 0.001, 0.0),
    )
    pt_arr = np.array([true_xyz], dtype=np.float64)
    px1 = tuple(project_points_3d(pt_arr, calib.camera_01)[0])
    px2 = tuple(project_points_3d(pt_arr, calib.camera_02, calib.R, calib.T)[0])

    tri = triangulate_points(px1, px2, calib, detector_confidence=0.92)
    assert tri is not None
    assert math.dist(tri.xyz_m, true_xyz) < 1e-4  # Sub-millimeter error on noise-free projection
    assert tri.reprojection_error_px < 1e-3
    assert tri.epipolar_error_px < 1e-3
    assert tri.ray_gap_m < 1e-5
    assert tri.parallax_deg > 3.0
    assert tri.confidence_score > 0.85
    assert tri.position_uncertainty_m > 0.0


def test_noisy_pixel_observations_bound_3d_error_and_report_reprojection_metrics():
    calib = create_synthetic_calibration(image_size=(1280, 720), focal_px=900.0, baseline_m=2.5)
    true_xyz = (1.0, -0.4, 8.0)
    pt_arr = np.array([true_xyz], dtype=np.float64)
    px1 = project_points_3d(pt_arr, calib.camera_01)[0]
    px2 = project_points_3d(pt_arr, calib.camera_02, calib.R, calib.T)[0]

    # Add 1.5 px vertical/horizontal perturbation
    noisy_px1 = (float(px1[0] + 1.2), float(px1[1] - 1.5))
    noisy_px2 = (float(px2[0] - 1.0), float(px2[1] + 1.5))
    tri = triangulate_points(noisy_px1, noisy_px2, calib)
    assert tri is not None
    error_m = math.dist(tri.xyz_m, true_xyz)
    assert error_m < 0.12  # Within 12 cm at 8m depth
    assert 0.5 < tri.reprojection_error_px < 5.0
    assert symmetric_epipolar_distance_px(noisy_px1, noisy_px2, calib) > 0.5


def test_cross_camera_matching_does_not_assume_same_bytetrack_ids():
    """Camera 1 assigns IDs (1, 2) while Camera 2 assigns swapped/arbitrary IDs (9, 3)."""
    calib = create_synthetic_calibration(
        image_size=(1280, 720), focal_px=900.0, baseline_m=2.0, convergence_deg=5.0
    )
    drone_a_xyz = (-1.2, -0.8, 7.0)
    drone_b_xyz = (2.4, 1.1, 10.5)
    pts = np.array([drone_a_xyz, drone_b_xyz], dtype=np.float64)
    px1 = project_points_3d(pts, calib.camera_01)
    px2 = project_points_3d(pts, calib.camera_02, calib.R, calib.T)

    # Notice: in Camera 1, Drone A is ID 1 and Drone B is ID 2.
    # In Camera 2, Drone B is listed first with ID 1, and Drone A is listed second with ID 2!
    det1 = _make_frame_det("camera-01", [
        _make_obs("camera-01", track_id=1, center=tuple(px1[0])),
        _make_obs("camera-01", track_id=2, center=tuple(px1[1])),
    ])
    det2 = _make_frame_det("camera-02", [
        _make_obs("camera-02", track_id=1, center=tuple(px2[1])),  # Same track_id=1, DIFFERENT drone!
        _make_obs("camera-02", track_id=2, center=tuple(px2[0])),
    ])

    tracker = StereoTracker3D()
    res = tracker.process(det1, det2, calib)
    assert res.status == STEREO_STATUS_OK
    assert len(res.drones) == 2

    pose_a = next(d for d in res.drones if d.cam1_track_id == 1)
    pose_b = next(d for d in res.drones if d.cam1_track_id == 2)
    assert pose_a.cam2_track_id == 2
    assert pose_b.cam2_track_id == 1
    assert math.dist(pose_a.xyz_m, drone_a_xyz) < 1e-3
    assert math.dist(pose_b.xyz_m, drone_b_xyz) < 1e-3


def test_multi_drone_epipolar_ambiguity_is_rejected_not_guessed():
    """Two drones on the exact same epipolar plane (Y=0) in a parallel horizontal stereo rig
    produce 4 valid triangulations (2 real + 2 ghost intersections) with 0 epipolar error.
    The tracker must detect this ambiguity and refuse to confirm coordinates.
    """
    calib = create_synthetic_calibration(
        image_size=(1280, 720), focal_px=900.0, baseline_m=2.0, convergence_deg=0.0
    )
    # Both drones have Y = 0.0 m, so both project to v = cy = 360.0 px in both cameras!
    pts = np.array([
        [0.5, 0.0, 8.0],
        [1.5, 0.0, 8.0],
    ], dtype=np.float64)
    px1 = project_points_3d(pts, calib.camera_01)
    px2 = project_points_3d(pts, calib.camera_02, calib.R, calib.T)

    det1 = _make_frame_det("camera-01", [
        _make_obs("camera-01", 1, tuple(px1[0])),
        _make_obs("camera-01", 2, tuple(px1[1])),
    ])
    det2 = _make_frame_det("camera-02", [
        _make_obs("camera-02", 10, tuple(px2[0])),
        _make_obs("camera-02", 20, tuple(px2[1])),
    ])

    tracker = StereoTracker3D()
    res = tracker.process(det1, det2, calib)
    assert res.status == STEREO_STATUS_AMBIGUOUS
    assert res.ambiguous_pairs == 2
    assert res.drones == ()


def test_frame_synchronization_rejection_and_offset_compensation():
    calib = create_synthetic_calibration(time_offset_s=0.0, max_sync_delta_s=0.08)
    xyz = np.array([[0.5, -0.3, 6.0]], dtype=np.float64)
    px1 = tuple(project_points_3d(xyz, calib.camera_01)[0])
    px2 = tuple(project_points_3d(xyz, calib.camera_02, calib.R, calib.T)[0])

    # 250 ms difference exceeds 80 ms threshold -> unsynchronized, no 3D output
    det1 = _make_frame_det("camera-01", [_make_obs("camera-01", 1, px1, t_mono=10.25)], t_mono=10.25)
    det2 = _make_frame_det("camera-02", [_make_obs("camera-02", 5, px2, t_mono=10.00)], t_mono=10.00)
    tracker = StereoTracker3D()
    res_bad = tracker.process(det1, det2, calib)
    assert res_bad.status == STEREO_STATUS_UNSYNCHRONIZED
    assert not res_bad.sync.synchronized
    assert res_bad.drones == ()

    # When time_offset_s = +0.25 s is configured on calibration, sync succeeds!
    calib_comp = calib.with_sync_params(time_offset_s=0.25, max_sync_delta_s=0.08)
    res_ok = tracker.process(det1, det2, calib_comp)
    assert res_ok.status == STEREO_STATUS_OK
    assert res_ok.sync.synchronized
    assert res_ok.sync.abs_delta_s == pytest.approx(0.0, abs=1e-6)
    assert len(res_ok.drones) == 1


def test_estimate_time_offset_recovers_camera_clock_skew():
    # Camera 2 timestamps lag behind Camera 1 by 0.12 s (so t2 + 0.12 == t1)
    true_offset = 0.12
    t1 = np.linspace(0.0, 4.0, 120)
    signal_1 = np.sin(2.0 * math.pi * 0.8 * t1) + 0.5 * np.cos(2.0 * math.pi * 1.7 * t1)
    t2 = t1 - true_offset
    signal_2 = signal_1.copy()

    estimated = estimate_time_offset_s(t1, signal_1, t2, signal_2, max_offset_s=0.5, step_s=0.002)
    assert estimated == pytest.approx(true_offset, abs=0.005)


def test_subframe_velocity_interpolation_improves_moving_drone_triangulation():
    calib = create_synthetic_calibration(max_sync_delta_s=0.10)
    tracker = StereoTracker3D()

    # Drone moves along X at 4.0 m/s at Z = 6.0 m
    for step in range(4):
        t1 = 1.0 + step * 0.10
        t2 = t1 + 0.04  # Camera 2 captures 40 ms later than Camera 1
        pos_t1 = np.array([[-1.0 + 4.0 * (t1 - 1.0), 0.2, 6.0]], dtype=np.float64)
        pos_t2 = np.array([[-1.0 + 4.0 * (t2 - 1.0), 0.2, 6.0]], dtype=np.float64)
        px1 = tuple(project_points_3d(pos_t1, calib.camera_01)[0])
        px2 = tuple(project_points_3d(pos_t2, calib.camera_02, calib.R, calib.T)[0])
        det1 = _make_frame_det("camera-01", [_make_obs("camera-01", 1, px1, t_mono=t1)], t_mono=t1)
        det2 = _make_frame_det("camera-02", [_make_obs("camera-02", 7, px2, t_mono=t2)], t_mono=t2)
        res = tracker.process(det1, det2, calib)

    assert res.status == STEREO_STATUS_OK
    t_mid = 0.5 * (t1 + t2)
    expected_mid_xyz = (-1.0 + 4.0 * (t_mid - 1.0), 0.2, 6.0)
    assert math.dist(res.drones[0].xyz_m, expected_mid_xyz) < 0.05
