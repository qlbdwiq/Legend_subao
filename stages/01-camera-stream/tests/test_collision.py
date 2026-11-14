"""Automated tests for Phase 04 3D collision risk analysis, CPA, and indeterminate state handling."""

import json
import math

import pytest

from drone_camera.collision import (
    RISK_CAUTION,
    RISK_DANGER,
    RISK_SAFE,
    RISK_UNKNOWN,
    CollisionAnalyzer,
    CollisionSettings,
    CollisionSettingsError,
    collision_settings_path_for,
    load_collision_settings,
    save_collision_settings,
)
from drone_camera.stereo_3d import (
    STEREO_STATUS_AMBIGUOUS,
    STEREO_STATUS_OK,
    STEREO_STATUS_UNCALIBRATED,
    STEREO_STATUS_UNSYNCHRONIZED,
    Drone3DPose,
    StereoFrameResult,
    SyncStatus,
)


def _pose(drone_id: int, xyz_m: tuple[float, float, float], t: float = 10.0,
          uncertainty_m: float = 0.05, confidence: float = 0.90,
          radius_px: float = 30.0) -> Drone3DPose:
    return Drone3DPose(
        drone_id=drone_id,
        cam1_track_id=drone_id,
        cam2_track_id=drone_id + 10,
        cam1_center=(640.0, 360.0),
        cam2_center=(580.0, 360.0),
        cam1_safe_radius_px=radius_px,
        cam2_safe_radius_px=radius_px,
        xyz_m=xyz_m,
        reprojection_error_px=0.8,
        epipolar_error_px=0.5,
        ray_gap_m=0.01,
        parallax_deg=12.0,
        position_uncertainty_m=uncertainty_m,
        confidence_score=confidence,
        detector_confidence=0.90,
        timestamp=t,
        wall_time=1700000000.0 + t,
    )


def _stereo_frame(drones: list[Drone3DPose], t: float = 10.0,
                  status: str = STEREO_STATUS_OK, is_calibrated: bool = True,
                  synchronized: bool = True, ambiguous_pairs: int = 0) -> StereoFrameResult:
    sync = SyncStatus(
        synchronized=synchronized,
        raw_delta_s=0.005 if synchronized else 0.35,
        corrected_delta_s=0.005 if synchronized else 0.35,
        abs_delta_s=0.005 if synchronized else 0.35,
        time_offset_s=0.0,
        max_sync_delta_s=0.10,
        reason="동기화 정상" if synchronized else "프레임 시간 차이 초과",
    )
    return StereoFrameResult(
        status=status,
        status_message=status,
        is_calibrated=is_calibrated,
        calibration_type="synthetic" if is_calibrated else "none",
        sync=sync,
        timestamp=t,
        wall_time=1700000000.0 + t,
        drones=tuple(drones),
        ambiguous_pairs=ambiguous_pairs,
    )


def test_collision_settings_save_and_load_roundtrip(tmp_path):
    path = collision_settings_path_for(tmp_path / "config.json")
    custom = CollisionSettings(
        warning_distance_m=6.0,
        danger_distance_m=3.0,
        prediction_horizon_s=4.5,
        min_closing_speed_mps=0.4,
        cpa_warning_time_s=3.5,
        cpa_danger_time_s=1.8,
        uncertainty_margin_sigma=1.5,
        max_position_uncertainty_m=1.8,
        min_confidence_score=0.45,
        visual_alert_enabled=True,
        audio_alert_enabled=False,
    )
    save_collision_settings(path, custom)
    assert load_collision_settings(path) == custom
    assert load_collision_settings(tmp_path / "missing.json") == CollisionSettings()


@pytest.mark.parametrize("bad_kwargs", [
    {"warning_distance_m": 2.0, "danger_distance_m": 2.5},  # danger >= warning
    {"cpa_warning_time_s": 1.0, "cpa_danger_time_s": 2.0},  # danger_time > warning_time
    {"warning_distance_m": -1.0},
    {"min_confidence_score": 1.5},
    {"visual_alert_enabled": "yes"},
])
def test_invalid_collision_settings_are_rejected(bad_kwargs):
    with pytest.raises(CollisionSettingsError):
        CollisionSettings(**bad_kwargs)


def test_uncalibrated_unsynchronized_or_incomplete_states_are_unknown_never_safe():
    """Critical safety requirement: never report SAFE when coordinates or sync are missing/uncertain."""
    analyzer = CollisionAnalyzer()

    # 1. Uncalibrated
    res_uncalib = analyzer.analyze(_stereo_frame([], status=STEREO_STATUS_UNCALIBRATED, is_calibrated=False))
    assert res_uncalib.risk_level == RISK_UNKNOWN
    assert "판단 불가" in res_uncalib.risk_label

    # 2. Unsynchronized even when 2 drones exist
    d1 = _pose(1, (-5.0, 0.0, 10.0))
    d2 = _pose(2, (5.0, 0.0, 10.0))
    res_unsync = analyzer.analyze(
        _stereo_frame([d1, d2], status=STEREO_STATUS_UNSYNCHRONIZED, synchronized=False)
    )
    assert res_unsync.risk_level == RISK_UNKNOWN
    assert res_unsync.risk_level != RISK_SAFE

    # 3. Only 1 confirmed drone in 3D
    res_one = analyzer.analyze(_stereo_frame([d1]))
    assert res_one.risk_level == RISK_UNKNOWN
    assert "1대뿐" in res_one.reason

    # 4. Ambiguous cross-camera match
    res_ambig = analyzer.analyze(
        _stereo_frame([], status=STEREO_STATUS_AMBIGUOUS, ambiguous_pairs=2)
    )
    assert res_ambig.risk_level == RISK_UNKNOWN

    # 5. Excessive position uncertainty (> max_position_uncertainty_m)
    d2_uncertain = _pose(2, (8.0, 0.0, 10.0), uncertainty_m=3.5)
    res_unc = analyzer.analyze(_stereo_frame([d1, d2_uncertain]))
    assert res_unc.risk_level == RISK_UNKNOWN
    assert "불확실성 과다" in res_unc.reason

    # 6. Low coordinate confidence (< min_confidence_score)
    d2_low_conf = _pose(2, (8.0, 0.0, 10.0), confidence=0.15)
    res_low = analyzer.analyze(_stereo_frame([d1, d2_low_conf]))
    assert res_low.risk_level == RISK_UNKNOWN
    assert "신뢰도 부족" in res_low.reason


def test_3d_distance_velocity_closing_speed_and_cpa_calculation():
    settings = CollisionSettings(
        warning_distance_m=5.0,
        danger_distance_m=2.0,
        prediction_horizon_s=5.0,
        uncertainty_margin_sigma=0.0,
    )
    analyzer = CollisionAnalyzer(settings=settings)

    # Drone 1 starts at x = -4.0 m moving +2.0 m/s along X.
    # Drone 2 starts at x = +4.0 m moving -2.0 m/s along X (head-on approach at 4.0 m/s closing speed!).
    # At t = 10.5 s: Drone 1 is at x = -3.0 m, Drone 2 is at x = +3.0 m -> distance = 6.0 m (> 5.0 m),
    # but CPA occurs in 1.5 s with CPA distance = 0.0 m -> must trigger DANGER!
    audio_triggers = []
    for step in range(6):
        t = 10.0 + step * 0.1
        p1 = (-4.0 + 2.0 * (t - 10.0), 0.0, 8.0)
        p2 = (4.0 - 2.0 * (t - 10.0), 0.0, 8.0)
        res = analyzer.analyze(_stereo_frame([_pose(1, p1, t=t), _pose(2, p2, t=t)], t=t))
        audio_triggers.append(res.audio_alert_triggered)

    assert res.pair_metrics is not None
    pm = res.pair_metrics
    assert pm.distance_m == pytest.approx(6.0, abs=1e-3)
    assert res.kinematics[0].velocity_valid and res.kinematics[1].velocity_valid
    assert res.kinematics[0].speed_mps == pytest.approx(2.0, abs=0.05)
    assert res.kinematics[1].speed_mps == pytest.approx(2.0, abs=0.05)
    assert pm.closing_speed_mps == pytest.approx(4.0, abs=0.1)
    assert pm.is_approaching
    assert pm.cpa_time_s == pytest.approx(1.5, abs=0.05)
    assert pm.cpa_distance_m == pytest.approx(0.0, abs=0.05)
    assert res.risk_level == RISK_DANGER
    assert res.visual_alert_active
    assert any(audio_triggers)
    assert not audio_triggers[-1]  # Cooldown suppresses repeated beeps within 1.5s


def test_diverging_drones_beyond_warning_distance_are_safe():
    settings = CollisionSettings(warning_distance_m=5.0, danger_distance_m=2.5, uncertainty_margin_sigma=0.0)
    analyzer = CollisionAnalyzer(settings=settings)

    # Moving apart from each other
    for step in range(5):
        t = 20.0 + step * 0.1
        p1 = (-3.0 - 1.5 * (t - 20.0), 0.0, 8.0)
        p2 = (3.0 + 1.5 * (t - 20.0), 0.0, 8.0)
        res = analyzer.analyze(_stereo_frame([_pose(1, p1, t=t), _pose(2, p2, t=t)], t=t))

    assert res.risk_level == RISK_SAFE
    assert res.pair_metrics.closing_speed_mps < -2.5
    assert not res.pair_metrics.is_approaching
    assert res.pair_metrics.cpa_time_s == pytest.approx(0.0)
    assert not res.visual_alert_active
    assert not res.audio_alert_triggered


def test_uncertainty_margin_triggers_caution_or_danger_conservatively():
    settings = CollisionSettings(
        warning_distance_m=5.0,
        danger_distance_m=2.5,
        uncertainty_margin_sigma=1.0,
        max_position_uncertainty_m=2.0,
    )
    analyzer = CollisionAnalyzer(settings=settings)

    # Measured distance is 5.6 m (> 5.0 m), but each drone has 0.45 m uncertainty (combined 0.90 m),
    # so effective_distance_m = 4.70 m <= 5.0 m -> CAUTION!
    d1 = _pose(1, (-2.8, 0.0, 9.0), uncertainty_m=0.45)
    d2 = _pose(2, (2.8, 0.0, 9.0), uncertainty_m=0.45)
    res = analyzer.analyze(_stereo_frame([d1, d2]))
    assert res.pair_metrics.distance_m == pytest.approx(5.6)
    assert res.pair_metrics.effective_distance_m == pytest.approx(4.7)
    assert res.risk_level == RISK_CAUTION
    assert len(analyzer.events) == 1
    assert analyzer.events[0].risk_level == RISK_CAUTION


def test_2d_pixel_bounding_circle_radius_never_affects_3d_collision_judgment():
    """Requirement: 2D bounding circle pixel radius is for screen display only, never 3D safety distance."""
    analyzer_small = CollisionAnalyzer()
    analyzer_huge = CollisionAnalyzer()

    res_small = analyzer_small.analyze(_stereo_frame([
        _pose(1, (-4.0, 0.0, 8.0), radius_px=12.0),
        _pose(2, (4.0, 0.0, 8.0), radius_px=12.0),
    ]))
    res_huge = analyzer_huge.analyze(_stereo_frame([
        _pose(1, (-4.0, 0.0, 8.0), radius_px=600.0),
        _pose(2, (4.0, 0.0, 8.0), radius_px=600.0),
    ]))
    assert res_small.risk_level == res_huge.risk_level == RISK_SAFE
    assert res_small.pair_metrics.distance_m == res_huge.pair_metrics.distance_m == pytest.approx(8.0)
