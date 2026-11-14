"""Phase 03 real-time 3D stereo triangulation, frame sync compensation and cross-camera matching.

Key invariants:
1. Per-camera ByteTrack IDs are NEVER assumed to match across cameras (Camera 1 ID 1
   and Camera 2 ID 1 are independent). Cross-camera association is purely geometric
   (epipolar + reprojection error + valid depth).
2. In multi-drone scenes, if two candidate correspondences lie near the same epipolar
   plane (cost difference smaller than ``ambiguity_margin_px``), the ambiguous pair is
   rejected rather than emitting a potentially phantom 3D position.
3. Without a valid ``StereoCalibration``, no 3D coordinates are produced
   (``status="uncalibrated"``).
"""

from dataclasses import asdict, dataclass, field
import itertools
import math

import cv2
import numpy as np

from .calibration import (
    StereoCalibration,
    project_points_3d,
    undistort_point_px,
    undistort_points_normalized,
)
from .detections import DroneObservation, FrameDetections

STEREO_STATUS_UNCALIBRATED = "uncalibrated"
STEREO_STATUS_WAITING = "waiting_detections"
STEREO_STATUS_STALE = "stale_detections"
STEREO_STATUS_UNSYNCHRONIZED = "unsynchronized"
STEREO_STATUS_NO_MATCH = "no_match"
STEREO_STATUS_AMBIGUOUS = "ambiguous_match"
STEREO_STATUS_PARTIAL = "partial_match"
STEREO_STATUS_OK = "ok"


@dataclass(frozen=True)
class SyncStatus:
    synchronized: bool
    raw_delta_s: float        # t1 - t2 (monotonic seconds)
    corrected_delta_s: float  # t1 - (t2 + time_offset_s)
    abs_delta_s: float        # |t1 - (t2 + time_offset_s)|
    time_offset_s: float
    max_sync_delta_s: float
    reason: str = ""

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class TriangulatedPoint:
    xyz_m: tuple[float, float, float]  # (X, Y, Z) in meters in Camera 1 coordinate frame
    depth_cam1_m: float
    depth_cam2_m: float
    reprojection_error_px: float       # RMS reprojection error across both cameras (px)
    reproj_error_cam1_px: float
    reproj_error_cam2_px: float
    epipolar_error_px: float           # Symmetric epipolar line distance (px)
    ray_gap_m: float                   # 3D shortest distance between viewing rays (m)
    parallax_deg: float                # Angle between viewing rays (deg)
    position_uncertainty_m: float      # Estimated 1-sigma 3D position uncertainty (m)
    confidence_score: float            # Overall coordinate reliability in [0.0, 1.0]

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class Drone3DPose:
    drone_id: int                      # Global 3D track ID (1, 2, ...), independent of single-camera IDs
    cam1_track_id: int | None
    cam2_track_id: int | None
    cam1_center: tuple[float, float]   # Observed (possibly time-compensated) 2D center in Camera 1 (px)
    cam2_center: tuple[float, float]   # Observed (possibly time-compensated) 2D center in Camera 2 (px)
    cam1_safe_radius_px: float         # 2D display bounding circle radius in Camera 1 (px, NOT 3D safety distance)
    cam2_safe_radius_px: float         # 2D display bounding circle radius in Camera 2 (px, NOT 3D safety distance)
    xyz_m: tuple[float, float, float]  # (X, Y, Z) in meters
    reprojection_error_px: float
    epipolar_error_px: float
    ray_gap_m: float
    parallax_deg: float
    position_uncertainty_m: float
    confidence_score: float
    detector_confidence: float
    timestamp: float                   # Fused monotonic timestamp (s)
    wall_time: float                   # Fused wall-clock timestamp (s)

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class StereoFrameResult:
    status: str
    status_message: str
    is_calibrated: bool
    calibration_type: str              # "none", "synthetic", or "field"
    sync: SyncStatus
    timestamp: float
    wall_time: float
    drones: tuple[Drone3DPose, ...] = field(default_factory=tuple)
    unmatched_cam1: int = 0
    unmatched_cam2: int = 0
    ambiguous_pairs: int = 0

    def to_dict(self) -> dict:
        return {
            "status": self.status,
            "status_message": self.status_message,
            "is_calibrated": self.is_calibrated,
            "calibration_type": self.calibration_type,
            "sync": self.sync.to_dict(),
            "timestamp": self.timestamp,
            "wall_time": self.wall_time,
            "drones": [d.to_dict() for d in self.drones],
            "unmatched_cam1": self.unmatched_cam1,
            "unmatched_cam2": self.unmatched_cam2,
            "ambiguous_pairs": self.ambiguous_pairs,
        }


# -------------------------------------------- frame time difference & sync
def evaluate_frame_sync(
    det_01: FrameDetections | None,
    det_02: FrameDetections | None,
    calibration: StereoCalibration | None,
    now_monotonic: float | None = None,
    max_frame_age_s: float = 1.0,
) -> SyncStatus:
    """Measure inter-camera frame timestamp difference and apply configured offset."""
    offset = calibration.time_offset_s if calibration is not None else 0.0
    max_delta = calibration.max_sync_delta_s if calibration is not None else 0.10
    if det_01 is None or det_02 is None:
        return SyncStatus(
            synchronized=False,
            raw_delta_s=0.0,
            corrected_delta_s=0.0,
            abs_delta_s=0.0,
            time_offset_s=offset,
            max_sync_delta_s=max_delta,
            reason="두 카메라의 AI 탐지 결과를 기다리는 중입니다.",
        )
    t1 = float(det_01.received_monotonic)
    t2 = float(det_02.received_monotonic)
    if now_monotonic is not None:
        age1 = now_monotonic - t1
        age2 = now_monotonic - t2
        if age1 > max_frame_age_s or age2 > max_frame_age_s:
            return SyncStatus(
                synchronized=False,
                raw_delta_s=t1 - t2,
                corrected_delta_s=t1 - (t2 + offset),
                abs_delta_s=abs(t1 - (t2 + offset)),
                time_offset_s=offset,
                max_sync_delta_s=max_delta,
                reason=f"프레임 데이터가 오래되었습니다 (최대 {max(age1, age2) * 1000:.0f} ms 경과).",
            )
    raw_delta = t1 - t2
    corrected_delta = t1 - (t2 + offset)
    abs_delta = abs(corrected_delta)
    if abs_delta > max_delta:
        return SyncStatus(
            synchronized=False,
            raw_delta_s=raw_delta,
            corrected_delta_s=corrected_delta,
            abs_delta_s=abs_delta,
            time_offset_s=offset,
            max_sync_delta_s=max_delta,
            reason=(
                f"카메라 간 프레임 시간 차이({abs_delta * 1000:.1f} ms)가 "
                f"허용 임계값({max_delta * 1000:.1f} ms)을 초과했습니다."
            ),
        )
    return SyncStatus(
        synchronized=True,
        raw_delta_s=raw_delta,
        corrected_delta_s=corrected_delta,
        abs_delta_s=abs_delta,
        time_offset_s=offset,
        max_sync_delta_s=max_delta,
        reason="동기화 정상",
    )


def estimate_time_offset_s(
    timestamps_01: list[float] | np.ndarray,
    signal_01: list[float] | np.ndarray,
    timestamps_02: list[float] | np.ndarray,
    signal_02: list[float] | np.ndarray,
    max_offset_s: float = 1.0,
    step_s: float = 0.002,
) -> float:
    """Estimate time_offset_s to add to Camera 2 timestamps so t2 + offset aligns with t1.

    Uses normalized cross-correlation over a uniform time grid on a shared motion
    feature (such as vertical image motion or flash intensity).
    """
    t1 = np.asarray(timestamps_01, dtype=np.float64)
    y1 = np.asarray(signal_01, dtype=np.float64)
    t2 = np.asarray(timestamps_02, dtype=np.float64)
    y2 = np.asarray(signal_02, dtype=np.float64)
    if len(t1) < 4 or len(t2) < 4 or len(t1) != len(y1) or len(t2) != len(y2):
        raise ValueError("시간 오프셋 추정을 위해 각 카메라당 최소 4개 이상의 시계열 샘플이 필요합니다.")
    y1_zero = y1 - np.mean(y1)
    y2_zero = y2 - np.mean(y2)
    s1 = float(np.std(y1_zero))
    s2 = float(np.std(y2_zero))
    if s1 <= 1e-9 or s2 <= 1e-9:
        return 0.0

    best_offset = 0.0
    best_corr = -2.0
    offsets = np.arange(-abs(max_offset_s), abs(max_offset_s) + step_s * 0.5, step_s)
    for delta in offsets:
        t2_shifted = t2 + delta
        t_start = max(float(t1[0]), float(t2_shifted[0]))
        t_end = min(float(t1[-1]), float(t2_shifted[-1]))
        if t_end - t_start <= step_s * 5:
            continue
        grid = np.arange(t_start, t_end, step_s)
        if len(grid) < 4:
            continue
        v1 = np.interp(grid, t1, y1_zero)
        v2 = np.interp(grid, t2_shifted, y2_zero)
        denom = float(np.linalg.norm(v1) * np.linalg.norm(v2))
        if denom <= 1e-9:
            continue
        corr = float(np.dot(v1, v2) / denom)
        if corr > best_corr:
            best_corr = corr
            best_offset = float(delta)
    return round(best_offset, 4)


# ----------------------------------------- stereo geometry & triangulation
def symmetric_epipolar_distance_px(
    pt1_px: tuple[float, float],
    pt2_px: tuple[float, float],
    calibration: StereoCalibration,
) -> float:
    """Symmetric distance (in pixels) between undistorted points and their epipolar lines."""
    u1, v1 = undistort_point_px(pt1_px[0], pt1_px[1], calibration.camera_01)
    u2, v2 = undistort_point_px(pt2_px[0], pt2_px[1], calibration.camera_02)
    f_mat = calibration.fundamental_matrix()
    x1 = np.array([u1, v1, 1.0], dtype=np.float64)
    x2 = np.array([u2, v2, 1.0], dtype=np.float64)

    # Line in camera 2: l2 = F x1 = (a2, b2, c2)
    l2 = f_mat @ x1
    # Line in camera 1: l1 = F^T x2 = (a1, b1, c1)
    l1 = f_mat.T @ x2

    denom1 = math.hypot(float(l1[0]), float(l1[1]))
    denom2 = math.hypot(float(l2[0]), float(l2[1]))
    if denom1 <= 1e-12 or denom2 <= 1e-12:
        return float("inf")
    algebraic = abs(float(x2.T @ f_mat @ x1))
    d1 = algebraic / denom1
    d2 = algebraic / denom2
    return 0.5 * (d1 + d2)


def _ray_intersection_and_parallax(
    norm1: np.ndarray,
    norm2: np.ndarray,
    calibration: StereoCalibration,
) -> tuple[float, float]:
    """Compute shortest 3D distance (m) and parallax angle (deg) between the two viewing rays."""
    # Ray 1 in Camera 1 frame: origin O1 = (0,0,0), direction d1
    d1 = np.array([norm1[0], norm1[1], 1.0], dtype=np.float64)
    d1 /= np.linalg.norm(d1)

    # Ray 2 in Camera 1 frame: origin O2 = -R^T T, direction d2 = R^T [x2_n, y2_n, 1]
    r_mat = calibration.R
    o2 = calibration.camera_2_center_in_cam1()
    d2_cam2 = np.array([norm2[0], norm2[1], 1.0], dtype=np.float64)
    d2 = r_mat.T @ d2_cam2
    d2 /= np.linalg.norm(d2)

    cos_angle = float(np.clip(np.dot(d1, d2), -1.0, 1.0))
    parallax_deg = math.degrees(math.acos(cos_angle))

    cross = np.cross(d1, d2)
    cross_norm = float(np.linalg.norm(cross))
    if cross_norm <= 1e-9:
        # Nearly parallel rays
        perp = o2 - np.dot(o2, d1) * d1
        return float(np.linalg.norm(perp)), parallax_deg
    ray_gap_m = abs(float(np.dot(o2, cross))) / cross_norm
    return ray_gap_m, parallax_deg


def triangulate_points(
    pt1_px: tuple[float, float],
    pt2_px: tuple[float, float],
    calibration: StereoCalibration,
    detector_confidence: float = 0.85,
) -> TriangulatedPoint | None:
    """Triangulate a 3D point (meters in Camera 1 frame) from a pair of 2D pixel coordinates.

    Returns None if the triangulated point lies behind either camera or is non-finite.
    """
    norm1 = undistort_points_normalized(np.array([pt1_px], dtype=np.float64), calibration.camera_01)[0]
    norm2 = undistort_points_normalized(np.array([pt2_px], dtype=np.float64), calibration.camera_02)[0]

    homog = cv2.triangulatePoints(
        calibration.P1_normalized,
        calibration.P2_normalized,
        norm1.reshape(2, 1),
        norm2.reshape(2, 1),
    )
    w = float(homog[3, 0])
    if abs(w) <= 1e-12 or not math.isfinite(w):
        return None
    xyz_cam1 = (homog[:3, 0] / w).astype(np.float64)
    if not np.all(np.isfinite(xyz_cam1)):
        return None

    z1 = float(xyz_cam1[2])
    xyz_cam2 = (calibration.R @ xyz_cam1.reshape(3, 1) + calibration.T).reshape(3)
    z2 = float(xyz_cam2[2])
    if z1 <= 0.0 or z2 <= 0.0:
        return None

    # Reproject 3D point back into both cameras (including lens distortion)
    reproj_1 = project_points_3d(xyz_cam1.reshape(1, 3), calibration.camera_01)[0]
    reproj_2 = project_points_3d(
        xyz_cam1.reshape(1, 3), calibration.camera_02, calibration.R, calibration.T
    )[0]
    err1 = math.hypot(float(reproj_1[0] - pt1_px[0]), float(reproj_1[1] - pt1_px[1]))
    err2 = math.hypot(float(reproj_2[0] - pt2_px[0]), float(reproj_2[1] - pt2_px[1]))
    rms_reproj = math.sqrt(0.5 * (err1 * err1 + err2 * err2))

    epipolar_err = symmetric_epipolar_distance_px(pt1_px, pt2_px, calibration)
    ray_gap_m, parallax_deg = _ray_intersection_and_parallax(norm1, norm2, calibration)

    # Estimate 3D position uncertainty (1-sigma in meters) from stereo geometry:
    # sigma_z ≈ (Z^2 / (f * B)) * sigma_px + 0.5 * ray_gap_m
    mean_focal = 0.5 * (calibration.camera_01.fx + calibration.camera_02.fx)
    effective_sigma_px = max(1.5, rms_reproj, calibration.rms_error)
    geom_sigma_m = (z1 * z1) / max(1.0, mean_focal * calibration.baseline_m) * effective_sigma_px
    conf_clamped = min(max(float(detector_confidence), 0.1), 1.0)
    uncertainty_m = (geom_sigma_m + 0.5 * ray_gap_m) / math.sqrt(conf_clamped)

    # Compute overall coordinate confidence score in [0, 1]
    reproj_factor = max(0.0, 1.0 - rms_reproj / max(1.0, calibration.max_reprojection_error_px * 1.5))
    epi_factor = max(0.0, 1.0 - epipolar_err / max(1.0, calibration.max_epipolar_error_px * 1.5))
    parallax_factor = min(1.0, parallax_deg / 5.0) if parallax_deg > 0.2 else 0.0
    confidence_score = float(np.clip(
        0.40 * reproj_factor + 0.25 * epi_factor + 0.20 * parallax_factor + 0.15 * conf_clamped,
        0.0,
        1.0,
    ))

    return TriangulatedPoint(
        xyz_m=(float(xyz_cam1[0]), float(xyz_cam1[1]), float(xyz_cam1[2])),
        depth_cam1_m=z1,
        depth_cam2_m=z2,
        reprojection_error_px=rms_reproj,
        reproj_error_cam1_px=err1,
        reproj_error_cam2_px=err2,
        epipolar_error_px=epipolar_err,
        ray_gap_m=ray_gap_m,
        parallax_deg=parallax_deg,
        position_uncertainty_m=float(uncertainty_m),
        confidence_score=confidence_score,
    )


# ------------------------------------ cross-camera matching & 3D track state
@dataclass
class _Active3DTrack:
    drone_id: int
    xyz_m: tuple[float, float, float]
    timestamp: float
    cam1_track_id: int | None
    cam2_track_id: int | None


class StereoTracker3D:
    """Matches drones across two cameras, rejects ambiguous correspondences, and
    maintains persistent 3D drone IDs without assuming per-camera ByteTrack IDs match.
    """

    def __init__(self, max_track_Coast_s: float = 1.5, max_3d_jump_m: float = 8.0):
        self.max_track_coast_s = float(max_track_Coast_s)
        self.max_3d_jump_m = float(max_3d_jump_m)
        self._tracks: list[_Active3DTrack] = []
        self._next_drone_id = 1
        # Per-camera previous observation cache for sub-frame 2D velocity interpolation:
        # camera_id -> {track_id: (timestamp, cx, cy, vx_px_s, vy_px_s)}
        self._cam_prev: dict[str, dict[int, tuple[float, float, float, float, float]]] = {
            "camera-01": {},
            "camera-02": {},
        }

    def reset(self) -> None:
        self._tracks.clear()
        self._next_drone_id = 1
        self._cam_prev["camera-01"].clear()
        self._cam_prev["camera-02"].clear()

    def _update_2d_velocities_and_interpolate(
        self,
        camera_id: str,
        observations: tuple[DroneObservation, ...],
        target_timestamp: float,
    ) -> list[tuple[DroneObservation, tuple[float, float]]]:
        """Estimate 2D pixel velocity per track_id within one camera and shift center to target_timestamp."""
        history = self._cam_prev.setdefault(camera_id, {})
        compensated: list[tuple[DroneObservation, tuple[float, float]]] = []
        for obs in observations:
            cx, cy = obs.center
            t_obs = float(obs.received_monotonic)
            vx, vy = 0.0, 0.0
            if obs.track_id is not None and obs.track_id in history:
                prev_t, prev_x, prev_y, prev_vx, prev_vy = history[obs.track_id]
                dt = t_obs - prev_t
                if 0.002 <= dt <= 0.5:
                    raw_vx = (cx - prev_x) / dt
                    raw_vy = (cy - prev_y) / dt
                    vx = 0.6 * raw_vx + 0.4 * prev_vx
                    vy = 0.6 * raw_vy + 0.4 * prev_vy
            if obs.track_id is not None and t_obs > 0:
                history[obs.track_id] = (t_obs, cx, cy, vx, vy)
            shift_dt = target_timestamp - t_obs
            if abs(shift_dt) <= 0.25:
                comp_center = (cx + vx * shift_dt, cy + vy * shift_dt)
            else:
                comp_center = (cx, cy)
            compensated.append((obs, comp_center))
        return compensated

    def _assign_global_drone_id(
        self,
        xyz_m: tuple[float, float, float],
        cam1_track_id: int | None,
        cam2_track_id: int | None,
        timestamp: float,
        used_ids: set[int],
    ) -> int:
        # Prune expired 3D tracks
        self._tracks = [t for t in self._tracks if timestamp - t.timestamp <= self.max_track_coast_s]

        best_track = None
        best_score = float("inf")
        for track in self._tracks:
            if track.drone_id in used_ids:
                continue
            dist_3d = math.dist(xyz_m, track.xyz_m)
            if dist_3d > self.max_3d_jump_m:
                continue
            # Bonus if the pair of per-camera IDs stayed consistent with the previous frame
            pair_bonus = 0.0
            if (cam1_track_id is not None and cam1_track_id == track.cam1_track_id
                    and cam2_track_id is not None and cam2_track_id == track.cam2_track_id):
                pair_bonus = -0.5
            score = dist_3d + pair_bonus
            if score < best_score:
                best_score = score
                best_track = track

        if best_track is not None:
            best_track.xyz_m = xyz_m
            best_track.timestamp = timestamp
            best_track.cam1_track_id = cam1_track_id
            best_track.cam2_track_id = cam2_track_id
            used_ids.add(best_track.drone_id)
            return best_track.drone_id

        drone_id = self._next_drone_id
        self._next_drone_id += 1
        self._tracks.append(_Active3DTrack(drone_id, xyz_m, timestamp, cam1_track_id, cam2_track_id))
        used_ids.add(drone_id)
        return drone_id

    def process(
        self,
        det_01: FrameDetections | None,
        det_02: FrameDetections | None,
        calibration: StereoCalibration | None,
        now_monotonic: float | None = None,
    ) -> StereoFrameResult:
        """Compute synchronized, cross-camera matched 3D drone coordinates."""
        sync = evaluate_frame_sync(det_01, det_02, calibration, now_monotonic=now_monotonic)
        t_ref = (
            0.5 * (det_01.received_monotonic + det_02.received_monotonic + (calibration.time_offset_s if calibration else 0.0))
            if (det_01 is not None and det_02 is not None)
            else (now_monotonic or 0.0)
        )
        wall_ref = (
            0.5 * (det_01.received_at + det_02.received_at)
            if (det_01 is not None and det_02 is not None)
            else 0.0
        )

        if calibration is None:
            return StereoFrameResult(
                status=STEREO_STATUS_UNCALIBRATED,
                status_message="카메라 보정 데이터가 없습니다 — 임의의 3D 좌표를 출력하지 않습니다.",
                is_calibrated=False,
                calibration_type="none",
                sync=sync,
                timestamp=t_ref,
                wall_time=wall_ref,
            )

        if det_01 is None or det_02 is None:
            return StereoFrameResult(
                status=STEREO_STATUS_WAITING,
                status_message=sync.reason,
                is_calibrated=True,
                calibration_type=calibration.calibration_type,
                sync=sync,
                timestamp=t_ref,
                wall_time=wall_ref,
            )

        if not sync.synchronized:
            status_code = (
                STEREO_STATUS_STALE if "오래되었습니다" in sync.reason else STEREO_STATUS_UNSYNCHRONIZED
            )
            return StereoFrameResult(
                status=status_code,
                status_message=sync.reason,
                is_calibrated=True,
                calibration_type=calibration.calibration_type,
                sync=sync,
                timestamp=t_ref,
                wall_time=wall_ref,
                unmatched_cam1=len(det_01.observations),
                unmatched_cam2=len(det_02.observations),
            )

        obs_1 = det_01.observations
        obs_2 = det_02.observations
        if not obs_1 or not obs_2:
            return StereoFrameResult(
                status=STEREO_STATUS_WAITING if (not obs_1 and not obs_2) else STEREO_STATUS_NO_MATCH,
                status_message=(
                    "탐지된 드론이 없습니다." if (not obs_1 and not obs_2)
                    else f"한쪽 카메라에서만 탐지됨 (Cam1: {len(obs_1)}대, Cam2: {len(obs_2)}대) — 3D 계산 불가"
                ),
                is_calibrated=True,
                calibration_type=calibration.calibration_type,
                sync=sync,
                timestamp=t_ref,
                wall_time=wall_ref,
                unmatched_cam1=len(obs_1),
                unmatched_cam2=len(obs_2),
            )

        # Compensate sub-frame timestamp difference using per-camera 2D track velocity
        t1_target = t_ref
        t2_target = t_ref - calibration.time_offset_s
        comp_1 = self._update_2d_velocities_and_interpolate("camera-01", obs_1, t1_target)
        comp_2 = self._update_2d_velocities_and_interpolate("camera-02", obs_2, t2_target)

        n1, n2 = len(comp_1), len(comp_2)
        candidates: dict[tuple[int, int], tuple[float, TriangulatedPoint]] = {}

        for i, (o1, center1) in enumerate(comp_1):
            for j, (o2, center2) in enumerate(comp_2):
                det_conf = 0.5 * (o1.confidence + o2.confidence)
                tri = triangulate_points(center1, center2, calibration, detector_confidence=det_conf)
                if tri is None:
                    continue
                if not (calibration.min_depth_m <= tri.depth_cam1_m <= calibration.max_depth_m
                        and calibration.min_depth_m <= tri.depth_cam2_m <= calibration.max_depth_m):
                    continue
                if tri.reprojection_error_px > calibration.max_reprojection_error_px:
                    continue
                if tri.epipolar_error_px > calibration.max_epipolar_error_px:
                    continue
                cost = 0.6 * tri.reprojection_error_px + 0.4 * tri.epipolar_error_px
                candidates[(i, j)] = (cost, tri)

        if not candidates:
            return StereoFrameResult(
                status=STEREO_STATUS_NO_MATCH,
                status_message="기하학적으로 일치하는 카메라 간 드론 대응점이 없습니다 (재투영/에피폴라 오차 초과).",
                is_calibrated=True,
                calibration_type=calibration.calibration_type,
                sync=sync,
                timestamp=t_ref,
                wall_time=wall_ref,
                unmatched_cam1=n1,
                unmatched_cam2=n2,
            )

        # Multi-drone ambiguity detection:
        # If an observation in Cam1 has >=2 plausible matches in Cam2 within ambiguity_margin_px,
        # or an observation in Cam2 has >=2 plausible matches in Cam1 within ambiguity_margin_px,
        # mark those observations as ambiguous so we never confirm uncertain 3D coordinates.
        margin = calibration.ambiguity_margin_px
        ambiguous_i: set[int] = set()
        ambiguous_j: set[int] = set()

        for i in range(n1):
            row_costs = sorted(cost for (ii, _jj), (cost, _tri) in candidates.items() if ii == i)
            if len(row_costs) >= 2 and (row_costs[1] - row_costs[0]) < margin:
                ambiguous_i.add(i)

        for j in range(n2):
            col_costs = sorted(cost for (_ii, jj), (cost, _tri) in candidates.items() if jj == j)
            if len(col_costs) >= 2 and (col_costs[1] - col_costs[0]) < margin:
                ambiguous_j.add(j)

        filtered_candidates = {
            (i, j): val for (i, j), val in candidates.items()
            if i not in ambiguous_i and j not in ambiguous_j
        }
        ambiguous_count = max(len(ambiguous_i), len(ambiguous_j))

        # Solve optimal bipartite assignment over non-ambiguous candidates
        matched_pairs = _solve_min_cost_matching(n1, n2, filtered_candidates)
        if not matched_pairs:
            status = STEREO_STATUS_AMBIGUOUS if ambiguous_count > 0 else STEREO_STATUS_NO_MATCH
            msg = (
                f"다중 드론 대응 불확실 ({ambiguous_count}쌍 모호) — 좌표를 확정하지 않습니다."
                if ambiguous_count > 0
                else "유효한 드론 대응점을 확정할 수 없습니다."
            )
            return StereoFrameResult(
                status=status,
                status_message=msg,
                is_calibrated=True,
                calibration_type=calibration.calibration_type,
                sync=sync,
                timestamp=t_ref,
                wall_time=wall_ref,
                unmatched_cam1=n1,
                unmatched_cam2=n2,
                ambiguous_pairs=ambiguous_count,
            )

        used_global_ids: set[int] = set()
        poses: list[Drone3DPose] = []
        # Sort matched pairs by X coordinate for deterministic initial ID ordering
        matched_sorted = sorted(matched_pairs, key=lambda pair: filtered_candidates[pair][1].xyz_m[0])
        for i, j in matched_sorted:
            _cost, tri = filtered_candidates[(i, j)]
            o1, center1 = comp_1[i]
            o2, center2 = comp_2[j]
            drone_id = self._assign_global_drone_id(
                tri.xyz_m, o1.track_id, o2.track_id, t_ref, used_global_ids
            )
            poses.append(
                Drone3DPose(
                    drone_id=drone_id,
                    cam1_track_id=o1.track_id,
                    cam2_track_id=o2.track_id,
                    cam1_center=center1,
                    cam2_center=center2,
                    cam1_safe_radius_px=o1.safe_radius,
                    cam2_safe_radius_px=o2.safe_radius,
                    xyz_m=tri.xyz_m,
                    reprojection_error_px=tri.reprojection_error_px,
                    epipolar_error_px=tri.epipolar_error_px,
                    ray_gap_m=tri.ray_gap_m,
                    parallax_deg=tri.parallax_deg,
                    position_uncertainty_m=tri.position_uncertainty_m,
                    confidence_score=tri.confidence_score,
                    detector_confidence=0.5 * (o1.confidence + o2.confidence),
                    timestamp=t_ref,
                    wall_time=wall_ref,
                )
            )
        poses.sort(key=lambda d: d.drone_id)
        unmatched_1 = n1 - len(poses)
        unmatched_2 = n2 - len(poses)
        if ambiguous_count > 0:
            status = STEREO_STATUS_AMBIGUOUS
            msg = f"{len(poses)}대 3D 확정 · {ambiguous_count}쌍 대응 불확실(제외됨)"
        elif unmatched_1 > 0 or unmatched_2 > 0:
            status = STEREO_STATUS_PARTIAL
            msg = f"{len(poses)}대 3D 확정 (미대응 Cam1: {unmatched_1}, Cam2: {unmatched_2})"
        else:
            status = STEREO_STATUS_OK
            calib_tag = "합성 보정(실험용)" if calibration.calibration_type == "synthetic" else "현장 보정"
            msg = f"{len(poses)}대 3D 좌표 확정 ({calib_tag})"

        return StereoFrameResult(
            status=status,
            status_message=msg,
            is_calibrated=True,
            calibration_type=calibration.calibration_type,
            sync=sync,
            timestamp=t_ref,
            wall_time=wall_ref,
            drones=tuple(poses),
            unmatched_cam1=unmatched_1,
            unmatched_cam2=unmatched_2,
            ambiguous_pairs=ambiguous_count,
        )


def _solve_min_cost_matching(
    n1: int,
    n2: int,
    candidates: dict[tuple[int, int], tuple[float, TriangulatedPoint]],
) -> list[tuple[int, int]]:
    """Find maximum-cardinality, minimum-cost bipartite matching among valid candidates."""
    if not candidates:
        return []
    active_i = sorted({i for i, _ in candidates})
    active_j = sorted({j for _, j in candidates})
    if len(active_i) <= 6 and len(active_j) <= 6:
        # Exact combinatorial search for small drone counts (maximizes matched pairs, then minimizes cost)
        best_count = 0
        best_cost = float("inf")
        best_pairs: list[tuple[int, int]] = []

        def backtrack(idx: int, used_j: set[int], current: list[tuple[int, int]], cost_so_far: float):
            nonlocal best_count, best_cost, best_pairs
            remaining = len(active_i) - idx
            if len(current) + remaining < best_count:
                return
            if idx == len(active_i):
                if len(current) > best_count or (len(current) == best_count and cost_so_far < best_cost):
                    best_count = len(current)
                    best_cost = cost_so_far
                    best_pairs = list(current)
                return
            i = active_i[idx]
            for j in active_j:
                if j not in used_j and (i, j) in candidates:
                    c, _ = candidates[(i, j)]
                    used_j.add(j)
                    current.append((i, j))
                    backtrack(idx + 1, used_j, current, cost_so_far + c)
                    current.pop()
                    used_j.remove(j)
            backtrack(idx + 1, used_j, current, cost_so_far)

        backtrack(0, set(), [], 0.0)
        return best_pairs

    # Greedy fallback for larger sets
    used_i: set[int] = set()
    used_j: set[int] = set()
    result: list[tuple[int, int]] = []
    for (i, j), ( _cost, _tri) in sorted(candidates.items(), key=lambda item: item[1][0]):
        if i not in used_i and j not in used_j:
            used_i.add(i)
            used_j.add(j)
            result.append((i, j))
    return result
