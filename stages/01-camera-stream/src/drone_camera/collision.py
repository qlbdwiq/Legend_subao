"""Phase 04 3D collision risk analysis, kinematics estimation, CPA prediction and event logging.

Safety and design invariants:
1. 3D coordinates from Phase 03 (in meters) are the ONLY spatial input for distance and
   collision risk. 2D pixel bounding circle radii are never used as 3D safety distances.
2. Whenever 3D coordinates cannot be reliably determined (uncalibrated, unsynchronized,
   ambiguous cross-camera match, fewer than 2 confirmed drones, or excessive position
   uncertainty / low confidence), the state MUST be ``RISK_UNKNOWN`` ("판단 불가"),
   NEVER ``RISK_SAFE`` ("안전").
3. This module is an experimental auxiliary warning system and does NOT perform any
   automatic drone control or forced avoidance.
"""

from collections import deque
from dataclasses import asdict, dataclass, field
import json
import math
import os
from pathlib import Path
import tempfile

import numpy as np

from .stereo_3d import (
    STEREO_STATUS_AMBIGUOUS,
    STEREO_STATUS_OK,
    STEREO_STATUS_UNCALIBRATED,
    STEREO_STATUS_UNSYNCHRONIZED,
    Drone3DPose,
    StereoFrameResult,
)

RISK_UNKNOWN = "unknown"
RISK_SAFE = "safe"
RISK_CAUTION = "caution"
RISK_DANGER = "danger"

RISK_LABELS = {
    RISK_UNKNOWN: "판단 불가 / UNKNOWN",
    RISK_SAFE: "안전 / SAFE",
    RISK_CAUTION: "주의 / CAUTION",
    RISK_DANGER: "위험 / DANGER",
}

COLLISION_SETTINGS_NAME = "collision_settings.json"
MAX_RISK_EVENTS = 500


class CollisionSettingsError(ValueError):
    pass


def _is_finite(val) -> bool:
    return not isinstance(val, bool) and isinstance(val, (int, float)) and math.isfinite(val)


@dataclass(frozen=True)
class CollisionSettings:
    warning_distance_m: float = 5.0          # Caution threshold (meters)
    danger_distance_m: float = 2.5           # Danger threshold (meters)
    prediction_horizon_s: float = 5.0        # CPA lookahead window (seconds)
    min_closing_speed_mps: float = 0.30      # Minimum closing speed to treat as approaching (m/s)
    cpa_warning_time_s: float = 4.0          # Caution if CPA <= warning_distance_m within this time
    cpa_danger_time_s: float = 2.0           # Danger if CPA <= danger_distance_m within this time
    uncertainty_margin_sigma: float = 1.0    # Subtract k * (sigma1 + sigma2) from measured distance
    max_position_uncertainty_m: float = 2.0  # Exceeding this forces RISK_UNKNOWN ("판단 불가")
    min_confidence_score: float = 0.35       # Below this forces RISK_UNKNOWN ("판단 불가")
    visual_alert_enabled: bool = True
    audio_alert_enabled: bool = True
    audio_cooldown_s: float = 1.5

    def __post_init__(self):
        for name, low, high in (
            ("warning_distance_m", 0.2, 200.0),
            ("danger_distance_m", 0.1, 100.0),
            ("prediction_horizon_s", 0.5, 60.0),
            ("min_closing_speed_mps", 0.05, 50.0),
            ("cpa_warning_time_s", 0.2, 60.0),
            ("cpa_danger_time_s", 0.1, 30.0),
            ("uncertainty_margin_sigma", 0.0, 5.0),
            ("max_position_uncertainty_m", 0.05, 50.0),
            ("min_confidence_score", 0.05, 0.99),
            ("audio_cooldown_s", 0.2, 30.0),
        ):
            val = getattr(self, name)
            if not _is_finite(val) or not low <= val <= high:
                raise CollisionSettingsError(f"{name} must be a finite number from {low} to {high}.")
            object.__setattr__(self, name, float(val))

        if self.danger_distance_m >= self.warning_distance_m:
            raise CollisionSettingsError("위험 거리(danger_distance_m)는 주의 거리(warning_distance_m)보다 작아야 합니다.")
        if self.cpa_danger_time_s > self.cpa_warning_time_s:
            raise CollisionSettingsError("위험 예측 시간(cpa_danger_time_s)은 주의 예측 시간(cpa_warning_time_s) 이하여야 합니다.")
        for flag in ("visual_alert_enabled", "audio_alert_enabled"):
            if type(getattr(self, flag)) is not bool:
                raise CollisionSettingsError(f"{flag} must be a boolean.")


def collision_settings_path_for(config_path: Path) -> Path:
    return config_path.with_name(COLLISION_SETTINGS_NAME)


def load_collision_settings(path: Path) -> CollisionSettings:
    if not path.exists():
        return CollisionSettings()
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or data.get("version") != 1:
            raise CollisionSettingsError("Expected collision settings version 1.")
        known = set(CollisionSettings.__dataclass_fields__)
        return CollisionSettings(**{k: v for k, v in data.items() if k in known})
    except CollisionSettingsError:
        raise
    except (OSError, ValueError, TypeError):
        raise CollisionSettingsError("충돌 경고 설정 파일을 읽을 수 없습니다.") from None


def save_collision_settings(path: Path, settings: CollisionSettings) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, prefix=".collision-", suffix=".tmp", delete=False
        ) as stream:
            tmp = Path(stream.name)
            json.dump({"version": 1, **asdict(settings)}, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, path)
    finally:
        if tmp is not None:
            tmp.unlink(missing_ok=True)


# ----------------------------------------------------------- data structures
@dataclass(frozen=True)
class DroneKinematics3D:
    drone_id: int
    xyz_m: tuple[float, float, float]
    velocity_mps: tuple[float, float, float]  # (vx, vy, vz) in m/s
    speed_mps: float
    direction_unit: tuple[float, float, float]
    direction_label: str
    velocity_valid: bool
    position_uncertainty_m: float
    confidence_score: float
    reprojection_error_px: float
    cam1_track_id: int | None
    cam2_track_id: int | None
    drone_model: str = "unknown"
    estimation_mode: str = "stereo"

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class PairCollisionMetrics:
    drone_a_id: int
    drone_b_id: int
    distance_m: float
    combined_uncertainty_m: float
    effective_distance_m: float
    relative_velocity_mps: tuple[float, float, float]
    relative_speed_mps: float
    closing_speed_mps: float          # Positive when approaching, negative when diverging
    is_approaching: bool
    cpa_time_s: float                 # Time to Closest Point of Approach within [0, horizon]
    cpa_distance_m: float             # Predicted distance at cpa_time_s
    effective_cpa_distance_m: float
    cpa_point_a_m: tuple[float, float, float]
    cpa_point_b_m: tuple[float, float, float]

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class RiskEvent:
    event_id: int
    timestamp: float
    wall_time: float
    risk_level: str
    reason: str
    distance_m: float | None
    effective_distance_m: float | None
    closing_speed_mps: float | None
    cpa_distance_m: float | None
    cpa_time_s: float | None
    drone_a_xyz_m: tuple[float, float, float] | None
    drone_b_xyz_m: tuple[float, float, float] | None
    calibration_type: str

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class CollisionAnalysisResult:
    risk_level: str
    risk_label: str
    reason: str
    visual_alert_active: bool
    audio_alert_triggered: bool
    stereo_status: str
    calibration_type: str
    is_calibrated: bool
    synchronized: bool
    sync_delta_ms: float
    timestamp: float
    wall_time: float
    kinematics: tuple[DroneKinematics3D, ...] = field(default_factory=tuple)
    pair_metrics: PairCollisionMetrics | None = None
    all_pairs: tuple[PairCollisionMetrics, ...] = field(default_factory=tuple)

    def to_dict(self) -> dict:
        return {
            "risk_level": self.risk_level,
            "risk_label": self.risk_label,
            "reason": self.reason,
            "visual_alert_active": self.visual_alert_active,
            "audio_alert_triggered": self.audio_alert_triggered,
            "stereo_status": self.stereo_status,
            "calibration_type": self.calibration_type,
            "is_calibrated": self.is_calibrated,
            "synchronized": self.synchronized,
            "sync_delta_ms": self.sync_delta_ms,
            "timestamp": self.timestamp,
            "wall_time": self.wall_time,
            "kinematics": [k.to_dict() for k in self.kinematics],
            "pair_metrics": self.pair_metrics.to_dict() if self.pair_metrics else None,
            "all_pairs": [p.to_dict() for p in self.all_pairs],
        }


# ----------------------------------------- pure geometric & kinematic math
def format_direction_label(velocity_mps: tuple[float, float, float], speed_mps: float) -> str:
    if speed_mps < 0.10:
        return "정지 / Stationary"
    vx, vy, vz = velocity_mps
    parts = []
    if abs(vx) >= 0.15:
        parts.append("우(+X)" if vx > 0 else "좌(-X)")
    if abs(vy) >= 0.15:
        parts.append("하(+Y)" if vy > 0 else "상(-Y)")
    if abs(vz) >= 0.15:
        parts.append("원거리(+Z)" if vz > 0 else "근접(-Z)")
    axis_text = "·".join(parts) if parts else "이동 중"
    return f"{axis_text} ({speed_mps:.2f} m/s)"


def compute_pair_collision_metrics(
    kin_a: DroneKinematics3D,
    kin_b: DroneKinematics3D,
    settings: CollisionSettings,
) -> PairCollisionMetrics:
    """Compute 3D distance, closing speed, and Closest Point of Approach (CPA) between two drones."""
    pa = np.array(kin_a.xyz_m, dtype=np.float64)
    pb = np.array(kin_b.xyz_m, dtype=np.float64)
    va = np.array(kin_a.velocity_mps, dtype=np.float64) if kin_a.velocity_valid else np.zeros(3, dtype=np.float64)
    vb = np.array(kin_b.velocity_mps, dtype=np.float64) if kin_b.velocity_valid else np.zeros(3, dtype=np.float64)

    r_vec = pb - pa
    v_rel = vb - va
    distance_m = float(np.linalg.norm(r_vec))
    rel_speed = float(np.linalg.norm(v_rel))

    if distance_m > 1e-9:
        # Positive when pb and pa are getting closer: -d/dt ||pb - pa|| = -(r . v_rel) / ||r||
        closing_speed = float(-np.dot(r_vec, v_rel) / distance_m)
    else:
        closing_speed = 0.0

    v_rel_sq = float(np.dot(v_rel, v_rel))
    if v_rel_sq > 1e-6:
        t_cpa_unclamped = float(-np.dot(r_vec, v_rel) / v_rel_sq)
        t_cpa = min(max(0.0, t_cpa_unclamped), settings.prediction_horizon_s)
    else:
        t_cpa = 0.0

    cpa_pa = pa + va * t_cpa
    cpa_pb = pb + vb * t_cpa
    cpa_distance_m = float(np.linalg.norm(cpa_pb - cpa_pa))

    combined_unc = float(kin_a.position_uncertainty_m + kin_b.position_uncertainty_m)
    margin = settings.uncertainty_margin_sigma * combined_unc
    effective_dist = max(0.0, distance_m - margin)
    effective_cpa_dist = max(0.0, cpa_distance_m - margin)
    is_approaching = closing_speed >= settings.min_closing_speed_mps

    return PairCollisionMetrics(
        drone_a_id=kin_a.drone_id,
        drone_b_id=kin_b.drone_id,
        distance_m=distance_m,
        combined_uncertainty_m=combined_unc,
        effective_distance_m=effective_dist,
        relative_velocity_mps=(float(v_rel[0]), float(v_rel[1]), float(v_rel[2])),
        relative_speed_mps=rel_speed,
        closing_speed_mps=closing_speed,
        is_approaching=is_approaching,
        cpa_time_s=t_cpa,
        cpa_distance_m=cpa_distance_m,
        effective_cpa_distance_m=effective_cpa_dist,
        cpa_point_a_m=(float(cpa_pa[0]), float(cpa_pa[1]), float(cpa_pa[2])),
        cpa_point_b_m=(float(cpa_pb[0]), float(cpa_pb[1]), float(cpa_pb[2])),
    )


def evaluate_pair_risk(
    pair: PairCollisionMetrics,
    kin_a: DroneKinematics3D,
    kin_b: DroneKinematics3D,
    settings: CollisionSettings,
) -> tuple[str, str]:
    """Return (risk_level, reason) for a pair of confirmed 3D drones."""
    # First check coordinate reliability & position uncertainty gates
    if (kin_a.confidence_score < settings.min_confidence_score
            or kin_b.confidence_score < settings.min_confidence_score):
        return (
            RISK_UNKNOWN,
            f"3D 좌표 신뢰도 부족 (최소 {min(kin_a.confidence_score, kin_b.confidence_score) * 100:.0f}% "
            f"< 기준 {settings.min_confidence_score * 100:.0f}%) — 판단 불가",
        )
    if (kin_a.position_uncertainty_m > settings.max_position_uncertainty_m
            or kin_b.position_uncertainty_m > settings.max_position_uncertainty_m):
        return (
            RISK_UNKNOWN,
            f"3D 위치 오차 불확실성 과다 (±{max(kin_a.position_uncertainty_m, kin_b.position_uncertainty_m):.2f} m "
            f"> 기준 ±{settings.max_position_uncertainty_m:.2f} m) — 판단 불가",
        )

    # Check immediate DANGER distance
    if pair.effective_distance_m <= settings.danger_distance_m:
        return (
            RISK_DANGER,
            f"위험 거리 침범! 실측 {pair.distance_m:.2f} m (유효 {pair.effective_distance_m:.2f} m "
            f"<= 위험 기준 {settings.danger_distance_m:.2f} m)",
        )

    # Check predicted CPA DANGER when approaching
    if (pair.is_approaching
            and pair.cpa_time_s <= settings.cpa_danger_time_s
            and pair.effective_cpa_distance_m <= settings.danger_distance_m):
        return (
            RISK_DANGER,
            f"고속 접근 충돌 위험! 접근 속도 {pair.closing_speed_mps:.2f} m/s, "
            f"{pair.cpa_time_s:.1f}초 후 예상 최근접 거리 {pair.cpa_distance_m:.2f} m "
            f"(유효 {pair.effective_cpa_distance_m:.2f} m <= {settings.danger_distance_m:.2f} m)",
        )

    # Check immediate CAUTION distance
    if pair.effective_distance_m <= settings.warning_distance_m:
        return (
            RISK_CAUTION,
            f"주의 거리 이내 근접: 실측 {pair.distance_m:.2f} m (유효 {pair.effective_distance_m:.2f} m "
            f"<= 주의 기준 {settings.warning_distance_m:.2f} m)",
        )

    # Check predicted CPA CAUTION when approaching
    if (pair.is_approaching
            and pair.cpa_time_s <= settings.cpa_warning_time_s
            and pair.effective_cpa_distance_m <= settings.warning_distance_m):
        return (
            RISK_CAUTION,
            f"접근 주의: 상대 접근 속도 {pair.closing_speed_mps:.2f} m/s, "
            f"{pair.cpa_time_s:.1f}초 후 예상 최근접 거리 {pair.cpa_distance_m:.2f} m "
            f"(유효 {pair.effective_cpa_distance_m:.2f} m <= {settings.warning_distance_m:.2f} m)",
        )

    return (
        RISK_SAFE,
        f"안전 거리 유지: 드론 간 거리 {pair.distance_m:.2f} m (유효 {pair.effective_distance_m:.2f} m, "
        f"상대 속도 {pair.closing_speed_mps:+.2f} m/s)",
    )


class CollisionAnalyzer:
    """Stateful 3D trajectory velocity estimator, collision risk evaluator, and event logger."""

    def __init__(self, settings: CollisionSettings | None = None, history_window_s: float = 0.8):
        self.settings = settings or CollisionSettings()
        self.history_window_s = float(history_window_s)
        # drone_id -> deque of (timestamp, xyz_m)
        self._trajectories: dict[int, deque[tuple[float, tuple[float, float, float]]]] = {}
        self._smoothed_vel: dict[int, tuple[float, float, float]] = {}
        self._events: deque[RiskEvent] = deque(maxlen=MAX_RISK_EVENTS)
        self._next_event_id = 1
        self._last_risk_level = RISK_UNKNOWN
        self._last_audio_at = -1e9
        self._had_active_tracking = False

    @property
    def events(self) -> tuple[RiskEvent, ...]:
        return tuple(self._events)

    def clear_events(self) -> None:
        self._events.clear()

    def reset(self) -> None:
        self._trajectories.clear()
        self._smoothed_vel.clear()
        self._last_risk_level = RISK_UNKNOWN
        self._had_active_tracking = False

    def _estimate_kinematics(self, pose: Drone3DPose) -> DroneKinematics3D:
        hist = self._trajectories.setdefault(pose.drone_id, deque(maxlen=20))
        # Drop samples older than history_window_s or out-of-order timestamps
        while hist and (pose.timestamp - hist[0][0] > self.history_window_s or pose.timestamp <= hist[-1][0]):
            if pose.timestamp <= hist[-1][0]:
                hist.clear()
                self._smoothed_vel.pop(pose.drone_id, None)
                break
            hist.popleft()
        hist.append((pose.timestamp, pose.xyz_m))

        velocity_valid = False
        vx, vy, vz = 0.0, 0.0, 0.0
        if len(hist) >= 2:
            dt_total = hist[-1][0] - hist[0][0]
            if dt_total >= 0.01:
                times = np.array([t - hist[0][0] for t, _ in hist], dtype=np.float64)
                coords = np.array([xyz for _, xyz in hist], dtype=np.float64)
                # Weighted linear least squares slope for (x(t), y(t), z(t))
                t_mean = float(np.mean(times))
                denom = float(np.sum((times - t_mean) ** 2))
                if denom > 1e-9:
                    raw_vel = tuple(
                        float(np.sum((times - t_mean) * (coords[:, axis] - np.mean(coords[:, axis]))) / denom)
                        for axis in range(3)
                    )
                    prev_v = self._smoothed_vel.get(pose.drone_id)
                    if prev_v is not None:
                        vx = 0.65 * raw_vel[0] + 0.35 * prev_v[0]
                        vy = 0.65 * raw_vel[1] + 0.35 * prev_v[1]
                        vz = 0.65 * raw_vel[2] + 0.35 * prev_v[2]
                    else:
                        vx, vy, vz = raw_vel
                    self._smoothed_vel[pose.drone_id] = (vx, vy, vz)
                    velocity_valid = True

        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if speed > 1e-6:
            unit = (vx / speed, vy / speed, vz / speed)
        else:
            unit = (0.0, 0.0, 0.0)
        return DroneKinematics3D(
            drone_id=pose.drone_id,
            xyz_m=pose.xyz_m,
            velocity_mps=(vx, vy, vz),
            speed_mps=speed,
            direction_unit=unit,
            direction_label=format_direction_label((vx, vy, vz), speed),
            velocity_valid=velocity_valid,
            position_uncertainty_m=pose.position_uncertainty_m,
            confidence_score=pose.confidence_score,
            reprojection_error_px=pose.reprojection_error_px,
            cam1_track_id=pose.cam1_track_id,
            cam2_track_id=pose.cam2_track_id,
            drone_model=pose.drone_model,
            estimation_mode=pose.estimation_mode,
        )

    def _record_event_if_needed(
        self,
        risk_level: str,
        reason: str,
        stereo: StereoFrameResult,
        primary_pair: PairCollisionMetrics | None,
        kinematics: list[DroneKinematics3D],
    ) -> None:
        should_log = False
        if risk_level in (RISK_CAUTION, RISK_DANGER) and risk_level != self._last_risk_level:
            should_log = True
        elif (risk_level == RISK_UNKNOWN
              and self._had_active_tracking
              and self._last_risk_level in (RISK_SAFE, RISK_CAUTION, RISK_DANGER)):
            should_log = True

        if should_log:
            kin_map = {k.drone_id: k.xyz_m for k in kinematics}
            xyz_a = kin_map.get(primary_pair.drone_a_id) if primary_pair else (kinematics[0].xyz_m if kinematics else None)
            xyz_b = kin_map.get(primary_pair.drone_b_id) if primary_pair else (kinematics[1].xyz_m if len(kinematics) > 1 else None)
            self._events.append(
                RiskEvent(
                    event_id=self._next_event_id,
                    timestamp=stereo.timestamp,
                    wall_time=stereo.wall_time,
                    risk_level=risk_level,
                    reason=reason,
                    distance_m=primary_pair.distance_m if primary_pair else None,
                    effective_distance_m=primary_pair.effective_distance_m if primary_pair else None,
                    closing_speed_mps=primary_pair.closing_speed_mps if primary_pair else None,
                    cpa_distance_m=primary_pair.cpa_distance_m if primary_pair else None,
                    cpa_time_s=primary_pair.cpa_time_s if primary_pair else None,
                    drone_a_xyz_m=xyz_a,
                    drone_b_xyz_m=xyz_b,
                    calibration_type=stereo.calibration_type,
                )
            )
            self._next_event_id += 1
        self._last_risk_level = risk_level

    def analyze(self, stereo: StereoFrameResult) -> CollisionAnalysisResult:
        """Analyze a Phase 03 StereoFrameResult and return Phase 04 collision risk analysis."""
        kinematics = [self._estimate_kinematics(pose) for pose in stereo.drones]
        sync_ms = stereo.sync.abs_delta_s * 1000.0

        # Rule: never display SAFE when coordinates or synchronization are uncertain!
        if not stereo.is_calibrated or stereo.status == STEREO_STATUS_UNCALIBRATED:
            reason = "판단 불가: 카메라 보정 데이터가 없습니다 (임의 좌표 사용 금지)."
            self._record_event_if_needed(RISK_UNKNOWN, reason, stereo, None, kinematics)
            return CollisionAnalysisResult(
                risk_level=RISK_UNKNOWN,
                risk_label=RISK_LABELS[RISK_UNKNOWN],
                reason=reason,
                visual_alert_active=False,
                audio_alert_triggered=False,
                stereo_status=stereo.status,
                calibration_type=stereo.calibration_type,
                is_calibrated=False,
                synchronized=stereo.sync.synchronized,
                sync_delta_ms=sync_ms,
                timestamp=stereo.timestamp,
                wall_time=stereo.wall_time,
                kinematics=tuple(kinematics),
            )

        if not stereo.sync.synchronized or stereo.status == STEREO_STATUS_UNSYNCHRONIZED:
            reason = f"판단 불가: {stereo.sync.reason}"
            self._record_event_if_needed(RISK_UNKNOWN, reason, stereo, None, kinematics)
            return CollisionAnalysisResult(
                risk_level=RISK_UNKNOWN,
                risk_label=RISK_LABELS[RISK_UNKNOWN],
                reason=reason,
                visual_alert_active=False,
                audio_alert_triggered=False,
                stereo_status=stereo.status,
                calibration_type=stereo.calibration_type,
                is_calibrated=True,
                synchronized=False,
                sync_delta_ms=sync_ms,
                timestamp=stereo.timestamp,
                wall_time=stereo.wall_time,
                kinematics=tuple(kinematics),
            )

        if stereo.status == STEREO_STATUS_AMBIGUOUS and len(kinematics) < 2:
            reason = f"판단 불가: {stereo.status_message}"
            self._record_event_if_needed(RISK_UNKNOWN, reason, stereo, None, kinematics)
            return CollisionAnalysisResult(
                risk_level=RISK_UNKNOWN,
                risk_label=RISK_LABELS[RISK_UNKNOWN],
                reason=reason,
                visual_alert_active=False,
                audio_alert_triggered=False,
                stereo_status=stereo.status,
                calibration_type=stereo.calibration_type,
                is_calibrated=True,
                synchronized=True,
                sync_delta_ms=sync_ms,
                timestamp=stereo.timestamp,
                wall_time=stereo.wall_time,
                kinematics=tuple(kinematics),
            )

        if len(kinematics) < 2:
            if len(kinematics) == 1:
                reason = "판단 불가: 3D 좌표가 확정된 드론이 1대뿐입니다 (두 드론 간 거리 계산에는 2대 이상 필요)."
            else:
                reason = f"판단 불가: {stereo.status_message}"
            self._record_event_if_needed(RISK_UNKNOWN, reason, stereo, None, kinematics)
            return CollisionAnalysisResult(
                risk_level=RISK_UNKNOWN,
                risk_label=RISK_LABELS[RISK_UNKNOWN],
                reason=reason,
                visual_alert_active=False,
                audio_alert_triggered=False,
                stereo_status=stereo.status,
                calibration_type=stereo.calibration_type,
                is_calibrated=True,
                synchronized=True,
                sync_delta_ms=sync_ms,
                timestamp=stereo.timestamp,
                wall_time=stereo.wall_time,
                kinematics=tuple(kinematics),
            )

        # Evaluate all confirmed pairs of drones and select the highest-risk pair
        self._had_active_tracking = True
        pair_results: list[tuple[int, str, str, PairCollisionMetrics]] = []
        severity_order = {RISK_SAFE: 0, RISK_UNKNOWN: 1, RISK_CAUTION: 2, RISK_DANGER: 3}

        for idx_a in range(len(kinematics)):
            for idx_b in range(idx_a + 1, len(kinematics)):
                ka, kb = kinematics[idx_a], kinematics[idx_b]
                metrics = compute_pair_collision_metrics(ka, kb, self.settings)
                level, pair_reason = evaluate_pair_risk(metrics, ka, kb, self.settings)
                pair_results.append((severity_order[level], level, pair_reason, metrics))

        # Sort by highest severity, then smallest effective distance
        pair_results.sort(key=lambda item: (-item[0], item[3].effective_distance_m))
        _sev, top_level, top_reason, primary_pair = pair_results[0]
        all_pairs = tuple(item[3] for item in pair_results)

        # If any ambiguous pairs were also rejected in this frame, never report SAFE
        if top_level == RISK_SAFE and stereo.ambiguous_pairs > 0:
            top_level = RISK_UNKNOWN
            top_reason = f"판단 불가: 일부 드론 대응이 불확실하여 안전으로 단정할 수 없습니다 ({stereo.ambiguous_pairs}쌍 모호)."

        visual_active = bool(self.settings.visual_alert_enabled and top_level in (RISK_CAUTION, RISK_DANGER))
        audio_triggered = False
        if (self.settings.audio_alert_enabled
                and top_level in (RISK_CAUTION, RISK_DANGER)
                and (stereo.timestamp - self._last_audio_at >= self.settings.audio_cooldown_s
                     or (top_level == RISK_DANGER and self._last_risk_level != RISK_DANGER))):
            audio_triggered = True
            self._last_audio_at = stereo.timestamp

        self._record_event_if_needed(top_level, top_reason, stereo, primary_pair, kinematics)

        return CollisionAnalysisResult(
            risk_level=top_level,
            risk_label=RISK_LABELS[top_level],
            reason=top_reason,
            visual_alert_active=visual_active,
            audio_alert_triggered=audio_triggered,
            stereo_status=stereo.status,
            calibration_type=stereo.calibration_type,
            is_calibrated=True,
            synchronized=True,
            sync_delta_ms=sync_ms,
            timestamp=stereo.timestamp,
            wall_time=stereo.wall_time,
            kinematics=tuple(kinematics),
            pair_metrics=primary_pair,
            all_pairs=all_pairs,
        )
