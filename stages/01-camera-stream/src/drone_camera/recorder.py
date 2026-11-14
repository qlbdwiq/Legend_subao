"""Phase 05 experiment data recorder, CSV exporter, and recorded session playback/analysis."""

import csv
from dataclasses import asdict, dataclass
from datetime import datetime
import json
from pathlib import Path
import time

import cv2
import numpy as np

from .collision import (
    RISK_CAUTION,
    RISK_DANGER,
    RISK_SAFE,
    RISK_UNKNOWN,
    CollisionAnalysisResult,
    RiskEvent,
)
from .detections import FrameDetections
from .stereo_3d import StereoFrameResult

CSV_COLUMNS = (
    "tick_index",
    "wall_time",
    "timestamp",
    "calibration_type",
    "stereo_status",
    "synchronized",
    "sync_delta_ms",
    "risk_level",
    "risk_reason",
    "drones_3d_count",
    "drone_1_id",
    "drone_1_x_m",
    "drone_1_y_m",
    "drone_1_z_m",
    "drone_1_vx_mps",
    "drone_1_vy_mps",
    "drone_1_vz_mps",
    "drone_1_speed_mps",
    "drone_1_reproj_px",
    "drone_1_uncertainty_m",
    "drone_1_confidence",
    "drone_2_id",
    "drone_2_x_m",
    "drone_2_y_m",
    "drone_2_z_m",
    "drone_2_vx_mps",
    "drone_2_vy_mps",
    "drone_2_vz_mps",
    "drone_2_speed_mps",
    "drone_2_reproj_px",
    "drone_2_uncertainty_m",
    "drone_2_confidence",
    "distance_m",
    "effective_distance_m",
    "closing_speed_mps",
    "cpa_time_s",
    "cpa_distance_m",
    "effective_cpa_distance_m",
)

EVENT_CSV_COLUMNS = (
    "event_id",
    "wall_time",
    "timestamp",
    "risk_level",
    "reason",
    "calibration_type",
    "distance_m",
    "effective_distance_m",
    "closing_speed_mps",
    "cpa_distance_m",
    "cpa_time_s",
    "drone_a_x_m",
    "drone_a_y_m",
    "drone_a_z_m",
    "drone_b_x_m",
    "drone_b_y_m",
    "drone_b_z_m",
)


class RecorderError(ValueError):
    pass


@dataclass(frozen=True)
class RecordedTick:
    tick_index: int
    timestamp: float
    wall_time: float
    cam1_sequence: int
    cam2_sequence: int
    cam1_detections: int
    cam2_detections: int
    stereo: dict
    collision: dict

    def to_dict(self) -> dict:
        return asdict(self)

    def to_csv_row(self) -> dict:
        kinematics = self.collision.get("kinematics") or []
        pair = self.collision.get("pair_metrics") or {}
        d1 = kinematics[0] if len(kinematics) >= 1 else {}
        d2 = kinematics[1] if len(kinematics) >= 2 else {}
        d1_xyz = d1.get("xyz_m") or (None, None, None)
        d2_xyz = d2.get("xyz_m") or (None, None, None)
        d1_vel = d1.get("velocity_mps") or (None, None, None)
        d2_vel = d2.get("velocity_mps") or (None, None, None)
        return {
            "tick_index": self.tick_index,
            "wall_time": f"{self.wall_time:.4f}",
            "timestamp": f"{self.timestamp:.4f}",
            "calibration_type": self.collision.get("calibration_type", "none"),
            "stereo_status": self.collision.get("stereo_status", ""),
            "synchronized": bool(self.collision.get("synchronized", False)),
            "sync_delta_ms": f"{float(self.collision.get('sync_delta_ms', 0.0)):.2f}",
            "risk_level": self.collision.get("risk_level", RISK_UNKNOWN),
            "risk_reason": self.collision.get("reason", ""),
            "drones_3d_count": len(kinematics),
            "drone_1_id": d1.get("drone_id", ""),
            "drone_1_x_m": _fmt_opt(d1_xyz[0]),
            "drone_1_y_m": _fmt_opt(d1_xyz[1]),
            "drone_1_z_m": _fmt_opt(d1_xyz[2]),
            "drone_1_vx_mps": _fmt_opt(d1_vel[0]),
            "drone_1_vy_mps": _fmt_opt(d1_vel[1]),
            "drone_1_vz_mps": _fmt_opt(d1_vel[2]),
            "drone_1_speed_mps": _fmt_opt(d1.get("speed_mps")),
            "drone_1_reproj_px": _fmt_opt(d1.get("reprojection_error_px")),
            "drone_1_uncertainty_m": _fmt_opt(d1.get("position_uncertainty_m")),
            "drone_1_confidence": _fmt_opt(d1.get("confidence_score")),
            "drone_2_id": d2.get("drone_id", ""),
            "drone_2_x_m": _fmt_opt(d2_xyz[0]),
            "drone_2_y_m": _fmt_opt(d2_xyz[1]),
            "drone_2_z_m": _fmt_opt(d2_xyz[2]),
            "drone_2_vx_mps": _fmt_opt(d2_vel[0]),
            "drone_2_vy_mps": _fmt_opt(d2_vel[1]),
            "drone_2_vz_mps": _fmt_opt(d2_vel[2]),
            "drone_2_speed_mps": _fmt_opt(d2.get("speed_mps")),
            "drone_2_reproj_px": _fmt_opt(d2.get("reprojection_error_px")),
            "drone_2_uncertainty_m": _fmt_opt(d2.get("position_uncertainty_m")),
            "drone_2_confidence": _fmt_opt(d2.get("confidence_score")),
            "distance_m": _fmt_opt(pair.get("distance_m")),
            "effective_distance_m": _fmt_opt(pair.get("effective_distance_m")),
            "closing_speed_mps": _fmt_opt(pair.get("closing_speed_mps")),
            "cpa_time_s": _fmt_opt(pair.get("cpa_time_s")),
            "cpa_distance_m": _fmt_opt(pair.get("cpa_distance_m")),
            "effective_cpa_distance_m": _fmt_opt(pair.get("effective_cpa_distance_m")),
        }


def _fmt_opt(value, digits: int = 4) -> str:
    if value is None or value == "":
        return ""
    return f"{float(value):.{digits}f}"


@dataclass(frozen=True)
class SessionSummary:
    total_ticks: int
    duration_s: float
    calibration_type: str
    valid_pair_ticks: int
    min_distance_m: float | None
    min_effective_distance_m: float | None
    max_closing_speed_mps: float | None
    min_cpa_distance_m: float | None
    mean_reprojection_error_px: float | None
    mean_sync_delta_ms: float | None
    risk_counts: dict[str, int]


class ExperimentRecorder:
    """Records frame-by-frame 2D/3D/collision telemetry to JSONL + optional AVI videos,
    exports CSV files, and loads recorded sessions for playback & analysis.
    """

    def __init__(self):
        self._active = False
        self._stream = None
        self._jsonl_path: Path | None = None
        self._video_paths: list[Path | None] = [None, None]
        self._writers: list[cv2.VideoWriter | None] = [None, None]
        self._record_video = False
        self._video_fps = 15.0
        self._ticks: list[RecordedTick] = []

    @property
    def active(self) -> bool:
        return self._active

    @property
    def jsonl_path(self) -> Path | None:
        return self._jsonl_path

    @property
    def video_paths(self) -> tuple[Path | None, Path | None]:
        return (self._video_paths[0], self._video_paths[1])

    @property
    def ticks(self) -> tuple[RecordedTick, ...]:
        return tuple(self._ticks)

    def start(
        self,
        output_dir: Path,
        session_name: str = "",
        record_video: bool = False,
        video_fps: float = 15.0,
    ) -> Path:
        if self._active:
            self.stop()
        output_dir = Path(output_dir).expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        prefix = f"{session_name}_{stamp}" if session_name else f"experiment_{stamp}"
        self._jsonl_path = output_dir / f"{prefix}.jsonl"
        self._stream = self._jsonl_path.open("w", encoding="utf-8")
        self._record_video = bool(record_video)
        self._video_fps = max(1.0, min(60.0, float(video_fps)))
        if self._record_video:
            self._video_paths = [
                output_dir / f"{prefix}_cam1.avi",
                output_dir / f"{prefix}_cam2.avi",
            ]
        else:
            self._video_paths = [None, None]
        self._writers = [None, None]
        self._ticks.clear()
        self._active = True
        return self._jsonl_path

    def record_tick(
        self,
        det_01: FrameDetections | None,
        det_02: FrameDetections | None,
        stereo: StereoFrameResult,
        collision: CollisionAnalysisResult,
        frame_01: np.ndarray | None = None,
        frame_02: np.ndarray | None = None,
    ) -> RecordedTick:
        tick = RecordedTick(
            tick_index=len(self._ticks) + 1,
            timestamp=collision.timestamp or time.monotonic(),
            wall_time=collision.wall_time or time.time(),
            cam1_sequence=det_01.frame_sequence if det_01 else 0,
            cam2_sequence=det_02.frame_sequence if det_02 else 0,
            cam1_detections=len(det_01.observations) if det_01 else 0,
            cam2_detections=len(det_02.observations) if det_02 else 0,
            stereo=stereo.to_dict(),
            collision=collision.to_dict(),
        )
        self._ticks.append(tick)
        if self._active and self._stream is not None:
            self._stream.write(json.dumps(tick.to_dict(), ensure_ascii=False) + "\n")
            self._stream.flush()
            if self._record_video:
                for idx, img in enumerate((frame_01, frame_02)):
                    if img is not None and isinstance(img, np.ndarray) and img.ndim == 3:
                        self._write_video_frame(idx, img)
        return tick

    def _write_video_frame(self, index: int, image: np.ndarray) -> None:
        vpath = self._video_paths[index]
        if vpath is None:
            return
        writer = self._writers[index]
        height, width = image.shape[:2]
        if writer is None:
            writer = cv2.VideoWriter(
                str(vpath), cv2.VideoWriter_fourcc(*"MJPG"), self._video_fps, (width, height)
            )
            self._writers[index] = writer
        if writer.isOpened():
            writer.write(image)

    def stop(self) -> Path | None:
        self._active = False
        if self._stream is not None:
            try:
                self._stream.flush()
                self._stream.close()
            except OSError:
                pass
            self._stream = None
        for idx, writer in enumerate(self._writers):
            if writer is not None:
                writer.release()
                self._writers[idx] = None
        return self._jsonl_path


def export_ticks_csv(csv_path: Path, ticks: list[RecordedTick] | tuple[RecordedTick, ...]) -> Path:
    """Export recorded ticks to a UTF-8 CSV file."""
    csv_path = Path(csv_path).expanduser().resolve()
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        for tick in ticks:
            writer.writerow(tick.to_csv_row())
    return csv_path


def export_events_csv(csv_path: Path, events: list[RiskEvent] | tuple[RiskEvent, ...]) -> Path:
    """Export recorded collision risk events to a UTF-8 CSV file."""
    csv_path = Path(csv_path).expanduser().resolve()
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=EVENT_CSV_COLUMNS)
        writer.writeheader()
        for ev in events:
            pa = ev.drone_a_xyz_m or (None, None, None)
            pb = ev.drone_b_xyz_m or (None, None, None)
            writer.writerow({
                "event_id": ev.event_id,
                "wall_time": f"{ev.wall_time:.4f}",
                "timestamp": f"{ev.timestamp:.4f}",
                "risk_level": ev.risk_level,
                "reason": ev.reason,
                "calibration_type": ev.calibration_type,
                "distance_m": _fmt_opt(ev.distance_m),
                "effective_distance_m": _fmt_opt(ev.effective_distance_m),
                "closing_speed_mps": _fmt_opt(ev.closing_speed_mps),
                "cpa_distance_m": _fmt_opt(ev.cpa_distance_m),
                "cpa_time_s": _fmt_opt(ev.cpa_time_s),
                "drone_a_x_m": _fmt_opt(pa[0]),
                "drone_a_y_m": _fmt_opt(pa[1]),
                "drone_a_z_m": _fmt_opt(pa[2]),
                "drone_b_x_m": _fmt_opt(pb[0]),
                "drone_b_y_m": _fmt_opt(pb[1]),
                "drone_b_z_m": _fmt_opt(pb[2]),
            })
    return csv_path


def load_session(jsonl_path: Path) -> list[RecordedTick]:
    """Load a recorded JSONL session file for playback and analysis."""
    path = Path(jsonl_path).expanduser().resolve()
    if not path.is_file():
        raise RecorderError(f"기록 파일을 찾을 수 없습니다: {path.name}")
    ticks: list[RecordedTick] = []
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            raw = json.loads(line)
            ticks.append(
                RecordedTick(
                    tick_index=int(raw["tick_index"]),
                    timestamp=float(raw["timestamp"]),
                    wall_time=float(raw["wall_time"]),
                    cam1_sequence=int(raw.get("cam1_sequence", 0)),
                    cam2_sequence=int(raw.get("cam2_sequence", 0)),
                    cam1_detections=int(raw.get("cam1_detections", 0)),
                    cam2_detections=int(raw.get("cam2_detections", 0)),
                    stereo=dict(raw["stereo"]),
                    collision=dict(raw["collision"]),
                )
            )
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise RecorderError(f"기록 파일 형식이 올바르지 않습니다: {exc}") from None
    if not ticks:
        raise RecorderError("기록 파일에 데이터가 없습니다.")
    return ticks


def summarize_session(ticks: list[RecordedTick] | tuple[RecordedTick, ...]) -> SessionSummary:
    """Compute statistical summary over a recorded or replayed session."""
    risk_counts = {RISK_UNKNOWN: 0, RISK_SAFE: 0, RISK_CAUTION: 0, RISK_DANGER: 0}
    if not ticks:
        return SessionSummary(
            total_ticks=0,
            duration_s=0.0,
            calibration_type="none",
            valid_pair_ticks=0,
            min_distance_m=None,
            min_effective_distance_m=None,
            max_closing_speed_mps=None,
            min_cpa_distance_m=None,
            mean_reprojection_error_px=None,
            mean_sync_delta_ms=None,
            risk_counts=risk_counts,
        )

    duration = max(0.0, float(ticks[-1].timestamp - ticks[0].timestamp))
    calib_types = {t.collision.get("calibration_type", "none") for t in ticks}
    calib_type = (
        "field" if "field" in calib_types else ("synthetic" if "synthetic" in calib_types else "none")
    )

    distances = []
    eff_distances = []
    closing_speeds = []
    cpa_distances = []
    reproj_errors = []
    sync_deltas = []

    for tick in ticks:
        level = tick.collision.get("risk_level", RISK_UNKNOWN)
        risk_counts[level] = risk_counts.get(level, 0) + 1
        if tick.collision.get("synchronized"):
            sync_deltas.append(float(tick.collision.get("sync_delta_ms", 0.0)))
        for k in tick.collision.get("kinematics") or []:
            if k.get("reprojection_error_px") is not None:
                reproj_errors.append(float(k["reprojection_error_px"]))
        pair = tick.collision.get("pair_metrics")
        if pair and pair.get("distance_m") is not None:
            distances.append(float(pair["distance_m"]))
            eff_distances.append(float(pair["effective_distance_m"]))
            closing_speeds.append(float(pair["closing_speed_mps"]))
            cpa_distances.append(float(pair["cpa_distance_m"]))

    return SessionSummary(
        total_ticks=len(ticks),
        duration_s=duration,
        calibration_type=calib_type,
        valid_pair_ticks=len(distances),
        min_distance_m=min(distances) if distances else None,
        min_effective_distance_m=min(eff_distances) if eff_distances else None,
        max_closing_speed_mps=max(closing_speeds) if closing_speeds else None,
        min_cpa_distance_m=min(cpa_distances) if cpa_distances else None,
        mean_reprojection_error_px=(sum(reproj_errors) / len(reproj_errors)) if reproj_errors else None,
        mean_sync_delta_ms=(sum(sync_deltas) / len(sync_deltas)) if sync_deltas else None,
        risk_counts=risk_counts,
    )
