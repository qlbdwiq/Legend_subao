"""Phase 05 Qt widgets: 3D spatial visualization, Calibration Wizard dialog, and Session Playback dialog."""

from dataclasses import replace
import math
from pathlib import Path

import cv2
import numpy as np
from PySide6.QtCore import QPointF, Qt, QTimer
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap, QPolygonF
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from .calibration import (
    CalibrationError,
    StereoCalibration,
    calibrate_stereo_cameras,
    create_synthetic_calibration,
    detect_chessboard_corners,
    draw_chessboard_preview,
    project_chessboard_corners_pair,
)
from .collision import (
    RISK_CAUTION,
    RISK_DANGER,
    RISK_SAFE,
    RISK_UNKNOWN,
    CollisionAnalysisResult,
)
from .recorder import (
    RecorderError,
    RecordedTick,
    export_ticks_csv,
    load_session,
    summarize_session,
)
from .stereo_3d import StereoFrameResult

RISK_COLORS = {
    RISK_UNKNOWN: "#94a3b8",
    RISK_SAFE: "#22c55e",
    RISK_CAUTION: "#f59e0b",
    RISK_DANGER: "#ef4444",
}

DRONE_3D_COLORS = ("#38bdf8", "#f472b6", "#a3e635", "#c084fc")


class DroneSpace3DWidget(QWidget):
    """Lightweight QPainter 3D visualization of cameras, 3D drones, velocities, distance, and CPA."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(280, 210)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._calibration: StereoCalibration | None = None
        self._stereo: StereoFrameResult | None = None
        self._collision: CollisionAnalysisResult | None = None
        self._view_mode = "3d"      # "3d", "top" (X-Z), "front" (X-Y)
        self._yaw_deg = -28.0
        self._pitch_deg = 22.0
        self._drag_pos = None

    def set_view_mode(self, mode: str) -> None:
        if mode in ("3d", "top", "front"):
            self._view_mode = mode
            self.update()

    def update_scene(
        self,
        calibration: StereoCalibration | None,
        stereo: StereoFrameResult | None,
        collision: CollisionAnalysisResult | None,
    ) -> None:
        self._calibration = calibration
        self._stereo = stereo
        self._collision = collision
        self.update()

    def _project_3d_to_2d(self, pt_xyz: tuple[float, float, float], width: float, height: float) -> tuple[float, float]:
        """Map 3D Camera-1 coordinates (X right, Y down, Z forward in meters) to widget pixels."""
        x, y, z = float(pt_xyz[0]), float(pt_xyz[1]), float(pt_xyz[2])
        # Convert to visualization frame where +X is right, +Up is -Y, and Z is centered around z_center = 8m
        up = -y
        z_centered = z - 8.0
        scale = min(width, height) / 18.0

        if self._view_mode == "top":
            # Top-down X (horizontal) vs Z (vertical upward)
            sx = width * 0.5 + x * scale
            sy = height * 0.85 - z * (height * 0.72 / 18.0)
            return sx, sy
        if self._view_mode == "front":
            # Front X (horizontal) vs Up=-Y (vertical)
            sx = width * 0.5 + x * scale
            sy = height * 0.55 - up * scale
            return sx, sy

        # 3D isometric / rotated projection
        yaw = math.radians(self._yaw_deg)
        pitch = math.radians(self._pitch_deg)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        cos_p, sin_p = math.cos(pitch), math.sin(pitch)

        rx = x * cos_y + z_centered * sin_y
        rz = -x * sin_y + z_centered * cos_y
        ry = up * cos_p - rz * sin_p
        sx = width * 0.5 + rx * scale
        sy = height * 0.56 - ry * scale
        return sx, sy

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_pos = event.position()

    def mouseMoveEvent(self, event):
        if self._drag_pos is not None and self._view_mode == "3d":
            delta = event.position() - self._drag_pos
            self._drag_pos = event.position()
            self._yaw_deg = max(-85.0, min(85.0, self._yaw_deg + delta.x() * 0.4))
            self._pitch_deg = max(5.0, min(80.0, self._pitch_deg + delta.y() * 0.4))
            self.update()

    def mouseReleaseEvent(self, event):
        self._drag_pos = None

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = float(self.width()), float(self.height())
        painter.fillRect(0, 0, int(w), int(h), QColor("#09101d"))

        # Draw ground grid (Y = +1.5m below camera level, X in [-6, 6], Z in [0, 16])
        grid_pen = QPen(QColor("#1e293b"), 1)
        painter.setPen(grid_pen)
        ground_y = 1.5
        for x_grid in range(-6, 7, 2):
            p1 = self._project_3d_to_2d((x_grid, ground_y, 0.0), w, h)
            p2 = self._project_3d_to_2d((x_grid, ground_y, 16.0), w, h)
            painter.drawLine(QPointF(*p1), QPointF(*p2))
        for z_grid in range(0, 17, 2):
            p1 = self._project_3d_to_2d((-6.0, ground_y, z_grid), w, h)
            p2 = self._project_3d_to_2d((6.0, ground_y, z_grid), w, h)
            painter.drawLine(QPointF(*p1), QPointF(*p2))

        # Draw Camera 1 at origin and Camera 2 if calibrated
        cam1_pt = self._project_3d_to_2d((0.0, 0.0, 0.0), w, h)
        painter.setPen(QPen(QColor("#60a5fa"), 2))
        painter.setBrush(QColor("#2563eb"))
        painter.drawEllipse(QPointF(*cam1_pt), 5, 5)
        painter.drawText(int(cam1_pt[0]) - 18, int(cam1_pt[1]) + 16, "Cam1(0,0,0)")

        if self._calibration is not None:
            c2_xyz = tuple(self._calibration.camera_2_center_in_cam1())
            cam2_pt = self._project_3d_to_2d(c2_xyz, w, h)
            painter.setPen(QPen(QColor("#fbbf24"), 1, Qt.PenStyle.DashLine))
            painter.drawLine(QPointF(*cam1_pt), QPointF(*cam2_pt))
            painter.setPen(QPen(QColor("#34d399"), 2))
            painter.setBrush(QColor("#059669"))
            painter.drawEllipse(QPointF(*cam2_pt), 5, 5)
            painter.drawText(
                int(cam2_pt[0]) - 12,
                int(cam2_pt[1]) + 16,
                f"Cam2 (B={self._calibration.baseline_m:.2f}m)",
            )

        # Draw 3D drones and collision line
        kinematics = self._collision.kinematics if self._collision is not None else ()
        pair = self._collision.pair_metrics if self._collision is not None else None
        risk_level = self._collision.risk_level if self._collision is not None else RISK_UNKNOWN

        if pair is not None and len(kinematics) >= 2:
            ka = next((k for k in kinematics if k.drone_id == pair.drone_a_id), kinematics[0])
            kb = next((k for k in kinematics if k.drone_id == pair.drone_b_id), kinematics[1])
            pa_2d = self._project_3d_to_2d(ka.xyz_m, w, h)
            pb_2d = self._project_3d_to_2d(kb.xyz_m, w, h)
            line_color = QColor(RISK_COLORS.get(risk_level, "#94a3b8"))
            painter.setPen(QPen(line_color, 2.5))
            painter.drawLine(QPointF(*pa_2d), QPointF(*pb_2d))

            mid_x = 0.5 * (pa_2d[0] + pb_2d[0])
            mid_y = 0.5 * (pa_2d[1] + pb_2d[1])
            font = QFont(painter.font())
            font.setBold(True)
            painter.setFont(font)
            painter.setPen(line_color)
            painter.drawText(int(mid_x) + 6, int(mid_y) - 6, f"{pair.distance_m:.2f} m")

            # Draw CPA predicted points if approaching
            if pair.is_approaching and pair.cpa_time_s > 0.05:
                cpa_a_2d = self._project_3d_to_2d(pair.cpa_point_a_m, w, h)
                cpa_b_2d = self._project_3d_to_2d(pair.cpa_point_b_m, w, h)
                painter.setPen(QPen(QColor("#f87171"), 1.5, Qt.PenStyle.DotLine))
                painter.drawLine(QPointF(*cpa_a_2d), QPointF(*cpa_b_2d))

        for idx, kin in enumerate(kinematics):
            color = QColor(DRONE_3D_COLORS[idx % len(DRONE_3D_COLORS)])
            pt_2d = self._project_3d_to_2d(kin.xyz_m, w, h)
            # Draw uncertainty halo
            unc_px = max(8.0, min(42.0, kin.position_uncertainty_m * (min(w, h) / 18.0)))
            halo = QColor(color)
            halo.setAlpha(45)
            painter.setPen(QPen(color, 1, Qt.PenStyle.DotLine))
            painter.setBrush(halo)
            painter.drawEllipse(QPointF(*pt_2d), unc_px, unc_px)

            # Draw velocity vector arrow (1 second lookahead)
            if kin.velocity_valid and kin.speed_mps >= 0.1:
                tip_3d = (
                    kin.xyz_m[0] + kin.velocity_mps[0],
                    kin.xyz_m[1] + kin.velocity_mps[1],
                    kin.xyz_m[2] + kin.velocity_mps[2],
                )
                tip_2d = self._project_3d_to_2d(tip_3d, w, h)
                painter.setPen(QPen(color, 2))
                painter.drawLine(QPointF(*pt_2d), QPointF(*tip_2d))

            painter.setPen(QPen(Qt.GlobalColor.white, 1.5))
            painter.setBrush(color)
            painter.drawEllipse(QPointF(*pt_2d), 6, 6)
            x, y, z = kin.xyz_m
            painter.setPen(color)
            painter.drawText(
                int(pt_2d[0]) + 8,
                int(pt_2d[1]) - 8,
                f"D{kin.drone_id} ({x:+.2f}, {y:+.2f}, {z:.2f})m",
            )

        # Header overlay inside 3D widget
        painter.setPen(QColor("#94a3b8"))
        mode_names = {"3d": "3D 사선 뷰 (드래그 회전)", "top": "상면도 (X-Z)", "front": "정면도 (X-Y)"}
        status_note = (
            "미보정 — 3D 미출력" if self._calibration is None
            else ("합성 보정(실험용)" if self._calibration.calibration_type == "synthetic" else "현장 실측 보정")
        )
        painter.drawText(10, 18, f"{mode_names.get(self._view_mode, '3D')} · {status_note}")
        painter.end()


class CalibrationWizardDialog(QDialog):
    """Step-by-step camera calibration wizard for field chessboard calibration & synthetic presets."""

    def __init__(self, current_calibration: StereoCalibration | None, frame_provider=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("스테레오 카메라 보정 마법사 / Stereo Calibration Wizard")
        self.setMinimumWidth(820)
        self.frame_provider = frame_provider  # Callable returning (frame1, frame2) or (None, None)
        self.result_calibration: StereoCalibration | None = None
        self.captured_corners_01: list[np.ndarray] = []
        self.captured_corners_02: list[np.ndarray] = []
        self.captured_size_01: tuple[int, int] = (1280, 720)
        self.captured_size_02: tuple[int, int] = (1280, 720)
        self.captured_type: str = "field"

        layout = QVBoxLayout(self)

        guide = QLabel(
            "1. [현장 보정] 체커보드 판을 두 카메라가 동시에 볼 수 있는 겹침 영역에 들고 다양한 각도·거리에서 최소 5쌍 이상 캡처하세요.\n"
            "2. 줄자로 두 카메라 렌즈 중심 사이의 실제 거리(기준선 길이, m)를 측정하여 입력하면 스케일 정확도가 향상됩니다.\n"
            "3. [소프트웨어 검증용] 실제 장비 없이 테스트할 때는 아래 '합성 보정 프리셋 생성'을 사용하세요 (현장 보정 완료로 표시되지 않음)."
        )
        guide.setWordWrap(True)
        guide.setStyleSheet("color: #cbd5e1; background: #0f172a; padding: 8px; border-radius: 6px;")
        layout.addWidget(guide)

        top_row = QHBoxLayout()

        # Left group: Chessboard & Extrinsic settings
        board_group = QGroupBox("체커보드 및 동기화·대응 기준 설정")
        form = QFormLayout(board_group)
        self.cols_spin = QSpinBox()
        self.cols_spin.setRange(3, 25)
        self.cols_spin.setValue(7)
        self.rows_spin = QSpinBox()
        self.rows_spin.setRange(3, 25)
        self.rows_spin.setValue(5)
        self.square_mm_spin = QDoubleSpinBox()
        self.square_mm_spin.setRange(5.0, 500.0)
        self.square_mm_spin.setDecimals(1)
        self.square_mm_spin.setValue(30.0)
        self.square_mm_spin.setSuffix(" mm")

        self.override_baseline_check = QCheckBox("실측 기준선 길이(Baseline) 직접 지정")
        self.baseline_spin = QDoubleSpinBox()
        self.baseline_spin.setRange(0.05, 500.0)
        self.baseline_spin.setDecimals(3)
        self.baseline_spin.setValue(current_calibration.baseline_m if current_calibration else 2.0)
        self.baseline_spin.setSuffix(" m")

        self.time_offset_spin = QDoubleSpinBox()
        self.time_offset_spin.setRange(-5.0, 5.0)
        self.time_offset_spin.setDecimals(3)
        self.time_offset_spin.setSingleStep(0.010)
        self.time_offset_spin.setValue(current_calibration.time_offset_s if current_calibration else 0.0)
        self.time_offset_spin.setSuffix(" s")

        self.max_sync_spin = QDoubleSpinBox()
        self.max_sync_spin.setRange(0.005, 2.0)
        self.max_sync_spin.setDecimals(3)
        self.max_sync_spin.setSingleStep(0.010)
        self.max_sync_spin.setValue(current_calibration.max_sync_delta_s if current_calibration else 0.10)
        self.max_sync_spin.setSuffix(" s")

        self.max_reproj_spin = QDoubleSpinBox()
        self.max_reproj_spin.setRange(1.0, 100.0)
        self.max_reproj_spin.setDecimals(1)
        self.max_reproj_spin.setValue(current_calibration.max_reprojection_error_px if current_calibration else 15.0)
        self.max_reproj_spin.setSuffix(" px")

        form.addRow("체커보드 가로 내부 코너 (Cols)", self.cols_spin)
        form.addRow("체커보드 세로 내부 코너 (Rows)", self.rows_spin)
        form.addRow("사각형 한 변 길이 (Square size)", self.square_mm_spin)
        form.addRow(self.override_baseline_check, self.baseline_spin)
        form.addRow("카메라 2 시간 보정 오프셋", self.time_offset_spin)
        form.addRow("최대 허용 프레임 시간차", self.max_sync_spin)
        form.addRow("최대 허용 재투영 오차", self.max_reproj_spin)
        top_row.addWidget(board_group, 1)

        # Right group: Capture & Calibration execution
        action_group = QGroupBox("코너 캡처 및 보정 계산")
        action_col = QVBoxLayout(action_group)
        self.capture_status_label = QLabel("캡처된 체커보드 쌍: 0쌍 (최소 3쌍 필요, 5~15쌍 권장)")
        self.capture_status_label.setWordWrap(True)
        action_col.addWidget(self.capture_status_label)

        btn_capture_live = QPushButton("현재 두 카메라 화면에서 체커보드 코너 캡처")
        btn_capture_live.clicked.connect(self.capture_from_live_cameras)
        btn_clear_pairs = QPushButton("캡처 목록 초기화")
        btn_clear_pairs.clicked.connect(self.clear_captured_pairs)
        btn_run_calib = QPushButton("체커보드 기반 스테레오 보정 계산 실행")
        btn_run_calib.clicked.connect(self.run_stereo_calibration)
        btn_synth_samples = QPushButton("테스트용 가상 체커보드 8쌍 생성 후 보정 검증")
        btn_synth_samples.clicked.connect(self.generate_and_calibrate_synthetic_board)
        btn_synth_preset = QPushButton("실험 전 검증용 합성(Synthetic) 보정 프리셋 바로 적용")
        btn_synth_preset.clicked.connect(self.apply_synthetic_preset)

        for b in (btn_capture_live, btn_clear_pairs, btn_run_calib, btn_synth_samples, btn_synth_preset):
            action_col.addWidget(b)

        self.result_label = QLabel("보정 결과: 아직 계산되지 않음")
        self.result_label.setWordWrap(True)
        self.result_label.setStyleSheet("color: #38bdf8; font-weight: 600;")
        action_col.addWidget(self.result_label)
        top_row.addWidget(action_group, 1)

        layout.addLayout(top_row)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self._accept_if_ready)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def clear_captured_pairs(self) -> None:
        self.captured_corners_01.clear()
        self.captured_corners_02.clear()
        self.capture_status_label.setText("캡처된 체커보드 쌍: 0쌍")

    def capture_from_live_cameras(self) -> None:
        if self.frame_provider is None:
            QMessageBox.warning(self, "체커보드 캡처", "연결된 카메라 영상 공급자가 없습니다.")
            return
        frame1, frame2 = self.frame_provider()
        if frame1 is None or frame2 is None:
            QMessageBox.warning(self, "체커보드 캡처", "두 카메라가 모두 연결되어 영상을 수신 중이어야 합니다.")
            return
        cols = self.cols_spin.value()
        rows = self.rows_spin.value()
        c1 = detect_chessboard_corners(frame1.image, cols, rows)
        c2 = detect_chessboard_corners(frame2.image, cols, rows)
        if c1 is None or c2 is None:
            missing = "카메라 1·2 모두" if (c1 is None and c2 is None) else ("카메라 1" if c1 is None else "카메라 2")
            QMessageBox.warning(
                self,
                "코너 미검출",
                f"{missing} 화면에서 ({cols}×{rows}) 체커보드 코너를 찾지 못했습니다. 조명과 각도를 조절하세요.",
            )
            return
        self.captured_corners_01.append(c1)
        self.captured_corners_02.append(c2)
        self.captured_size_01 = frame1.resolution
        self.captured_size_02 = frame2.resolution
        self.captured_type = "field"
        self.capture_status_label.setText(
            f"캡처된 체커보드 쌍: {len(self.captured_corners_01)}쌍 (현장 카메라 캡처)"
        )

    def generate_and_calibrate_synthetic_board(self) -> None:
        cols = self.cols_spin.value()
        rows = self.rows_spin.value()
        sq_m = self.square_mm_spin.value() / 1000.0
        baseline = self.baseline_spin.value()
        synth_rig = create_synthetic_calibration(baseline_m=baseline, convergence_deg=6.0)
        poses = [
            ((0.05, -0.08, 0.02), (-0.30, -0.20, 2.2)),
            ((-0.12, 0.10, -0.05), (0.10, -0.20, 2.5)),
            ((0.18, 0.05, 0.12), (-0.20, 0.05, 2.8)),
            ((-0.08, -0.18, -0.10), (0.20, 0.10, 2.4)),
            ((0.15, -0.14, 0.20), (-0.10, -0.15, 2.0)),
            ((-0.20, 0.16, -0.15), (0.05, -0.05, 3.1)),
            ((0.02, 0.22, -0.08), (-0.25, -0.10, 2.6)),
            ((-0.16, -0.06, 0.18), (0.15, -0.22, 2.3)),
        ]
        self.captured_corners_01.clear()
        self.captured_corners_02.clear()
        for rvec, tvec in poses:
            c1, c2 = project_chessboard_corners_pair(synth_rig, cols, rows, sq_m, rvec, tvec)
            self.captured_corners_01.append(c1)
            self.captured_corners_02.append(c2)
        self.captured_size_01 = synth_rig.camera_01.image_size
        self.captured_size_02 = synth_rig.camera_02.image_size
        self.captured_type = "synthetic"
        self.capture_status_label.setText(f"캡처된 체커보드 쌍: {len(self.captured_corners_01)}쌍 (합성 검증 데이터)")
        self.run_stereo_calibration()

    def run_stereo_calibration(self) -> None:
        try:
            cols = self.cols_spin.value()
            rows = self.rows_spin.value()
            sq_m = self.square_mm_spin.value() / 1000.0
            baseline_override = self.baseline_spin.value() if self.override_baseline_check.isChecked() else None
            calib = calibrate_stereo_cameras(
                self.captured_corners_01,
                self.captured_corners_02,
                self.captured_size_01,
                self.captured_size_02,
                cols,
                rows,
                sq_m,
                baseline_m=baseline_override,
                calibration_type=self.captured_type,
                time_offset_s=self.time_offset_spin.value(),
                max_sync_delta_s=self.max_sync_spin.value(),
            )
            calib = replace(calib, max_reprojection_error_px=self.max_reproj_spin.value())
            self.result_calibration = calib
            tag = "합성 보정(실험용)" if calib.calibration_type == "synthetic" else "현장 실측 보정"
            self.result_label.setText(
                f"보정 완료 [{tag}] · 기준선 {calib.baseline_m:.3f} m · "
                f"스테레오 RMS {calib.rms_error:.3f} px "
                f"(Cam1 {calib.camera_01.rms_error:.3f} px, Cam2 {calib.camera_02.rms_error:.3f} px)"
            )
        except CalibrationError as exc:
            QMessageBox.warning(self, "스테레오 보정 오류", str(exc))

    def apply_synthetic_preset(self) -> None:
        try:
            calib = create_synthetic_calibration(
                image_size=(1280, 720),
                focal_px=900.0,
                baseline_m=self.baseline_spin.value(),
                convergence_deg=0.0,
                time_offset_s=self.time_offset_spin.value(),
                max_sync_delta_s=self.max_sync_spin.value(),
            )
            calib = replace(calib, max_reprojection_error_px=self.max_reproj_spin.value())
            self.result_calibration = calib
            self.result_label.setText(
                f"합성 보정 프리셋 준비됨 [실험용] · 기준선 {calib.baseline_m:.3f} m · "
                f"동기화 허용 {calib.max_sync_delta_s * 1000:.0f} ms"
            )
        except CalibrationError as exc:
            QMessageBox.warning(self, "합성 프리셋 오류", str(exc))

    def _accept_if_ready(self) -> None:
        if self.result_calibration is None:
            QMessageBox.warning(self, "보정 저장", "먼저 보정 계산을 실행하거나 합성 보정 프리셋을 적용하세요.")
            return
        self.accept()


class PlaybackDialog(QDialog):
    """Replay and analyze recorded experiment JSONL telemetry and optional synchronized AVI videos."""

    def __init__(self, jsonl_path: Path | None = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("실험 기록 재생 및 분석 / Recorded Session Playback & Analysis")
        self.setMinimumSize(880, 620)
        self.ticks: list[RecordedTick] = []
        self.current_index = 0
        self.video_captures: list[cv2.VideoCapture | None] = [None, None]

        layout = QVBoxLayout(self)

        top_bar = QHBoxLayout()
        btn_open = QPushButton("기록 파일(.jsonl) 불러오기…")
        btn_open.clicked.connect(self.choose_session_file)
        btn_export_csv = QPushButton("현재 세션 CSV 내보내기…")
        btn_export_csv.clicked.connect(self.export_current_csv)
        self.file_label = QLabel("기록 파일을 선택하세요.")
        top_bar.addWidget(btn_open)
        top_bar.addWidget(btn_export_csv)
        top_bar.addWidget(self.file_label, 1)
        layout.addLayout(top_bar)

        self.summary_label = QLabel("세션 요약 대기 중")
        self.summary_label.setWordWrap(True)
        self.summary_label.setStyleSheet("background: #0f172a; padding: 8px; border-radius: 6px;")
        layout.addWidget(self.summary_label)

        center_row = QHBoxLayout()
        self.space_3d = DroneSpace3DWidget()
        center_row.addWidget(self.space_3d, 2)

        right_col = QVBoxLayout()
        self.video_labels = [QLabel("Cam 1 녹화 영상 없음"), QLabel("Cam 2 녹화 영상 없음")]
        for lbl in self.video_labels:
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setMinimumSize(220, 124)
            lbl.setStyleSheet("background: #05080d; color: #64748b; border-radius: 6px;")
            right_col.addWidget(lbl, 1)
        self.tick_detail_label = QLabel("프레임 상세 정보")
        self.tick_detail_label.setWordWrap(True)
        self.tick_detail_label.setMinimumWidth(300)
        right_col.addWidget(self.tick_detail_label)
        center_row.addLayout(right_col, 1)
        layout.addLayout(center_row, 1)

        controls = QHBoxLayout()
        self.btn_play = QPushButton("재생 / Play")
        self.btn_play.clicked.connect(self.toggle_play)
        btn_prev = QPushButton("◀ 이전")
        btn_prev.clicked.connect(lambda: self.seek_to(self.current_index - 1))
        btn_next = QPushButton("다음 ▶")
        btn_next.clicked.connect(lambda: self.seek_to(self.current_index + 1))
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, 0)
        self.slider.valueChanged.connect(self.seek_to)
        controls.addWidget(self.btn_play)
        controls.addWidget(btn_prev)
        controls.addWidget(btn_next)
        controls.addWidget(self.slider, 1)
        layout.addLayout(controls)

        self.play_timer = QTimer(self)
        self.play_timer.setInterval(100)
        self.play_timer.timeout.connect(self._advance_playback)

        if jsonl_path is not None and jsonl_path.is_file():
            self.load_file(jsonl_path)

    def choose_session_file(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "실험 기록 파일 선택", str(Path.home()), "Experiment Log (*.jsonl)"
        )
        if path:
            self.load_file(Path(path))

    def load_file(self, path: Path) -> None:
        self.play_timer.stop()
        self.btn_play.setText("재생 / Play")
        try:
            self.ticks = load_session(path)
        except RecorderError as exc:
            QMessageBox.warning(self, "기록 불러오기 오류", str(exc))
            return

        self.file_label.setText(f"파일: {path.name} ({len(self.ticks)} 프레임)")
        summary = summarize_session(self.ticks)
        min_d = f"{summary.min_distance_m:.2f} m" if summary.min_distance_m is not None else "—"
        max_v = f"{summary.max_closing_speed_mps:.2f} m/s" if summary.max_closing_speed_mps is not None else "—"
        min_cpa = f"{summary.min_cpa_distance_m:.2f} m" if summary.min_cpa_distance_m is not None else "—"
        reproj = f"{summary.mean_reprojection_error_px:.2f} px" if summary.mean_reprojection_error_px is not None else "—"
        sync_ms = f"{summary.mean_sync_delta_ms:.1f} ms" if summary.mean_sync_delta_ms is not None else "—"
        rc = summary.risk_counts
        self.summary_label.setText(
            f"총 {summary.total_ticks}틱 ({summary.duration_s:.1f}초) · 보정: {summary.calibration_type} · "
            f"최소 3D 거리: {min_d} · 최대 접근 속도: {max_v} · 최소 예상 CPA: {min_cpa}\n"
            f"평균 재투영 오차: {reproj} · 평균 동기화 오차: {sync_ms} · "
            f"단계 분포 — 위험: {rc.get(RISK_DANGER, 0)} / 주의: {rc.get(RISK_CAUTION, 0)} / "
            f"안전: {rc.get(RISK_SAFE, 0)} / 판단 불가: {rc.get(RISK_UNKNOWN, 0)}"
        )

        # Open companion AVI videos if present
        self._close_videos()
        stem = path.stem
        for idx, suffix in enumerate(("_cam1.avi", "_cam2.avi")):
            vpath = path.parent / f"{stem}{suffix}"
            if vpath.is_file():
                cap = cv2.VideoCapture(str(vpath))
                if cap.isOpened():
                    self.video_captures[idx] = cap

        self.slider.blockSignals(True)
        self.slider.setRange(0, max(0, len(self.ticks) - 1))
        self.slider.setValue(0)
        self.slider.blockSignals(False)
        self.seek_to(0)

    def seek_to(self, index: int) -> None:
        if not self.ticks:
            return
        idx = max(0, min(len(self.ticks) - 1, int(index)))
        self.current_index = idx
        if self.slider.value() != idx:
            self.slider.blockSignals(True)
            self.slider.setValue(idx)
            self.slider.blockSignals(False)

        tick = self.ticks[idx]
        col = tick.collision
        row = tick.to_csv_row()
        dist_txt = f"{row['distance_m']} m (유효 {row['effective_distance_m']} m)" if row["distance_m"] else "—"
        close_txt = f"{row['closing_speed_mps']} m/s" if row["closing_speed_mps"] else "—"
        cpa_txt = f"{row['cpa_distance_m']} m (@ {row['cpa_time_s']}s)" if row["cpa_distance_m"] else "—"
        d1_txt = (
            f"({row['drone_1_x_m']}, {row['drone_1_y_m']}, {row['drone_1_z_m']}) m · {row['drone_1_speed_mps']} m/s"
            if row["drone_1_x_m"] else "—"
        )
        d2_txt = (
            f"({row['drone_2_x_m']}, {row['drone_2_y_m']}, {row['drone_2_z_m']}) m · {row['drone_2_speed_mps']} m/s"
            if row["drone_2_x_m"] else "—"
        )
        self.tick_detail_label.setText(
            f"[{tick.tick_index} / {len(self.ticks)}]  위험 단계: {col.get('risk_label', col.get('risk_level'))}\n"
            f"사유: {col.get('reason', '')}\n"
            f"드론 1 (3D): {d1_txt}\n"
            f"드론 2 (3D): {d2_txt}\n"
            f"3D 거리: {dist_txt}\n"
            f"상대 접근 속도: {close_txt}  ·  CPA: {cpa_txt}\n"
            f"동기화 오차: {row['sync_delta_ms']} ms"
        )

        # Reconstruct lightweight objects for DroneSpace3DWidget
        from .collision import DroneKinematics3D, PairCollisionMetrics
        kin_list = [DroneKinematics3D(**k) for k in (col.get("kinematics") or [])]
        pm_raw = col.get("pair_metrics")
        pm = PairCollisionMetrics(**pm_raw) if pm_raw else None
        fake_collision = CollisionAnalysisResult(
            risk_level=col.get("risk_level", RISK_UNKNOWN),
            risk_label=col.get("risk_label", ""),
            reason=col.get("reason", ""),
            visual_alert_active=bool(col.get("visual_alert_active", False)),
            audio_alert_triggered=False,
            stereo_status=col.get("stereo_status", ""),
            calibration_type=col.get("calibration_type", "none"),
            is_calibrated=bool(col.get("is_calibrated", False)),
            synchronized=bool(col.get("synchronized", False)),
            sync_delta_ms=float(col.get("sync_delta_ms", 0.0)),
            timestamp=tick.timestamp,
            wall_time=tick.wall_time,
            kinematics=tuple(kin_list),
            pair_metrics=pm,
        )
        calib = create_synthetic_calibration() if fake_collision.is_calibrated else None
        self.space_3d.update_scene(calib, None, fake_collision)

        # Update companion video frames if available
        for cam_i, cap in enumerate(self.video_captures):
            if cap is not None:
                cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
                ok, frame = cap.read()
                if ok and frame is not None:
                    h, w = frame.shape[:2]
                    qimg = QImage(frame.data, w, h, frame.strides[0], QImage.Format.Format_BGR888).copy()
                    self.video_labels[cam_i].setPixmap(
                        QPixmap.fromImage(qimg).scaled(
                            self.video_labels[cam_i].size(),
                            Qt.AspectRatioMode.KeepAspectRatio,
                            Qt.TransformationMode.SmoothTransformation,
                        )
                    )

    def toggle_play(self) -> None:
        if not self.ticks:
            return
        if self.play_timer.isActive():
            self.play_timer.stop()
            self.btn_play.setText("재생 / Play")
        else:
            if self.current_index >= len(self.ticks) - 1:
                self.seek_to(0)
            self.play_timer.start()
            self.btn_play.setText("일시정지 / Pause")

    def _advance_playback(self) -> None:
        if self.current_index + 1 < len(self.ticks):
            self.seek_to(self.current_index + 1)
        else:
            self.play_timer.stop()
            self.btn_play.setText("재생 / Play")

    def export_current_csv(self) -> None:
        if not self.ticks:
            QMessageBox.warning(self, "CSV 내보내기", "먼저 기록 파일을 불러오세요.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "CSV로 내보내기", str(Path.home() / "experiment_telemetry.csv"), "CSV Files (*.csv)"
        )
        if path:
            export_ticks_csv(Path(path), self.ticks)

    def _close_videos(self) -> None:
        for idx, cap in enumerate(self.video_captures):
            if cap is not None:
                cap.release()
                self.video_captures[idx] = None

    def closeEvent(self, event):
        self.play_timer.stop()
        self._close_videos()
        super().closeEvent(event)
