"""Qt presentation; acquisition, AI, 3D stereo triangulation, collision analysis, and recording remain usable without Qt."""

from dataclasses import replace
from pathlib import Path
import time

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QFileDialog, QFormLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
    QMainWindow, QMessageBox, QPushButton, QSizePolicy, QSpinBox, QTabWidget,
    QVBoxLayout, QWidget,
)

from .ai_pipeline import AiPipeline, InferenceParams
from .ai_settings import (
    DEVICES, IMAGE_SIZES, AiSettings, AiSettingsError, load_ai_settings,
    save_ai_settings, settings_path_for,
)
from .calibration import (
    DRONE_SIZE_PRESETS, CalibrationError, StereoCalibration,
    auto_calibrate_from_scene_frames, calibration_path_for,
    create_synthetic_calibration, load_calibration, save_calibration,
)
from .capabilities import (
    STANDARD_FPS, STANDARD_RESOLUTIONS, CapabilityError, StreamMode,
    best_fps_mode, best_resolution_mode, fetch_capabilities, fps_options,
    is_supported, probe_usb_modes,
)
from .capture import CameraSource, State
from .collision import (
    RISK_CAUTION, RISK_DANGER, RISK_LABELS, RISK_SAFE, RISK_UNKNOWN,
    CollisionAnalysisResult, CollisionAnalyzer, CollisionSettings,
    CollisionSettingsError, collision_settings_path_for,
    load_collision_settings, save_collision_settings,
)
from .config import CameraConfig, ConfigError, resolved_config, save_config
from .dashboard_widgets import (
    CalibrationWizardDialog, DroneSpace3DWidget, PlaybackDialog,
)
from .geometry import MAX_SAFETY_FACTOR, MIN_SAFETY_FACTOR
from .recorder import (
    ExperimentRecorder, export_events_csv, export_ticks_csv,
)
from .stereo_3d import StereoFrameResult, StereoTracker3D
from .usb import UsbError, UsbTunnel, companion_url

# An AI result older than this is hidden rather than shown as the current position.
OVERLAY_MAX_AGE_SECONDS = 2.0
TRACK_COLORS = ("#ff5252", "#40c4ff", "#ffd740", "#69f0ae", "#e040fb", "#ff9100")
DEVICE_LABELS = {"auto": "자동 / Auto", "cpu": "CPU", "cuda": "NVIDIA CUDA", "mps": "Apple GPU (MPS)"}

RISK_BANNER_STYLES = {
    RISK_UNKNOWN: "background: #1e293b; color: #f8fafc; border: 2px solid #64748b; border-radius: 8px; padding: 8px;",
    RISK_SAFE: "background: #064e3b; color: #ecfdf5; border: 2px solid #10b981; border-radius: 8px; padding: 8px;",
    RISK_CAUTION: "background: #78350f; color: #fffbeb; border: 2px solid #f59e0b; border-radius: 8px; padding: 8px;",
    RISK_DANGER: "background: #7f1d1d; color: #fef2f2; border: 2px solid #ef4444; border-radius: 8px; padding: 8px;",
}


class VideoView(QLabel):
    def __init__(self):
        super().__init__("연결 대기 / Waiting for connection")
        self._image = None
        self._detections = None
        self._options = AiSettings()
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(200, 140)
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.setStyleSheet("background: #05080d; color: #93a4bd; border-radius: 8px;")

    def set_frame(self, frame):
        height, width = frame.image.shape[:2]
        self._image = QImage(frame.image.data, width, height, frame.image.strides[0],
                             QImage.Format.Format_BGR888).copy()
        self._redraw()

    def set_detections(self, detections, options=None):
        """Overlay one analysed frame's drones. None removes the overlay."""
        if detections is self._detections and (options is None or options is self._options):
            return
        self._detections = detections
        if options is not None:
            self._options = options
        self._redraw()

    def clear_frame(self, text):
        self._image = None
        self._detections = None
        self.setPixmap(QPixmap())
        self.setText(text)

    def _redraw(self):
        if self._image is not None:
            pixmap = QPixmap.fromImage(self._image).scaled(
                self.size(), Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation)
            if self._detections is not None:
                self._paint_overlay(pixmap)
            self.setPixmap(pixmap)

    def _paint_overlay(self, pixmap):
        detections = self._detections
        # Never draw coordinates of another resolution onto this image.
        if detections.image_size != (self._image.width(), self._image.height()) or not (
                self._options.show_circle or self._options.show_box):
            return
        scale = pixmap.width() / self._image.width()
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        font = QFont(painter.font())
        font.setPixelSize(13)
        font.setBold(True)
        painter.setFont(font)
        for observation in detections.observations:
            color = QColor(TRACK_COLORS[observation.track_id % len(TRACK_COLORS)]
                           if observation.track_id is not None else "#b0bec5")
            painter.setPen(QPen(color, 2))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            cx, cy = observation.center[0] * scale, observation.center[1] * scale
            if self._options.show_box:
                x1, y1, x2, y2 = (value * scale for value in observation.bbox)
                painter.drawRect(round(x1), round(y1), round(x2 - x1), round(y2 - y1))
            parts = []
            if self._options.show_circle:
                radius = observation.safe_radius * scale
                painter.drawEllipse(round(cx - radius), round(cy - radius), round(2 * radius), round(2 * radius))
                painter.setBrush(color)
                painter.drawEllipse(round(cx - 3), round(cy - 3), 6, 6)
                parts.append(f"r {observation.safe_radius:.0f}px")
            if self._options.show_id:
                parts.insert(0, "ID —" if observation.track_id is None else f"ID {observation.track_id}")
            if self._options.show_confidence:
                parts.append(f"{observation.confidence * 100:.0f}%")
            if parts:
                text = "  ".join(parts)
                top = cy - (observation.safe_radius * scale if self._options.show_circle else 0) - 22
                metrics = painter.fontMetrics()
                width = metrics.horizontalAdvance(text) + 8
                left = min(max(0, cx - width / 2), max(0, pixmap.width() - width))
                top = min(max(0, top), max(0, pixmap.height() - 20))
                painter.setPen(Qt.PenStyle.NoPen)
                painter.setBrush(QColor(5, 8, 13, 190))
                painter.drawRect(round(left), round(top), width, 20)
                painter.setPen(color)
                painter.drawText(round(left) + 4, round(top) + 15, text)
        painter.end()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._redraw()


class SettingsDialog(QDialog):
    def __init__(self, configs, parent=None):
        super().__init__(parent)
        self.setWindowTitle("카메라 설정 / Camera settings")
        self.setMinimumWidth(760)
        self.configs = configs
        self.result_configs = None
        self.transports = []
        self.serials = [c.usb_serial for c in configs]
        self.fields = []
        # Enumerated live-streaming modes per camera; None means the
        # device has not been probed yet, so only standard choices show.
        self.device_modes = [None, None]
        self.supported_labels = []
        layout = QVBoxLayout(self)
        columns = QHBoxLayout()
        for index, config in enumerate(configs):
            group = QGroupBox(config.id)
            form = QFormLayout(group)
            transport = QComboBox()
            transport.addItems(["Wi-Fi / 일반 입력", "Android USB", "iPhone USB"])
            transport.setCurrentIndex(["", "android", "ios"].index(config.usb_platform))
            self.transports.append(transport)
            form.addRow("연결 방식", transport)
            name = QLineEdit(config.name)
            kind = QComboBox()
            kind.addItems(["mjpeg", "rtsp", "usb", "file"])
            kind.setCurrentText(config.input_type)
            source = QLineEdit(str(config.source))
            source.setPlaceholderText("URL / device number / video path")
            show = QCheckBox("주소 표시 / Show source")
            def visibility(*_, source=source, kind=kind, show=show):
                source.setEchoMode(QLineEdit.EchoMode.Password
                    if kind.currentText() in ("mjpeg", "rtsp") and not show.isChecked()
                    else QLineEdit.EchoMode.Normal)
            kind.currentTextChanged.connect(visibility)
            show.toggled.connect(visibility)
            visibility()
            timeout = QDoubleSpinBox()
            timeout.setRange(0.2, 60)
            timeout.setValue(config.timeout_seconds)
            timeout.setSuffix(" s")
            retry = QCheckBox()
            retry.setChecked(config.auto_reconnect)
            delay = QDoubleSpinBox()
            delay.setRange(0.1, 60)
            delay.setValue(config.reconnect_delay_seconds)
            delay.setSuffix(" s")
            attempts = QSpinBox()
            attempts.setRange(0, 100)
            attempts.setValue(config.max_reconnect_attempts)
            loop = QCheckBox()
            loop.setChecked(config.loop_file)
            resolution = QComboBox()
            fps_select = QComboBox()
            facing = QComboBox()
            facing.addItems(["자동 / Auto", "후면 / Back", "전면 / Front"])
            facing.setCurrentIndex(["", "back", "front"].index(config.facing))
            supported = QLabel("지원 범위 미확인 — 확인하면 기기가 실제 지원하는 조합만 선택할 수 있습니다.")
            supported.setWordWrap(True)
            self.supported_labels.append(supported)
            self.fields.append((name, kind, source, timeout, retry, delay, attempts, loop,
                                resolution, fps_select, facing))
            self._fill_resolution(index)
            resolution.currentIndexChanged.connect(
                lambda _value, i=index: self._fill_fps(i))
            self._fill_fps(index)
            probe = QPushButton("지원 범위 확인 / Probe device")
            probe.clicked.connect(lambda checked=False, i=index: self.probe_capabilities(i))
            best_resolution = QPushButton("최고 해상도 / Max resolution")
            best_resolution.clicked.connect(lambda checked=False, i=index: self.apply_best(i, False))
            best_fps = QPushButton("최고 FPS / Max FPS")
            best_fps.clicked.connect(lambda checked=False, i=index: self.apply_best(i, True))
            for label, widget in (("이름 / Name", name), ("입력 종류 / Type", kind),
                ("입력 / Source", source), ("", show), ("타임아웃 / Timeout", timeout),
                ("자동 재연결 / Auto retry", retry), ("재시도 간격 / Delay", delay),
                ("최대 재시도 / Attempts", attempts), ("파일 반복 / Loop file", loop),
                ("해상도 / Resolution", resolution), ("FPS", fps_select),
                ("카메라 방향 / Facing", facing)):
                form.addRow(label, widget)
            form.addRow(supported)
            form.addRow(probe)
            form.addRow(best_resolution)
            form.addRow(best_fps)
            usb = QPushButton("유선 USB 연결 도우미")
            usb.clicked.connect(lambda checked=False, i=index: self.configure_usb(i))
            form.addRow(usb)
            columns.addWidget(group)
        layout.addLayout(columns)
        notice = QLabel("Drone Camera 앱의 연결 주소를 입력하고 mjpeg를 선택하세요. 유선은 USB 연결 도우미를 사용하세요.\n"
                        "설정은 이 컴퓨터에 저장됩니다. URL에 포함된 비밀번호도 저장되므로 공유하지 마세요.")
        notice.setWordWrap(True)
        layout.addWidget(notice)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self._validate)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _fill_resolution(self, index):
        config = self.configs[index]
        combo = self.fields[index][8]
        current = combo.currentData()
        combo.blockSignals(True)
        combo.clear()
        combo.addItem("자동 (기기 기본값) / Auto", (0, 0))
        modes = self.device_modes[index]
        choices = ([(m.width, m.height) for m in modes] if modes
                   else list(STANDARD_RESOLUTIONS))
        wanted = tuple(current) if current else (config.requested_width, config.requested_height)
        if wanted != (0, 0) and wanted not in choices:
            choices.append(wanted)
        for width, height in choices:
            combo.addItem(f"{width} × {height}", (width, height))
        combo.setCurrentIndex(choices.index(wanted) + 1 if wanted in choices else 0)
        combo.blockSignals(False)

    def _fill_fps(self, index):
        config = self.configs[index]
        resolution = self.fields[index][8]
        combo = self.fields[index][9]
        current_fps = combo.currentData()
        combo.blockSignals(True)
        combo.clear()
        combo.addItem("자동 / Auto", 0)
        modes = self.device_modes[index]
        selected = resolution.currentData()
        if modes and selected and selected != (0, 0):
            mode = next((m for m in modes
                         if (m.width, m.height) == tuple(selected)), None)
            choices = list(fps_options(mode)) if mode else list(STANDARD_FPS)
        else:
            choices = list(STANDARD_FPS)
        wanted_fps = current_fps if current_fps else config.requested_fps
        if wanted_fps and wanted_fps not in choices:
            choices.append(wanted_fps)
            choices.sort()
        for fps in choices:
            combo.addItem(f"{fps} FPS", fps)
        combo.setCurrentIndex(choices.index(wanted_fps) + 1
                              if wanted_fps in choices else 0)
        combo.blockSignals(False)

    def probe_capabilities(self, index):
        """Enumerate what the selected device can really stream."""
        fields = self.fields[index]
        kind = fields[1].currentText()
        source = fields[2].text().strip()
        try:
            if kind == "mjpeg":
                modes = fetch_capabilities(source)
            elif kind == "usb":
                modes = probe_usb_modes(int(source))
            elif kind == "file":
                import cv2
                capture = cv2.VideoCapture(source)
                try:
                    if not capture.isOpened():
                        raise CapabilityError("동영상 파일을 열 수 없습니다.")
                    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
                    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    fps = int(capture.get(cv2.CAP_PROP_FPS) or 30)
                finally:
                    capture.release()
                modes = (StreamMode(width, height, max(1, fps)),)
            else:
                raise CapabilityError("RTSP는 기기별 지원 범위 조회 규격이 없어 실제 수신값으로 확인합니다.")
        except (CapabilityError, ValueError) as error:
            self.supported_labels[index].setText(str(error))
            QMessageBox.warning(self, "지원 범위 확인", str(error))
            return
        self.device_modes[index] = modes
        best = best_resolution_mode(modes)
        fastest = best_fps_mode(modes)
        self.supported_labels[index].setText(
            f"지원 {len(modes)}개 해상도 · 최고 {best.label} @ {best.max_fps} FPS · "
            f"최고 속도 {fastest.label} @ {fastest.max_fps} FPS\n"
            "최종 확인은 연결 후 실제 수신 해상도·FPS 표시로 합니다.")
        self._fill_resolution(index)
        self._fill_fps(index)

    def apply_best(self, index, fps_priority):
        if self.device_modes[index] is None:
            self.probe_capabilities(index)
        modes = self.device_modes[index]
        if not modes:
            return
        mode = best_fps_mode(modes) if fps_priority else best_resolution_mode(modes)
        resolution = self.fields[index][8]
        for row in range(resolution.count()):
            if tuple(resolution.itemData(row)) == (mode.width, mode.height):
                resolution.setCurrentIndex(row)
                break
        self._fill_fps(index)
        fps_combo = self.fields[index][9]
        for row in range(fps_combo.count()):
            if fps_combo.itemData(row) == mode.max_fps:
                fps_combo.setCurrentIndex(row)
                break

    def _validate(self):
        result = []
        try:
            for index, (config, fields) in enumerate(zip(self.configs, self.fields)):
                name, kind, source, timeout, retry, delay, attempts, loop, \
                    resolution, fps_select, facing = fields
                value = source.text().strip()
                if kind.currentText() == "usb":
                    try:
                        value = int(value)
                    except ValueError:
                        raise ConfigError("USB 입력에는 장치 번호를 입력하세요 (0–99).") from None
                width, height = tuple(resolution.currentData() or (0, 0))
                fps = int(fps_select.currentData() or 0)
                modes = self.device_modes[index]
                if modes is not None and not is_supported(modes, width, height, fps):
                    raise ConfigError(
                        f"{config.id}: 기기가 지원하지 않는 해상도/FPS 조합입니다. "
                        "지원 범위 확인 후 목록에서 선택하세요.")
                result.append(CameraConfig(config.id, name.text().strip(), kind.currentText(), value,
                    timeout.value(), retry.isChecked(), delay.value(), attempts.value(), loop.isChecked(),
                    ["", "android", "ios"][self.transports[index].currentIndex()], self.serials[index],
                    width, height, fps, ["", "back", "front"][facing.currentIndex()]))
        except ConfigError as error:
            QMessageBox.warning(self, "설정 확인", str(error))
            return
        self.result_configs = result
        self.accept()

    def configure_usb(self, index):
        dialog = QDialog(self)
        dialog.setWindowTitle("USB 연결 — Wi-Fi 없이 사용 가능")
        form = QFormLayout(dialog)
        platform = QComboBox(); platform.addItems(["Android / Galaxy", "iPhone / iPad"])
        address = QLineEdit(); address.setPlaceholderText("휴대폰 앱의 전체 URL (token 포함)")
        address.setEchoMode(QLineEdit.EchoMode.Password)
        serial = QLineEdit(); serial.setPlaceholderText("한 대면 비워두기; 여러 대면 adb serial / iPhone UDID")
        notice = QLabel("Android: Platform-Tools adb + USB 디버깅 승인\niPhone: 잠금 해제 + 컴퓨터 신뢰 (Windows: Apple Devices/iTunes 드라이버)\n"
                        "앱에서 송출을 시작하고 주소를 붙여 넣으세요. 연결 확인은 영상 수신 시 수행합니다.")
        notice.setWordWrap(True)
        form.addRow(notice); form.addRow("휴대폰", platform); form.addRow("앱 연결 주소", address); form.addRow("장치 번호 (선택)", serial)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        form.addRow(buttons)
        buttons.rejected.connect(dialog.reject)
        def apply():
            try:
                companion_url(address.text())
            except UsbError as error:
                QMessageBox.warning(dialog, "USB 설정", str(error))
                return
            self.transports[index].setCurrentIndex(platform.currentIndex() + 1)
            self.serials[index] = serial.text().strip()
            self.fields[index][1].setCurrentText("mjpeg")
            self.fields[index][2].setText(address.text().strip())
            dialog.accept()
        buttons.accepted.connect(apply)
        dialog.exec()


STATE_LABELS = {
    State.DISCONNECTED: "연결 대기 / Disconnected",
    State.CONNECTING: "연결 중 / Connecting",
    State.CONNECTED: "수신 중 / Connected",
    State.RECONNECTING: "재연결 중 / Reconnecting",
    State.ERROR: "연결 오류 / Error",
    State.ENDED: "파일 재생 완료 / End of file",
    State.STOPPING: "연결 해제 중 / Stopping",
}


class MainWindow(QMainWindow):
    def __init__(
        self,
        configs: list[CameraConfig],
        config_path: Path,
        ai_settings_path: Path | None = None,
        pipeline: AiPipeline | None = None,
        calibration_path: Path | None = None,
        collision_settings_path: Path | None = None,
    ):
        super().__init__()
        self.configs = configs
        self.config_path = config_path
        self.retired_tunnels = []
        self.setup_sources()
        self.sequences = [0, 0]
        self._latest_frames = [None, None]
        # AI is created lazily: with AI unused, no thread starts and torch is never imported.
        self.pipeline = pipeline
        self.ai_settings_path = ai_settings_path or settings_path_for(config_path)
        self.ai_notice = ""
        try:
            self.ai_settings = load_ai_settings(self.ai_settings_path)
        except AiSettingsError as error:
            self.ai_settings = AiSettings()
            self.ai_notice = f"AI 설정을 읽지 못해 기본값을 사용합니다: {error}"

        # Phase 03: Stereo Calibration & 3D Tracker
        self.calibration_path = calibration_path or calibration_path_for(config_path)
        self.calibration_notice = ""
        try:
            self.calibration: StereoCalibration | None = load_calibration(self.calibration_path)
        except CalibrationError as error:
            self.calibration = None
            self.calibration_notice = f"보정 파일을 읽지 못해 미보정 상태로 시작합니다: {error}"
        self.stereo_tracker = StereoTracker3D()
        self.latest_stereo_result: StereoFrameResult | None = None

        # Phase 04: Collision Risk Analysis
        self.collision_settings_path = collision_settings_path or collision_settings_path_for(config_path)
        try:
            self.collision_settings = load_collision_settings(self.collision_settings_path)
        except CollisionSettingsError:
            self.collision_settings = CollisionSettings()
        self.collision_analyzer = CollisionAnalyzer(settings=self.collision_settings)
        self.latest_collision_result: CollisionAnalysisResult | None = None
        self.audio_beep_count = 0

        # Phase 05: Experiment Recorder
        self.recorder = ExperimentRecorder()

        self._connected_before = [False, False]
        self._shown_detections = [None, None]
        self._last_model_text = None
        self._ai_save_timer = QTimer(self)
        self._ai_save_timer.setSingleShot(True)
        self._ai_save_timer.setInterval(500)
        self._ai_save_timer.timeout.connect(self._save_ai_settings)

        self.setWindowTitle("Drone Collision Detection System — Phase 01~05 Integrated")
        self.resize(1380, 900)
        self.setMinimumSize(800, 620)
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(8)

        header = QHBoxLayout()
        title = QLabel(
            "Drone Collision Detection System\n"
            "PHASE 01~05  ·  DUAL CAMERA · YOLO11 + BYTETRACK · 3D STEREO · COLLISION WARNING & DASHBOARD"
        )
        title.setStyleSheet("font-size: 16px; font-weight: 600;")
        self.summary = QLabel()
        self.summary.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        btn_calib_wizard = QPushButton("보정 마법사 / Calibration")
        btn_calib_wizard.clicked.connect(self.open_calibration_wizard)
        btn_playback = QPushButton("재생·분석 / Playback")
        btn_playback.clicked.connect(self.open_playback_dialog)
        settings = QPushButton("카메라 설정 / Settings")
        settings.clicked.connect(self.edit_settings)
        header.addWidget(title, 1)
        header.addWidget(self.summary)
        header.addWidget(btn_calib_wizard)
        header.addWidget(btn_playback)
        header.addWidget(settings)
        layout.addLayout(header)
        layout.addWidget(self._build_ai_panel())

        row = QHBoxLayout()
        self.views, self.details, self.buttons, self.titles = [], [], [], []
        for i in range(2):
            card = QGroupBox()
            column = QVBoxLayout(card)
            column.setContentsMargins(8, 8, 8, 8)
            heading = QLabel()
            heading.setStyleSheet("font-size: 15px; font-weight: 600;")
            view = VideoView()
            details = QLabel()
            details.setWordWrap(True)
            details.setMinimumHeight(105)
            details.setTextFormat(Qt.TextFormat.PlainText)
            button = QPushButton()
            button.clicked.connect(lambda checked=False, index=i: self.toggle(index))
            heading.setTextFormat(Qt.TextFormat.PlainText)
            for widget in (heading, view, details, button):
                column.addWidget(widget, 1 if widget is view else 0)
            row.addWidget(card, 2)
            self.views.append(view)
            self.details.append(details)
            self.buttons.append(button)
            self.titles.append(heading)

        # Right Zone: Phase 03-05 Integrated Control & Telemetry Dashboard
        row.addWidget(self._build_dashboard_zone(), 2)
        layout.addLayout(row, 1)

        bottom = QHBoxLayout()
        for text, action in (
            ("전체 연결 / Connect all", self.connect_all),
            ("전체 해제 / Disconnect all", self.disconnect_all),
            ("보정 마법사 / Calibration", self.open_calibration_wizard),
            ("기록 시작·중지 / Record", self.toggle_recording),
            ("CSV 내보내기 / Export CSV", self.export_recorded_csv),
            ("설정 / Settings", self.edit_settings),
            ("종료 / Exit", self.close),
        ):
            button = QPushButton(text)
            button.clicked.connect(action)
            bottom.addWidget(button)
        layout.addLayout(bottom)

        footer = QLabel(
            "실험용 보조 경고 시스템 (실제 비행 안전 미보장 · 자동 비행 제어 없음)  •  "
            "2D 바운딩 서클은 화면 표시 전용  •  "
            "미보정·미동기화·대응 불확실 시 '안전'이 아닌 '판단 불가' 처리"
        )
        footer.setStyleSheet("color: #93a4bd;")
        layout.addWidget(footer)
        self.setStyleSheet("""
            QMainWindow, QDialog { background: #111827; color: #e5edf8; }
            QWidget { color: #e5edf8; font-size: 12px; }
            QGroupBox { background: #192438; border: 1px solid #33435d; border-radius: 10px; padding: 8px; }
            QPushButton { background: #245a87; border: 1px solid #3975a6; border-radius: 6px; padding: 7px; }
            QPushButton:hover { background: #3072a4; }
            QPushButton:disabled { color: #8290a5; background: #28354a; }
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox { background: #0e1726; padding: 4px; border: 1px solid #52627c; }
            QComboBox QAbstractItemView { background: #192438; color: #e5edf8; }
            QTabWidget::pane { border: 1px solid #33435d; background: #192438; border-radius: 8px; }
            QTabBar::tab { background: #0e1726; color: #93a4bd; padding: 6px 10px; border-top-left-radius: 6px; border-top-right-radius: 6px; }
            QTabBar::tab:selected { background: #245a87; color: #ffffff; font-weight: 600; }
            QMessageBox { background: #192438; }
        """)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(33)
        self.refresh()
        if self.ai_settings.enabled and self.ai_settings.model_path:
            self._start_ai()

    # ------------------------------------------------- Phase 03-05 Dashboard
    def _build_dashboard_zone(self) -> QWidget:
        container = QGroupBox("3D 관제 · 충돌 위험 분석 · 기록 (Phase 03~05)")
        container.setMinimumWidth(280)
        col = QVBoxLayout(container)
        col.setContentsMargins(8, 8, 8, 8)
        col.setSpacing(6)

        # Always-visible Risk Banner
        self.risk_banner = QLabel("판단 불가 / UNKNOWN\n보정 및 AI 탐지 대기 중")
        self.risk_banner.setWordWrap(True)
        self.risk_banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.risk_banner.setStyleSheet(RISK_BANNER_STYLES[RISK_UNKNOWN])
        col.addWidget(self.risk_banner)

        # Key status summary bar (Calibration, Sync, Distance, Closing speed/CPA)
        self.status_overview_label = QLabel()
        self.status_overview_label.setWordWrap(True)
        self.status_overview_label.setTextFormat(Qt.TextFormat.PlainText)
        self.status_overview_label.setStyleSheet(
            "background: #0f172a; padding: 6px; border-radius: 6px; font-size: 11px;"
        )
        col.addWidget(self.status_overview_label)

        self.dashboard_tabs = QTabWidget()
        self.dashboard_tabs.addTab(self._build_3d_telemetry_tab(), "3D 관제·좌표")
        self.dashboard_tabs.addTab(self._build_calib_threshold_tab(), "보정·경고 기준")
        self.dashboard_tabs.addTab(self._build_recording_tab(), "데이터 기록·재생")
        col.addWidget(self.dashboard_tabs, 1)
        return container

    def _build_3d_telemetry_tab(self) -> QWidget:
        tab = QWidget()
        col = QVBoxLayout(tab)
        col.setContentsMargins(6, 6, 6, 6)

        view_bar = QHBoxLayout()
        view_bar.addWidget(QLabel("3D 시점:"))
        for label, mode in (("3D 사선", "3d"), ("상면도(X-Z)", "top"), ("정면도(X-Y)", "front")):
            b = QPushButton(label)
            b.clicked.connect(lambda _checked=False, m=mode: self.space_3d_widget.set_view_mode(m))
            view_bar.addWidget(b)
        col.addLayout(view_bar)

        self.space_3d_widget = DroneSpace3DWidget()
        col.addWidget(self.space_3d_widget, 1)

        self.drone_telemetry_label = QLabel("확정된 3D 드론 좌표 없음")
        self.drone_telemetry_label.setWordWrap(True)
        self.drone_telemetry_label.setTextFormat(Qt.TextFormat.PlainText)
        self.drone_telemetry_label.setStyleSheet(
            "background: #0f172a; padding: 6px; border-radius: 6px; font-family: monospace; font-size: 11px;"
        )
        col.addWidget(self.drone_telemetry_label)
        return tab

    def _build_calib_threshold_tab(self) -> QWidget:
        tab = QWidget()
        col = QVBoxLayout(tab)
        col.setContentsMargins(6, 6, 6, 6)

        self.calib_detail_label = QLabel()
        self.calib_detail_label.setWordWrap(True)
        self.calib_detail_label.setStyleSheet("background: #0f172a; padding: 6px; border-radius: 6px;")
        col.addWidget(self.calib_detail_label)

        calib_btns_1 = QHBoxLayout()
        btn_wiz = QPushButton("보정 마법사…")
        btn_wiz.clicked.connect(self.open_calibration_wizard)
        btn_load_cal = QPushButton("보정 불러오기…")
        btn_load_cal.clicked.connect(self.choose_calibration_file)
        btn_save_cal = QPushButton("보정 저장…")
        btn_save_cal.clicked.connect(self.save_calibration_as)
        for b in (btn_wiz, btn_load_cal, btn_save_cal):
            calib_btns_1.addWidget(b)
        col.addLayout(calib_btns_1)

        calib_btns_2 = QHBoxLayout()
        btn_quick_layout = QPushButton("상·하/좌·우 간편 보정 (체커보드 없음)")
        btn_quick_layout.clicked.connect(self.apply_quick_layout_calibration)
        btn_auto_scene = QPushButton("배경 특징점 자동 보정")
        btn_auto_scene.clicked.connect(self.apply_auto_scene_calibration)
        btn_auto_traj = QPushButton("드론 궤적 자동 보정")
        btn_auto_traj.clicked.connect(self.apply_auto_trajectory_calibration)
        for b in (btn_quick_layout, btn_auto_scene, btn_auto_traj):
            calib_btns_2.addWidget(b)
        col.addLayout(calib_btns_2)

        calib_btns_3 = QHBoxLayout()
        btn_synth = QPushButton("합성 보정 적용 (실험 전 검증용)")
        btn_synth.clicked.connect(self.apply_quick_synthetic_calibration)
        btn_clear_cal = QPushButton("보정 해제 (미보정 전환)")
        btn_clear_cal.clicked.connect(self.clear_calibration)
        calib_btns_3.addWidget(btn_synth)
        calib_btns_3.addWidget(btn_clear_cal)
        col.addLayout(calib_btns_3)

        # Drone Preset (Mavic 3 + drone model B), Camera Layout (Vertical/Horizontal), & Monocular Fallback
        preset_row = QHBoxLayout()
        self.camera_layout_combo = QComboBox()
        self.camera_layout_combo.addItem("상·하 배치 (위 Cam1·아래 Cam2)", "vertical")
        self.camera_layout_combo.addItem("좌·우 배치 (좌 Cam1·우 Cam2)", "horizontal")
        self.drone_preset_combo = QComboBox()
        for preset_key, (preset_label, _p_m, _s_m) in DRONE_SIZE_PRESETS.items():
            self.drone_preset_combo.addItem(preset_label, preset_key)
        self.drone_preset_combo.currentIndexChanged.connect(self._drone_preset_changed)
        preset_row.addWidget(QLabel("배치:"))
        preset_row.addWidget(self.camera_layout_combo)
        preset_row.addWidget(QLabel("기체:"))
        preset_row.addWidget(self.drone_preset_combo, 1)
        col.addLayout(preset_row)

        self.monocular_fallback_check = QCheckBox(
            "카메라 단안 거리 예측 보조 (한쪽 카메라만 잡혀도 드론 A·드론 B 크기로 3D 추정)"
        )
        self.monocular_fallback_check.setChecked(self.stereo_tracker.monocular_fallback)
        self.monocular_fallback_check.toggled.connect(self._monocular_fallback_toggled)
        col.addWidget(self.monocular_fallback_check)

        # Collision threshold controls
        thresh_group = QGroupBox("충돌 위험 임계값 및 경고 설정")
        form = QFormLayout(thresh_group)
        cs = self.collision_settings

        self.warn_dist_spin = QDoubleSpinBox()
        self.warn_dist_spin.setRange(0.5, 200.0)
        self.warn_dist_spin.setDecimals(2)
        self.warn_dist_spin.setSingleStep(0.5)
        self.warn_dist_spin.setValue(cs.warning_distance_m)
        self.warn_dist_spin.setSuffix(" m")

        self.danger_dist_spin = QDoubleSpinBox()
        self.danger_dist_spin.setRange(0.2, 100.0)
        self.danger_dist_spin.setDecimals(2)
        self.danger_dist_spin.setSingleStep(0.25)
        self.danger_dist_spin.setValue(cs.danger_distance_m)
        self.danger_dist_spin.setSuffix(" m")

        self.horizon_spin = QDoubleSpinBox()
        self.horizon_spin.setRange(0.5, 30.0)
        self.horizon_spin.setDecimals(1)
        self.horizon_spin.setValue(cs.prediction_horizon_s)
        self.horizon_spin.setSuffix(" s")

        self.closing_speed_spin = QDoubleSpinBox()
        self.closing_speed_spin.setRange(0.05, 30.0)
        self.closing_speed_spin.setDecimals(2)
        self.closing_speed_spin.setValue(cs.min_closing_speed_mps)
        self.closing_speed_spin.setSuffix(" m/s")

        self.unc_sigma_spin = QDoubleSpinBox()
        self.unc_sigma_spin.setRange(0.0, 5.0)
        self.unc_sigma_spin.setDecimals(2)
        self.unc_sigma_spin.setSingleStep(0.25)
        self.unc_sigma_spin.setValue(cs.uncertainty_margin_sigma)

        self.max_unc_spin = QDoubleSpinBox()
        self.max_unc_spin.setRange(0.1, 20.0)
        self.max_unc_spin.setDecimals(2)
        self.max_unc_spin.setValue(cs.max_position_uncertainty_m)
        self.max_unc_spin.setSuffix(" m")

        self.visual_alert_check = QCheckBox("시각적 경고 활성화")
        self.visual_alert_check.setChecked(cs.visual_alert_enabled)
        self.audio_alert_check = QCheckBox("청각적 경고음(Beep) 활성화")
        self.audio_alert_check.setChecked(cs.audio_alert_enabled)

        form.addRow("주의 거리 (Caution)", self.warn_dist_spin)
        form.addRow("위험 거리 (Danger)", self.danger_dist_spin)
        form.addRow("최근접 예측 범위 (Horizon)", self.horizon_spin)
        form.addRow("최소 접근 속도 기준", self.closing_speed_spin)
        form.addRow("불확실성 차감 계수 (σ)", self.unc_sigma_spin)
        form.addRow("최대 허용 위치 오차", self.max_unc_spin)
        form.addRow(self.visual_alert_check, self.audio_alert_check)
        col.addWidget(thresh_group)

        for spin in (
            self.warn_dist_spin, self.danger_dist_spin, self.horizon_spin,
            self.closing_speed_spin, self.unc_sigma_spin, self.max_unc_spin,
        ):
            spin.valueChanged.connect(self._collision_widgets_changed)
        self.visual_alert_check.toggled.connect(self._collision_widgets_changed)
        self.audio_alert_check.toggled.connect(self._collision_widgets_changed)
        return tab

    def _build_recording_tab(self) -> QWidget:
        tab = QWidget()
        col = QVBoxLayout(tab)
        col.setContentsMargins(6, 6, 6, 6)

        self.record_video_check = QCheckBox("카메라 1·2 영상(AVI)도 함께 녹화")
        self.record_video_check.setChecked(False)
        col.addWidget(self.record_video_check)

        rec_btns = QHBoxLayout()
        self.btn_record_toggle = QPushButton("실험 데이터 기록 시작 / Start Recording")
        self.btn_record_toggle.clicked.connect(self.toggle_recording)
        btn_csv = QPushButton("기록 CSV 내보내기…")
        btn_csv.clicked.connect(self.export_recorded_csv)
        rec_btns.addWidget(self.btn_record_toggle)
        rec_btns.addWidget(btn_csv)
        col.addLayout(rec_btns)

        rec_btns_2 = QHBoxLayout()
        btn_ev_csv = QPushButton("위험 이벤트 CSV 내보내기…")
        btn_ev_csv.clicked.connect(self.export_risk_events_csv)
        btn_play = QPushButton("녹화·기록 재생 분석기 열기…")
        btn_play.clicked.connect(self.open_playback_dialog)
        rec_btns_2.addWidget(btn_ev_csv)
        rec_btns_2.addWidget(btn_play)
        col.addLayout(rec_btns_2)

        self.record_status_label = QLabel("기록 상태: 대기 중 (0 프레임)")
        self.record_status_label.setWordWrap(True)
        self.record_status_label.setStyleSheet("background: #0f172a; padding: 6px; border-radius: 6px;")
        col.addWidget(self.record_status_label)

        col.addWidget(QLabel("최근 위험/상태 전환 이벤트 로그:"))
        self.risk_events_label = QLabel("기록된 위험 이벤트 없음")
        self.risk_events_label.setWordWrap(True)
        self.risk_events_label.setTextFormat(Qt.TextFormat.PlainText)
        self.risk_events_label.setStyleSheet(
            "background: #0f172a; padding: 6px; border-radius: 6px; font-family: monospace; font-size: 11px;"
        )
        col.addWidget(self.risk_events_label, 1)
        return tab

    # ------------------------------------------------------------------ AI
    def _build_ai_panel(self):
        settings = self.ai_settings
        group = QGroupBox("AI 탐지 / YOLO11 + ByteTrack")
        column = QVBoxLayout(group)
        column.setContentsMargins(8, 6, 8, 6)
        top = QHBoxLayout()
        load = QPushButton("모델 불러오기 / Load model…")
        load.clicked.connect(self.choose_model)
        reload_button = QPushButton("다시 불러오기 / Reload")
        reload_button.clicked.connect(self.reload_model)
        self.ai_enabled = QCheckBox("AI 탐지 ON / Detection")
        self.ai_enabled.setChecked(bool(settings.enabled and settings.model_path))
        self.model_label = QLabel()
        self.model_label.setWordWrap(True)
        self.model_label.setTextFormat(Qt.TextFormat.PlainText)
        for widget in (load, reload_button, self.ai_enabled):
            top.addWidget(widget)
        top.addWidget(self.model_label, 1)
        column.addLayout(top)
        options = QHBoxLayout()
        self.ai_confidence = QDoubleSpinBox()
        self.ai_confidence.setRange(0.05, 0.95)
        self.ai_confidence.setSingleStep(0.05)
        self.ai_confidence.setDecimals(2)
        self.ai_confidence.setValue(settings.confidence)
        self.ai_iou = QDoubleSpinBox()
        self.ai_iou.setRange(0.1, 0.95)
        self.ai_iou.setSingleStep(0.05)
        self.ai_iou.setDecimals(2)
        self.ai_iou.setValue(settings.iou)
        self.ai_imgsz = QComboBox()
        for size in IMAGE_SIZES:
            self.ai_imgsz.addItem(f"{size} px", size)
        self.ai_imgsz.setCurrentIndex(IMAGE_SIZES.index(settings.imgsz))
        self.ai_device = QComboBox()
        for device in DEVICES:
            self.ai_device.addItem(DEVICE_LABELS[device], device)
        self.ai_device.setCurrentIndex(DEVICES.index(settings.device))
        self.ai_safety = QDoubleSpinBox()
        self.ai_safety.setRange(MIN_SAFETY_FACTOR, MAX_SAFETY_FACTOR)
        self.ai_safety.setSingleStep(0.05)
        self.ai_safety.setDecimals(2)
        self.ai_safety.setValue(settings.safety_factor)
        self.ai_safety.setToolTip("바운딩 서클 반지름 = 박스 대각선의 절반 × 계수. 화면 표시 전용이며 실제 3D 안전거리가 아닙니다.")
        self.ai_show = {}
        for label, widget in (("신뢰도 / Confidence", self.ai_confidence), ("NMS IoU", self.ai_iou),
                              ("추론 해상도 / Size", self.ai_imgsz), ("장치 / Device", self.ai_device),
                              ("서클 여유 계수 / Safety", self.ai_safety)):
            options.addWidget(QLabel(label))
            options.addWidget(widget)
        for key, label in (("show_box", "박스"), ("show_circle", "서클"),
                           ("show_id", "ID"), ("show_confidence", "신뢰도 표시")):
            box = QCheckBox(label)
            box.setChecked(getattr(settings, key))
            self.ai_show[key] = box
            options.addWidget(box)
        options.addStretch(1)
        column.addLayout(options)
        self.ai_enabled.toggled.connect(self._ai_enabled_toggled)
        for widget in (self.ai_confidence, self.ai_iou, self.ai_safety):
            widget.valueChanged.connect(self._ai_options_changed)
        self.ai_imgsz.currentIndexChanged.connect(self._ai_options_changed)
        self.ai_device.currentIndexChanged.connect(self._ai_device_changed)
        for box in self.ai_show.values():
            box.toggled.connect(self._ai_options_changed)
        return group

    def _ensure_pipeline(self) -> AiPipeline:
        if self.pipeline is None:
            self.pipeline = AiPipeline(config_dir=self.config_path.parent / "ultralytics")
        return self.pipeline

    def _inference_params(self) -> InferenceParams:
        s = self.ai_settings
        return InferenceParams(s.confidence, s.iou, s.imgsz, s.safety_factor)

    def _read_ai_widgets(self):
        """Build validated settings from the widgets; keep the old ones if invalid."""
        try:
            self.ai_settings = AiSettings(
                self.ai_settings.model_path, self.ai_enabled.isChecked(), self.ai_confidence.value(),
                self.ai_iou.value(), self.ai_imgsz.currentData(), self.ai_device.currentData(),
                self.ai_safety.value(), **{key: box.isChecked() for key, box in self.ai_show.items()})
        except AiSettingsError as error:
            self.ai_notice = str(error)
        if self.pipeline is not None:
            self.pipeline.configure(self._inference_params())
        self._ai_save_timer.start()

    def _ai_options_changed(self, *_):
        self._read_ai_widgets()

    def _ai_device_changed(self, *_):
        self._read_ai_widgets()
        if self.ai_settings.model_path and self.pipeline is not None:
            self._load_model()

    def _ai_enabled_toggled(self, checked):
        if checked and not self.ai_settings.model_path:
            self.ai_enabled.blockSignals(True)
            self.ai_enabled.setChecked(False)
            self.ai_enabled.blockSignals(False)
            self.ai_notice = "AI 탐지를 켜려면 먼저 모델(.pt) 파일을 불러오세요."
            return
        self._read_ai_widgets()
        if checked:
            self._start_ai()
        elif self.pipeline is not None:
            self.pipeline.set_enabled(False)
            self.stereo_tracker.reset()
            self.collision_analyzer.reset()

    def _start_ai(self):
        pipeline = self._ensure_pipeline()
        pipeline.configure(self._inference_params())
        if pipeline.model_status().state in ("none", "error"):
            self._load_model()
        pipeline.set_enabled(True)

    def _load_model(self):
        pipeline = self._ensure_pipeline()
        pipeline.configure(self._inference_params())
        self.ai_notice = ""
        pipeline.load_model(self.ai_settings.model_path, self.ai_settings.device)
        self._last_model_text = None

    def choose_model(self):
        current = Path(self.ai_settings.model_path).expanduser().parent if self.ai_settings.model_path else Path.home()
        path, _ = QFileDialog.getOpenFileName(
            self, "YOLO 모델 선택 / Select YOLO model",
            str(current if current.is_dir() else Path.home()), "PyTorch 모델 (*.pt)")
        if not path:
            return
        self.ai_settings = replace(self.ai_settings, model_path=path)
        self._ai_save_timer.start()
        self._load_model()

    def reload_model(self):
        if not self.ai_settings.model_path:
            self.choose_model()
        else:
            self._load_model()

    def _save_ai_settings(self):
        try:
            save_ai_settings(self.ai_settings_path, self.ai_settings)
        except OSError:
            self.ai_notice = "AI 설정을 저장할 수 없습니다. 폴더 쓰기 권한을 확인하세요."

    def _model_text(self):
        status = self.pipeline.model_status() if self.pipeline is not None else None
        name = Path(self.ai_settings.model_path).name if self.ai_settings.model_path else ""
        if status is None or status.state == "none":
            if self.ai_notice:
                return self.ai_notice
            return (f"모델 대기: {name} (AI 탐지를 켜거나 다시 불러오세요)" if name
                    else "모델 미선택 — '모델 불러오기'로 학습된 .pt 파일을 선택하세요. (신뢰할 수 있는 파일만 사용)")
        if status.state == "loading":
            return f"모델 불러오는 중… {name}"
        if status.state == "error":
            return f"⚠ 모델 오류: {status.message}"
        info = status.info
        text = (f"모델: {name} · 장치 {info.device} · 클래스 {info.names} · "
                f"torch {info.torch_version} / ultralytics {info.ultralytics_version}")
        return text + (f"\n{status.message}" if status.message else "")

    # ---------------------------------------- Phase 03-05 Actions & Handlers
    def set_calibration(self, calibration: StereoCalibration | None, persist: bool = True) -> None:
        self.calibration = calibration
        self.calibration_notice = ""
        self.stereo_tracker.reset()
        self.collision_analyzer.reset()
        if persist:
            if calibration is not None:
                try:
                    save_calibration(self.calibration_path, calibration)
                except OSError:
                    self.calibration_notice = "보정 파일을 저장할 수 없습니다."
            elif self.calibration_path.exists():
                try:
                    self.calibration_path.unlink()
                except OSError:
                    pass
        self.refresh()

    def apply_quick_synthetic_calibration(self) -> None:
        calib = create_synthetic_calibration()
        self.set_calibration(calib, persist=True)

    def apply_quick_layout_calibration(self) -> None:
        cam_layout = self.camera_layout_combo.currentData() if hasattr(self, "camera_layout_combo") else "vertical"
        baseline = self.calibration.baseline_m if self.calibration is not None else 1.5
        calib = create_synthetic_calibration(
            baseline_m=baseline,
            layout=cam_layout or "vertical",
            calibration_type="auto",
        )
        self.set_calibration(calib, persist=True)

    def apply_auto_scene_calibration(self) -> None:
        f1, f2 = self._latest_frames[0], self._latest_frames[1]
        if f1 is None or f2 is None:
            QMessageBox.warning(
                self,
                "배경 특징점 자동 보정",
                "두 카메라가 모두 연결되어 같은 배경을 비추고 있어야 합니다.",
            )
            return
        try:
            baseline = self.calibration.baseline_m if self.calibration is not None else 1.5
            calib = auto_calibrate_from_scene_frames(f1.image, f2.image, baseline_m=baseline)
            self.set_calibration(calib, persist=True)
        except CalibrationError as exc:
            QMessageBox.warning(self, "배경 특징점 자동 보정 오류", str(exc))

    def apply_auto_trajectory_calibration(self) -> None:
        try:
            baseline = self.calibration.baseline_m if self.calibration is not None else 1.5
            calib = self.stereo_tracker.auto_calibrate_from_trajectories(fallback_baseline_m=baseline)
            self.set_calibration(calib, persist=True)
        except (CalibrationError, ValueError) as exc:
            QMessageBox.warning(self, "드론 궤적 자동 보정 안내", str(exc))

    def _drone_preset_changed(self, *_) -> None:
        preset = self.drone_preset_combo.currentData()
        if preset in DRONE_SIZE_PRESETS:
            self.stereo_tracker.drone_preset = preset
            self.refresh()

    def _monocular_fallback_toggled(self, checked: bool) -> None:
        self.stereo_tracker.monocular_fallback = bool(checked)
        self.refresh()

    def clear_calibration(self) -> None:
        self.set_calibration(None, persist=True)

    def open_calibration_wizard(self) -> None:
        dialog = CalibrationWizardDialog(
            self.calibration,
            frame_provider=lambda: (self._latest_frames[0], self._latest_frames[1]),
            stereo_tracker=self.stereo_tracker,
            parent=self,
        )
        if dialog.exec() == QDialog.DialogCode.Accepted and dialog.result_calibration is not None:
            if hasattr(self, "drone_preset_combo"):
                idx = self.drone_preset_combo.findData(self.stereo_tracker.drone_preset)
                if idx >= 0:
                    self.drone_preset_combo.blockSignals(True)
                    self.drone_preset_combo.setCurrentIndex(idx)
                    self.drone_preset_combo.blockSignals(False)
            self.set_calibration(dialog.result_calibration, persist=True)

    def choose_calibration_file(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "스테레오 보정 파일 열기", str(self.config_path.parent), "JSON (*.json)"
        )
        if not path:
            return
        try:
            loaded = load_calibration(Path(path))
            if loaded is None:
                raise CalibrationError("선택한 보정 파일이 존재하지 않습니다.")
            self.set_calibration(loaded, persist=True)
        except CalibrationError as exc:
            QMessageBox.warning(self, "보정 파일 오류", str(exc))

    def save_calibration_as(self) -> None:
        if self.calibration is None:
            QMessageBox.warning(self, "보정 저장", "현재 적용된 보정 데이터가 없습니다.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "스테레오 보정 파일 저장", str(self.calibration_path), "JSON (*.json)"
        )
        if path:
            try:
                save_calibration(Path(path), self.calibration)
            except OSError as exc:
                QMessageBox.warning(self, "보정 저장 오류", str(exc))

    def _collision_widgets_changed(self, *_) -> None:
        warn_d = self.warn_dist_spin.value()
        danger_d = self.danger_dist_spin.value()
        if danger_d >= warn_d:
            # Keep danger strictly below warning
            danger_d = max(0.1, round(warn_d - 0.1, 2))
            self.danger_dist_spin.blockSignals(True)
            self.danger_dist_spin.setValue(danger_d)
            self.danger_dist_spin.blockSignals(False)
        try:
            new_settings = CollisionSettings(
                warning_distance_m=warn_d,
                danger_distance_m=danger_d,
                prediction_horizon_s=self.horizon_spin.value(),
                min_closing_speed_mps=self.closing_speed_spin.value(),
                cpa_warning_time_s=min(self.horizon_spin.value(), self.collision_settings.cpa_warning_time_s),
                cpa_danger_time_s=min(
                    self.horizon_spin.value(),
                    self.collision_settings.cpa_warning_time_s,
                    self.collision_settings.cpa_danger_time_s,
                ),
                uncertainty_margin_sigma=self.unc_sigma_spin.value(),
                max_position_uncertainty_m=self.max_unc_spin.value(),
                min_confidence_score=self.collision_settings.min_confidence_score,
                visual_alert_enabled=self.visual_alert_check.isChecked(),
                audio_alert_enabled=self.audio_alert_check.isChecked(),
            )
            self.collision_settings = new_settings
            self.collision_analyzer.settings = new_settings
            save_collision_settings(self.collision_settings_path, new_settings)
        except (CollisionSettingsError, OSError):
            pass

    def toggle_recording(self) -> None:
        if self.recorder.active:
            saved = self.recorder.stop()
            self.btn_record_toggle.setText("실험 데이터 기록 시작 / Start Recording")
            if saved is not None:
                self.record_status_label.setText(
                    f"기록 완료: {saved.name} ({len(self.recorder.ticks)}틱 저장됨)"
                )
        else:
            out_dir = self.config_path.parent / "recordings"
            path = self.recorder.start(
                out_dir, session_name="flight_exp", record_video=self.record_video_check.isChecked()
            )
            self.btn_record_toggle.setText("실험 데이터 기록 중지 / Stop Recording")
            self.record_status_label.setText(f"● 기록 중: {path.name} (0틱)")

    def export_recorded_csv(self) -> None:
        if not self.recorder.ticks:
            QMessageBox.warning(self, "CSV 내보내기", "기록된 프레임 데이터가 없습니다. 먼저 실험 기록을 진행하세요.")
            return
        default_path = self.config_path.parent / "recordings" / "experiment_telemetry.csv"
        path, _ = QFileDialog.getSaveFileName(
            self, "실험 데이터 CSV 내보내기", str(default_path), "CSV (*.csv)"
        )
        if path:
            out = export_ticks_csv(Path(path), self.recorder.ticks)
            self.record_status_label.setText(f"CSV 내보내기 완료: {out.name} ({len(self.recorder.ticks)}행)")

    def export_risk_events_csv(self) -> None:
        if not self.collision_analyzer.events:
            QMessageBox.warning(self, "이벤트 CSV 내보내기", "기록된 위험 이벤트가 없습니다.")
            return
        default_path = self.config_path.parent / "recordings" / "risk_events.csv"
        path, _ = QFileDialog.getSaveFileName(
            self, "위험 이벤트 CSV 내보내기", str(default_path), "CSV (*.csv)"
        )
        if path:
            out = export_events_csv(Path(path), self.collision_analyzer.events)
            self.record_status_label.setText(
                f"이벤트 CSV 내보내기 완료: {out.name} ({len(self.collision_analyzer.events)}건)"
            )

    def open_playback_dialog(self) -> None:
        initial = self.recorder.jsonl_path if (self.recorder.jsonl_path and not self.recorder.active) else None
        dialog = PlaybackDialog(jsonl_path=initial, parent=self)
        dialog.exec()

    # ------------------------------------------------------- Camera sources
    def connect_camera(self, index):
        if not self.sources[index].active:
            self.sequences[index] = 0
            if self.pipeline is not None:
                self.pipeline.reset_camera(index)
            self.stereo_tracker.reset()
            self.collision_analyzer.reset()
            self.sources[index].connect()

    def setup_sources(self):
        self.sources, self.tunnels, self.usb_errors = [], [], []
        for config in self.configs:
            tunnel = None
            error = ""
            runtime = resolved_config(config, self.config_path.parent)
            if config.usb_platform:
                try:
                    tunnel = UsbTunnel(config.usb_platform, companion_url(str(config.source)).port, config.usb_serial)
                    runtime = replace(runtime, source=tunnel.rewrite(str(config.source)), usb_platform="")
                except (UsbError, OSError):
                    error = "USB 전달 포트를 열 수 없습니다. 설정을 다시 저장하세요."
                    runtime = replace(runtime, source="http://127.0.0.1:1/stream.mjpg", usb_platform="")
            self.sources.append(CameraSource(runtime))
            self.tunnels.append(tunnel)
            self.usb_errors.append(error)

    def toggle(self, index):
        if self.sources[index].active:
            self.sources[index].disconnect()
        else:
            self.connect_camera(index)

    def connect_all(self):
        for i in range(2):
            self.connect_camera(i)

    def disconnect_all(self):
        for source in self.sources:
            source.disconnect()

    def edit_settings(self):
        dialog = SettingsDialog(self.configs, self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        self.apply_configs(dialog.result_configs)

    def apply_configs(self, new_configs):
        old_configs = self.configs
        was_active = [source.active for source in self.sources]
        try:
            save_config(self.config_path, new_configs)
        except OSError:
            QMessageBox.warning(self, "저장 오류", "설정 파일을 저장할 수 없습니다. 폴더 쓰기 권한을 확인하세요.")
            return
        self._teardown_sources()
        self.configs = new_configs
        self.setup_sources()
        self.sequences = [0, 0]
        for index, active in enumerate(was_active):
            if active:
                self.connect_camera(index)
        if any(was_active):
            self._pending_rollback = (old_configs, was_active)
            QTimer.singleShot(8000, self._verify_settings_rollback)
        self.refresh()

    def _teardown_sources(self):
        for tunnel in self.tunnels:
            if tunnel is not None:
                tunnel.close()
                self.retired_tunnels.append(tunnel)
        for source in self.sources:
            source.close()

    def _verify_settings_rollback(self):
        pending = getattr(self, "_pending_rollback", None)
        if pending is None:
            return
        self._pending_rollback = None
        old_configs, was_active = pending
        failed = [index for index, active in enumerate(was_active)
                  if active and self.sources[index].status().state == State.ERROR]
        if not failed:
            return
        try:
            save_config(self.config_path, old_configs)
        except OSError:
            QMessageBox.warning(self, "복구 오류", "이전 설정을 저장할 수 없습니다.")
            return
        self._teardown_sources()
        self.configs = old_configs
        self.setup_sources()
        self.sequences = [0, 0]
        for index, active in enumerate(was_active):
            if active:
                self.connect_camera(index)
        self.refresh()
        QMessageBox.warning(self, "설정 복구",
            "새 설정으로 연결할 수 없어 이전에 정상 작동하던 설정으로 복구했습니다. "
            "휴대폰 앱의 해상도/FPS와 지원 범위를 확인하세요.")

    def refresh(self):
        connected = 0
        pipeline = self.pipeline
        ai_on = pipeline is not None and pipeline.enabled
        current_detections = [None, None]
        new_ai_frame = False

        for i, source in enumerate(self.sources):
            source.poll()
            status = source.status()
            label = STATE_LABELS[status.state]
            self.titles[i].setText(f"{i + 1:02d}  {self.configs[i].name}")
            detection = None
            if status.state == State.CONNECTED:
                connected += 1
                frame = source.latest_frame(self.sequences[i])
                if frame is not None:
                    self._latest_frames[i] = frame
                    self.views[i].set_frame(frame)
                    self.sequences[i] = frame.sequence
                    if ai_on:
                        pipeline.submit(i, frame)   # Newest frame only; never blocks the GUI.
                if ai_on:
                    detection = pipeline.latest(i)
                    if detection is not None and time.monotonic() - detection.completed_monotonic > OVERLAY_MAX_AGE_SECONDS:
                        detection = None            # Stale: do not present as the current position.
                self.views[i].set_detections(detection, self.ai_settings)
                current_detections[i] = detection
                if detection is not None and detection is not self._shown_detections[i]:
                    new_ai_frame = True
                self._shown_detections[i] = detection
            else:
                self._latest_frames[i] = None
                self._shown_detections[i] = None
                self.views[i].clear_frame(label)
                if self._connected_before[i] and pipeline is not None:
                    pipeline.reset_camera(i)
            self._connected_before[i] = status.state == State.CONNECTED
            age = f"{max(0, time.monotonic() - status.last_received):.1f} s" if status.last_received else "—"
            resolution = f"{status.width} × {status.height}" if status.width else "—"
            requested = (f"{status.requested_width} × {status.requested_height} · "
                         f"{status.requested_fps or 'Auto'} FPS" if status.requested_width
                         else "자동 / Auto")
            capture_fps = f"{status.capture_fps:.1f}" if status.capture_fps else "—"
            mismatch = ""
            if status.state == State.CONNECTED and status.requested_width and status.width \
                    and (status.width, status.height) != (status.requested_width, status.requested_height):
                mismatch = " · 설정과 실제 해상도 다름 / differs from requested"
            if ai_on:
                stats = pipeline.stats(i)
                ai_line = (f"AI: {len(detection.observations) if detection else 0}대 · AI {stats.fps:.1f} FPS · "
                           f"추론 {stats.inference_ms:.0f} ms · 수신→결과 {stats.latency_ms:.0f} ms"
                           f"{' · ' + stats.error if stats.error else ''}")
            else:
                ai_line = "AI: OFF"
            self.details[i].setText(
                f"{self.configs[i].name}  ·  {self.configs[i].input_type.upper()}\n"
                f"설정 / Requested: {requested}\n"
                f"캡처 / Capture: {capture_fps} FPS  ·  수신·출력 / Receive: {status.fps:.1f} FPS\n"
                f"실제 / Actual: {resolution}{mismatch}  ·  마지막 수신 / Last frame: {age}\n"
                f"{ai_line}\n"
                f"{label}\n{self.usb_errors[i] or (self.tunnels[i].error if self.tunnels[i] and self.tunnels[i].error else status.message)}")
            self.buttons[i].setText(f"Camera {i + 1:02d} " + ("해제 / Disconnect" if source.active else "연결 / Connect"))
            self.buttons[i].setEnabled(status.state != State.STOPPING)

        model_text = self._model_text()
        if model_text != self._last_model_text:
            self.model_label.setText(model_text)
            self._last_model_text = model_text
        self.summary.setText(f"{connected} / 2 CONNECTED\n" + (
            "전체 수신 중" if connected == 2 else "일부 수신 중" if connected else "수신 대기"))

        # Phase 03-05 update
        self._update_stereo_and_collision_dashboard(current_detections[0], current_detections[1], new_ai_frame)

    @staticmethod
    def _set_text_if_changed(label: QLabel, text: str) -> None:
        if label.text() != text:
            label.setText(text)

    def _update_stereo_and_collision_dashboard(self, det_01, det_02, new_ai_frame: bool) -> None:
        stereo = self.stereo_tracker.process(det_01, det_02, self.calibration)
        collision = self.collision_analyzer.analyze(stereo)
        self.latest_stereo_result = stereo
        self.latest_collision_result = collision

        if collision.audio_alert_triggered:
            self.audio_beep_count += 1
            QApplication.beep()

        if self.recorder.active and (new_ai_frame or (det_01 is None and det_02 is None)):
            f1 = self._latest_frames[0].image if self._latest_frames[0] is not None else None
            f2 = self._latest_frames[1].image if self._latest_frames[1] is not None else None
            self.recorder.record_tick(det_01, det_02, stereo, collision, frame_01=f1, frame_02=f2)
            self._set_text_if_changed(
                self.record_status_label,
                f"● 기록 중: {self.recorder.jsonl_path.name if self.recorder.jsonl_path else ''} "
                f"({len(self.recorder.ticks)}틱)",
            )

        # Risk Banner
        style_key = collision.risk_level if (collision.visual_alert_active or collision.risk_level in (RISK_UNKNOWN, RISK_SAFE)) else RISK_SAFE
        if getattr(self, "_last_banner_style_key", None) != style_key:
            self._last_banner_style_key = style_key
            self.risk_banner.setStyleSheet(RISK_BANNER_STYLES.get(style_key, RISK_BANNER_STYLES[RISK_UNKNOWN]))
        self._set_text_if_changed(self.risk_banner, f"[{collision.risk_label}]\n{collision.reason}")

        # Status overview
        if self.calibration is None:
            calib_txt = "미보정 (3D 미출력)"
        elif self.calibration.calibration_type == "synthetic":
            calib_txt = f"합성 보정 [실험용] · 기준선 {self.calibration.baseline_m:.2f} m"
        elif self.calibration.calibration_type == "auto":
            calib_txt = (
                f"자동 보정 [체커보드 없음] · 기준선 {self.calibration.baseline_m:.2f} m · "
                f"RMS {self.calibration.rms_error:.2f} px"
            )
        else:
            calib_txt = (
                f"현장 보정 완료 · 기준선 {self.calibration.baseline_m:.2f} m · "
                f"RMS {self.calibration.rms_error:.2f} px"
            )
        sync_txt = (
            f"정상 (Δt={stereo.sync.abs_delta_s * 1000:.1f} ms, 오프셋 {stereo.sync.time_offset_s * 1000:+.0f} ms)"
            if stereo.sync.synchronized
            else f"불일치/대기 ({stereo.sync.reason})"
        )
        pm = collision.pair_metrics
        if pm is not None:
            dist_txt = f"{pm.distance_m:.2f} m (유효 {pm.effective_distance_m:.2f} m, 오차 ±{pm.combined_uncertainty_m:.2f} m)"
            cpa_txt = (
                f"상대 접근 {pm.closing_speed_mps:+.2f} m/s · "
                f"CPA {pm.cpa_distance_m:.2f} m ({pm.cpa_time_s:.1f}초 후)"
            )
        else:
            dist_txt = "— (계산 불가)"
            cpa_txt = "— (계산 불가)"

        self._set_text_if_changed(
            self.status_overview_label,
            f"보정 상태: {calib_txt}\n"
            f"동기화: {sync_txt}\n"
            f"3D 상태: {stereo.status_message}\n"
            f"드론 간 3D 거리: {dist_txt}\n"
            f"접근 속도 · CPA: {cpa_txt}",
        )

        # 3D widget & telemetry text
        self.space_3d_widget.update_scene(self.calibration, stereo, collision)
        if collision.kinematics:
            lines = []
            for k in collision.kinematics:
                x, y, z = k.xyz_m
                vx, vy, vz = k.velocity_mps
                model_str = f" ({k.drone_model})" if getattr(k, "drone_model", "unknown") not in ("", "unknown") else ""
                mode_str = "단안 크기추정 3D" if getattr(k, "estimation_mode", "stereo") == "monocular" else "스테레오 3D"
                lines.append(
                    f"드론 #{k.drone_id}{model_str} [{mode_str} · Cam1 ID:{k.cam1_track_id} ↔ Cam2 ID:{k.cam2_track_id}]\n"
                    f"  위치(X,Y,Z): ({x:+.2f}, {y:+.2f}, {z:.2f}) m  (±{k.position_uncertainty_m:.2f} m)\n"
                    f"  속도(Vx,Vy,Vz): ({vx:+.2f}, {vy:+.2f}, {vz:+.2f}) m/s · {k.direction_label}\n"
                    f"  신뢰도: {k.confidence_score * 100:.0f}% · 재투영 오차: {k.reprojection_error_px:.2f} px"
                )
            self._set_text_if_changed(self.drone_telemetry_label, "\n".join(lines))
        else:
            self._set_text_if_changed(
                self.drone_telemetry_label,
                f"확정된 3D 좌표 없음 — {stereo.status_message}",
            )

        # Calibration detail label in Tab 2
        if self.calibration is None:
            self._set_text_if_changed(
                self.calib_detail_label,
                self.calibration_notice
                or "현재 상태: 미보정 (Uncalibrated)\n"
                   "보정 데이터가 없으면 임의의 3D 좌표를 생성하지 않으며 충돌 위험은 '판단 불가'로 유지됩니다.",
            )
        else:
            c = self.calibration
            if c.calibration_type == "synthetic":
                tag = "합성 보정 (소프트웨어 검증용)"
            elif c.calibration_type == "auto":
                tag = f"자동 보정 (체커보드 없음 · {c.notes})"
            else:
                tag = "현장 실측 보정"
            self._set_text_if_changed(
                self.calib_detail_label,
                f"보정 유형: {tag}\n"
                f"기준선(Baseline): {c.baseline_m:.3f} m · 스테레오 RMS: {c.rms_error:.3f} px\n"
                f"Cam1 f=({c.camera_01.fx:.1f}, {c.camera_01.fy:.1f}) px · "
                f"Cam2 f=({c.camera_02.fx:.1f}, {c.camera_02.fy:.1f}) px\n"
                f"시간 오프셋: {c.time_offset_s * 1000:+.1f} ms · 허용 동기화 오차: {c.max_sync_delta_s * 1000:.0f} ms",
            )

        # Risk Events list in Tab 3
        events = self.collision_analyzer.events
        if events:
            recent = events[-5:]
            ev_lines = []
            for ev in reversed(recent):
                d_str = f"{ev.distance_m:.2f}m" if ev.distance_m is not None else "—"
                ev_lines.append(f"[#{ev.event_id} {ev.risk_level.upper()}] 거리:{d_str} · {ev.reason}")
            self._set_text_if_changed(self.risk_events_label, "\n".join(ev_lines))
        else:
            self._set_text_if_changed(self.risk_events_label, "기록된 위험 이벤트 없음")

    def closeEvent(self, event):
        self.timer.stop()
        try:
            self.shutdown()
        except RuntimeError as error:
            QMessageBox.warning(self, "종료 오류", str(error))
            self.timer.start(33)
            event.ignore()
            return
        event.accept()

    def shutdown(self):
        if self.recorder.active:
            self.recorder.stop()
        if self._ai_save_timer.isActive():
            self._ai_save_timer.stop()
            self._save_ai_settings()
        if self.pipeline is not None:
            self.pipeline.stop(timeout=3)
        for source in self.sources:
            source.disconnect()
        all_tunnels = self.tunnels + self.retired_tunnels
        for tunnel in all_tunnels:
            if tunnel is not None:
                tunnel.close()
        for source in self.sources:
            source.close()
        deadline = time.monotonic() + 9
        for tunnel in all_tunnels:
            if tunnel is not None:
                tunnel.wait_closed(timeout=max(0, deadline - time.monotonic()))
