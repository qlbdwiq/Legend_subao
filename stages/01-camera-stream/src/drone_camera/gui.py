"""Qt presentation; acquisition and configuration remain usable without Qt."""

from pathlib import Path
import time

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QFormLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QMessageBox, QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

from .capture import CameraSource, State
from .config import CameraConfig, ConfigError, resolved_config, save_config


class VideoView(QLabel):
    def __init__(self):
        super().__init__("연결 대기 / Waiting for connection")
        self._image = None
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(240, 180)
        self.setStyleSheet("background: #05080d; color: #93a4bd; border-radius: 8px;")

    def set_frame(self, frame):
        height, width = frame.image.shape[:2]
        self._image = QImage(frame.image.data, width, height, frame.image.strides[0],
                             QImage.Format.Format_BGR888).copy()
        self._redraw()

    def clear_frame(self, text):
        self._image = None
        self.setPixmap(QPixmap())
        self.setText(text)

    def _redraw(self):
        if self._image is not None:
            self.setPixmap(QPixmap.fromImage(self._image).scaled(
                self.size(), Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._redraw()


class SettingsDialog(QDialog):
    def __init__(self, configs, parent=None):
        super().__init__(parent)
        self.setWindowTitle("카메라 설정 / Camera settings")
        self.setMinimumWidth(700)
        self.configs = configs
        self.result_configs = None
        self.fields = []
        layout = QVBoxLayout(self)
        columns = QHBoxLayout()
        for config in configs:
            group = QGroupBox(config.id)
            form = QFormLayout(group)
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
            for label, widget in (("이름 / Name", name), ("입력 종류 / Type", kind),
                ("입력 / Source", source), ("", show), ("타임아웃 / Timeout", timeout),
                ("자동 재연결 / Auto retry", retry), ("재시도 간격 / Delay", delay),
                ("최대 재시도 / Attempts", attempts), ("파일 반복 / Loop file", loop)):
                form.addRow(label, widget)
            self.fields.append((name, kind, source, timeout, retry, delay, attempts, loop))
            columns.addWidget(group)
        layout.addLayout(columns)
        notice = QLabel("iOS/Android 스트리밍 앱의 직접 MJPEG/RTSP 주소를 입력하세요.\n"
                        "설정은 이 컴퓨터에 저장됩니다. URL에 포함된 비밀번호도 저장되므로 공유하지 마세요.")
        notice.setWordWrap(True)
        layout.addWidget(notice)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self._validate)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _validate(self):
        result = []
        try:
            for config, fields in zip(self.configs, self.fields):
                name, kind, source, timeout, retry, delay, attempts, loop = fields
                value = source.text().strip()
                if kind.currentText() == "usb":
                    try:
                        value = int(value)
                    except ValueError:
                        raise ConfigError("USB 입력에는 장치 번호를 입력하세요 (0–99).") from None
                result.append(CameraConfig(config.id, name.text().strip(), kind.currentText(), value,
                    timeout.value(), retry.isChecked(), delay.value(), attempts.value(), loop.isChecked()))
        except ConfigError as error:
            QMessageBox.warning(self, "설정 확인", str(error))
            return
        self.result_configs = result
        self.accept()


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
    def __init__(self, configs: list[CameraConfig], config_path: Path):
        super().__init__()
        self.configs = configs
        self.config_path = config_path
        self.sources = [CameraSource(resolved_config(c, config_path.parent)) for c in configs]
        self.sequences = [0, 0]
        self.setWindowTitle("Drone Collision Detection System — Phase 01")
        self.resize(1280, 760)
        self.setMinimumSize(800, 560)
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(20, 20, 20, 20)
        header = QHBoxLayout()
        title = QLabel("Drone Collision Detection System\nPHASE 01  ·  DUAL CAMERA STREAMING")
        title.setStyleSheet("font-size: 20px; font-weight: 600;")
        self.summary = QLabel()
        self.summary.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        settings = QPushButton("설정 / Settings")
        settings.clicked.connect(self.edit_settings)
        header.addWidget(title, 1)
        header.addWidget(self.summary)
        header.addWidget(settings)
        layout.addLayout(header)
        row = QHBoxLayout()
        self.views, self.details, self.buttons, self.titles = [], [], [], []
        for i in range(2):
            card = QGroupBox()
            column = QVBoxLayout(card)
            heading = QLabel()
            heading.setStyleSheet("font-size: 18px; font-weight: 600;")
            view = VideoView()
            details = QLabel()
            details.setWordWrap(True)
            details.setMinimumHeight(104)
            details.setTextFormat(Qt.TextFormat.PlainText)
            button = QPushButton()
            button.clicked.connect(lambda checked=False, index=i: self.toggle(index))
            heading.setTextFormat(Qt.TextFormat.PlainText)
            for widget in (heading, view, details, button):
                column.addWidget(widget, 1 if widget is view else 0)
            row.addWidget(card, 1)
            self.views.append(view)
            self.details.append(details)
            self.buttons.append(button)
            self.titles.append(heading)
        layout.addLayout(row, 1)
        bottom = QHBoxLayout()
        for text, action in (("전체 연결 / Connect all", self.connect_all),
                             ("전체 해제 / Disconnect all", self.disconnect_all),
                             ("설정 / Settings", self.edit_settings),
                             ("종료 / Exit", self.close)):
            button = QPushButton(text)
            button.clicked.connect(action)
            bottom.addWidget(button)
        layout.addLayout(bottom)
        footer = QLabel("CPU ONLY  •  수신 시각 기준 / Reception timestamps  •  객체 인식 및 충돌 판단 기능 미포함")
        footer.setStyleSheet("color: #93a4bd;")
        layout.addWidget(footer)
        self.setStyleSheet("""
            QMainWindow, QDialog { background: #111827; color: #e5edf8; }
            QWidget { color: #e5edf8; font-size: 12px; }
            QGroupBox { background: #192438; border: 1px solid #33435d; border-radius: 10px; padding: 12px; }
            QPushButton { background: #245a87; border: 1px solid #3975a6; border-radius: 6px; padding: 10px; }
            QPushButton:hover { background: #3072a4; }
            QPushButton:disabled { color: #8290a5; background: #28354a; }
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox { background: #0e1726; padding: 5px; border: 1px solid #52627c; }
            QComboBox QAbstractItemView { background: #192438; color: #e5edf8; }
            QMessageBox { background: #192438; }
        """)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(33)
        self.refresh()

    def connect_camera(self, index):
        if not self.sources[index].active:
            self.sequences[index] = 0
            self.sources[index].connect()

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
        if any(source.active for source in self.sources):
            QMessageBox.information(self, "설정", "설정을 변경하려면 카메라 연결을 모두 해제하세요.")
            return
        dialog = SettingsDialog(self.configs, self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        try:
            save_config(self.config_path, dialog.result_configs)
        except OSError:
            QMessageBox.warning(self, "저장 오류", "설정 파일을 저장할 수 없습니다. 폴더 쓰기 권한을 확인하세요.")
            return
        for source in self.sources:
            source.close()
        self.configs = dialog.result_configs
        self.sources = [CameraSource(resolved_config(c, self.config_path.parent)) for c in self.configs]
        self.sequences = [0, 0]
        self.refresh()

    def refresh(self):
        connected = 0
        for i, source in enumerate(self.sources):
            source.poll()
            status = source.status()
            label = STATE_LABELS[status.state]
            self.titles[i].setText(f"{i + 1:02d}  {self.configs[i].name}")
            if status.state == State.CONNECTED:
                connected += 1
                frame = source.latest_frame(self.sequences[i])
                if frame is not None:
                    self.views[i].set_frame(frame)
                    self.sequences[i] = frame.sequence
            else:
                self.views[i].clear_frame(label)
            age = f"{max(0, time.monotonic() - status.last_received):.1f} s" if status.last_received else "—"
            resolution = f"{status.width} × {status.height}" if status.width else "—"
            self.details[i].setText(
                f"{self.configs[i].name}  ·  {self.configs[i].input_type.upper()}\n"
                f"{status.fps:.1f} FPS  ·  {resolution}  ·  마지막 수신 / Last frame: {age}\n"
                f"{label}\n{status.message}")
            self.buttons[i].setText(f"Camera {i + 1:02d} " + ("해제 / Disconnect" if source.active else "연결 / Connect"))
            self.buttons[i].setEnabled(status.state != State.STOPPING)
        self.summary.setText(f"{connected} / 2 CONNECTED\n" + (
            "전체 수신 중" if connected == 2 else "일부 수신 중" if connected else "수신 대기"))

    def closeEvent(self, event):
        self.timer.stop()
        for source in self.sources:
            source.disconnect()
        try:
            for source in self.sources:
                source.close()
        except RuntimeError as error:
            QMessageBox.warning(self, "종료 오류", str(error))
            self.timer.start(33)
            event.ignore()
            return
        event.accept()
