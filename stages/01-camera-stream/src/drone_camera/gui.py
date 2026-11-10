"""Qt presentation; acquisition and configuration remain usable without Qt."""

from pathlib import Path
from dataclasses import replace
import time

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QFormLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QMessageBox, QPushButton, QSizePolicy, QSpinBox, QVBoxLayout, QWidget,
)

from .capture import CameraSource, State
from .config import CameraConfig, ConfigError, resolved_config, save_config
from .usb import UsbError, UsbTunnel, companion_url


class VideoView(QLabel):
    def __init__(self):
        super().__init__("연결 대기 / Waiting for connection")
        self._image = None
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(240, 180)
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
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
        self.transports = []
        self.serials = [c.usb_serial for c in configs]
        self.fields = []
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
            for label, widget in (("이름 / Name", name), ("입력 종류 / Type", kind),
                ("입력 / Source", source), ("", show), ("타임아웃 / Timeout", timeout),
                ("자동 재연결 / Auto retry", retry), ("재시도 간격 / Delay", delay),
                ("최대 재시도 / Attempts", attempts), ("파일 반복 / Loop file", loop)):
                form.addRow(label, widget)
            self.fields.append((name, kind, source, timeout, retry, delay, attempts, loop))
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

    def _validate(self):
        result = []
        try:
            for index, (config, fields) in enumerate(zip(self.configs, self.fields)):
                name, kind, source, timeout, retry, delay, attempts, loop = fields
                value = source.text().strip()
                if kind.currentText() == "usb":
                    try:
                        value = int(value)
                    except ValueError:
                        raise ConfigError("USB 입력에는 장치 번호를 입력하세요 (0–99).") from None
                result.append(CameraConfig(config.id, name.text().strip(), kind.currentText(), value,
                    timeout.value(), retry.isChecked(), delay.value(), attempts.value(), loop.isChecked(),
                    ["", "android", "ios"][self.transports[index].currentIndex()], self.serials[index]))
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
    def __init__(self, configs: list[CameraConfig], config_path: Path):
        super().__init__()
        self.configs = configs
        self.config_path = config_path
        self.setup_sources()
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
                    # Do not silently fall back to Wi-Fi when USB was requested.
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
        for tunnel in self.tunnels:
            if tunnel is not None:
                tunnel.close()
        for source in self.sources:
            source.close()
        self.configs = dialog.result_configs
        self.setup_sources()
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
                f"{label}\n{self.usb_errors[i] or (self.tunnels[i].error if self.tunnels[i] and self.tunnels[i].error else status.message)}")
            self.buttons[i].setText(f"Camera {i + 1:02d} " + ("해제 / Disconnect" if source.active else "연결 / Connect"))
            self.buttons[i].setEnabled(status.state != State.STOPPING)
        self.summary.setText(f"{connected} / 2 CONNECTED\n" + (
            "전체 수신 중" if connected == 2 else "일부 수신 중" if connected else "수신 대기"))

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
        for source in self.sources:
            source.disconnect()
        for tunnel in self.tunnels:
            if tunnel is not None:
                tunnel.close()
        for source in self.sources:
            source.close()
        for tunnel in self.tunnels:
            if tunnel is not None:
                tunnel.wait_closed()
