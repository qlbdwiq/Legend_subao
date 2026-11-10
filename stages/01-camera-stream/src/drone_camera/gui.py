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

from .capabilities import (
    STANDARD_FPS, STANDARD_RESOLUTIONS, CapabilityError, StreamMode,
    best_fps_mode, best_resolution_mode, fetch_capabilities, fps_options,
    is_supported, probe_usb_modes,
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
        """Enumerate what the selected device can really stream.

        Phone apps answer /capabilities from their Camera2/AVFoundation
        streaming formats; USB webcams are probed by applying sizes.
        Only then are unsupported combinations removed from the menus.
        """
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
    def __init__(self, configs: list[CameraConfig], config_path: Path):
        super().__init__()
        self.configs = configs
        self.config_path = config_path
        self.retired_tunnels = []
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
            details.setMinimumHeight(132)
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
        dialog = SettingsDialog(self.configs, self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        self.apply_configs(dialog.result_configs)

    def apply_configs(self, new_configs):
        """Save and apply settings, reconnecting cameras that were live.

        If a previously working camera ends in ERROR under the new
        settings, the last known-good configuration is restored and
        reconnected automatically (see _verify_settings_rollback).
        """
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
            requested = (f"{status.requested_width} × {status.requested_height} · "
                         f"{status.requested_fps or 'Auto'} FPS" if status.requested_width
                         else "자동 / Auto")
            capture_fps = f"{status.capture_fps:.1f}" if status.capture_fps else "—"
            mismatch = ""
            if status.state == State.CONNECTED and status.requested_width and status.width \
                    and (status.width, status.height) != (status.requested_width, status.requested_height):
                mismatch = " · 설정과 실제 해상도 다름 / differs from requested"
            self.details[i].setText(
                f"{self.configs[i].name}  ·  {self.configs[i].input_type.upper()}\n"
                f"설정 / Requested: {requested}\n"
                f"캡처 / Capture: {capture_fps} FPS  ·  수신·출력 / Receive: {status.fps:.1f} FPS\n"
                f"실제 / Actual: {resolution}{mismatch}  ·  마지막 수신 / Last frame: {age}\n"
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
