from dataclasses import replace
import time

from PySide6.QtCore import QTimer

from drone_camera.capture import State
from drone_camera.config import defaults
from drone_camera.gui import MainWindow, SettingsDialog
from conftest import wait_for


def test_headless_gui_dual_playback_resizing_and_shutdown(qapp, video, tmp_path):
    configs = [replace(c, input_type="file", source=str(video), loop_file=True) for c in defaults()]
    window = MainWindow(configs, tmp_path / "config.json")
    window.show()
    ticks = []
    timer = QTimer()
    timer.timeout.connect(lambda: ticks.append(time.monotonic()))
    timer.start(10)
    try:
        window.connect_all()
        wait_for(lambda: all(s.status().state == State.CONNECTED for s in window.sources), app=qapp)
        wait_for(lambda: all(view._image is not None for view in window.views), app=qapp)
        assert "2 / 2" in window.summary.text()
        # Fast startup on macOS can finish in only two event-loop turns.
        # Observe responsiveness for a bounded interval, independent of startup speed.
        baseline = len(ticks)
        wait_for(lambda: len(ticks) >= baseline + 5, timeout=2, app=qapp)
        assert len(ticks) >= baseline + 5
        window.resize(1000, 650)
        qapp.processEvents()
        for view in window.views:
            pixmap = view.pixmap()
            assert abs(pixmap.width() / pixmap.height() - 320 / 180) < 0.03
        window.toggle(0)
        wait_for(lambda: not window.sources[0].active, app=qapp)
        assert window.sources[1].status().state == State.CONNECTED
        window.connect_camera(0)
        wait_for(lambda: window.sources[0].status().state == State.CONNECTED, app=qapp)
    finally:
        timer.stop()
        window.close()
        qapp.processEvents()
    assert all(not s.active for s in window.sources)


def test_settings_dialog_roundtrip(qapp):
    dialog = SettingsDialog(defaults())
    dialog.fields[0][0].setText("Android Camera")
    dialog.fields[0][1].setCurrentText("mjpeg")
    dialog.fields[0][2].setText("http://127.0.0.1:8080/video")
    dialog._validate()
    assert dialog.result_configs[0].input_type == "mjpeg"
    assert dialog.result_configs[0].name == "Android Camera"


def test_settings_dialog_resolution_and_fps_are_independent(qapp):
    dialog = SettingsDialog(defaults())
    resolution = dialog.fields[0][8]
    fps = dialog.fields[0][9]
    for row in range(resolution.count()):
        if tuple(resolution.itemData(row)) == (1920, 1080):
            resolution.setCurrentIndex(row)
    for row in range(fps.count()):
        if fps.itemData(row) == 120:
            fps.setCurrentIndex(row)
    dialog.fields[0][1].setCurrentText("mjpeg")
    dialog.fields[0][2].setText("http://127.0.0.1:8080/video")
    dialog._validate()
    assert dialog.result_configs[0].requested_width == 1920
    assert dialog.result_configs[0].requested_height == 1080
    assert dialog.result_configs[0].requested_fps == 120
    # The second phone keeps its own independent (Auto) settings.
    assert dialog.result_configs[1].requested_width == 0
    assert dialog.result_configs[1].requested_fps == 0


def test_settings_dialog_rejects_unsupported_probed_combination(qapp, monkeypatch):
    from drone_camera.capabilities import StreamMode
    dialog = SettingsDialog(defaults())
    dialog.fields[0][1].setCurrentText("mjpeg")
    dialog.fields[0][2].setText("http://127.0.0.1:8080/video")
    dialog.device_modes[0] = (StreamMode(3840, 2160, 60), StreamMode(1920, 1080, 240))
    dialog._fill_resolution(0)
    dialog._fill_fps(0)
    resolution = dialog.fields[0][8]
    for row in range(resolution.count()):
        if tuple(resolution.itemData(row)) == (3840, 2160):
            resolution.setCurrentIndex(row)
    dialog._fill_fps(0)
    offered = [dialog.fields[0][9].itemData(row) for row in range(dialog.fields[0][9].count())]
    assert 120 not in offered  # 4K is capped at 60 FPS on this device.
    warnings = []
    monkeypatch.setattr("drone_camera.gui.QMessageBox.warning",
                        lambda *args: warnings.append(args))
    dialog.fields[0][9].addItem("120 FPS", 120)
    dialog.fields[0][9].setCurrentIndex(dialog.fields[0][9].count() - 1)
    dialog._validate()
    assert dialog.result_configs is None
    assert warnings
