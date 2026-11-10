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
        assert len(ticks) > 2  # Qt event loop remained responsive while connecting.
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
