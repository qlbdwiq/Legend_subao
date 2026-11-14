from dataclasses import replace

import pytest

from drone_camera.capture import State
from drone_camera.config import defaults, load_config
from drone_camera.gui import MainWindow
from conftest import wait_for


@pytest.fixture
def live_window(qapp, video, tmp_path, monkeypatch):
    monkeypatch.setattr("drone_camera.gui.QMessageBox.warning", lambda *args: None)
    configs = [replace(c, input_type="file", source=str(video), loop_file=True) for c in defaults()]
    window = MainWindow(configs, tmp_path / "config.json")
    window.connect_all()
    try:
        wait_for(lambda: all(s.status().state == State.CONNECTED for s in window.sources), app=qapp)
        yield window, configs
    finally:
        window.close()


def failing_configs(configs):
    # A closed local port fails quickly but the configured retry delay keeps
    # the connection pending long enough to observe rollback monitoring.
    return [replace(configs[0], input_type="mjpeg", source="http://127.0.0.1:1/video",
                    timeout_seconds=0.2, reconnect_delay_seconds=0.1,
                    max_reconnect_attempts=10), configs[1]]


def test_rollback_waits_for_retries_and_restores_saved_working_config(qapp, live_window):
    window, good = live_window
    window.apply_configs(failing_configs(good))
    wait_for(lambda: window.sources[0].status().state == State.RECONNECTING, app=qapp)
    window._verify_settings_rollback()
    assert window._pending_rollback is not None
    assert window.rollback_timer.isActive()
    wait_for(lambda: window.configs == good, app=qapp)
    assert load_config(window.config_path) == good
    assert window._pending_rollback is None
    assert not window.rollback_timer.isActive()
    wait_for(lambda: all(s.status().state == State.CONNECTED for s in window.sources), app=qapp)


def test_repeated_edits_keep_original_fallback_and_manual_disconnect_cancels(qapp, live_window):
    window, good = live_window
    first = failing_configs(good)
    window.apply_configs(first)
    second = [replace(first[0], name="Second unverified edit"), first[1]]
    window.apply_configs(second)
    assert window._pending_rollback[0] == good
    window.disconnect_all()
    assert window._pending_rollback is None
    assert not window.rollback_timer.isActive()
    wait_for(lambda: all(not s.active for s in window.sources), app=qapp)
    window._verify_settings_rollback()
    assert window.configs == second
    assert load_config(window.config_path) == second


def test_successful_settings_change_finishes_monitoring(qapp, live_window):
    window, good = live_window
    updated = [replace(c, name="Updated " + c.name) for c in good]
    window.apply_configs(updated)
    wait_for(lambda: all(s.status().state == State.CONNECTED for s in window.sources), app=qapp)
    window._verify_settings_rollback()
    assert window.configs == updated
    assert window._pending_rollback is None
    assert not window.rollback_timer.isActive()
