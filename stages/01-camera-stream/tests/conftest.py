import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
# Tests must never touch the user's global Ultralytics settings or the network.
import tempfile
os.environ.setdefault("YOLO_CONFIG_DIR", tempfile.mkdtemp(prefix="yolo-test-config-"))
os.environ.setdefault("YOLO_OFFLINE", "1")
os.environ.setdefault("YOLO_AUTOINSTALL", "False")
os.environ.setdefault("YOLO_VERBOSE", "False")

import cv2
import numpy as np
import pytest


@pytest.fixture
def video(tmp_path):
    """Real, portable MJPEG AVI; fail if the installed codec cannot write it."""
    path = tmp_path / "test video.avi"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 30, (320, 180))
    assert writer.isOpened()
    for index in range(90):
        image = np.full((180, 320, 3), (index * 2, 80, 180), dtype=np.uint8)
        cv2.putText(image, str(index), (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        writer.write(image)
    writer.release()
    return path


@pytest.fixture(scope="session")
def qapp():
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    yield app


def wait_for(predicate, timeout=12, app=None):
    import time
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if app:
            app.processEvents()
        value = predicate()
        if value:
            return value
        time.sleep(0.02)
    raise AssertionError("Timed out waiting for expected camera behavior")


@pytest.fixture(scope="session")
def real_model_path():
    value = os.environ.get("DRONE_YOLO_MODEL", "")
    if not value or not Path(value).is_file():
        pytest.skip("real_model: set DRONE_YOLO_MODEL to the trained .pt file")
    pytest.importorskip("ultralytics")
    return value


@pytest.fixture(scope="session")
def samples_dir():
    value = os.environ.get("DRONE_YOLO_SAMPLES", "")
    if not value or not Path(value).is_dir():
        pytest.skip("real_model: set DRONE_YOLO_SAMPLES to a folder of labelled drone images")
    return Path(value)
