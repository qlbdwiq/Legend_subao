"""Frozen-app integration probe used by Windows CI; never needs a camera."""
import json
import multiprocessing
from pathlib import Path
import subprocess
import sys
import time


def run(config_path: Path, report_path: Path) -> int:
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication
    from .capture import State
    from .config import load_config
    from .gui import MainWindow
    from .runtime import adb_executable
    app = QApplication(sys.argv[:1])
    app.setApplicationName("DroneCameraSelfTest")
    window = MainWindow(load_config(config_path), config_path)
    window.show()
    started = time.monotonic()
    result = {"ok": False, "frozen": bool(getattr(sys, "frozen", False))}
    ticks = []
    timer = QTimer()

    def finish(ok, reason=""):
        timer.stop()
        result.update(ok=ok, reason=reason, gui_ticks=len(ticks))
        result["cameras"] = [{"state": s.status().state.name, "fps": s.status().fps,
            "resolution": [s.status().width, s.status().height], "frames": window.sequences[i]}
            for i, s in enumerate(window.sources)]
        window.grab().save(str(report_path.with_suffix(".png")))
        window.close()
        result["workers_stopped"] = all(not s.active for s in window.sources)
        result["active_children"] = len(multiprocessing.active_children())
        if ok and sys.platform == "win32" and getattr(sys, "frozen", False):
            try:
                adb = adb_executable()
                command = subprocess.run([adb, "version"], capture_output=True, timeout=10,
                                         creationflags=subprocess.CREATE_NO_WINDOW)
                result["bundled_adb"] = command.returncode == 0 and b"Android Debug Bridge" in command.stdout
            except (OSError, ValueError, subprocess.SubprocessError):
                result["bundled_adb"] = False
            result["ok"] = result["ok"] and result["bundled_adb"]
        if ok and sys.platform == "darwin" and getattr(sys, "frozen", False):
            try:
                command = subprocess.run([adb_executable(), "version"], capture_output=True, timeout=10)
                result["bundled_adb"] = command.returncode == 0 and b"Android Debug Bridge" in command.stdout
            except (OSError, ValueError, TypeError, subprocess.SubprocessError):
                result["bundled_adb"] = False
            result["ok"] = result["ok"] and result["bundled_adb"]
        result["ok"] = result["ok"] and result["workers_stopped"] and result["active_children"] == 0
        report_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
        app.exit(0 if result["ok"] else 1)

    def check():
        ticks.append(time.monotonic())
        statuses = [s.status() for s in window.sources]
        if (time.monotonic() - started > 2 and len(ticks) >= 10
                and all(s.state == State.CONNECTED and s.fps > 0 for s in statuses)
                and all(n >= 5 for n in window.sequences)
                and all(view._image is not None for view in window.views)):
            finish(True)
        elif time.monotonic() - started > 40:
            finish(False, "Timed out waiting for two live frames in the frozen GUI")

    timer.timeout.connect(check)
    timer.start(100)
    QTimer.singleShot(0, window.connect_all)
    return app.exec()
