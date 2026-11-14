"""Frozen-app integration probe used by Windows CI; never needs a camera."""
import json
import multiprocessing
import os
from pathlib import Path
import subprocess
import sys
import time


def model_check(config_path: Path) -> dict:
    """Optional real-model check: DRONE_SELFTEST_MODEL (.pt) and DRONE_SELFTEST_IMAGE (a drone photo)."""
    model, image = os.environ.get("DRONE_SELFTEST_MODEL"), os.environ.get("DRONE_SELFTEST_IMAGE")
    if not model or not image:
        return {"ok": True, "configured": False}
    try:
        import cv2
        from .engine import YoloTrackerEngine
        engine = YoloTrackerEngine(model, "cpu", config_path.parent / "ultralytics")
        info = engine.load()
        picture = cv2.imread(image)
        detections = engine.track(picture, 0.25, 0.5, 960)
        return {"ok": info.names == {0: "drone"} and len(detections) >= 1, "configured": True,
                "names": {str(k): v for k, v in info.names.items()}, "device": info.device,
                "detections": len(detections), "ultralytics": info.ultralytics_version,
                "torch": info.torch_version}
    except Exception as error:
        return {"ok": False, "configured": True, "error": f"{type(error).__name__}: {error}"[:300]}


def stereo_3d_collision_check() -> dict:
    """Verify Phase 03~04 synthetic stereo 3D triangulation and collision risk analysis in frozen runtime."""
    try:
        from .calibration import create_synthetic_calibration, project_points_3d
        from .collision import CollisionAnalyzer, RISK_UNKNOWN
        from .stereo_3d import StereoTracker3D, triangulate_points
        import numpy as np

        calib = create_synthetic_calibration(baseline_m=2.0, convergence_deg=5.0)
        pt = np.array([[0.3, -0.4, 6.0]], dtype=np.float64)
        px1 = tuple(project_points_3d(pt, calib.camera_01)[0])
        px2 = tuple(project_points_3d(pt, calib.camera_02, calib.R, calib.T)[0])
        tri = triangulate_points(px1, px2, calib, detector_confidence=0.92)
        err_m = float(np.linalg.norm(np.array(tri.xyz_m) - pt[0])) if tri is not None else 999.0
        tracker = StereoTracker3D()
        uncalib_res = tracker.process(None, None, calibration=None)
        analyzer = CollisionAnalyzer()
        unk = analyzer.analyze(uncalib_res)
        return {
            "ok": bool(tri is not None and err_m < 1e-3 and unk.risk_level == RISK_UNKNOWN),
            "triangulation_error_m": round(err_m, 6),
            "uncalibrated_risk": unk.risk_level,
        }
    except Exception as error:
        return {"ok": False, "error": f"{type(error).__name__}: {error}"[:300]}


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
        if ok:
            # Phase 02: prove torch, torchvision NMS, bundled Ultralytics config files, lap and
            # ByteTrack work inside this (frozen) build. Random weights: says nothing about accuracy.
            from .engine import runtime_report
            result["ai_runtime"] = runtime_report(config_path.parent / "ultralytics")
            result["ok"] = result["ok"] and result["ai_runtime"]["ok"]
            result["stereo_3d_collision"] = stereo_3d_collision_check()
            result["ok"] = result["ok"] and result["stereo_3d_collision"]["ok"]
            result["model_check"] = model_check(config_path)
            result["ok"] = result["ok"] and result["model_check"].get("ok", True)
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
