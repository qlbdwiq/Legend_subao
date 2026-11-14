"""Mount the built DMG, check its contents, and run the app copied out of it.

Runs the frozen app with no Python/venv in the environment, from a folder with
spaces and Korean characters, against two real video files and two MJPEG streams.
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import platform
from pathlib import Path
import plistlib
import shutil
import subprocess
import tempfile
import threading
import time

import cv2
import numpy as np

APP = "DroneCollisionDetection.app"


def run_checked(*command):
    return subprocess.run(command, check=True, capture_output=True, text=True).stdout


def main(dmg: Path, evidence: Path):
    evidence.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="drone-dmg-") as temporary:
        folder = Path(temporary) / "한글 폴더 with spaces"
        folder.mkdir()
        mount = Path(temporary) / "mount"
        mount.mkdir()
        run_checked("hdiutil", "attach", str(dmg), "-mountpoint", str(mount), "-nobrowse", "-readonly")
        try:
            names = {p.name for p in mount.iterdir()}
            print("DMG contents:", sorted(names))
            assert {APP, "Applications", "README-KO.md", "LICENSE"} <= names
            assert (mount / "Applications").is_symlink() and os.readlink(mount / "Applications") == "/Applications"
            run_checked("ditto", str(mount / APP), str(folder / APP))
        finally:
            subprocess.run(["hdiutil", "detach", str(mount), "-force"], capture_output=True)
        app = folder / APP
        info = plistlib.loads((app / "Contents/Info.plist").read_bytes())
        assert info["CFBundleIdentifier"] == "com.dronecollision.detection" and info["NSCameraUsageDescription"]
        run_checked("codesign", "--verify", "--deep", "--strict", str(app))
        binary = app / "Contents/MacOS/DroneCollisionDetection"
        wanted = {"arm64": "arm64", "x86_64": "x86_64"}[platform.machine()]
        assert wanted in run_checked("lipo", "-archs", str(binary)).split(), "wrong CPU architecture"
        adb = next(app.rglob("platform-tools/adb"))
        assert os.access(adb, os.X_OK)

        env = {"HOME": str(folder / "home"), "PATH": "/usr/bin:/bin", "LANG": "ko_KR.UTF-8"}
        for name in ("DRONE_SELFTEST_MODEL", "DRONE_SELFTEST_IMAGE"):   # optional real-model check
            if os.environ.get(name):
                env[name] = os.environ[name]
        (folder / "home").mkdir()
        video = folder / "테스트 영상.avi"
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), 30, (1280, 720))
        assert writer.isOpened()
        for frame in range(90):
            image = np.full((720, 1280, 3), (40, 90, 150), np.uint8)
            cv2.putText(image, str(frame), (40, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
            writer.write(image)
        writer.release()

        def run(kind, source, name):
            cameras = [{"id": f"camera-{i:02d}", "name": f"Camera {i}", "input_type": kind,
                        "source": source, "loop_file": True} for i in (1, 2)]
            config = folder / "config.json"
            config.write_text(json.dumps({"version": 1, "cameras": cameras}), encoding="utf-8")
            report = evidence.resolve() / (name + ".json")
            report.unlink(missing_ok=True)
            result = subprocess.run([str(binary), "--config", str(config), "--self-test-report", str(report)],
                                    cwd=folder, env=env, timeout=420)
            assert report.is_file(), f"App did not write {name} report (exit {result.returncode})"
            data = json.loads(report.read_text(encoding="utf-8"))
            print(name, json.dumps(data))
            assert result.returncode == 0 and data["ok"] and data["frozen"] and data["bundled_adb"]
            assert data["workers_stopped"] and data["active_children"] == 0
            assert data["ai_runtime"]["ok"] and len(set(data["ai_runtime"]["bytetrack_ids"])) == 2, data["ai_runtime"]
            assert data["stereo_3d_collision"]["ok"], data["stereo_3d_collision"]
            assert data["model_check"]["ok"], data["model_check"]
            assert len(data["cameras"]) == 2 and all(c["frames"] >= 5 for c in data["cameras"])

        run("file", video.name, "dual-file")
        ok, jpeg = cv2.imencode(".jpg", np.full((180, 320, 3), 120, np.uint8))
        assert ok
        payload = jpeg.tobytes()

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    for _ in range(600):
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                         str(len(payload)).encode() + b"\r\n\r\n" + payload + b"\r\n")
                        self.wfile.flush()
                        time.sleep(1 / 30)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            run("mjpeg", f"http://127.0.0.1:{server.server_port}/video", "dual-mjpeg")
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)
    print("macOS DMG smoke test passed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("dmg", type=Path)
    parser.add_argument("evidence", type=Path)
    args = parser.parse_args()
    main(args.dmg.resolve(), args.evidence.resolve())
