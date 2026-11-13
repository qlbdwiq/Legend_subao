"""Exercise the built EXE with no Python/SDK paths and a non-build working dir."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time

import cv2
import numpy as np


def main(executable: Path, evidence: Path):
    evidence.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="drone-exe-") as temporary:
        folder = Path(temporary) / "한글 폴더 with spaces"
        folder.mkdir()
        exe = folder / executable.name
        shutil.copyfile(executable, exe)  # A single copied EXE, no adjacent DLLs.
        env = os.environ.copy()
        for name in list(env):
            if name.upper().startswith(("PYTHON", "VIRTUAL_ENV", "QT_", "PYSIDE", "CONDA")):
                env.pop(name)
        env["PATH"] = str(Path(env["SYSTEMROOT"]) / "System32")
        env["QT_QPA_PLATFORM"] = "windows"  # Check qwindows, not only the offscreen plugin.
        env["LOCALAPPDATA"] = str(folder / "AppData")
        video = folder / "테스트 영상.avi"
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), 30, (1280, 720))
        assert writer.isOpened()
        for frame in range(90):
            image = np.full((720,1280,3), (40,90,150), np.uint8)
            cv2.putText(image,str(frame),(40,100),cv2.FONT_HERSHEY_SIMPLEX,2,(255,255,255),3)
            writer.write(image)
        writer.release()

        def run(kind, source, name):
            cameras = [{"id": f"camera-{i:02d}", "name": f"Camera {i}", "input_type": kind,
                        "source": source, "loop_file": True} for i in (1,2)]
            config = folder / "config.json"
            config.write_text(json.dumps({"version":1,"cameras":cameras}),encoding="utf-8")
            report = evidence.resolve() / (name + ".json")
            command = [str(exe), "--config", str(config), "--self-test-report", str(report)]
            try:
                result = subprocess.run(command, cwd=folder, env=env, timeout=420)
            except subprocess.TimeoutExpired:
                subprocess.run(["taskkill", "/IM", exe.name, "/T", "/F"], capture_output=True)
                raise
            assert report.is_file(), f"Frozen app did not write {name} report (exit {result.returncode})"
            data = json.loads(report.read_text(encoding="utf-8"))
            print(name, json.dumps(data))
            assert result.returncode == 0 and data["ok"] and data["frozen"] and data["bundled_adb"]
            assert data["workers_stopped"] and data["active_children"] == 0
            assert data["ai_runtime"]["ok"] and len(set(data["ai_runtime"]["bytetrack_ids"])) == 2, data["ai_runtime"]
            assert data["model_check"]["ok"], data["model_check"]
            assert len(data["cameras"]) == 2 and all(c["frames"] >= 5 for c in data["cameras"])

        run("file", video.name, "dual-file")
        ok, jpeg = cv2.imencode(".jpg", np.full((180,320,3),120,np.uint8))
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
                            str(len(payload)).encode()+b"\r\n\r\n"+payload+b"\r\n")
                        self.wfile.flush(); time.sleep(1/30)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass
        server = ThreadingHTTPServer(("127.0.0.1",0), Handler)
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever,daemon=True); thread.start()
        try:
            run("mjpeg", f"http://127.0.0.1:{server.server_port}/video", "dual-mjpeg")
        finally:
            server.shutdown(); server.server_close(); thread.join(timeout=2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("executable", type=Path)
    parser.add_argument("evidence", type=Path)
    args = parser.parse_args()
    main(args.executable.resolve(), args.evidence.resolve())
