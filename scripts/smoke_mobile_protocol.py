"""Real Android JVM HTTP server -> optional loopback relay -> OpenCV capture.

Uses the exact server shipped in the APK. Camera input/USB device transport are
substituted with a generated JPEG/local socket; this is not a physical device test.
"""
from pathlib import Path
import os
import socket
import subprocess
import sys
import tempfile
import time
from urllib.request import urlopen
from urllib.error import HTTPError

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "stages" / "01-camera-stream" / "src"))
from drone_camera.config import CameraConfig
from drone_camera.capture import CameraSource
from drone_camera.usb import UsbTunnel, companion_url


def main():
    with tempfile.TemporaryDirectory(prefix="drone-mobile-smoke-") as directory:
        work = Path(directory)
        jpeg = work / "frame.jpg"
        assert cv2.imwrite(str(jpeg), np.full((180, 320, 3), 120, np.uint8))
        java = ROOT / "apps/android-camera/app/src/main/java/com/dronecollision/camera"
        subprocess.run(["javac", "--release", "17", "-d", str(work), str(java/"FrameHub.java"),
                        str(java/"MjpegServer.java"), str(ROOT/"apps/android-camera/MjpegFixture.java")], check=True)
        process = subprocess.Popen(["java", "-cp", str(work), "MjpegFixture", str(jpeg)], stdout=subprocess.PIPE, text=True)
        relay = None
        try:
            url = process.stdout.readline().strip()
            parsed = companion_url(url)
            try:
                urlopen(url.replace(parsed.query, "token=wrong"), timeout=3)
                raise AssertionError("Unauthorized stream must be rejected")
            except HTTPError as error:
                assert error.code == 403
            relay = UsbTunnel("ios", parsed.port)
            # Substitute only the physical device transport; exercise real TCP relay.
            relay._connect = lambda: socket.create_connection(("127.0.0.1", parsed.port), timeout=3)
            for endpoint in (url, relay.rewrite(url)):
                source = CameraSource(CameraConfig("camera-01", "Companion", "mjpeg", endpoint))
                try:
                    source.connect()
                    deadline = time.monotonic() + 10
                    frame = None
                    while time.monotonic() < deadline:
                        source.poll()
                        frame = source.latest_frame()
                        if frame is not None:
                            break
                        time.sleep(0.02)
                    assert frame is not None and frame.resolution == (320,180)
                finally:
                    source.close()
            print("PASS: Android server auth + MJPEG -> OpenCV, direct and through bounded relay")
        finally:
            if relay is not None:
                relay.close(); relay.wait_closed()
            process.terminate(); process.wait(timeout=5)


if __name__ == "__main__":
    main()
