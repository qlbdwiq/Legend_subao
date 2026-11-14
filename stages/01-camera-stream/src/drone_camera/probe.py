"""Cancellable capability queries, isolated from Qt and blocking native I/O."""

import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import time

from .capabilities import (
    CapabilityError, StreamMode, fetch_capabilities, normalize_modes, probe_usb_modes,
)
from .usb import UsbError, UsbTunnel, companion_url


def _probe_worker(kind, source, http_timeout, result, length):
    # Native diagnostics must not print URLs or credentials.
    os.environ["OPENCV_FFMPEG_LOGLEVEL"] = "-8"
    try:
        import cv2
        cv2.setLogLevel(0)
        if kind == "mjpeg":
            modes = fetch_capabilities(source, timeout=http_timeout)
        elif kind == "usb":
            modes = probe_usb_modes(int(source))
        elif kind == "file":
            capture = cv2.VideoCapture(source)
            try:
                if not capture.isOpened():
                    raise CapabilityError("동영상 파일을 열 수 없습니다.")
                width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = capture.get(cv2.CAP_PROP_FPS)
                fps = int(fps) if math.isfinite(fps) and fps >= 1 else 30
                modes = (StreamMode(width, height, fps),)
            finally:
                capture.release()
        else:
            raise CapabilityError("RTSP는 지원 범위 조회 규격이 없어 실제 수신값으로 확인합니다.")
        modes = normalize_modes(modes)
        if not modes:
            raise CapabilityError("수신기가 지원하는 해상도/FPS 조합이 없습니다.")
        payload = {"modes": [[m.width, m.height, m.max_fps] for m in modes]}
    except CapabilityError as error:
        payload = {"error": str(error)}
    except Exception:
        payload = {"error": "지원 범위를 확인할 수 없습니다. 장치와 입력 설정을 확인하세요."}
    encoded = json.dumps(payload).encode("utf-8")
    if len(encoded) > len(result):
        encoded = json.dumps({"error": "기기 지원 범위 응답이 너무 큽니다."}).encode("utf-8")
    result[:len(encoded)] = encoded
    # The synchronized length publishes a complete bounded result. No pipe read
    # can block the GUI if a worker dies partway through writing its response.
    length.value = len(encoded)


class CapabilityProbe:
    """One short-lived process and, for phones, an owned USB relay.

    The relay stays in the parent so cancellation also removes adb forwarding
    when a worker must be killed. poll() never waits for native camera reads.
    """

    def __init__(self, kind, source, usb_platform="", serial="", directory=None,
                 timeout=30.0):
        self._process = None
        self._tunnel = None
        self._deadline = time.monotonic() + timeout
        ctx = mp.get_context("spawn")
        self._result = ctx.RawArray("B", 65536)
        self._length = ctx.Value("i", 0)
        try:
            if usb_platform:
                if kind != "mjpeg":
                    raise CapabilityError("휴대폰 USB 연결은 MJPEG 입력을 선택하세요.")
                parsed = companion_url(source)
                self._tunnel = UsbTunnel(usb_platform, parsed.port, serial)
                source = self._tunnel.rewrite(source)
            elif kind == "file":
                path = Path(source).expanduser()
                source = str(((directory or Path.cwd()) / path).resolve())
            http_timeout = 20.0 if usb_platform else 3.0
            process = ctx.Process(target=_probe_worker,
                args=(kind, source, http_timeout, self._result, self._length),
                daemon=True, name="camera-capabilities")
            process.start()
            self._process = process
        except (CapabilityError, UsbError):
            self.close()
            raise
        except Exception:
            self.close()
            raise CapabilityError("지원 범위 조회를 시작할 수 없습니다.") from None

    def poll(self):
        """Return modes when ready, None while waiting, or a sanitized error."""
        length = self._length.value
        if length:
            payload = json.loads(bytes(self._result[:length]))
            self.close()
            if "error" in payload:
                raise CapabilityError(payload["error"])
            return tuple(StreamMode(*item) for item in payload["modes"])
        if self._process is None:
            return None
        if time.monotonic() >= self._deadline:
            self.close()
            raise CapabilityError("지원 범위 조회 시간이 초과되었습니다. 장치 연결을 확인하세요.")
        if not self._process.is_alive():
            # Check the published result again on the next event-loop turn.
            if self._length.value:
                return None
            self.close()
            raise CapabilityError("지원 범위 조회가 중단되었습니다. 다시 시도하세요.")
        return None

    def close(self):
        if self._process is not None:
            process = self._process
            process.join(timeout=0)
            if process.is_alive():
                process.terminate()
                process.join(timeout=0.2)
            if process.is_alive():
                process.kill()
                process.join(timeout=1)
            process.close()
            self._process = None
        if self._tunnel is not None:
            # adb cleanup runs on the relay's own thread, never on Qt's thread.
            self._tunnel.close()
