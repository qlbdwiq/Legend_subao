"""Device stream capabilities: supported resolution/FPS combinations.

A phone's still/video recording maximum is NOT its live MJPEG streaming
maximum. Modes in this module always describe the live streaming pipeline:
on the phones they are enumerated from Camera2/AVFoundation streaming
formats, on the desktop they describe what the receiver can ingest.

The desktop ingest buffer is fixed at 3840x2160 BGR (see capture.py), so
modes above that are dropped here and reported as pipeline limits rather
than silently accepted.
"""

from dataclasses import dataclass
import json
from urllib.parse import urlsplit, urlunsplit
from urllib.request import Request, urlopen

MAX_STREAM_WIDTH = 3840
MAX_STREAM_HEIGHT = 2160

STANDARD_RESOLUTIONS = ((640, 480), (1280, 720), (1920, 1080),
                        (2560, 1440), (3840, 2160))
STANDARD_FPS = (15, 24, 30, 60, 120, 240)


class CapabilityError(RuntimeError):
    """Raised when a device capability query fails; never contains tokens."""


@dataclass(frozen=True, order=True)
class StreamMode:
    width: int
    height: int
    max_fps: int

    @property
    def pixels(self) -> int:
        return self.width * self.height

    @property
    def label(self) -> str:
        return f"{self.width} × {self.height}"


def fps_options(mode: StreamMode) -> tuple[int, ...]:
    """Standard frame rates a mode can actually sustain, plus its exact max."""
    options = [fps for fps in STANDARD_FPS if fps <= mode.max_fps]
    if mode.max_fps >= 1 and mode.max_fps not in options:
        options.append(mode.max_fps)
    return tuple(sorted(options))


def normalize_modes(modes) -> tuple[StreamMode, ...]:
    """Dedupe by resolution (keeping max fps), drop pipeline-oversize modes."""
    best: dict[tuple[int, int], StreamMode] = {}
    for mode in modes:
        if not (16 <= mode.width <= MAX_STREAM_WIDTH
                and 16 <= mode.height <= MAX_STREAM_HEIGHT):
            continue
        if mode.width % 2 or mode.height % 2 or mode.max_fps < 1:
            continue
        key = (mode.width, mode.height)
        if key not in best or mode.max_fps > best[key].max_fps:
            best[key] = mode
    return tuple(sorted(best.values(), key=lambda m: (m.pixels, m.max_fps),
                        reverse=True))


def is_supported(modes, width: int, height: int, fps: int) -> bool:
    """True only for an enumerated resolution whose fps ceiling covers fps."""
    if width == 0 and height == 0 and fps == 0:
        return True  # Auto always defers to the device.
    for mode in modes:
        if mode.width == width and mode.height == height:
            return fps == 0 or 1 <= fps <= mode.max_fps
    return False


def best_resolution_mode(modes) -> StreamMode | None:
    modes = normalize_modes(modes)
    return modes[0] if modes else None


def best_fps_mode(modes) -> StreamMode | None:
    modes = normalize_modes(modes)
    if not modes:
        return None
    return max(modes, key=lambda m: (m.max_fps, m.pixels))


def parse_capabilities(payload: bytes) -> tuple[StreamMode, ...]:
    """Parse a phone /capabilities JSON document into normalized modes."""
    try:
        data = json.loads(payload.decode("utf-8"))
        items = data["modes"]
        modes = [StreamMode(int(item["width"]), int(item["height"]),
                            int(item["max_fps"])) for item in items]
    except (ValueError, KeyError, TypeError, UnicodeDecodeError):
        raise CapabilityError("기기 지원 범위 응답을 읽을 수 없습니다.") from None
    return normalize_modes(modes)


def capabilities_url_for(source: str) -> str:
    """Derive the authenticated /capabilities URL from a stream URL."""
    url = urlsplit(source)
    if url.scheme not in ("http", "https") or not url.hostname:
        raise CapabilityError("지원 범위 확인은 HTTP(S) 스트림에서만 가능합니다.")
    return urlunsplit((url.scheme, url.netloc, "/capabilities", url.query, ""))


def fetch_capabilities(source: str, timeout: float = 3.0) -> tuple[StreamMode, ...]:
    """Query a companion phone app for its live-streaming modes.

    Only the phone's own /capabilities endpoint is contacted. Errors are
    reported without the URL so stream tokens never reach logs or dialogs
    beyond a generic message.
    """
    try:
        request = Request(capabilities_url_for(source),
                          headers={"Accept": "application/json"})
        with urlopen(request, timeout=timeout) as response:
            return parse_capabilities(response.read(65536))
    except CapabilityError:
        raise
    except Exception:
        raise CapabilityError(
            "기기 지원 범위를 가져올 수 없습니다. 휴대폰 송출 시작과 주소를 확인하세요."
        ) from None


def probe_usb_modes(device_index: int) -> tuple[StreamMode, ...]:
    """Probe a desktop USB webcam by actually applying each resolution.

    OpenCV's reported FPS after set() is a request echo on many backends,
    so max_fps here is the backend-reported value; the trustworthy figure
    is the measured receive FPS shown after connecting.
    """
    import cv2
    capture = cv2.VideoCapture(device_index)
    modes = []
    try:
        if not capture.isOpened():
            raise CapabilityError("USB 카메라를 열 수 없습니다. 장치 번호와 권한을 확인하세요.")
        for width, height in STANDARD_RESOLUTIONS:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            capture.set(cv2.CAP_PROP_FPS, 240)
            ok, frame = capture.read()
            if not ok or frame is None:
                continue
            actual_h, actual_w = frame.shape[:2]
            reported = capture.get(cv2.CAP_PROP_FPS)
            max_fps = int(reported) if reported and 1 <= reported <= 1000 else 30
            modes.append(StreamMode(int(actual_w), int(actual_h), max_fps))
    finally:
        capture.release()
    result = normalize_modes(modes)
    if not result:
        raise CapabilityError("USB 카메라에서 읽을 수 있는 해상도가 없습니다.")
    return result
