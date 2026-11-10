"""Validated, versioned configuration. Never log stream addresses or credentials."""

from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import tempfile
from urllib.parse import urlsplit


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class CameraConfig:
    id: str
    name: str
    input_type: str
    source: str | int
    timeout_seconds: float = 5.0
    auto_reconnect: bool = True
    reconnect_delay_seconds: float = 2.0
    max_reconnect_attempts: int = 5
    loop_file: bool = False

    def __post_init__(self):
        if self.id not in ("camera-01", "camera-02"):
            raise ConfigError("Camera id must be camera-01 or camera-02.")
        if not isinstance(self.name, str) or not self.name.strip() or len(self.name) > 80:
            raise ConfigError("Camera name must contain 1–80 characters.")
        if self.input_type not in ("mjpeg", "rtsp", "usb", "file"):
            raise ConfigError("Input type must be mjpeg, rtsp, usb or file.")
        if self.input_type == "usb":
            if type(self.source) is not int or not 0 <= self.source <= 99:
                raise ConfigError("USB device number must be an integer from 0 to 99.")
        elif not isinstance(self.source, str) or not self.source.strip():
            raise ConfigError("A stream address or video file path is required.")
        elif self.input_type in ("mjpeg", "rtsp"):
            try:
                url = urlsplit(self.source)
                valid = url.hostname and url.port != 0 and url.scheme in (
                    ("http", "https") if self.input_type == "mjpeg" else ("rtsp",)
                )
            except ValueError:
                valid = False
            if not valid:
                raise ConfigError("Enter an HTTP(S) MJPEG or RTSP URL matching the input type.")
        for label, value, low, high in (
            ("Timeout", self.timeout_seconds, 0.2, 60),
            ("Reconnect delay", self.reconnect_delay_seconds, 0.1, 60),
        ):
            if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                raise ConfigError(f"{label} must be a finite number from {low} to {high} seconds.")
        if type(self.max_reconnect_attempts) is not int or not 0 <= self.max_reconnect_attempts <= 100:
            raise ConfigError("Reconnect attempts must be an integer from 0 to 100.")
        if type(self.auto_reconnect) is not bool or type(self.loop_file) is not bool:
            raise ConfigError("Reconnect and file loop settings must be booleans.")


def defaults() -> list[CameraConfig]:
    return [CameraConfig(f"camera-{i:02d}", f"Camera {i:02d}", "usb", i - 1) for i in (1, 2)]


def load_config(path: Path) -> list[CameraConfig]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or type(data.get("version")) is not int or data["version"] != 1:
            raise ConfigError("Expected configuration version 1.")
        items = data.get("cameras")
        if not isinstance(items, list) or len(items) != 2:
            raise ConfigError("Exactly two camera configurations are required.")
        configs = [CameraConfig(**item) for item in items]
        if {c.id for c in configs} != {"camera-01", "camera-02"}:
            raise ConfigError("Camera IDs must be unique.")
        return sorted(configs, key=lambda c: c.id)
    except ConfigError:
        raise
    except (OSError, ValueError, TypeError):
        raise ConfigError("Cannot read configuration. Check JSON format and camera fields.") from None


def save_config(path: Path, cameras: list[CameraConfig]) -> None:
    if len(cameras) != 2 or {c.id for c in cameras} != {"camera-01", "camera-02"}:
        raise ConfigError("Exactly two unique camera IDs are required.")
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=".camera-", suffix=".tmp", delete=False) as stream:
            tmp = Path(stream.name)
            json.dump({"version": 1, "cameras": [asdict(c) for c in cameras]}, stream, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, path)
    finally:
        if tmp is not None:
            tmp.unlink(missing_ok=True)


def resolved_config(config: CameraConfig, directory: Path) -> CameraConfig:
    """Relative video paths are relative to the config file, not the shell cwd."""
    from dataclasses import replace
    if config.input_type == "file":
        path = Path(str(config.source)).expanduser()
        if not path.is_absolute():
            path = directory / path
        return replace(config, source=str(path.resolve()))
    return config
