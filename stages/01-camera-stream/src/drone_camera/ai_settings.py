"""Validated AI settings in their own JSON file, next to config.json.

Kept separate from the camera configuration so existing config files (version 1)
and their validation stay unchanged. The model path is chosen by the user and is
never hard-coded; a moved or deleted model is reported when it is loaded.
"""

from dataclasses import asdict, dataclass, fields
import json
import math
import os
from pathlib import Path
import tempfile

from .geometry import DEFAULT_SAFETY_FACTOR, GeometryError, validate_safety_factor

IMAGE_SIZES = (320, 480, 640, 960, 1280)   # YOLO strides require multiples of 32.
DEVICES = ("auto", "cpu", "cuda", "mps")
AI_SETTINGS_NAME = "ai_settings.json"


class AiSettingsError(ValueError):
    pass


@dataclass(frozen=True)
class AiSettings:
    model_path: str = ""
    enabled: bool = False
    confidence: float = 0.25
    iou: float = 0.5           # 0.7 let the project model emit a nested duplicate box on one drone.
    imgsz: int = 960           # Matches the training resolution of the project model.
    device: str = "auto"
    safety_factor: float = DEFAULT_SAFETY_FACTOR
    show_box: bool = False
    show_circle: bool = True
    show_id: bool = True
    show_confidence: bool = True

    def __post_init__(self):
        if not isinstance(self.model_path, str) or len(self.model_path) > 4096 or "\x00" in self.model_path:
            raise AiSettingsError("Model path must be a text path.")
        for name in ("enabled", "show_box", "show_circle", "show_id", "show_confidence"):
            if type(getattr(self, name)) is not bool:
                raise AiSettingsError(f"{name} must be a boolean.")
        for name, low, high in (("confidence", 0.05, 0.95), ("iou", 0.1, 0.95)):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not math.isfinite(value) or not low <= value <= high:
                raise AiSettingsError(f"{name} must be a number from {low} to {high}.")
        if type(self.imgsz) is not int or self.imgsz not in IMAGE_SIZES:
            raise AiSettingsError(f"Inference size must be one of {IMAGE_SIZES}.")
        if self.device not in DEVICES:
            raise AiSettingsError(f"Device must be one of {DEVICES}.")
        try:
            validate_safety_factor(self.safety_factor)
        except GeometryError as error:
            raise AiSettingsError(str(error)) from None


def settings_path_for(config_path: Path) -> Path:
    return config_path.with_name(AI_SETTINGS_NAME)


def load_ai_settings(path: Path) -> AiSettings:
    """Missing file means defaults; a damaged file raises AiSettingsError."""
    if not path.exists():
        return AiSettings()
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or type(data.get("version")) is not int or data["version"] != 1:
            raise AiSettingsError("Expected AI settings version 1.")
        known = {f.name for f in fields(AiSettings)}
        return AiSettings(**{k: v for k, v in data.items() if k in known})
    except AiSettingsError:
        raise
    except (OSError, ValueError, TypeError):
        raise AiSettingsError("Cannot read AI settings. Check the JSON file.") from None


def save_ai_settings(path: Path, settings: AiSettings) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=".ai-", suffix=".tmp", delete=False) as stream:
            tmp = Path(stream.name)
            json.dump({"version": 1, **asdict(settings)}, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, path)
    finally:
        if tmp is not None:
            tmp.unlink(missing_ok=True)
