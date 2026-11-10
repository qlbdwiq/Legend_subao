import json

import pytest

from drone_camera.capabilities import (
    StreamMode,
    best_fps_mode,
    best_resolution_mode,
    capabilities_url_for,
    fps_options,
    is_supported,
    normalize_modes,
    parse_capabilities,
)
from drone_camera.config import CameraConfig, ConfigError, defaults, load_config, save_config
from dataclasses import replace


def test_fps_options_are_per_resolution():
    assert fps_options(StreamMode(3840, 2160, 60)) == (15, 24, 30, 60)
    assert fps_options(StreamMode(1920, 1080, 240)) == (15, 24, 30, 60, 120, 240)
    assert fps_options(StreamMode(1280, 720, 30)) == (15, 24, 30)


def test_supported_combinations_distinguish_resolution():
    modes = [StreamMode(3840, 2160, 60), StreamMode(1920, 1080, 240)]
    assert is_supported(modes, 3840, 2160, 60)
    assert not is_supported(modes, 3840, 2160, 120)
    assert is_supported(modes, 1920, 1080, 240)
    assert not is_supported(modes, 2560, 1440, 30)


def test_best_modes():
    modes = normalize_modes([StreamMode(1920, 1080, 240), StreamMode(3840, 2160, 60),
                             StreamMode(3840, 2160, 30), StreamMode(1280, 720, 30)])
    assert best_resolution_mode(modes) == StreamMode(3840, 2160, 60)
    assert best_fps_mode(modes) == StreamMode(1920, 1080, 240)
    # Duplicate resolutions keep the highest fps.
    assert len([m for m in modes if m.width == 3840]) == 1


def test_normalize_drops_oversize_and_odd_modes():
    modes = normalize_modes([StreamMode(7680, 4320, 30), StreamMode(1280, 721, 30),
                             StreamMode(640, 480, 30)])
    assert modes == (StreamMode(640, 480, 30),)


def test_parse_phone_capabilities_json():
    payload = json.dumps({"version": 1, "facing": "back", "modes": [
        {"width": 1920, "height": 1080, "max_fps": 120},
        {"width": 3840, "height": 2160, "max_fps": 60},
    ]}).encode()
    modes = parse_capabilities(payload)
    assert StreamMode(3840, 2160, 60) in modes
    assert StreamMode(1920, 1080, 120) in modes


def test_capabilities_url_keeps_token_but_targets_capabilities_path():
    url = capabilities_url_for("http://192.168.0.10:8080/stream.mjpg?token=" + "a" * 32)
    assert url.startswith("http://192.168.0.10:8080/capabilities?token=")


def test_requested_stream_settings_roundtrip_and_defaults(tmp_path):
    path = tmp_path / "config.json"
    configs = defaults()
    configs[0] = replace(configs[0], requested_width=1920, requested_height=1080,
                         requested_fps=120, facing="back")
    save_config(path, configs)
    loaded = load_config(path)
    assert loaded[0].requested_width == 1920
    assert loaded[0].requested_fps == 120
    assert loaded[0].facing == "back"
    assert loaded[1].requested_width == 0  # Auto.


def test_old_config_without_stream_fields_still_loads(tmp_path):
    path = tmp_path / "config.json"
    path.write_text(json.dumps({"version": 1, "cameras": [
        {"id": "camera-01", "name": "Camera 01", "input_type": "usb", "source": 0},
        {"id": "camera-02", "name": "Camera 02", "input_type": "usb", "source": 1},
    ]}))
    loaded = load_config(path)
    assert loaded[0].requested_fps == 0
    assert loaded[0].facing == ""


@pytest.mark.parametrize("patch", [
    {"requested_width": 1920},                      # Height missing.
    {"requested_width": 1920, "requested_height": 1080, "requested_fps": 999},
    {"requested_width": 7680, "requested_height": 4320},
    {"requested_width": 1280, "requested_height": 721},
    {"requested_fps": -1},
    {"facing": "side"},
])
def test_reject_invalid_requested_stream_settings(patch):
    with pytest.raises(ConfigError):
        replace(defaults()[0], **patch)
