from dataclasses import asdict, replace
import json
from pathlib import Path

import pytest

from drone_camera.app import main
from drone_camera.config import CameraConfig, ConfigError, defaults, load_config, resolved_config, save_config


def test_example_and_cli():
    example = Path(__file__).parents[1] / "config.example.json"
    assert len(load_config(example)) == 2
    assert main(["--validate-config", str(example)]) == 0


def test_roundtrip_and_atomic_overwrite(tmp_path):
    path = tmp_path / "nested" / "config.json"
    configs = defaults()
    configs[0] = replace(configs[0], name="iPhone / Android 카메라", input_type="mjpeg",
                         source="http://127.0.0.1:8080/video")
    save_config(path, configs)
    assert load_config(path) == configs
    save_config(path, defaults())
    assert load_config(path) == defaults()
    assert list(path.parent.iterdir()) == [path]


@pytest.mark.parametrize("patch", [
    {"input_type": "lidar"}, {"source": -1}, {"source": True}, {"source": "0"},
    {"timeout_seconds": 0}, {"timeout_seconds": float("nan")}, {"timeout_seconds": True},
    {"reconnect_delay_seconds": -1}, {"max_reconnect_attempts": 1.5},
    {"auto_reconnect": "yes"}, {"loop_file": 1}, {"id": "unknown"}, {"name": " "},
    {"input_type": "rtsp", "source": "http://host/video"},
    {"input_type": "mjpeg", "source": "http://host:bad/video"},
    {"input_type": "file", "source": ""},
])
def test_reject_invalid_values(patch):
    with pytest.raises(ConfigError):
        replace(defaults()[0], **patch)


@pytest.mark.parametrize("content", ["{", "[]", '{"version": true}', '{"version":2}',
    '{"version":1,"cameras":[]}', '{"version":1,"cameras":[null,null]}'])
def test_malformed_configuration(tmp_path, content):
    path = tmp_path / "config.json"
    path.write_text(content)
    with pytest.raises(ConfigError):
        load_config(path)
    assert main(["--validate-config", str(path)]) == 2


def test_duplicate_ids_and_unknown_fields(tmp_path):
    config = asdict(defaults()[0])
    path = tmp_path / "config.json"
    path.write_text(json.dumps({"version": 1, "cameras": [config, config]}))
    with pytest.raises(ConfigError):
        load_config(path)
    config["password"] = "not-a-real-password"
    path.write_text(json.dumps({"version": 1, "cameras": [config, asdict(defaults()[1])]}))
    with pytest.raises(ConfigError) as error:
        load_config(path)
    assert "not-a-real-password" not in str(error.value)


def test_relative_file_resolved_against_configuration(tmp_path):
    config = replace(defaults()[0], input_type="file", source="videos/test.avi")
    assert resolved_config(config, tmp_path).source == str(tmp_path / "videos" / "test.avi")
