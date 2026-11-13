import json

import pytest

from drone_camera.ai_settings import (
    AiSettings, AiSettingsError, load_ai_settings, save_ai_settings, settings_path_for)
from pathlib import Path


def test_defaults_match_documented_values():
    s = AiSettings()
    assert (s.confidence, s.iou, s.imgsz, s.device, s.safety_factor) == (0.25, 0.5, 960, "auto", 1.2)
    assert s.enabled is False and s.model_path == ""


def test_roundtrip_keeps_model_path_and_options(tmp_path):
    path = tmp_path / "ai_settings.json"
    original = AiSettings(model_path=str(tmp_path / "한글 폴더" / "best.pt"), enabled=True,
                          confidence=0.4, iou=0.6, imgsz=640, device="cpu", safety_factor=1.5,
                          show_box=True, show_circle=False, show_id=False, show_confidence=False)
    save_ai_settings(path, original)
    assert load_ai_settings(path) == original
    assert [p.name for p in tmp_path.iterdir()] == ["ai_settings.json"]   # no temp leftovers


def test_missing_file_gives_defaults_and_path_is_next_to_config(tmp_path):
    assert load_ai_settings(tmp_path / "none.json") == AiSettings()
    assert settings_path_for(Path("/x/config.json")) == Path("/x/ai_settings.json")


@pytest.mark.parametrize("override", [
    {"confidence": 0.0}, {"confidence": 1.0}, {"confidence": float("nan")}, {"iou": 2},
    {"imgsz": 100}, {"imgsz": "960"}, {"device": "tpu"}, {"safety_factor": 0.5},
    {"safety_factor": 9}, {"enabled": "yes"}, {"show_box": 1}, {"model_path": 5},
    {"model_path": "a\x00b"}])
def test_invalid_values_are_rejected(override):
    with pytest.raises(AiSettingsError):
        AiSettings(**override)


def test_damaged_file_reports_error_and_unknown_keys_are_ignored(tmp_path):
    path = tmp_path / "ai_settings.json"
    path.write_text("{not json")
    with pytest.raises(AiSettingsError):
        load_ai_settings(path)
    path.write_text(json.dumps({"version": 2}))
    with pytest.raises(AiSettingsError):
        load_ai_settings(path)
    path.write_text(json.dumps({"version": 1, "confidence": 0.3, "future_option": True}))
    assert load_ai_settings(path).confidence == 0.3
    path.write_text(json.dumps({"version": 1, "confidence": 7}))
    with pytest.raises(AiSettingsError):
        load_ai_settings(path)
