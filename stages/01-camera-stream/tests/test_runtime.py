import hashlib
import json

import pytest

from drone_camera.runtime import stage_tools


def bundle(tmp_path):
    source = tmp_path / "extracted"
    source.mkdir()
    (source / "adb.exe").write_bytes(b"test tool bytes")
    manifest = {"id": "tools-test", "files": {"adb.exe": hashlib.sha256(b"test tool bytes").hexdigest()}}
    (source / "bundle.json").write_text(json.dumps(manifest))
    return source


def test_cache_survives_bundle_removal_and_repairs_corrupt_copy(tmp_path):
    source = bundle(tmp_path)
    target = stage_tools(source, tmp_path / "user-cache")
    assert target.read_bytes() == b"test tool bytes"
    target.write_bytes(b"corrupted cache")
    assert stage_tools(source, tmp_path / "user-cache").read_bytes() == b"test tool bytes"
    (source / "adb.exe").unlink()
    assert stage_tools(source, tmp_path / "user-cache") == target  # Reuse verified cached binary.


def test_reject_modified_embedded_tool(tmp_path):
    source = bundle(tmp_path)
    (source / "adb.exe").write_bytes(b"changed")
    with pytest.raises(ValueError, match="integrity"):
        stage_tools(source, tmp_path / "user-cache")


def test_reject_manifest_path_escape(tmp_path):
    source = bundle(tmp_path)
    (source / "bundle.json").write_text(json.dumps({"id":"../outside", "files":{}}))
    with pytest.raises(ValueError):
        stage_tools(source, tmp_path / "user-cache")
