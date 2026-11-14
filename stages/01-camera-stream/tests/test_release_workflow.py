"""Exercise the Linux release shell with local artifacts and a fake gh CLI."""

import hashlib
import os
from pathlib import Path
import subprocess
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]
pytestmark = pytest.mark.skipif(
    sys.platform == "win32" or not (ROOT / ".github/workflows/release.yml").is_file(),
    reason="Linux/bash release automation is available in the Git checkout",
)


@pytest.fixture
def release_job(tmp_path):
    workflow = yaml.safe_load((ROOT / ".github/workflows/release.yml").read_text())
    steps = workflow["jobs"]["publish"]["steps"]
    commands = {step.get("name"): step["run"] for step in steps if "run" in step}
    # Resolve the mobile files from the actual producer output names. The
    # desktop release version intentionally differs from the mobile version.
    mobile_names = ("drone-camera-android-v0.4.0-debug.apk",
                    "drone-camera-ios-v0.4.0-unsigned.ipa",
                    "drone-camera-ios-v0.4.0-xcode-project.zip")
    for folder in ("dist", "mobile-dist", "checksums", "docs/releases", "bin"):
        (tmp_path / folder).mkdir(parents=True)
    for name in ("DroneCollisionDetection-v0.6.0-windows-x64.exe",
                 "DroneCollisionDetection-v0.6.0-windows-x64.zip",
                 "DroneCollisionDetection-v0.6.0-macos-arm64.dmg",
                 "DroneCollisionDetection-v0.6.0-macos-x64.dmg",
                 "phase-01-camera-stream-v0.6.0.zip", "SHA256SUMS.txt"):
        (tmp_path / "dist" / name).write_bytes(b"desktop fixture\n")
    (tmp_path / "checksums/SHA256SUMS-macos-arm64.txt").write_text("mac checksum fixture\n")
    (tmp_path / "docs/releases/v0.6.0.md").write_text("# Test release\n")
    for name in mobile_names:
        (tmp_path / "mobile-dist" / name).write_bytes(name.encode())
    gh = tmp_path / "bin/gh"
    gh.write_text('#!/bin/sh\nif [ "$2" = view ]; then exit 1; fi\nprintf "%s\\n" "$@" > "$RELEASE_ARGS"\n')
    gh.chmod(0o755)
    env = {**os.environ, "PATH": str(gh.parent) + os.pathsep + os.environ["PATH"],
           "VERSION": "0.6.0", "PHASE": "01", "RELEASE_TAG": "v0.6.0",
           "RELEASE_ARGS": str(tmp_path / "release-args.txt")}
    return tmp_path, steps, commands, mobile_names, env


def test_release_attaches_downloaded_mobile_artifacts_and_correct_checksums(release_job):
    work, steps, commands, mobile_names, env = release_job
    producer = yaml.safe_load((ROOT / ".github/workflows/mobile.yml").read_text())
    for platform in ("android", "ios"):
        artifact = next(step["with"]["name"] for step in producer["jobs"][platform]["steps"]
                        if step.get("uses") == "actions/upload-artifact@v4")
        assert any(step.get("uses") == "actions/download-artifact@v4"
                   and step["with"].get("name") == artifact for step in steps)
    subprocess.run(["bash", "-euo", "pipefail", "-c", commands["Combine checksums"]],
                   cwd=work, env=env, check=True, capture_output=True)
    checksums = (work / "dist/SHA256SUMS.txt").read_text()
    for name in mobile_names:
        digest = hashlib.sha256((work / "mobile-dist" / name).read_bytes()).hexdigest()
        assert digest + "  " + name in checksums
    subprocess.run(["bash", "-euo", "pipefail", "-c",
                    commands["Publish release without overwriting an existing one"]],
                   cwd=work, env=env, check=True, capture_output=True)
    arguments = (work / "release-args.txt").read_text().splitlines()
    assert all("mobile-dist/" + name in arguments for name in mobile_names)


def test_release_refuses_to_publish_when_mobile_file_is_missing(release_job):
    work, _, commands, mobile_names, env = release_job
    (work / "mobile-dist" / mobile_names[1]).unlink()
    result = subprocess.run(["bash", "-euo", "pipefail", "-c",
                             commands["Publish release without overwriting an existing one"]],
                            cwd=work, env=env, capture_output=True)
    assert result.returncode != 0
    assert not (work / "release-args.txt").exists()
