"""Build a deterministic source ZIP from an explicit public-file allowlist.

No git traversal, virtualenvs, local settings, videos or credentials are included.
Run from any working directory: python scripts/package_release.py
"""
from pathlib import Path
import zipfile
import argparse
import sys

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / "stages" / "01-camera-stream"
sys.path.insert(0, str(STAGE / "src"))
from drone_camera import __version__


def package(output: Path | None = None, version: str = __version__) -> Path:
    output = output or ROOT / "dist" / f"phase-01-camera-stream-v{version}.zip"
    output.parent.mkdir(parents=True, exist_ok=True)
    paths = [ROOT / "docs" / "macos-app.md", ROOT / "LICENSE", ROOT / "README.md", ROOT / "docs" / "roadmap.md",
             ROOT / "docs" / "releases" / f"v{version}.md",
             ROOT / "docs" / "mobile" / "README.md",
             ROOT / "docs" / "windows-exe.md",
             ROOT / "scripts" / "make_test_video.py", ROOT / "scripts" / "package_release.py",
             STAGE / "requirements.txt", STAGE / "config.example.json",
             STAGE / "README.md", STAGE / "pytest.ini"]
    paths += sorted((STAGE / "src").rglob("*.py"))
    paths += sorted((STAGE / "tests").rglob("*.py"))
    with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as archive:
        for path in paths:
            relative = path.relative_to(ROOT)
            info = zipfile.ZipInfo(f"phase-01-camera-stream-v{version}/" + relative.as_posix(),
                                   date_time=(2026, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o644 << 16
            archive.writestr(info, path.read_bytes())
    print(output)
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--version", default=__version__, choices=[__version__])
    package(version=parser.parse_args().version)
