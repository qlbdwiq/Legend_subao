"""Resolve bundled tools without depending on a Python/SDK installation."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile


def adb_executable() -> str | None:
    if getattr(sys, "frozen", False) and os.name == "nt":
        bundle = Path(sys._MEIPASS) / "platform-tools"
        # adb can run a persistent shared server. Stage it outside PyInstaller's
        # temporary extraction directory so exit can clean up the single EXE.
        base = Path(os.environ.get("LOCALAPPDATA", str(Path.home() / "AppData" / "Local")))
        return str(stage_tools(bundle, base / "DroneCollisionDetection" / "Tools"))
    if getattr(sys, "frozen", False) and sys.platform == "darwin":
        # The macOS .app ships Google's universal (arm64 + x86_64) adb.
        bundled = Path(sys._MEIPASS) / "platform-tools" / "adb"
        if bundled.is_file() and os.access(bundled, os.X_OK):
            return str(bundled)
    return shutil.which("adb")


def stage_tools(bundle: Path, base: Path) -> Path:
    manifest = json.loads((bundle / "bundle.json").read_text(encoding="utf-8"))
    if Path(manifest["id"]).name != manifest["id"]:
        raise ValueError("Invalid bundled tool directory")
    cache = base / manifest["id"]
    cache.mkdir(parents=True, exist_ok=True)
    for name, expected in manifest["files"].items():
        if Path(name).name != name:
            raise ValueError("Invalid bundled tool filename")
        target = cache / name
        if target.is_file() and hashlib.sha256(target.read_bytes()).hexdigest() == expected:
            continue
        data = (bundle / name).read_bytes()
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError("Bundled Android tool integrity check failed")
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=cache, delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(data)
            os.replace(temporary, target)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    return cache / "adb.exe"
