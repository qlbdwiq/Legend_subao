"""Build DroneCollisionDetection.app and a drag-to-Applications DMG on macOS.

Builds for the architecture of the machine running it (arm64 or x86_64); the
CI matrix runs it on both. The app is ad-hoc signed only (required for arm64 to
launch). It is NOT Developer ID signed or notarized: no Apple credentials are used.
"""
import hashlib
from importlib import metadata
import json
import os
import platform
from pathlib import Path
import plistlib
import shutil
import subprocess
import sys
from urllib.request import urlopen
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "stages/01-camera-stream/src"))
from drone_camera import __version__


def torchvision_binaries():
    """--add-binary args for torchvision's own extension libraries (e.g. _C_stable.so / *.pyd).

    PyInstaller's torchvision hook only knows the old ``torchvision._C`` name and its dynamic
    library patterns skip these files, so they would silently be missing from the bundle."""
    import importlib.util
    root = Path(importlib.util.find_spec("torchvision").submodule_search_locations[0])
    found = sorted(p for p in root.iterdir() if p.is_file() and p.suffix in (".so", ".pyd", ".dll", ".dylib"))
    if not found:
        raise RuntimeError("torchvision extension libraries not found; torchvision::nms would be missing")
    return [arg for p in found for arg in ("--add-binary", f"{p}:torchvision")]

APP_NAME = "DroneCollisionDetection"
BUNDLE_ID = "com.dronecollision.detection"


def main():
    if sys.platform != "darwin":
        raise SystemExit("macOS apps must be built on macOS. Use the macOS CI job.")
    arch = {"arm64": "arm64", "x86_64": "x64"}[platform.machine()]
    work = ROOT / "build" / "macos"
    dist = ROOT / "dist" / "macos"
    tools = work / "platform-tools"
    licenses = work / "licenses"
    for folder in (tools, licenses, dist):
        folder.mkdir(parents=True, exist_ok=True)
    pin = json.loads((ROOT / "packaging/macos/platform-tools.json").read_text())
    archive = work / "platform-tools.zip"
    if not archive.exists() or hashlib.sha256(archive.read_bytes()).hexdigest() != pin["sha256"]:
        with urlopen(pin["url"], timeout=60) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != pin["sha256"]:
            raise RuntimeError("Official Android platform-tools SHA256 mismatch")
        archive.write_bytes(data)
    with zipfile.ZipFile(archive) as source:
        for name in ("platform-tools/adb", "platform-tools/NOTICE.txt", "platform-tools/source.properties"):
            (tools / Path(name).name).write_bytes(source.read(name))
    (tools / "adb").chmod(0o755)
    for name in ("numpy", "opencv-python-headless", "PySide6", "PySide6-Essentials", "PySide6-Addons", "shiboken6", "pyinstaller",
                 "torch", "torchvision", "ultralytics", "ultralytics-thop", "lap", "pillow", "pyyaml", "psutil", "requests"):
        distribution = metadata.distribution(name)
        for file in distribution.files or []:
            if any(part.lower().startswith(("license", "copying", "notice", "copyright")) for part in file.parts):
                path = distribution.locate_file(file)
                if path.is_file():
                    destination = licenses / name / Path(*file.parts)
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(path, destination)
    shutil.rmtree(dist / f"{APP_NAME}.app", ignore_errors=True)
    subprocess.run([sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean", "--windowed", "--noupx",
                    "--name", APP_NAME, "--osx-bundle-identifier", BUNDLE_ID,
                    "--paths", str(ROOT / "stages/01-camera-stream/src"),
                    "--exclude-module", "pytest", "--exclude-module", "tkinter",
                    # Ultralytics loads model/tracker modules lazily; PyInstaller cannot see them statically.
                    "--collect-submodules", "ultralytics", "--copy-metadata", "ultralytics",
                    # torchvision >= 0.2x names its ops libraries *_stable.so/.pyd, which the bundled hook
                    # (it only knows torchvision._C) misses; without them torchvision::nms does not exist.
                    "--collect-binaries", "torchvision", *torchvision_binaries(),
                    "--add-data", f"{tools}:platform-tools", "--add-data", f"{licenses}:third-party-licenses",
                    "--distpath", str(dist), "--workpath", str(work / "pyinstaller"),
                    "--specpath", str(work), str(ROOT / "stages/01-camera-stream/src/main.py")], cwd=ROOT, check=True)
    app = dist / f"{APP_NAME}.app"
    # PyInstaller creates a one-dir bundle next to the .app; only the .app is shipped.
    shutil.rmtree(dist / APP_NAME, ignore_errors=True)
    plist_path = app / "Contents/Info.plist"
    info = plistlib.loads(plist_path.read_bytes())
    info.update({
        "CFBundleName": "Drone Collision Detection", "CFBundleDisplayName": "Drone Collision Detection",
        "CFBundleShortVersionString": __version__, "CFBundleVersion": __version__,
        "LSMinimumSystemVersion": "11.0", "NSHighResolutionCapable": True,
        "NSCameraUsageDescription": "USB 웹캠 등 컴퓨터에 연결된 카메라 영상을 표시하기 위해 카메라 접근이 필요합니다.",
        "NSLocalNetworkUsageDescription": "같은 Wi-Fi의 휴대폰 카메라 앱(MJPEG/RTSP) 영상을 받기 위해 로컬 네트워크 접근이 필요합니다.",
    })
    plist_path.write_bytes(plistlib.dumps(info))
    # Ad-hoc signature only: needed so arm64 will launch the bundle. Not Developer ID / notarized.
    subprocess.run(["codesign", "--force", "--deep", "--sign", "-", str(app)], check=True)
    subprocess.run(["codesign", "--verify", "--deep", "--strict", str(app)], check=True)

    stage = work / "dmg"
    shutil.rmtree(stage, ignore_errors=True)
    stage.mkdir(parents=True)
    subprocess.run(["ditto", str(app), str(stage / app.name)], check=True)
    os.symlink("/Applications", stage / "Applications")
    shutil.copyfile(ROOT / "docs/macos-app.md", stage / "README-KO.md")
    shutil.copyfile(ROOT / "LICENSE", stage / "LICENSE")
    dmg = dist / f"{APP_NAME}-v{__version__}-macos-{arch}.dmg"
    dmg.unlink(missing_ok=True)
    subprocess.run(["hdiutil", "create", "-volname", "Drone Collision Detection", "-srcfolder", str(stage),
                    "-ov", "-format", "UDZO", str(dmg)], check=True)
    checksums = dist / f"SHA256SUMS-macos-{arch}.txt"
    checksums.write_text(hashlib.sha256(dmg.read_bytes()).hexdigest() + "  " + dmg.name + "\n", encoding="utf-8")
    print("macOS files:", dmg.name, "arch", arch)


if __name__ == "__main__":
    main()
