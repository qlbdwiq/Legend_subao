"""Package a compiled unsigned iPhone app and a project for Personal Team signing.

An unsigned IPA is NOT installable. Signing happens in Xcode on the user's Mac.
"""
from pathlib import Path
import argparse
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def package(app: Path, output: Path):
    if not app.is_dir() or not (app / "Info.plist").is_file() or not (app / "DroneCamera").is_file():
        raise ValueError("A real compiled DroneCamera.app is required.")
    output.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output / "drone-camera-ios-v0.2.0-unsigned.ipa", "w", zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(app.rglob("*")):
            if path.is_file():
                archive.write(path, "Payload/DroneCamera.app/" + path.relative_to(app).as_posix())
    project = ROOT / "apps" / "ios-camera"
    if not (project / "DroneCamera.xcodeproj" / "project.pbxproj").is_file():
        raise ValueError("Generate the Xcode project before packaging.")
    with zipfile.ZipFile(output / "drone-camera-ios-v0.2.0-xcode-project.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(project.rglob("*")):
            if path.is_file() and not any(p in ("xcuserdata", "DerivedData", "build", ".DS_Store") for p in path.parts):
                archive.write(path, "DroneCamera-iOS/" + path.relative_to(project).as_posix())
        archive.writestr("DroneCamera-iOS/INSTALL.txt", "Open DroneCamera.xcodeproj in Xcode on your Mac.\nSelect Signing & Capabilities > Personal Team, set a unique bundle identifier, select your trusted iPhone, and Run.\nThe unsigned IPA is NOT directly installable.\n")
        archive.write(ROOT / "docs" / "mobile" / "README.md", "DroneCamera-iOS/INSTALL-KO.md")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("app", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    package(args.app, args.output)
