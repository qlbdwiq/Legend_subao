"""Build a single Windows EXE including Python, Qt, OpenCV and official adb."""
import hashlib
from importlib import metadata
import json
from pathlib import Path
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
    return [arg for p in found for arg in ("--add-binary", f"{p};torchvision")]


def main():
    if sys.platform != "win32":
        raise SystemExit("Windows EXEs must be built on Windows. Use the Windows CI job.")
    work = ROOT / "build" / "windows"
    tools = work / "platform-tools"
    licenses = work / "licenses"
    tools.mkdir(parents=True, exist_ok=True); licenses.mkdir(parents=True, exist_ok=True)
    pin = json.loads((ROOT / "packaging/windows/platform-tools.json").read_text())
    archive = work / "platform-tools.zip"
    if not archive.exists() or hashlib.sha256(archive.read_bytes()).hexdigest() != pin["sha256"]:
        with urlopen(pin["url"], timeout=60) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != pin["sha256"]:
            raise RuntimeError("Official Android platform-tools SHA256 mismatch")
        archive.write_bytes(data)
    included = {}
    with zipfile.ZipFile(archive) as source:
        for name in source.namelist():
            path = Path(name)
            if len(path.parts) == 2 and (path.name == "adb.exe" or path.suffix.lower() == ".dll"
                                        or path.name in ("NOTICE.txt", "source.properties")):
                data = source.read(name); (tools / path.name).write_bytes(data)
                included[path.name] = hashlib.sha256(data).hexdigest()
    assert "adb.exe" in included and "AdbWinApi.dll" in included
    (tools / "bundle.json").write_text(json.dumps({"id": "platform-tools-" + pin["version"], "files": included}), encoding="utf-8")
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
    subprocess.run([sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean", "--onefile", "--windowed",
                    "--noupx", "--name", "DroneCollisionDetection", "--paths", str(ROOT / "stages/01-camera-stream/src"),
                    "--exclude-module", "pytest", "--exclude-module", "tkinter",
                    # Ultralytics loads model/tracker modules lazily; PyInstaller cannot see them statically.
                    "--collect-submodules", "ultralytics", "--copy-metadata", "ultralytics",
                    # torchvision >= 0.2x names its ops libraries *_stable.so/.pyd, which the bundled hook
                    # (it only knows torchvision._C) misses; without them torchvision::nms does not exist.
                    "--collect-binaries", "torchvision", *torchvision_binaries(),
                    "--add-data", str(tools) + ";platform-tools", "--add-data", str(licenses) + ";third-party-licenses",
                    "--distpath", str(ROOT / "dist/windows"), "--workpath", str(work / "pyinstaller"),
                    "--specpath", str(work), str(ROOT / "stages/01-camera-stream/src/main.py")], cwd=ROOT, check=True)
    executable = ROOT / "dist/windows" / f"DroneCollisionDetection-v{__version__}-windows-x64.exe"
    (ROOT / "dist/windows/DroneCollisionDetection.exe").replace(executable)
    output = ROOT / "dist/windows" / f"DroneCollisionDetection-v{__version__}-windows-x64.zip"
    with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as bundle:
        bundle.write(executable, "DroneCollisionDetection.exe")
        bundle.write(ROOT / "docs/windows-exe.md", "README-KO.md")
        bundle.write(ROOT / "LICENSE", "LICENSE")
        bundle.write(ROOT / "stages/01-camera-stream/config.example.json", "config.example.json")
        bundle.write(tools / "NOTICE.txt", "licenses/android-platform-tools-NOTICE.txt")
        for file in licenses.rglob("*"):
            if file.is_file():
                bundle.write(file, "licenses/" + file.relative_to(licenses).as_posix())
    checksums = ROOT / "dist/windows/SHA256SUMS.txt"
    checksums.write_text("\n".join(hashlib.sha256(p.read_bytes()).hexdigest()+"  "+p.name for p in (executable, output))+"\n", encoding="utf-8")
    print("Windows standalone files:", executable.name, output.name)


if __name__ == "__main__":
    main()
