"""Launcher guarded for Windows/macOS multiprocessing spawn."""
import multiprocessing

from drone_camera.app import main

if __name__ == "__main__":
    multiprocessing.freeze_support()
    raise SystemExit(main())
