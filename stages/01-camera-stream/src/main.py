"""Launcher guarded for Windows/macOS multiprocessing spawn."""
import multiprocessing

if __name__ == "__main__":
    # Divert frozen Windows capture/resource-tracker children before GUI imports.
    multiprocessing.freeze_support()
    from drone_camera.app import main
    raise SystemExit(main())
