import argparse
from pathlib import Path
import sys

from .config import ConfigError, defaults, load_config


def main(argv=None):
    parser = argparse.ArgumentParser(description="Phase 01 dual camera streaming")
    parser.add_argument("--config", type=Path, help="Configuration JSON; relative video paths use its directory")
    parser.add_argument("--validate-config", type=Path, help="Validate JSON and exit without loading Qt")
    parser.add_argument("--self-test-report", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.self_test_report:
        if not args.config:
            parser.error("--self-test-report requires --config")
        from .selftest import run
        return run(args.config.resolve(), args.self_test_report.resolve())
    if args.validate_config:
        try:
            load_config(args.validate_config)
        except ConfigError as error:
            print(str(error), file=sys.stderr)
            return 2
        print("Configuration valid: version 1, two cameras.")
        return 0
    from PySide6.QtCore import QStandardPaths
    from PySide6.QtWidgets import QApplication, QMessageBox
    from .gui import MainWindow
    app = QApplication(sys.argv[:1])
    app.setApplicationName("DroneCameraPhase01")
    app.setOrganizationName("DroneCollisionDetection")
    path = args.config or Path(QStandardPaths.writableLocation(
        QStandardPaths.StandardLocation.AppConfigLocation)) / "config.json"
    path = path.expanduser().resolve()
    try:
        configs = load_config(path) if path.exists() else defaults()
        if args.config and not path.exists():
            raise ConfigError("The specified configuration file does not exist.")
    except ConfigError as error:
        QMessageBox.critical(None, "Configuration error", str(error))
        return 2
    window = MainWindow(configs, path)
    window.show()
    # Also close workers on non-window event-loop shutdown.
    app.aboutToQuit.connect(window.shutdown)
    return app.exec()
