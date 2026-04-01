import sys
from pathlib import Path
from PyQt6.QtWidgets import QApplication
from pinky_station.config import StationConfig, load_station_config
from pinky_station.gui.main_window import PinkyStationWindow

# Default config search paths (first match wins)
_CONFIG_SEARCH = [
    Path(__file__).resolve().parent.parent.parent / "config" / "station_config.yaml",
    Path.home() / ".pinky" / "station_config.yaml",
]


def main():
    app = QApplication(sys.argv)

    # Load dark theme
    qss_path = Path(__file__).parent / "resources" / "styles" / "dark.qss"
    if qss_path.exists():
        app.setStyleSheet(qss_path.read_text())

    # Load station config (CLI --config <path> or auto-detect)
    config = StationConfig()
    config_path: str | None = None
    args = sys.argv[1:]
    for i, arg in enumerate(args):
        if arg == "--config" and i + 1 < len(args):
            config_path = args[i + 1]
            break

    if config_path:
        config = load_station_config(config_path)
    else:
        for candidate in _CONFIG_SEARCH:
            if candidate.exists():
                config = load_station_config(candidate)
                break

    window = PinkyStationWindow(config)
    window.resize(config.gui.window_width, config.gui.window_height)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
