import sys
from PyQt6.QtWidgets import QApplication
from pinky_station.gui.main_window import PinkyStationWindow

def main():
    app = QApplication(sys.argv)
    window = PinkyStationWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
