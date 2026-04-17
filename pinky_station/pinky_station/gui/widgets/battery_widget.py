from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QProgressBar, QLabel
from PyQt6.QtCore import Qt


class BatteryWidget(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("Battery Status", parent)
        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        self.lbl_voltage = QLabel("Voltage: -- V")
        self.lbl_voltage.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_voltage.setStyleSheet(
            "font-weight: bold; font-size: 14px; color: #eaf0fb; background: transparent;"
        )

        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.progress.setTextVisible(True)
        self.progress.setFormat("%p%")

        layout.addWidget(self.lbl_voltage)
        layout.addWidget(self.progress)

    def update_status(self, msg):
        """Accept protobuf battery message with .voltage and .percentage fields."""
        try:
            voltage = float(msg.voltage)
            pct = int(max(0, min(float(msg.percentage), 100)))

            self.lbl_voltage.setText(f"Voltage: {voltage:.2f} V")
            self.progress.setValue(pct)

            if pct > 50:
                chunk_color = "#a6e3a1"
            elif pct > 20:
                chunk_color = "#fab387"
            else:
                chunk_color = "#f38ba8"

            self.progress.setStyleSheet(
                f"QProgressBar {{ background-color: #181825; border: 1px solid #313244; "
                f"border-radius: 6px; text-align: center; color: #cdd6f4; min-height: 18px; }}"
                f"QProgressBar::chunk {{ background-color: {chunk_color}; border-radius: 5px; }}"
            )
        except Exception:
            pass
