from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QProgressBar, QLabel
from PyQt6.QtCore import Qt
import struct
from pinky_station.protocol.serializer import ParsedMessage

class BatteryWidget(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("Battery Status", parent)
        layout = QVBoxLayout(self)
        
        self.lbl_voltage = QLabel("Voltage: -- V")
        self.lbl_voltage.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_voltage.setStyleSheet("font-weight: bold; font-size: 14px;")
        
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.progress.setTextVisible(True)
        
        layout.addWidget(self.lbl_voltage)
        layout.addWidget(self.progress)
        
    def update_status(self, msg: ParsedMessage):
        try:
            # Expected format based on typical ROS2 BatteryState logic we used:
            # <QffB : stamp(uint64), voltage(float32), percentage(float32), status(uint8)
            # In types.h: uint64_t stamp, float voltage, float percentage, uint8_t status
            unpacked = struct.unpack('<QffB', msg.payload[:17]) # 8+4+4+1 = 17
            stamp, voltage, percentage, status = unpacked
            
            # Map voltage correctly (assuming 12V system or 8.4V system)
            pct = int(max(0, min(percentage, 100)))
            
            self.lbl_voltage.setText(f"Voltage: {voltage:.2f} V")
            self.progress.setValue(pct)
            
            # Update colors
            if pct > 50:
                color = "green"
            elif pct > 20:
                color = "orange"
            else:
                color = "red"
                
            self.progress.setStyleSheet(
                f"QProgressBar::chunk {{ background-color: {color}; }}"
            )
        except Exception as e:
            pass
