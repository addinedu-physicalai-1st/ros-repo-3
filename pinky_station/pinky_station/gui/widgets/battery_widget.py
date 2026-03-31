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
            # C++ SerializeBattery: voltage(f32) + percentage(f32) + status(u8) = 9 bytes
            voltage, percentage, status = struct.unpack('<ffB', msg.payload[:9])
            
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
