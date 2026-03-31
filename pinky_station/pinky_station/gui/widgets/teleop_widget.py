from PyQt6.QtWidgets import QGroupBox, QGridLayout, QPushButton, QSlider, QLabel, QVBoxLayout, QHBoxLayout
from PyQt6.QtCore import Qt, pyqtSignal

class TeleopWidget(QGroupBox):
    # Signals for commands to be passed to CommandWorker
    sig_cmd_vel = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__("Teleop Control", parent)
        layout = QVBoxLayout(self)
        
        # Speed slider
        speed_layout = QHBoxLayout()
        self.slider_speed = QSlider(Qt.Orientation.Horizontal)
        self.slider_speed.setRange(5, 50) # 0.05 to 0.50 m/s
        self.slider_speed.setValue(20) # 0.2 m/s default
        self.lbl_speed = QLabel("Speed: 0.20 m/s")
        
        self.slider_speed.valueChanged.connect(self._on_speed_changed)
        speed_layout.addWidget(self.lbl_speed)
        speed_layout.addWidget(self.slider_speed)
        
        # Grid of buttons
        grid_layout = QGridLayout()
        
        self.btn_fwd = QPushButton("W")
        self.btn_bwd = QPushButton("S")
        self.btn_left = QPushButton("A")
        self.btn_right = QPushButton("D")
        self.btn_stop = QPushButton("STOP")
        
        # Add basic style
        btn_style = "QPushButton { min-height: 40px; font-weight: bold; font-size: 16px; }"
        self.btn_fwd.setStyleSheet(btn_style)
        self.btn_bwd.setStyleSheet(btn_style)
        self.btn_left.setStyleSheet(btn_style)
        self.btn_right.setStyleSheet(btn_style)
        self.btn_stop.setStyleSheet("QPushButton { min-height: 40px; font-weight: bold; font-size: 16px; background-color: #8b0000; color: white; }")
        
        grid_layout.addWidget(self.btn_fwd, 0, 1)
        grid_layout.addWidget(self.btn_left, 1, 0)
        grid_layout.addWidget(self.btn_stop, 1, 1)
        grid_layout.addWidget(self.btn_right, 1, 2)
        grid_layout.addWidget(self.btn_bwd, 2, 1)
        
        layout.addLayout(speed_layout)
        layout.addLayout(grid_layout)
        
        # Connections
        self.btn_fwd.pressed.connect(lambda: self._send_cmd(1.0, 0.0))
        self.btn_bwd.pressed.connect(lambda: self._send_cmd(-1.0, 0.0))
        self.btn_left.pressed.connect(lambda: self._send_cmd(0.0, 1.0))
        self.btn_right.pressed.connect(lambda: self._send_cmd(0.0, -1.0))
        self.btn_stop.pressed.connect(lambda: self._send_cmd(0.0, 0.0))

        # Auto-stop on release for safety
        self.btn_fwd.released.connect(lambda: self._send_cmd(0.0, 0.0))
        self.btn_bwd.released.connect(lambda: self._send_cmd(0.0, 0.0))
        self.btn_left.released.connect(lambda: self._send_cmd(0.0, 0.0))
        self.btn_right.released.connect(lambda: self._send_cmd(0.0, 0.0))

    def _on_speed_changed(self, value):
        speed = value / 100.0
        self.lbl_speed.setText(f"Speed: {speed:.2f} m/s")

    def _send_cmd(self, linear_mult, angular_mult):
        speed = self.slider_speed.value() / 100.0
        angular_speed = 1.0 # fixed angular or scale by slider
        self.sig_cmd_vel.emit(linear_mult * speed, angular_mult * angular_speed)
