from PyQt6.QtWidgets import QGroupBox, QGridLayout, QPushButton, QSlider, QLabel, QVBoxLayout, QHBoxLayout
from PyQt6.QtCore import Qt, pyqtSignal

class TeleopWidget(QGroupBox):
    # Signals for commands to be passed to CommandWorker
    sig_cmd_vel = pyqtSignal(float, float)

    def __init__(self, default_speed: float = 0.20, max_speed: float = 0.50,
                 angular_speed: float = 1.0, parent=None):
        super().__init__("Teleop Control", parent)
        self._angular_speed = angular_speed
        layout = QVBoxLayout(self)

        # Speed slider (range: 0.05 .. max_speed in 0.01 steps)
        speed_layout = QHBoxLayout()
        self.slider_speed = QSlider(Qt.Orientation.Horizontal)
        self.slider_speed.setRange(5, int(max_speed * 100))
        default_val = int(default_speed * 100)
        self.slider_speed.setValue(default_val)
        self.lbl_speed = QLabel(f"Speed: {default_speed:.2f} m/s")
        
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
        self.sig_cmd_vel.emit(linear_mult * speed, angular_mult * self._angular_speed)
