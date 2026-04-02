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

        # Speed sliders layout
        sliders_layout = QVBoxLayout()
        
        # Linear Speed slider
        lin_layout = QHBoxLayout()
        self.slider_speed = QSlider(Qt.Orientation.Horizontal)
        self.slider_speed.setRange(5, int(max_speed * 100))
        default_val = int(default_speed * 100)
        self.slider_speed.setValue(default_val)
        self.lbl_speed = QLabel(f"L-Speed: {default_speed:.2f} m/s")
        self.slider_speed.valueChanged.connect(self._on_speed_changed)
        lin_layout.addWidget(self.lbl_speed)
        lin_layout.addWidget(self.slider_speed)
        
        # Angular Speed slider
        ang_layout = QHBoxLayout()
        self.slider_ang_speed = QSlider(Qt.Orientation.Horizontal)
        self.slider_ang_speed.setRange(10, 300) # 0.1 to 3.0 rad/s
        default_ang_val = int(angular_speed * 100)
        self.slider_ang_speed.setValue(default_ang_val)
        self.lbl_ang_speed = QLabel(f"A-Speed: {angular_speed:.2f} rad/s")
        self.slider_ang_speed.valueChanged.connect(self._on_ang_speed_changed)
        ang_layout.addWidget(self.lbl_ang_speed)
        ang_layout.addWidget(self.slider_ang_speed)
        
        sliders_layout.addLayout(lin_layout)
        sliders_layout.addLayout(ang_layout)
        
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
        
        layout.addLayout(sliders_layout)
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
        self.lbl_speed.setText(f"L-Speed: {speed:.2f} m/s")

    def _on_ang_speed_changed(self, value):
        speed = value / 100.0
        self.lbl_ang_speed.setText(f"A-Speed: {speed:.2f} rad/s")

    def _send_cmd(self, linear_mult, angular_mult):
        linear_speed = self.slider_speed.value() / 100.0
        angular_speed = self.slider_ang_speed.value() / 100.0
        self.sig_cmd_vel.emit(linear_mult * linear_speed, angular_mult * angular_speed)

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in (Qt.Key.Key_W, Qt.Key.Key_S, Qt.Key.Key_A, Qt.Key.Key_D):
            self._pressed_keys.add(key)
            self._evaluate_keys()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in self._pressed_keys:
            self._pressed_keys.remove(key)
            self._evaluate_keys()
        else:
            super().keyReleaseEvent(event)

    def _evaluate_keys(self):
        lin = 0.0
        ang = 0.0
        
        if Qt.Key.Key_W in self._pressed_keys:
            lin += 1.0
        if Qt.Key.Key_S in self._pressed_keys:
            lin -= 1.0
            
        if Qt.Key.Key_A in self._pressed_keys:
            ang += 1.0
        if Qt.Key.Key_D in self._pressed_keys:
            ang -= 1.0
            
        self._send_cmd(lin, ang)
