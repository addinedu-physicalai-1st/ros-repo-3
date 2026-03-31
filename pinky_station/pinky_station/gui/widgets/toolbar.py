from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel, QLineEdit, QPushButton
from PyQt6.QtCore import pyqtSignal

class ToolbarWidget(QWidget):
    sig_connect_toggled = pyqtSignal(str) # Emits IP address string or empty string to disconnect
    sig_set_pose_mode = pyqtSignal(bool)  # Emits true if entering 2D Pose Estimate mode

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.input_ip = QLineEdit("127.0.0.1")
        self.input_ip.setFixedWidth(150)
        
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setFixedWidth(100)
        
        self.lbl_status = QLabel("DISCONNECTED")
        self.lbl_status.setStyleSheet("color: red; font-weight: bold; margin-left: 10px; margin-right: 20px;")
        
        self.btn_pose = QPushButton("2D Pose Estimate")
        self.btn_pose.setCheckable(True)
        self.btn_pose.setFixedWidth(150)
        
        layout.addWidget(QLabel("Robot IP:"))
        layout.addWidget(self.input_ip)
        layout.addWidget(self.btn_connect)
        layout.addWidget(self.lbl_status)
        layout.addWidget(self.btn_pose)
        layout.addStretch()
        
        # Internal state
        self.is_connected = False
        
        self.btn_connect.clicked.connect(self._on_connect_clicked)
        self.btn_pose.toggled.connect(self.sig_set_pose_mode.emit)

    def _on_connect_clicked(self):
        if not self.is_connected:
            self.sig_connect_toggled.emit(self.input_ip.text())
        else:
            self.sig_connect_toggled.emit("") # Empty triggers disconnect

    def set_status(self, connected: bool, text: str, color: str):
        self.is_connected = connected
        self.lbl_status.setText(text)
        self.lbl_status.setStyleSheet(f"color: {color}; font-weight: bold; margin-left: 10px; margin-right: 20px;")
        self.btn_connect.setText("Disconnect" if connected else "Connect")
        # Ensure we don't accidentally get stuck if connecting state
        if text == "CONNECTING":
            self.is_connected = False 
            self.btn_connect.setText("Cancel")
