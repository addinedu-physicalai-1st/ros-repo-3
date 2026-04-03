from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel, QLineEdit, QPushButton, QComboBox
from PyQt6.QtCore import pyqtSignal

class ToolbarWidget(QWidget):
    sig_add_robot = pyqtSignal(str, str) # Emits robot_id, ip
    sig_active_robot_changed = pyqtSignal(str) # Emits active robot_id
    sig_set_pose_mode = pyqtSignal(bool)  # Emits true if entering 2D Pose Estimate mode
    sig_load_map = pyqtSignal()

    def __init__(self, default_host: str = "127.0.0.1", parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.cb_active_robot = QComboBox()
        self.cb_active_robot.setFixedWidth(150)
        self.cb_active_robot.currentTextChanged.connect(self.sig_active_robot_changed.emit)

        self.input_id = QLineEdit("robot1")
        self.input_id.setFixedWidth(100)
        self.input_ip = QLineEdit(default_host)
        self.input_ip.setFixedWidth(120)
        
        self.btn_add_robot = QPushButton("Add Robot")
        self.btn_add_robot.setFixedWidth(100)
        
        self.lbl_status = QLabel("Ready")
        self.lbl_status.setStyleSheet("color: green; font-weight: bold; margin-left: 10px; margin-right: 20px;")
        
        self.btn_pose = QPushButton("2D Pose Estimate")
        self.btn_pose.setCheckable(True)
        self.btn_pose.setFixedWidth(150)
        
        self.btn_load_map = QPushButton("Load Map")
        self.btn_load_map.setFixedWidth(100)

        layout.addWidget(QLabel("Active Robot:"))
        layout.addWidget(self.cb_active_robot)
        layout.addWidget(QLabel("ID:"))
        layout.addWidget(self.input_id)
        layout.addWidget(QLabel("IP:"))
        layout.addWidget(self.input_ip)
        layout.addWidget(self.btn_add_robot)
        layout.addWidget(self.lbl_status)
        layout.addWidget(self.btn_pose)
        layout.addWidget(self.btn_load_map)
        layout.addStretch()
        
        self.btn_add_robot.clicked.connect(self._on_add_robot_clicked)
        self.btn_pose.toggled.connect(self.sig_set_pose_mode.emit)
        self.btn_load_map.clicked.connect(self.sig_load_map.emit)

    def _on_add_robot_clicked(self):
        robot_id = self.input_id.text().strip()
        ip = self.input_ip.text().strip()
        if robot_id and ip:
            exists = False
            for i in range(self.cb_active_robot.count()):
                if self.cb_active_robot.itemText(i) == robot_id:
                    exists = True
                    break
            if not exists:
                self.cb_active_robot.addItem(robot_id)
            self.sig_add_robot.emit(robot_id, ip)

    def set_status(self, text: str, color: str):
        self.lbl_status.setText(text)
        self.lbl_status.setStyleSheet(f"color: {color}; font-weight: bold; margin-left: 10px; margin-right: 20px;")
