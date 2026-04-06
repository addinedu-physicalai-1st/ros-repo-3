from PyQt6.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QLabel, QLineEdit, QPushButton, QComboBox
from PyQt6.QtCore import pyqtSignal

class ToolbarWidget(QWidget):
    sig_add_robot = pyqtSignal(str, str) # Emits robot_id, ip
    sig_disconnect_robot = pyqtSignal(str) # Emits robot_id
    sig_active_robot_changed = pyqtSignal(str) # Emits active robot_id
    sig_set_pose_mode = pyqtSignal(bool)  # Emits true if entering 2D Pose Estimate mode
    sig_load_map = pyqtSignal()
    
    sig_nav_start = pyqtSignal()
    sig_nav_stop = pyqtSignal()
    sig_nav_resume = pyqtSignal()
    sig_nav_reset = pyqtSignal()
    sig_add_waypoint = pyqtSignal()
    sig_map_build_toggle = pyqtSignal(bool)  # True = start, False = stop
    sig_map_save = pyqtSignal()

    def __init__(self, default_host: str = "127.0.0.1", parent=None):
        super().__init__(parent)
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(5, 5, 5, 5)

        # Row 1: Connection and Status
        row1 = QHBoxLayout()
        self.cb_active_robot = QComboBox()
        self.cb_active_robot.setFixedWidth(150)
        self.cb_active_robot.currentTextChanged.connect(self.sig_active_robot_changed.emit)

        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.setObjectName("btn_disconnect")
        self.btn_disconnect.setFixedWidth(100)
        self.btn_disconnect.clicked.connect(self._on_disconnect_clicked)

        self.input_id = QLineEdit()
        self.input_id.setPlaceholderText("Robot ID")
        self.input_id.setFixedWidth(100)
        self.input_ip = QLineEdit(default_host)
        self.input_ip.setFixedWidth(120)
        
        self.btn_add_robot = QPushButton("Add Robot")
        self.btn_add_robot.setObjectName("btn_add_robot")
        self.btn_add_robot.setFixedWidth(100)
        
        self.lbl_status = QLabel("Ready")
        self.lbl_status.setStyleSheet("color: green; font-weight: bold; margin-left: 10px; margin-right: 20px;")
        
        row1.addWidget(QLabel("Active Robot:"))
        row1.addWidget(self.cb_active_robot)
        row1.addWidget(self.btn_disconnect)
        row1.addSpacing(20)
        row1.addWidget(QLabel("ID:"))
        row1.addWidget(self.input_id)
        row1.addWidget(QLabel("IP:"))
        row1.addWidget(self.input_ip)
        row1.addWidget(self.btn_add_robot)
        row1.addWidget(self.lbl_status)
        row1.addStretch()
        main_layout.addLayout(row1)

        # Row 2: Map and Navigation
        row2 = QHBoxLayout()
        self.btn_pose = QPushButton("⊕ 2D Pose Estimate")
        self.btn_pose.setObjectName("btn_pose")
        self.btn_pose.setCheckable(True)
        self.btn_pose.setFixedWidth(160)
        
        self.btn_load_map = QPushButton("Load Map")
        self.btn_load_map.setFixedWidth(100)

        self.btn_map_build = QPushButton("Map Build")
        self.btn_map_build.setObjectName("btn_map_build")
        self.btn_map_build.setCheckable(True)
        self.btn_map_build.setFixedWidth(100)

        self.btn_map_save = QPushButton("Save Map")
        self.btn_map_save.setObjectName("btn_map_save")
        self.btn_map_save.setFixedWidth(100)
        self.btn_map_save.setEnabled(False)

        self.btn_start = QPushButton("▶  Start")
        self.btn_start.setObjectName("btn_start")
        self.btn_stop = QPushButton("⏸  Stop")
        self.btn_stop.setObjectName("btn_stop")
        self.btn_reset = QPushButton("↺  Reset")
        self.btn_reset.setObjectName("btn_reset")
        self.btn_add_waypoint = QPushButton("＋ Add Waypoint")
        self.btn_add_waypoint.setObjectName("btn_add_waypoint")
        self.btn_add_waypoint.setEnabled(False)

        row2.addWidget(self.btn_pose)
        row2.addWidget(self.btn_load_map)
        row2.addWidget(self.btn_map_build)
        row2.addWidget(self.btn_map_save)
        row2.addSpacing(40)
        row2.addWidget(self.btn_start)
        row2.addWidget(self.btn_stop)
        row2.addWidget(self.btn_reset)
        row2.addWidget(self.btn_add_waypoint)
        row2.addStretch()
        main_layout.addLayout(row2)
        
        self.btn_add_robot.clicked.connect(self._on_add_robot_clicked)
        self.btn_pose.toggled.connect(self.sig_set_pose_mode.emit)
        self.btn_load_map.clicked.connect(self.sig_load_map.emit)

        self.btn_start.clicked.connect(self._on_start_clicked)
        self.btn_stop.clicked.connect(self._on_stop_clicked)
        self.btn_reset.clicked.connect(self._on_reset_clicked)
        self.btn_add_waypoint.clicked.connect(self.sig_add_waypoint.emit)
        self.btn_map_build.toggled.connect(self._on_map_build_toggled)
        self.btn_map_save.clicked.connect(self.sig_map_save.emit)

    def _on_start_clicked(self):
        self.sig_nav_start.emit()
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_add_waypoint.setEnabled(False)

    def _on_add_robot_clicked(self):
        robot_id = self.input_id.text().strip()
        ip = self.input_ip.text().strip()
        if not robot_id or not ip:
            return
        self.sig_add_robot.emit(robot_id, ip)

    def confirm_robot_added(self, robot_id: str):
        """Called by MainWindow after successful connection verification."""
        for i in range(self.cb_active_robot.count()):
            if self.cb_active_robot.itemText(i) == robot_id:
                return
        self.cb_active_robot.addItem(robot_id)

    def _on_disconnect_clicked(self):
        active_id = self.cb_active_robot.currentText()
        if active_id:
            self.sig_disconnect_robot.emit(active_id)
            index = self.cb_active_robot.currentIndex()
            self.cb_active_robot.removeItem(index)

    def _on_stop_clicked(self):
        if "Stop" in self.btn_stop.text():
            self.sig_nav_stop.emit()
            self.btn_stop.setText("▶  Resume")
        else:
            self.sig_nav_resume.emit()
            self.btn_stop.setText("⏸  Stop")

    def _on_reset_clicked(self):
        self.btn_stop.setText("⏸  Stop")
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(True)
        self.sig_nav_reset.emit()

    def _on_map_build_toggled(self, checked: bool):
        self.sig_map_build_toggle.emit(checked)
        self.btn_map_save.setEnabled(checked or self.btn_map_save.isEnabled())
        if checked:
            self.btn_map_build.setText("Stop Build")
        else:
            self.btn_map_build.setText("Map Build")

    def set_status(self, text: str, color: str):
        self.lbl_status.setText(text)
        self.lbl_status.setStyleSheet(f"color: {color}; font-weight: bold; margin-left: 10px; margin-right: 20px;")
