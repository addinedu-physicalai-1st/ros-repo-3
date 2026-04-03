import sys
import math
from PyQt6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFileDialog
from PyQt6.QtCore import Qt

from pinky_station.config import StationConfig
from pinky_station.net.zmq_client import ZmqClient

# Widgets
from pinky_station.gui.widgets.toolbar import ToolbarWidget
from pinky_station.gui.widgets.video_view import VideoViewWidget
from pinky_station.gui.widgets.teleop_widget import TeleopWidget
from pinky_station.gui.widgets.battery_widget import BatteryWidget
from pinky_station.gui.widgets.terminal_widget import TerminalWidget
from pinky_station.gui.widgets.map_widget import MapWidget

class PinkyStationWindow(QMainWindow):
    def __init__(self, config: StationConfig | None = None):
        super().__init__()
        self.cfg = config or StationConfig()
        self.setWindowTitle("Pinky Station (PyQt6) - Multi-Robot ZMQ Edition")
        
        self.active_robot_id = None

        self.zmq_client = ZmqClient()
        self.zmq_client.odom_received.connect(self._on_odom)
        self.zmq_client.battery_received.connect(self._on_battery)
        self.zmq_client.log_received.connect(self._on_log)
        self.zmq_client.frame_received.connect(self._on_frame)
        
        # Central widget and layout
        central = QWidget()
        main_layout = QVBoxLayout(central)
        self.setCentralWidget(central)

        # Toolbar
        self.toolbar = ToolbarWidget(default_host=self.cfg.connection.default_host)
        self.toolbar.sig_add_robot.connect(self._on_add_robot)
        self.toolbar.sig_active_robot_changed.connect(self._on_active_robot_changed)
        self.toolbar.sig_set_pose_mode.connect(self._on_pose_mode_changed)
        self.toolbar.sig_load_map.connect(self._on_load_map)
        main_layout.addWidget(self.toolbar)
        
        # Split layout
        content_layout = QHBoxLayout()

        # Left Panel
        left_layout = QVBoxLayout()
        self.battery_view = BatteryWidget()
        self.teleop_view = TeleopWidget(
            default_speed=self.cfg.teleop.default_linear_speed,
            max_speed=self.cfg.teleop.max_linear_speed,
            angular_speed=self.cfg.teleop.default_angular_speed,
        )
        self.teleop_view.sig_cmd_vel.connect(self._on_cmd_vel)
        
        self.terminal_view = TerminalWidget(
            max_lines=self.cfg.terminal.max_lines,
            default_filter=self.cfg.terminal.default_filter,
        )
        
        left_layout.addWidget(self.battery_view)
        left_layout.addWidget(self.teleop_view)
        left_layout.addWidget(QLabel("Terminal Log:"))
        left_layout.addWidget(self.terminal_view)
        
        # Center Panel
        center_layout = QVBoxLayout()
        self.video_view = VideoViewWidget()
        center_layout.addWidget(QLabel("Camera view"))
        center_layout.addWidget(self.video_view, 1)
        
        # Right Panel
        right_layout = QVBoxLayout()
        self.map_view = MapWidget(scale=self.cfg.gui.map_scale)
        self.map_view.sig_set_goal.connect(self._on_set_goal)
        self.map_view.sig_set_pose.connect(self._on_set_pose)
        right_layout.addWidget(QLabel("2D Map & Navigation"))
        right_layout.addWidget(self.map_view, stretch=1)
        
        content_layout.addLayout(left_layout, 2)
        content_layout.addLayout(center_layout, 3)
        content_layout.addLayout(right_layout, 3)
        main_layout.addLayout(content_layout)

    def _on_load_map(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open Map YAML", "", "YAML Files (*.yaml *.yml)")
        if file_path:
            self.map_view.load_map(file_path)

    def _on_add_robot(self, robot_id: str, ip: str):
        self.terminal_view.append_log(2, f"Adding robot {robot_id} at {ip}...")
        self.zmq_client.add_robot(
            robot_id, 
            ip, 
            req_port=self.cfg.connection.tcp_port, 
            pub_port=self.cfg.connection.udp_port
        )
        if not self.active_robot_id:
            self._on_active_robot_changed(robot_id)
        self.toolbar.set_status(f"{len(self.zmq_client.robots)} Connected", "green")

    def _on_active_robot_changed(self, robot_id: str):
        self.active_robot_id = robot_id
        self.map_view.set_active_robot(robot_id)
        self.terminal_view.append_log(2, f"Active robot changed to {robot_id}")

    def _on_pose_mode_changed(self, active: bool):
        self.map_view.set_pose_mode(active)

    def _on_cmd_vel(self, linear: float, angular: float):
        if self.active_robot_id:
            self.zmq_client.send_cmd_vel(self.active_robot_id, linear, angular)

    def _on_set_goal(self, x: float, y: float, theta: float):
        if self.active_robot_id:
            self.zmq_client.send_nav_goal(self.active_robot_id, x, y, theta)
            
    def _on_set_pose(self, x: float, y: float, theta: float):
        if self.active_robot_id:
            self.zmq_client.set_pose(self.active_robot_id, x, y, theta)

    def _on_odom(self, robot_id: str, msg):
        self.map_view.update_odom(robot_id, msg)

    def _on_battery(self, robot_id: str, msg):
        if robot_id == self.active_robot_id:
            class BattMock:
                pass
            b = BattMock()
            b.voltage = msg.voltage
            b.percentage = msg.percentage
            self.battery_view.update_status(b)

    def _on_log(self, robot_id: str, msg):
        self.terminal_view.append_log(msg.severity, f"[{robot_id}] {msg.text}")
        
    def _on_frame(self, robot_id: str, frame_data: bytes):
        if robot_id == self.active_robot_id:
            self.video_view.update_frame(frame_data)

    def closeEvent(self, event):
        self.zmq_client.stop()
        event.accept()
