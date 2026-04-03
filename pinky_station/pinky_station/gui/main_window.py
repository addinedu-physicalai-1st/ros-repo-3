import sys
import math
from PyQt6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel
from PyQt6.QtCore import Qt

from pinky_station.config import StationConfig
from pinky_station.net.zmq_client import ZmqClient

# Widgets
from pinky_station.gui.widgets.toolbar import ToolbarWidget
from pinky_station.gui.widgets.lidar_view import LidarViewWidget
from pinky_station.gui.widgets.video_view import VideoViewWidget
from pinky_station.gui.widgets.teleop_widget import TeleopWidget
from pinky_station.gui.widgets.battery_widget import BatteryWidget
from pinky_station.gui.widgets.terminal_widget import TerminalWidget
from pinky_station.gui.widgets.map_widget import MapWidget

class PinkyStationWindow(QMainWindow):
    def __init__(self, config: StationConfig | None = None):
        super().__init__()
        self.cfg = config or StationConfig()
        self.setWindowTitle("Pinky Station (PyQt6) - ZMQ Edition")

        self.zmq_client = ZmqClient(
            pub_port=self.cfg.connection.udp_port,
            req_port=self.cfg.connection.tcp_port
        )
        self.zmq_client.odom_received.connect(self._on_odom)
        self.zmq_client.lidar_received.connect(self._on_lidar)
        self.zmq_client.battery_received.connect(self._on_battery)
        self.zmq_client.log_received.connect(self._on_log)
        
        # Central widget and layout
        central = QWidget()
        main_layout = QVBoxLayout(central)
        self.setCentralWidget(central)

        # Toolbar
        self.toolbar = ToolbarWidget(default_host=self.cfg.connection.default_host)
        self.toolbar.sig_connect_toggled.connect(self._toggle_connection)
        self.toolbar.sig_set_pose_mode.connect(self._on_pose_mode_changed)
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
        self.teleop_view.sig_cmd_vel.connect(self.zmq_client.send_cmd_vel)
        
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
        self.zmq_client.frame_received.connect(self.video_view.update_frame)
        self.lidar_view = LidarViewWidget()
        center_layout.addWidget(QLabel("Camera view"))
        center_layout.addWidget(self.video_view, 1)
        center_layout.addWidget(QLabel("Raw Lidar Scan [PyQtGraph]"))
        center_layout.addWidget(self.lidar_view, 1)
        
        # Right Panel
        right_layout = QVBoxLayout()
        self.map_view = MapWidget(scale=self.cfg.gui.map_scale)
        self.map_view.sig_set_goal.connect(self.zmq_client.send_nav_goal)
        self.map_view.sig_set_pose.connect(self.zmq_client.set_pose)
        right_layout.addWidget(QLabel("2D Map & Navigation"))
        right_layout.addWidget(self.map_view, stretch=1)
        
        content_layout.addLayout(left_layout, 2)
        content_layout.addLayout(center_layout, 3)
        content_layout.addLayout(right_layout, 3)
        main_layout.addLayout(content_layout)

    def _toggle_connection(self, ip_target: str):
        if not ip_target:
            self.zmq_client.stop()
            self.toolbar.set_status(False, "DISCONNECTED", "red")
            self.terminal_view.append_log(2, "Disconnected.")
        else:
            self.terminal_view.append_log(2, f"Connecting to ZMQ {ip_target}...")
            self.zmq_client.connect_to(ip_target)
            self.toolbar.set_status(True, "CONNECTED", "green")
            self.terminal_view.append_log(2, "Connected ZMQ sockets.")

    def _on_pose_mode_changed(self, active: bool):
        self.map_view.set_pose_mode(active)

    def _on_odom(self, msg):
        self.map_view.update_odom(msg)

    def _on_lidar(self, msg):
        if hasattr(msg, 'sectors') and len(msg.sectors) > 0:
            sectors = msg.sectors
            max_range = 12.0
            ranges = [s * max_range for s in sectors]
            self.lidar_view.update_scan(ranges, 0.0, 2.0 * math.pi / 24.0)
        elif hasattr(msg, 'ranges'):
            self.lidar_view.update_scan(msg.ranges, msg.angle_min, msg.angle_increment)

    def _on_battery(self, msg):
        class BattMock:
            pass
        b = BattMock()
        b.voltage = msg.voltage
        b.percentage = msg.percentage
        self.battery_view.update_status(b)

    def _on_log(self, msg):
        self.terminal_view.append_log(msg.severity, msg.text)

    def closeEvent(self, event):
        self.zmq_client.stop()
        event.accept()