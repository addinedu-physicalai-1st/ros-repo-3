import sys
import struct
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel
)
from PyQt6.QtCore import Qt

# Net
from pinky_station.net.connection import ConnectionManager, ConnectionState
from pinky_station.protocol import message_types as mt

# Widgets
from pinky_station.gui.widgets.toolbar import ToolbarWidget
from pinky_station.gui.widgets.lidar_view import LidarViewWidget
from pinky_station.gui.widgets.video_view import VideoViewWidget
from pinky_station.gui.widgets.teleop_widget import TeleopWidget
from pinky_station.gui.widgets.battery_widget import BatteryWidget
from pinky_station.gui.widgets.terminal_widget import TerminalWidget
from pinky_station.gui.widgets.map_widget import MapWidget

# Workers
from pinky_station.workers.sensor_worker import SensorWorker
from pinky_station.workers.camera_worker import CameraWorker
from pinky_station.workers.command_worker import CommandWorker

class PinkyStationWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pinky Station (PyQt6)")
        self.resize(1200, 800)
        
        self.conn = ConnectionManager()
        self.conn.on_state_change = self._on_conn_state_change
        
        self.sensor_worker = None
        self.camera_worker = None
        self.command_worker = None
        
        # Central widget and layout
        central = QWidget()
        main_layout = QVBoxLayout(central)
        self.setCentralWidget(central)
        
        # Toolbar
        self.toolbar = ToolbarWidget()
        self.toolbar.sig_connect_toggled.connect(self._toggle_connection)
        self.toolbar.sig_set_pose_mode.connect(self._on_pose_mode_changed)
        main_layout.addWidget(self.toolbar)
        
        # Split layout
        content_layout = QHBoxLayout()
        
        # Left Panel
        left_layout = QVBoxLayout()
        self.battery_view = BatteryWidget()
        self.teleop_view = TeleopWidget()
        self.terminal_view = TerminalWidget()
        
        left_layout.addWidget(self.battery_view)
        left_layout.addWidget(self.teleop_view)
        left_layout.addWidget(QLabel("Terminal Log:"))
        left_layout.addWidget(self.terminal_view)
        
        # Center Panel
        center_layout = QVBoxLayout()
        self.video_view = VideoViewWidget()
        self.lidar_view = LidarViewWidget()
        center_layout.addWidget(QLabel("Camera view"))
        center_layout.addWidget(self.video_view, 1)
        center_layout.addWidget(QLabel("Raw Lidar Scan [PyQtGraph]"))
        center_layout.addWidget(self.lidar_view, 1)
        
        # Right Panel
        right_layout = QVBoxLayout()
        self.map_view = MapWidget()
        right_layout.addWidget(QLabel("2D Map & Navigation"))
        right_layout.addWidget(self.map_view, stretch=1)
        
        content_layout.addLayout(left_layout, 2)
        content_layout.addLayout(center_layout, 3)
        content_layout.addLayout(right_layout, 3)
        main_layout.addLayout(content_layout)

    def _toggle_connection(self, ip_target: str):
        if not ip_target: # User wants disconnect or cancel
            self._cleanup_workers()
            self.conn.disconnect()
        else:
            self.terminal_view.append_log(2, f"Connecting to {ip_target}...")
            self.conn.connect(ip_target)

    def _on_pose_mode_changed(self, active: bool):
        # Notify Map widget to expect 2D Pose estimate instead of nav goal clicks 
        # For simplicity, we can pass state to map_view
        pass

    def _on_conn_state_change(self, state: ConnectionState):
        if state == ConnectionState.CONNECTED:
            self.toolbar.set_status(True, "CONNECTED", "green")
            self.terminal_view.append_log(2, "Connected safely.")
            self._start_workers()
            
        elif state == ConnectionState.DISCONNECTED:
            self.toolbar.set_status(False, "DISCONNECTED", "red")
            self.terminal_view.append_log(2, "Disconnected.")
            self._cleanup_workers()
            
        elif state == ConnectionState.CONNECTING:
            self.toolbar.set_status(False, "CONNECTING", "orange")

    def _start_workers(self):
        if self.conn.udp and not self.sensor_worker:
            self.sensor_worker = SensorWorker(self.conn.udp)
            self.sensor_worker.sig_battery.connect(self.battery_view.update_status)
            self.sensor_worker.sig_odom.connect(self.map_view.update_odom)
            # Lidar 1D or 2D mapping can be routed here as well
            self.sensor_worker.start()

        if self.conn.tcp and not self.camera_worker:
            self.camera_worker = CameraWorker(self.conn.tcp)
            self.camera_worker.sig_frame.connect(self.video_view.update_frame)
            self.camera_worker.start()

        if self.conn.tcp and not self.command_worker:
            self.command_worker = CommandWorker(self.conn.tcp)
            self.command_worker.sig_log.connect(lambda txt: self.terminal_view.append_log(4, txt))
            self.command_worker.start()
            
            # Map UI events to CommandWorker TCP sends
            self.teleop_view.sig_cmd_vel.connect(self.command_worker.send_cmd_vel)
            self.map_view.sig_set_goal.connect(self.command_worker.send_nav_goal)
            self.map_view.sig_set_pose.connect(self.command_worker.set_pose)
            
            # Sub-routing for tcp logs
            original_tcp_cb = self.conn.tcp.on_message
            def combined_cb(msg):
                if msg.msg_type == mt.MSG_DEBUG_LOG:
                    self.terminal_view.append_msg(msg)
                if original_tcp_cb:
                    original_tcp_cb(msg)
            self.conn.tcp.on_message = combined_cb

    def _cleanup_workers(self):
        if self.sensor_worker:
            self.sensor_worker.stop()
            self.sensor_worker.wait()
            self.sensor_worker = None
        if self.camera_worker:
            self.camera_worker.stop()
            self.camera_worker.wait()
            self.camera_worker = None
        if self.command_worker:
            self.command_worker.stop()
            self.command_worker.wait()
            self.command_worker = None

    def closeEvent(self, event):
        self._cleanup_workers()
        self.conn.disconnect()
        event.accept()
