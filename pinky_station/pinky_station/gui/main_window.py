import sys
import math
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFileDialog, QFrame,
)
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
from pinky_station.workers.nav_worker import NavWorker


def _section_label(text: str) -> QLabel:
    lbl = QLabel(text.upper())
    lbl.setObjectName("section_label")
    lbl.setStyleSheet(
        "color: #89b4fa; font-size: 11px; font-weight: bold; "
        "letter-spacing: 1px; padding: 4px 6px 2px 6px;"
    )
    return lbl


def _hline() -> QFrame:
    line = QFrame()
    line.setFrameShape(QFrame.Shape.HLine)
    line.setStyleSheet("color: #313244;")
    return line


class PinkyStationWindow(QMainWindow):
    def __init__(self, config: StationConfig | None = None):
        super().__init__()
        self.cfg = config or StationConfig()
        self.setWindowTitle("Pinky Station")

        self.active_robot_id = None
        self.current_waypoint_idx = -1

        self.zmq_client = ZmqClient()
        self.zmq_client.odom_received.connect(self._on_odom)
        self.zmq_client.battery_received.connect(self._on_battery)
        self.zmq_client.log_received.connect(self._on_log)
        self.zmq_client.lidar_received.connect(self._on_lidar)
        self.zmq_client.frame_received.connect(self._on_frame)
        self.zmq_client.sig_command_failed.connect(self._on_command_failed)

        central = QWidget()
        self.main_layout = QVBoxLayout(central)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)
        self.setCentralWidget(central)

        self._setup_ui()

        self.nav_worker = NavWorker(self.zmq_client)
        self.nav_worker.sig_log.connect(
            lambda log: self.terminal_view.append_log(2, f"[NavWorker] {log}")
        )
        self.nav_worker.sig_path_ready.connect(self._on_path_ready)
        self.nav_worker.start()

        # Connect ROS 2 bridge to robot data
        self.zmq_client.odom_received.connect(self.nav_worker.on_odom_received)
        self.zmq_client.lidar_scan_received.connect(self.nav_worker.on_lidar_scan_received)

    def _setup_ui(self):
        # ── Toolbar ──────────────────────────────────────────────────
        self.toolbar = ToolbarWidget(default_host=self.cfg.connection.default_host)
        self.toolbar.setObjectName("toolbar_widget")
        self.toolbar.sig_add_robot.connect(self._on_add_robot)
        self.toolbar.sig_disconnect_robot.connect(self._on_disconnect_robot)
        self.toolbar.sig_active_robot_changed.connect(self._on_active_robot_changed)
        self.toolbar.sig_set_pose_mode.connect(self._on_pose_mode_changed)
        self.toolbar.sig_load_map.connect(self._on_load_map)
        self.toolbar.sig_nav_start.connect(self._on_nav_start)
        self.toolbar.sig_nav_stop.connect(self._on_nav_stop)
        self.toolbar.sig_nav_resume.connect(self._on_nav_resume)
        self.toolbar.sig_nav_reset.connect(self._on_nav_reset)
        self.toolbar.sig_add_waypoint.connect(self._on_add_waypoint)
        self.main_layout.addWidget(self.toolbar)
        self.main_layout.addWidget(_hline())

        # ── Content area ─────────────────────────────────────────────
        content = QWidget()
        content_layout = QHBoxLayout(content)
        content_layout.setContentsMargins(10, 10, 10, 10)
        content_layout.setSpacing(10)

        # ── Left panel ───────────────────────────────────────────────
        left_widget = QWidget()
        left_widget.setObjectName("panel_left")
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(8, 8, 8, 8)
        left_layout.setSpacing(6)

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
        left_layout.addWidget(_section_label("Terminal Log"))
        left_layout.addWidget(self.terminal_view, stretch=1)

        # ── Center panel ─────────────────────────────────────────────
        center_widget = QWidget()
        center_widget.setObjectName("panel_center")
        center_layout = QVBoxLayout(center_widget)
        center_layout.setContentsMargins(8, 8, 8, 8)
        center_layout.setSpacing(4)

        self.video_view = VideoViewWidget()
        center_layout.addWidget(_section_label("Camera Feed"))
        center_layout.addWidget(self.video_view, stretch=1)

        # ── Right panel ──────────────────────────────────────────────
        right_widget = QWidget()
        right_widget.setObjectName("panel_right")
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(8, 8, 8, 8)
        right_layout.setSpacing(4)

        self.map_view = MapWidget(scale=self.cfg.gui.map_scale)
        self.map_view.sig_set_goal.connect(self._on_set_goal)
        self.map_view.sig_set_pose.connect(self._on_set_pose)
        self.map_view.sig_potential_waypoint_selected.connect(
            self.toolbar.btn_add_waypoint.setEnabled
        )
        right_layout.addWidget(_section_label("2D Map & Navigation"))
        right_layout.addWidget(self.map_view, stretch=1)

        content_layout.addWidget(left_widget, 2)
        content_layout.addWidget(center_widget, 3)
        content_layout.addWidget(right_widget, 3)
        self.main_layout.addWidget(content, stretch=1)

    # ── Map loading ───────────────────────────────────────────────────
    def _on_load_map(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Open Map YAML", "", "YAML Files (*.yaml *.yml)"
        )
        if file_path:
            self.map_view.load_map(file_path)

    # ── Robot connection ──────────────────────────────────────────────
    def _on_add_robot(self, robot_id: str, ip: str):
        self.terminal_view.append_log(2, f"Connecting to robot '{robot_id}' at {ip}...")
        self.zmq_client.add_robot(
            robot_id,
            ip,
            req_port=self.cfg.connection.tcp_port,
            pub_port=self.cfg.connection.udp_port,
        )

        ok, msg = self.zmq_client.verify_connection(robot_id)
        if not ok:
            self.zmq_client.remove_robot(robot_id)
            self.terminal_view.append_log(3, f"Connection FAILED: {msg}")
            self.toolbar.set_status("Connection Failed", "red")
            return

        self.toolbar.confirm_robot_added(robot_id)
        if not self.active_robot_id:
            self._on_active_robot_changed(robot_id)
        self.toolbar.set_status(f"{len(self.zmq_client.robots)} Connected", "#a6e3a1")
        self.terminal_view.append_log(2, f"Robot '{robot_id}' connected successfully.")

    def _on_active_robot_changed(self, robot_id: str):
        self.active_robot_id = robot_id
        self.map_view.set_active_robot(robot_id)
        self.terminal_view.append_log(2, f"Active robot changed to {robot_id}")

    def _on_pose_mode_changed(self, active: bool):
        self.map_view.set_pose_mode(active)

    # ── Teleop ────────────────────────────────────────────────────────
    def _on_cmd_vel(self, linear: float, angular: float):
        if self.active_robot_id:
            self.zmq_client.send_cmd_vel(self.active_robot_id, linear, angular)

    # ── Pose estimate ─────────────────────────────────────────────────
    def _on_set_goal(self, x: float, y: float, theta: float):
        """Called programmatically (e.g., Nav2 bridge). NOT from map clicks."""
        if not self.active_robot_id:
            return

        has_nav2 = (self.nav_worker.ros_node is not None)
        has_pose = (self.active_robot_id in self.map_view.robots_pose)

        if has_nav2 and has_pose:
            start_pose = self.map_view.robots_pose[self.active_robot_id]
            self.terminal_view.append_log(2, f"Requesting Nav2 path to ({x:.2f}, {y:.2f})...")
            self.nav_worker.request_global_path(start_pose, (x, y, theta))
        else:
            self.terminal_view.append_log(
                2, "Nav2 unavailable — use Add Waypoint + Start."
            )

    def _on_path_ready(self, waypoints):
        self.map_view.waypoints = waypoints
        self.terminal_view.append_log(2, f"Global path: {len(waypoints)} waypoints.")
        self.map_view.update()

    def _on_set_pose(self, x: float, y: float, theta: float):
        if self.active_robot_id:
            self.zmq_client.set_pose(self.active_robot_id, x, y, theta)
            self.terminal_view.append_log(
                2, f"2D Pose set: ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}deg)"
            )
        self.toolbar.btn_pose.setChecked(False)

    # ── Navigation ────────────────────────────────────────────────────
    def _on_nav_start(self):
        if self.active_robot_id and self.map_view.waypoints:
            self.current_waypoint_idx = 0
            self.map_view.current_waypoint_idx = 0
            wx, wy = self.map_view.waypoints[0]
            self.zmq_client.send_nav_goal(self.active_robot_id, wx, wy, 0.0)
            self.terminal_view.append_log(
                2, f"Navigation started to Waypoint 1 ({wx:.2f}, {wy:.2f})"
            )

    def _on_nav_stop(self):
        if self.active_robot_id:
            self.zmq_client.send_nav_cancel(self.active_robot_id)
            self.terminal_view.append_log(2, "Navigation paused")

    def _on_nav_resume(self):
        if (self.active_robot_id and self.map_view.waypoints
                and self.current_waypoint_idx >= 0):
            self.map_view.current_waypoint_idx = self.current_waypoint_idx
            wx, wy = self.map_view.waypoints[self.current_waypoint_idx]
            self.zmq_client.send_nav_goal(self.active_robot_id, wx, wy, 0.0)
            self.terminal_view.append_log(
                2,
                f"Navigation resumed to Waypoint {self.current_waypoint_idx+1} "
                f"({wx:.2f}, {wy:.2f})"
            )

    def _on_nav_reset(self):
        if self.active_robot_id:
            self.zmq_client.send_nav_cancel(self.active_robot_id)
        self.map_view.clear_waypoints()
        self.current_waypoint_idx = -1
        self.map_view.current_waypoint_idx = -1
        self.terminal_view.append_log(2, "Navigation reset")

    def _on_add_waypoint(self):
        self.map_view.add_waypoint()

    def _on_disconnect_robot(self, robot_id: str):
        self.zmq_client.remove_robot(robot_id)
        if self.active_robot_id == robot_id:
            self.active_robot_id = None
            self.current_waypoint_idx = -1
            self.map_view.current_waypoint_idx = -1
        self.terminal_view.append_log(2, f"Robot {robot_id} disconnected")
        count = len(self.zmq_client.robots)
        self.toolbar.set_status(
            f"{count} Connected" if count else "Disconnected",
            "#a6e3a1" if count else "#7f849c",
        )

    # ── Data callbacks ────────────────────────────────────────────────
    def _on_odom(self, robot_id: str, msg):
        self.map_view.update_odom(robot_id, msg)

        if robot_id != self.active_robot_id or self.current_waypoint_idx < 0:
            return
        if self.current_waypoint_idx >= len(self.map_view.waypoints):
            return

        wx, wy = self.map_view.waypoints[self.current_waypoint_idx]
        dist = math.sqrt((msg.x - wx) ** 2 + (msg.y - wy) ** 2)

        if dist < 0.3:
            self.current_waypoint_idx += 1
            self.map_view.current_waypoint_idx = self.current_waypoint_idx
            if self.current_waypoint_idx < len(self.map_view.waypoints):
                nwx, nwy = self.map_view.waypoints[self.current_waypoint_idx]
                self.zmq_client.send_nav_goal(self.active_robot_id, nwx, nwy, 0.0)
                self.terminal_view.append_log(
                    2, f"Waypoint reached. Moving to next ({nwx:.2f}, {nwy:.2f})"
                )
            else:
                self.current_waypoint_idx = -1
                self.map_view.current_waypoint_idx = -1
                self.zmq_client.send_nav_cancel(self.active_robot_id)
                self.terminal_view.append_log(2, "Final goal reached!")
                self.toolbar.btn_start.setEnabled(True)

    def _on_battery(self, robot_id: str, msg):
        if robot_id == self.active_robot_id:
            self.battery_view.update_status(msg)

    def _on_lidar(self, robot_id: str, msg):
        # LiDAR data received but graph removed — no UI update needed
        pass

    def _on_command_failed(self, robot_id: str, reason: str):
        self.terminal_view.append_log(3, f"[{robot_id}] Command failed: {reason}")

    def _on_log(self, robot_id: str, msg):
        self.terminal_view.append_log(msg.severity, f"[{robot_id}] {msg.text}")

    def _on_frame(self, robot_id: str, frame_data: bytes):
        if robot_id == self.active_robot_id:
            self.video_view.update_frame(frame_data)

    def closeEvent(self, event):
        self.nav_worker.stop()
        self.zmq_client.stop()
        event.accept()
