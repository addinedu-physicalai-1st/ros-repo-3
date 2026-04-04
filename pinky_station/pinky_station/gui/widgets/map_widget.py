from collections import deque
import yaml
from pathlib import Path

from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtGui import QPainter, QColor, QPen, QTransform, QPolygonF, QImage
from PyQt6.QtCore import Qt, pyqtSignal, QPointF, QRectF
import struct
import math

class MapWidget(QWidget):
    sig_set_goal = pyqtSignal(float, float, float)
    sig_set_pose = pyqtSignal(float, float, float)
    sig_potential_waypoint_selected = pyqtSignal(bool)

    def __init__(self, scale: float = 50.0, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 400)

        self.robots_pose = {} # {robot_id: (x, y, theta)}
        self.active_robot_id = None

        self.waypoints = [] # List of (x, y)
        self.potential_waypoint = None

        self.scale_factor = scale
        self.offset_x = 0.0
        self.offset_y = 0.0

        self.last_mouse_pos = None
        self.pose_mode = False
        self.is_dragging_pose = False
        self.pose_start_pt = None # (x, y) in world
        self.pose_current_theta = 0.0

        self.trail: deque[tuple[float, float]] = deque(maxlen=2000)
        self._trail_skip = 0
        
        self.map_image: QImage | None = None
        self.map_resolution = 0.05
        self.map_origin = (0.0, 0.0)

    def load_map(self, yaml_path: str | Path):
        yaml_path = Path(yaml_path)
        if not yaml_path.exists():
            return

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                
            image_name = data.get('image')
            if not image_name:
                return
                
            img_path = yaml_path.parent / image_name
            if not img_path.exists():
                return
                
            self.map_image = QImage(str(img_path))
            self.map_resolution = float(data.get('resolution', 0.05))
            origin = data.get('origin', [0.0, 0.0, 0.0])
            self.map_origin = (float(origin[0]), float(origin[1]))
            self.update()
        except Exception:
            pass

    def set_active_robot(self, robot_id: str):
        self.active_robot_id = robot_id
        self.trail.clear()
        self.update()

    def add_waypoint(self):
        if self.potential_waypoint:
            self.waypoints.append(self.potential_waypoint)
            self.potential_waypoint = None
            self.sig_potential_waypoint_selected.emit(False)
            self.update()

    def clear_waypoints(self):
        self.waypoints.clear()
        self.potential_waypoint = None
        self.update()

    def update_odom(self, robot_id: str, msg):
        try:
            if hasattr(msg, 'payload'):
                # Extract from payload if it's a raw bytes message (though it shouldn't be here)
                pass
            else:
                x = msg.x
                y = msg.y
                theta = msg.theta
                
            self.robots_pose[robot_id] = (x, y, theta)
            
            if robot_id == self.active_robot_id:
                self._trail_skip += 1
                if self._trail_skip >= 10:
                    self._trail_skip = 0
                    self.trail.append((x, y))
            self.update()
        except Exception:
            pass

    def get_world_to_screen_transform(self):
        cx = self.rect().width() / 2.0
        cy = self.rect().height() / 2.0
        
        # World to Screen:
        # 1. Scale
        # 2. Invert Y (world Y is up, screen Y is down)
        # 3. Translate by offset
        # 4. Translate to center
        t = QTransform()
        t.translate(cx, cy)
        t.scale(self.scale_factor, -self.scale_factor)
        t.translate(self.offset_x, self.offset_y)
        return t

    def get_screen_to_world_transform(self):
        t, success = self.get_world_to_screen_transform().inverted()
        return t

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        painter.fillRect(self.rect(), QColor(30, 30, 30))
        
        world_to_screen = self.get_world_to_screen_transform()

        # Draw Map
        if self.map_image and not self.map_image.isNull():
            w_m = self.map_image.width() * self.map_resolution
            h_m = self.map_image.height() * self.map_resolution
            
            # Map top-left in world: (origin_x, origin_y + height)
            map_rect_world = QRectF(self.map_origin[0], self.map_origin[1], w_m, h_m)
            map_rect_screen = world_to_screen.mapRect(map_rect_world)
            
            # Draw image flipped to match world coordinates
            painter.drawImage(map_rect_screen, self.map_image)
        else:
            # Draw simple grid if no map
            pass
        
        # Draw Origin Axes
        painter.setPen(QPen(QColor(255, 0, 0), 2))
        p0 = world_to_screen.map(QPointF(0, 0))
        px = world_to_screen.map(QPointF(1, 0))
        py = world_to_screen.map(QPointF(0, 1))
        painter.drawLine(p0, px)
        painter.setPen(QPen(QColor(0, 255, 0), 2))
        painter.drawLine(p0, py)

        # Draw Path (lines between waypoints)
        if len(self.waypoints) >= 2:
            painter.setPen(QPen(QColor(255, 255, 0, 100), 2, Qt.PenStyle.DashLine))
            for i in range(len(self.waypoints) - 1):
                p0 = world_to_screen.map(QPointF(*self.waypoints[i]))
                p1 = world_to_screen.map(QPointF(*self.waypoints[i+1]))
                painter.drawLine(p0, p1)

        # Draw Odometry trail for active robot
        if len(self.trail) >= 2:
            painter.setPen(QPen(QColor(80, 200, 80, 160), 2))
            pts = list(self.trail)
            for i in range(len(pts) - 1):
                p0 = world_to_screen.map(QPointF(*pts[i]))
                p1 = world_to_screen.map(QPointF(*pts[i+1]))
                painter.drawLine(p0, p1)

        # Draw Waypoints
        for i, (wx, wy) in enumerate(self.waypoints):
            w_screen = world_to_screen.map(QPointF(wx, wy))
            painter.setPen(Qt.GlobalColor.transparent)
            painter.setBrush(QColor(255, 0, 100))
            painter.drawEllipse(w_screen, 6, 6)
            
            # Label background for clarity
            painter.setPen(QPen(QColor(0, 0, 0, 150), 1))
            painter.setBrush(QColor(0, 0, 0, 150))
            label = f"GOAL {i+1}"
            painter.drawText(int(w_screen.x() + 10), int(w_screen.y() + 5), label)
            painter.setPen(QPen(Qt.GlobalColor.white))
            painter.drawText(int(w_screen.x() + 10), int(w_screen.y() + 5), label)

        # Draw Potential Waypoint
        if self.potential_waypoint:
            p_screen = world_to_screen.map(QPointF(*self.potential_waypoint))
            painter.setPen(QPen(QColor(255, 0, 100), 2, Qt.PenStyle.DotLine))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawEllipse(p_screen, 8, 8)

        # Draw Pose Estimate Arrow (while dragging)
        if self.is_dragging_pose and self.pose_start_pt:
            ps_screen = world_to_screen.map(QPointF(*self.pose_start_pt))
            painter.setPen(QPen(QColor(0, 255, 255), 3))
            painter.setBrush(QColor(0, 255, 255, 100))
            painter.drawEllipse(ps_screen, 10, 10)
            
            # Arrow head
            arrow_len = 40
            hx = ps_screen.x() + math.cos(-self.pose_current_theta) * arrow_len
            hy = ps_screen.y() + math.sin(-self.pose_current_theta) * arrow_len
            painter.drawLine(ps_screen, QPointF(hx, hy))
            
            # Small arrow tips
            tip_angle = 0.5
            tip_len = 10
            t1x = hx + math.cos(-self.pose_current_theta + math.pi - tip_angle) * tip_len
            t1y = hy + math.sin(-self.pose_current_theta + math.pi - tip_angle) * tip_len
            t2x = hx + math.cos(-self.pose_current_theta + math.pi + tip_angle) * tip_len
            t2y = hy + math.sin(-self.pose_current_theta + math.pi + tip_angle) * tip_len
            painter.drawLine(QPointF(hx, hy), QPointF(t1x, t1y))
            painter.drawLine(QPointF(hx, hy), QPointF(t2x, t2y))

        # Draw Robots
        robot_radius = 8
        for rid, (rx, ry, rtheta) in self.robots_pose.items():
            r_screen = world_to_screen.map(QPointF(rx, ry))
            
            if rid == self.active_robot_id:
                # Highlight active robot with cyan glow/border
                painter.setPen(QPen(QColor(0, 255, 255), 3))
                painter.setBrush(QColor(0, 200, 255))
            else:
                painter.setPen(QPen(QColor(150, 150, 150), 1))
                painter.setBrush(QColor(100, 100, 100))
                
            painter.drawEllipse(r_screen, robot_radius, robot_radius)
            
            # Heading line
            painter.setPen(QPen(Qt.GlobalColor.white, 2))
            hx = r_screen.x() + math.cos(-rtheta) * robot_radius * 2
            hy = r_screen.y() + math.sin(-rtheta) * robot_radius * 2
            painter.drawLine(r_screen, QPointF(hx, hy))

    def set_pose_mode(self, active: bool):
        self.pose_mode = active

    def mousePressEvent(self, event):
        world_pt = self.get_screen_to_world_transform().map(event.position())

        if event.button() == Qt.MouseButton.LeftButton:
            if self.pose_mode:
                self.is_dragging_pose = True
                self.pose_start_pt = (world_pt.x(), world_pt.y())
                self.pose_current_theta = 0.0
            else:
                self.potential_waypoint = (world_pt.x(), world_pt.y())
                self.sig_potential_waypoint_selected.emit(True)
            self.update()
        elif event.button() == Qt.MouseButton.RightButton:
            self.last_mouse_pos = event.position()

    def mouseMoveEvent(self, event):
        world_pt = self.get_screen_to_world_transform().map(event.position())

        if self.is_dragging_pose:
            dx = world_pt.x() - self.pose_start_pt[0]
            dy = world_pt.y() - self.pose_start_pt[1]
            self.pose_current_theta = math.atan2(dy, dx)
            self.update()
        elif self.last_mouse_pos is not None:
            # Panning
            dx = event.position().x() - self.last_mouse_pos.x()
            dy = event.position().y() - self.last_mouse_pos.y()
            
            self.offset_x += dx / self.scale_factor
            self.offset_y -= dy / self.scale_factor
            self.last_mouse_pos = event.position()
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            if self.is_dragging_pose:
                self.sig_set_pose.emit(self.pose_start_pt[0], self.pose_start_pt[1], self.pose_current_theta)
                self.is_dragging_pose = False
                self.pose_start_pt = None
                self.update()
        elif event.button() == Qt.MouseButton.RightButton:
            self.last_mouse_pos = None

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta > 0:
            self.scale_factor *= 1.1
        else:
            self.scale_factor *= 0.9
        self.update()
