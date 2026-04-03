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

    def __init__(self, scale: float = 50.0, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 400)

        self.robots_pose = {} # {robot_id: (x, y, theta)}
        self.active_robot_id = None

        self.goal_x = None
        self.goal_y = None

        self.scale_factor = scale
        self.offset_x = 0.0
        self.offset_y = 0.0

        self.last_mouse_pos = None
        self.pose_mode = False

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

    def update_odom(self, robot_id: str, msg):
        try:
            if hasattr(msg, 'payload'):
                x, y, theta, vx, vth = struct.unpack('<5f', msg.payload[:20])
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

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        painter.fillRect(self.rect(), QColor(30, 30, 30))
        
        cx = self.rect().width() / 2.0
        cy = self.rect().height() / 2.0
        
        tx = cx + self.offset_x * self.scale_factor
        ty = cy - self.offset_y * self.scale_factor

        if self.map_image and not self.map_image.isNull():
            w_m = self.map_image.width() * self.map_resolution
            h_m = self.map_image.height() * self.map_resolution
            
            sx = tx + self.map_origin[0] * self.scale_factor
            sy = ty - (self.map_origin[1] + h_m) * self.scale_factor
            sw = w_m * self.scale_factor
            sh = h_m * self.scale_factor
            
            painter.drawImage(QRectF(sx, sy, sw, sh), self.map_image)
        else:
            pen_grid = QPen(QColor(60, 60, 60), 1)
            painter.setPen(pen_grid)
        
        painter.setPen(QPen(QColor(255, 0, 0), 2))
        painter.drawLine(int(tx), int(ty), int(tx + self.scale_factor), int(ty))
        painter.setPen(QPen(QColor(0, 255, 0), 2))
        painter.drawLine(int(tx), int(ty), int(tx), int(ty - self.scale_factor))

        robot_radius = 8
        
        # Draw odometry trail for active robot
        if len(self.trail) >= 2:
            painter.setPen(QPen(QColor(80, 200, 80, 160), 2))
            pts = list(self.trail)
            for i in range(len(pts) - 1):
                x0 = tx + pts[i][0] * self.scale_factor
                y0 = ty - pts[i][1] * self.scale_factor
                x1 = tx + pts[i + 1][0] * self.scale_factor
                y1 = ty - pts[i + 1][1] * self.scale_factor
                painter.drawLine(int(x0), int(y0), int(x1), int(y1))

        # Draw all robots
        for rid, (rx, ry, rtheta) in self.robots_pose.items():
            painter.setPen(Qt.GlobalColor.transparent)
            if rid == self.active_robot_id:
                painter.setBrush(QColor(0, 200, 255)) # Cyan for active
                pen_color = QColor(255, 255, 255)
            else:
                painter.setBrush(QColor(150, 150, 150)) # Gray for others
                pen_color = QColor(200, 200, 200)
                
            rx_screen = tx + rx * self.scale_factor
            ry_screen = ty - ry * self.scale_factor
            
            painter.drawEllipse(QPointF(rx_screen, ry_screen), robot_radius, robot_radius)
            
            painter.setPen(QPen(pen_color, 2))
            hx = rx_screen + math.cos(rtheta) * robot_radius * 2
            hy = ry_screen - math.sin(rtheta) * robot_radius * 2
            painter.drawLine(int(rx_screen), int(ry_screen), int(hx), int(hy))

        if self.goal_x is not None and self.goal_y is not None:
            gx_screen = tx + self.goal_x * self.scale_factor
            gy_screen = ty - self.goal_y * self.scale_factor
            painter.setPen(Qt.GlobalColor.transparent)
            painter.setBrush(QColor(255, 0, 100))
            painter.drawEllipse(QPointF(gx_screen, gy_screen), 6, 6)
            
    def set_pose_mode(self, active: bool):
        self.pose_mode = active

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            cx = self.rect().width() / 2.0
            cy = self.rect().height() / 2.0

            x_world = (event.position().x() - cx) / self.scale_factor - self.offset_x
            y_world = -(event.position().y() - cy) / self.scale_factor + self.offset_y

            if self.pose_mode:
                self.sig_set_pose.emit(x_world, y_world, 0.0)
            else:
                self.goal_x = x_world
                self.goal_y = y_world
                self.sig_set_goal.emit(self.goal_x, self.goal_y, 0.0)
            self.update()
        elif event.button() == Qt.MouseButton.RightButton:
            self.last_mouse_pos = event.position()

    def mouseMoveEvent(self, event):
        if self.last_mouse_pos is not None:
            dx = event.position().x() - self.last_mouse_pos.x()
            dy = event.position().y() - self.last_mouse_pos.y()
            
            self.offset_x += dx / self.scale_factor
            self.offset_y -= dy / self.scale_factor
            self.last_mouse_pos = event.position()
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.RightButton:
            self.last_mouse_pos = None

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta > 0:
            self.scale_factor *= 1.1
        else:
            self.scale_factor *= 0.9
        self.update()
