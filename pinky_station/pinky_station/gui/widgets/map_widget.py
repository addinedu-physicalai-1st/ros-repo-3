from collections import deque
import yaml
from pathlib import Path

from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtGui import QPainter, QColor, QPen, QTransform, QPolygonF, QImage
from PyQt6.QtCore import Qt, pyqtSignal, QPointF, QRectF
import struct
import math

# Simple Map Widget that draws robot position based on odometry
class MapWidget(QWidget):
    sig_set_goal = pyqtSignal(float, float, float)
    sig_set_pose = pyqtSignal(float, float, float)

    def __init__(self, scale: float = 50.0, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 400)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.goal_x = None
        self.goal_y = None

        # Simple panning/zooming variables
        self.scale_factor = scale  # pixels per meter
        self.offset_x = 0.0
        self.offset_y = 0.0

        self.last_mouse_pos = None
        self.pose_mode = False

        # Odometry trail (world coordinates)
        self.trail: deque[tuple[float, float]] = deque(maxlen=2000)
        self._trail_skip = 0
        
        # ROS 2 Map properties
        self.map_image: QImage | None = None
        self.map_resolution = 0.05
        self.map_origin = (0.0, 0.0)

    def load_map(self, yaml_path: str | Path):
        yaml_path = Path(yaml_path)
        if not yaml_path.exists():
            print(f"Map yaml {yaml_path} does not exist.")
            return

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                
            image_name = data.get('image')
            if not image_name:
                return
                
            img_path = yaml_path.parent / image_name
            if not img_path.exists():
                print(f"Map image {img_path} does not exist.")
                return
                
            self.map_image = QImage(str(img_path))
            self.map_resolution = float(data.get('resolution', 0.05))
            origin = data.get('origin', [0.0, 0.0, 0.0])
            self.map_origin = (float(origin[0]), float(origin[1]))
            print(f"Loaded map {img_path} with resolution {self.map_resolution} and origin {self.map_origin}")
            self.update()
        except Exception as e:
            print(f"Failed to load map {yaml_path}: {e}")

    def update_odom(self, msg):
        try:
            # We support both the raw payload (from UDP) or direct attributes (from decoded Protobuf)
            if hasattr(msg, 'payload'):
                # C++ SerializeOdom: x,y,theta,vx,vth as 5 x float32 = 20 bytes
                x, y, theta, vx, vth = struct.unpack('<5f', msg.payload[:20])
            else:
                x = msg.x
                y = msg.y
                theta = msg.theta
            self.robot_x = x
            self.robot_y = y
            self.robot_theta = theta
            # Record trail at ~5Hz (odom arrives at 50Hz)
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
        
        # Fill background
        painter.fillRect(self.rect(), QColor(30, 30, 30))
        
        # Center of widget
        cx = self.rect().width() / 2.0
        cy = self.rect().height() / 2.0
        
        tx = cx + self.offset_x * self.scale_factor
        ty = cy - self.offset_y * self.scale_factor

        # Draw Map Image if loaded
        if self.map_image and not self.map_image.isNull():
            w_m = self.map_image.width() * self.map_resolution
            h_m = self.map_image.height() * self.map_resolution
            
            sx = tx + self.map_origin[0] * self.scale_factor
            sy = ty - (self.map_origin[1] + h_m) * self.scale_factor
            sw = w_m * self.scale_factor
            sh = h_m * self.scale_factor
            
            painter.drawImage(QRectF(sx, sy, sw, sh), self.map_image)
        else:
            # Draw grid if no map
            pen_grid = QPen(QColor(60, 60, 60), 1)
            painter.setPen(pen_grid)
        
        # Draw world origin axes (Red=X, Green=Y) (Reverted change #3-1)
        painter.setPen(QPen(QColor(255, 0, 0), 2)) # X-axis
        painter.drawLine(int(tx), int(ty), int(tx + self.scale_factor), int(ty))
        painter.setPen(QPen(QColor(0, 255, 0), 2)) # Y-axis
        painter.drawLine(int(tx), int(ty), int(tx), int(ty - self.scale_factor))

        # Draw robot
        painter.setPen(Qt.GlobalColor.transparent)
        painter.setBrush(QColor(0, 200, 255))
        
        rx_screen = tx + self.robot_x * self.scale_factor
        ry_screen = ty - self.robot_y * self.scale_factor
        
        robot_radius = 8
        painter.drawEllipse(QPointF(rx_screen, ry_screen), robot_radius, robot_radius)
        
        # Draw robot heading
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        hx = rx_screen + math.cos(self.robot_theta) * robot_radius * 2
        hy = ry_screen - math.sin(self.robot_theta) * robot_radius * 2
        painter.drawLine(int(rx_screen), int(ry_screen), int(hx), int(hy))

        # Draw odometry trail
        if len(self.trail) >= 2:
            painter.setPen(QPen(QColor(80, 200, 80, 160), 2))
            pts = list(self.trail)
            for i in range(len(pts) - 1):
                x0 = tx + pts[i][0] * self.scale_factor
                y0 = ty - pts[i][1] * self.scale_factor
                x1 = tx + pts[i + 1][0] * self.scale_factor
                y1 = ty - pts[i + 1][1] * self.scale_factor
                painter.drawLine(int(x0), int(y0), int(x1), int(y1))

        # Draw goal if set
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
