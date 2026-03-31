from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtGui import QPainter, QColor, QPen, QTransform, QPolygonF
from PyQt6.QtCore import Qt, pyqtSignal, QPointF
import struct
import math

# Simple Map Widget that draws robot position based on odometry
class MapWidget(QWidget):
    sig_set_goal = pyqtSignal(float, float, float)
    sig_set_pose = pyqtSignal(float, float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 400)
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        self.goal_x = None
        self.goal_y = None
        
        # Simple panning/zooming variables
        self.scale_factor = 50.0 # pixels per meter
        self.offset_x = 0.0
        self.offset_y = 0.0
        
        self.last_mouse_pos = None

    def update_odom(self, msg):
        try:
            # C++ SerializeOdom: x,y,theta,vx,vth as 5 x float32 = 20 bytes
            x, y, theta, vx, vth = struct.unpack('<5f', msg.payload[:20])
            self.robot_x = x
            self.robot_y = y
            self.robot_theta = theta
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
        
        # Draw grid
        pen_grid = QPen(QColor(60, 60, 60), 1)
        painter.setPen(pen_grid)
        
        # Calculate transform parameters (robot -> screen)
        # We'll center on the robot for simplicity unless an offset is set
        tx = cx + (self.offset_x - self.robot_x) * self.scale_factor
        ty = cy - (self.offset_y - self.robot_y) * self.scale_factor
        
        # Draw axes
        painter.setPen(QPen(QColor(255, 0, 0), 2)) # X-axis
        painter.drawLine(int(tx), int(ty), int(tx + self.scale_factor), int(ty))
        painter.setPen(QPen(QColor(0, 255, 0), 2)) # Y-axis
        painter.drawLine(int(tx), int(ty), int(tx), int(ty - self.scale_factor))

        # Draw robot
        painter.setPen(Qt.GlobalColor.transparent)
        painter.setBrush(QColor(0, 200, 255))
        
        rx_screen = cx + self.offset_x * self.scale_factor
        ry_screen = cy + self.offset_y * self.scale_factor
        
        robot_radius = 8
        painter.drawEllipse(QPointF(rx_screen, ry_screen), robot_radius, robot_radius)
        
        # Draw robot heading
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        hx = rx_screen + math.cos(self.robot_theta) * robot_radius * 2
        hy = ry_screen - math.sin(self.robot_theta) * robot_radius * 2
        painter.drawLine(int(rx_screen), int(ry_screen), int(hx), int(hy))

        # Draw goal if set
        if self.goal_x is not None and self.goal_y is not None:
            gx_screen = tx + self.goal_x * self.scale_factor
            gy_screen = ty - self.goal_y * self.scale_factor
            painter.setPen(Qt.GlobalColor.transparent)
            painter.setBrush(QColor(255, 0, 100))
            painter.drawEllipse(QPointF(gx_screen, gy_screen), 6, 6)
            
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            # Map screen coordinates to world coordinates
            cx = self.rect().width() / 2.0
            cy = self.rect().height() / 2.0
            
            # rx_screen = cx + offset_x * scale
            # tx = cx + (offset_x - robot_x) * scale => tx = rx_screen - robot_x * scale
            # sx = tx + x_world * scale => sx = cx + (offset_x - robot_x + x_world) * scale
            
            x_world = (event.position().x() - cx) / self.scale_factor - self.offset_x + self.robot_x
            y_world = -(event.position().y() - cy) / self.scale_factor + self.offset_y + self.robot_y
            
            self.goal_x = x_world
            self.goal_y = y_world
            
            # Emit navigation goal. Theta is simplified to 0 for now.
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
