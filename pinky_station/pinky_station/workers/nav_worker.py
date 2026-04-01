import math
import struct
import threading
from typing import Optional

from PyQt6.QtCore import QThread, pyqtSignal

from pinky_station.protocol.serializer import ParsedMessage
from pinky_station.protocol import message_types as mt

# ROS 2 imports (will fail if ROS 2 is not sourced, handle gracefully)
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
    from tf2_ros import TransformBroadcaster
    HAS_ROS2 = True
    Node_Base = Node
except ImportError:
    HAS_ROS2 = False
    Node_Base = object


class RosBridgeNode(Node_Base):
    """ROS 2 Node that bridges pinky_station messages to/from ROS 2 topics."""

    def __init__(self, command_worker):
        super().__init__('pinky_station_bridge')
        self.cmd_worker = command_worker

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self._goal_callback, 10)

        self.get_logger().info('Pinky Station ROS 2 Bridge Node Started')

    def _cmd_vel_callback(self, msg: Twist):
        # Forward ROS 2 cmd_vel to the robot
        if self.cmd_worker:
            self.cmd_worker.send_cmd_vel(msg.linear.x, msg.angular.z)

    def _goal_callback(self, msg: PoseStamped):
        # Forward ROS 2 goal_pose to the robot
        if self.cmd_worker:
            x = msg.pose.position.x
            y = msg.pose.position.y
            
            # Extract yaw from quaternion
            q = msg.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            self.cmd_worker.send_nav_goal(x, y, yaw)

    def publish_odom(self, x, y, theta, vx, vth):
        now = self.get_clock().now().to_msg()

        # TF Broadcast
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        
        # yaw to quaternion
        t.transform.rotation.z = math.sin(theta / 2.0)
        t.transform.rotation.w = math.cos(theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

        # Odom Publish
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation = t.transform.rotation
        
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vth
        
        self.odom_pub.publish(odom_msg)


class NavWorker(QThread):
    """
    QThread that spins the ROS 2 Bridge Node.
    Connects PyQt signals (from SensorWorker) to ROS 2 publishers.
    """
    sig_log = pyqtSignal(str)

    def __init__(self, command_worker, parent=None):
        super().__init__(parent)
        self.command_worker = command_worker
        self.ros_node: Optional[RosBridgeNode] = None
        self._running = False

    def run(self):
        if not HAS_ROS2:
            self.sig_log.emit("ROS 2 (rclpy) not found. NavWorker disabled.")
            return

        try:
            # Check if rclpy is already initialized
            if not rclpy.ok():
                rclpy.init()

            self.ros_node = RosBridgeNode(self.command_worker)
            self._running = True
            
            self.sig_log.emit("Nav2 Bridge (ROS 2) started successfully.")
            
            # Spin until stopped
            while self._running and rclpy.ok():
                rclpy.spin_once(self.ros_node, timeout_sec=0.1)

        except Exception as e:
            self.sig_log.emit(f"NavWorker failed: {e}")
        finally:
            if self.ros_node:
                self.ros_node.destroy_node()
                self.ros_node = None

    def stop(self):
        self._running = False
        self.wait()

    def on_odom_received(self, msg: ParsedMessage):
        """Slot to receive Odom messages from SensorWorker."""
        if not self._running or not self.ros_node:
            return
            
        try:
            # C++ SerializeOdom: x, y, theta, vx, vth (5 x float32)
            x, y, theta, vx, vth = struct.unpack('<5f', msg.payload[:20])
            self.ros_node.publish_odom(x, y, theta, vx, vth)
        except Exception as e:
            self.sig_log.emit(f"Failed to publish odom to ROS: {e}")
