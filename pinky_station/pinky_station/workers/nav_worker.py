from __future__ import annotations

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
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
    from sensor_msgs.msg import LaserScan
    from nav2_msgs.srv import ComputePathToPose
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
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Service Clients
        self.path_client = self.create_client(ComputePathToPose, 'compute_path_to_pose')

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
        # This is triggered when a goal is set via RViz or other ROS tools
        if self.cmd_worker:
            x = msg.pose.position.x
            y = msg.pose.position.y
            
            q = msg.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            self.cmd_worker.send_nav_goal(x, y, yaw)

    def request_path(self, start_pose, goal_pose, callback):
        if not self.path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Nav2 ComputePathToPose service not available')
            return False

        req = ComputePathToPose.Request()
        req.start.header.frame_id = 'map'
        req.start.pose.position.x = start_pose[0]
        req.start.pose.position.y = start_pose[1]
        
        req.goal.header.frame_id = 'map'
        req.goal.pose.position.x = goal_pose[0]
        req.goal.pose.position.y = goal_pose[1]

        future = self.path_client.call_async(req)
        future.add_done_callback(callback)
        return True

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

    def publish_scan(self, scan):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'rplidar_link'
        scan_msg.angle_min = scan.angle_min
        scan_msg.angle_max = scan.angle_max
        scan_msg.angle_increment = scan.angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1  # assuming 10Hz
        scan_msg.range_min = scan.range_min
        scan_msg.range_max = scan.range_max
        scan_msg.ranges = list(scan.ranges)
        self.scan_pub.publish(scan_msg)


class NavWorker(QThread):
    """
    QThread that spins the ROS 2 Bridge Node.
    """
    sig_log = pyqtSignal(str)
    sig_path_ready = pyqtSignal(list) # List of (x, y) waypoints

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
            if not rclpy.ok():
                rclpy.init()

            self.ros_node = RosBridgeNode(self.command_worker)
            self._running = True
            self.sig_log.emit("Nav2 Bridge (ROS 2) started successfully.")
            
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

    def request_global_path(self, start_pose, goal_pose):
        if not self.ros_node:
            return
            
        def path_callback(future):
            try:
                response = future.result()
                if response and response.path:
                    # Sample path every N points to create waypoints (e.g., every 5th point)
                    # Or based on distance. Let's take points at ~0.3m intervals.
                    waypoints = []
                    last_pt = None
                    for pose in response.path.poses:
                        pt = (pose.pose.position.x, pose.pose.position.y)
                        if last_pt is None:
                            waypoints.append(pt)
                            last_pt = pt
                        else:
                            dist = math.sqrt((pt[0]-last_pt[0])**2 + (pt[1]-last_pt[1])**2)
                            if dist > 0.3: # 30cm interval
                                waypoints.append(pt)
                                last_pt = pt
                    
                    # Ensure final goal is included
                    final_goal = (goal_pose[0], goal_pose[1])
                    if waypoints[-1] != final_goal:
                        waypoints.append(final_goal)
                        
                    self.sig_path_ready.emit(waypoints)
                else:
                    self.sig_log.emit("Nav2 failed to find a path.")
            except Exception as e:
                self.sig_log.emit(f"Path request error: {e}")

        self.ros_node.request_path(start_pose, goal_pose, path_callback)

    def on_odom_received(self, robot_id, msg):
        """Slot to receive Odom data."""
        if not self._running or not self.ros_node:
            return
        
        # In this multi-robot version, msg is already an object from ZmqClient
        try:
            self.ros_node.publish_odom(msg.x, msg.y, msg.theta, msg.vx, msg.vth)
        except Exception as e:
            print(f"Failed to publish odom to ROS: {e}")

    def on_lidar_scan_received(self, robot_id, scan):
        """Slot to receive LiDAR scan data."""
        if not self._running or not self.ros_node:
            return
        
        try:
            self.ros_node.publish_scan(scan)
        except Exception as e:
            print(f"Failed to publish scan to ROS: {e}")
