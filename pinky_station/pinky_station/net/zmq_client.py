import zmq
from PyQt6.QtCore import QThread, pyqtSignal, QObject
from pinky_station.protocol import pinky_pb2 as pb
import time

class ZmqReceiverThread(QThread):
    odom_received = pyqtSignal(str, object)
    lidar_received = pyqtSignal(str, object)
    battery_received = pyqtSignal(str, object)
    log_received = pyqtSignal(str, object)
    frame_received = pyqtSignal(str, bytes)  # emits jpeg bytes

    def __init__(self, robot_id: str, host: str, port: int):
        super().__init__()
        self.robot_id = robot_id
        self.host = host
        self.port = port
        self.running = False
        self.ctx = zmq.Context.instance()
        self.sub_socket = None

    def run(self):
        self.running = True
        self.sub_socket = self.ctx.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://{self.host}:{self.port}")
        # Subscribe to Telemetry ("T") and Video ("V")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "T")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "V")

        poller = zmq.Poller()
        poller.register(self.sub_socket, zmq.POLLIN)

        while self.running:
            try:
                events = dict(poller.poll(100))
                if self.sub_socket in events:
                    # Receive multipart: [topic, payload]
                    parts = self.sub_socket.recv_multipart(zmq.NOBLOCK)
                    if len(parts) != 2:
                        continue
                    
                    topic, data = parts
                    if topic == b"T":
                        telemetry = pb.SensorTelemetry()
                        telemetry.ParseFromString(data)
                        
                        if telemetry.HasField("odom"):
                            self.odom_received.emit(self.robot_id, telemetry.odom)
                        elif telemetry.HasField("lidar_scan"):
                            self.lidar_received.emit(self.robot_id, telemetry.lidar_scan)
                        elif telemetry.HasField("lidar_sectors"):
                            self.lidar_received.emit(self.robot_id, telemetry.lidar_sectors)
                        elif telemetry.HasField("battery"):
                            self.battery_received.emit(self.robot_id, telemetry.battery)
                        elif telemetry.HasField("log"):
                            self.log_received.emit(self.robot_id, telemetry.log)
                            
                    elif topic == b"V":
                        video = pb.VideoStream()
                        video.ParseFromString(data)
                        if video.HasField("frame"):
                            self.frame_received.emit(self.robot_id, video.frame.jpeg_data)

            except zmq.ZMQError:
                pass
            except Exception as e:
                print(f"ZmqReceiverThread error: {e}")

        if self.sub_socket:
            self.sub_socket.close()

    def stop(self):
        self.running = False
        self.wait()

class ZmqClient(QObject):
    odom_received = pyqtSignal(str, object)
    lidar_received = pyqtSignal(str, object)
    battery_received = pyqtSignal(str, object)
    log_received = pyqtSignal(str, object)
    frame_received = pyqtSignal(str, bytes)
    sig_command_failed = pyqtSignal(str, str)  # robot_id, reason

    def __init__(self):
        super().__init__()
        self.ctx = zmq.Context.instance()
        self.robots = {}
        self.request_id = 0

    def add_robot(self, robot_id: str, host: str, req_port: int, pub_port: int):
        if robot_id in self.robots:
            self.remove_robot(robot_id)
            
        req_socket = self.ctx.socket(zmq.REQ)
        req_socket.connect(f"tcp://{host}:{req_port}")
        
        receiver = ZmqReceiverThread(robot_id, host, pub_port)
        receiver.odom_received.connect(self.odom_received.emit)
        receiver.lidar_received.connect(self.lidar_received.emit)
        receiver.battery_received.connect(self.battery_received.emit)
        receiver.log_received.connect(self.log_received.emit)
        receiver.frame_received.connect(self.frame_received.emit)
        receiver.start()
        
        self.robots[robot_id] = {
            'req_socket': req_socket,
            'host': host,
            'req_port': req_port,
            'pub_port': pub_port,
            'receiver': receiver
        }

    def remove_robot(self, robot_id: str):
        if robot_id in self.robots:
            robot_data = self.robots.pop(robot_id)
            receiver = robot_data['receiver']
            req_socket = robot_data['req_socket']
            if receiver:
                receiver.stop()
            if req_socket:
                req_socket.close()

    def stop(self):
        for robot_id in list(self.robots.keys()):
            self.remove_robot(robot_id)

    def _send_command(self, robot_id: str, cmd: pb.ControlCommand):
        if robot_id not in self.robots:
            return False
            
        robot_data = self.robots[robot_id]
        req_socket = robot_data['req_socket']
            
        self.request_id += 1
        cmd.request_id = self.request_id
        try:
            req_socket.send(cmd.SerializeToString(), zmq.NOBLOCK)
            poller = zmq.Poller()
            poller.register(req_socket, zmq.POLLIN)
            if poller.poll(500):
                ack_data = req_socket.recv()
                ack = pb.CommandAck()
                ack.ParseFromString(ack_data)
                return ack.success
            else:
                # Timeout: recreate socket
                req_socket.close()
                new_req_socket = self.ctx.socket(zmq.REQ)
                new_req_socket.connect(f"tcp://{robot_data['host']}:{robot_data['req_port']}")
                self.robots[robot_id]['req_socket'] = new_req_socket
                self.sig_command_failed.emit(robot_id, "Command timeout — robot not responding")
                return False
        except Exception as e:
            self.sig_command_failed.emit(robot_id, str(e))
            return False

    def send_nav_goal(self, robot_id: str, x: float, y: float, theta: float):
        cmd = pb.ControlCommand()
        cmd.robot_id = robot_id
        cmd.nav_goal.x = x
        cmd.nav_goal.y = y
        cmd.nav_goal.theta = theta
        cmd.nav_goal.robot_id = robot_id
        self._send_command(robot_id, cmd)

    def send_nav_cancel(self, robot_id: str):
        cmd = pb.ControlCommand()
        cmd.robot_id = robot_id
        cmd.nav_cancel.robot_id = robot_id
        self._send_command(robot_id, cmd)

    def send_cmd_vel(self, robot_id: str, linear: float, angular: float):
        cmd = pb.ControlCommand()
        cmd.robot_id = robot_id
        cmd.cmd_vel.linear_x = linear
        cmd.cmd_vel.angular_z = angular
        self._send_command(robot_id, cmd)

    def set_pose(self, robot_id: str, x: float, y: float, theta: float):
        cmd = pb.ControlCommand()
        cmd.robot_id = robot_id
        cmd.set_pose.x = x
        cmd.set_pose.y = y
        cmd.set_pose.theta = theta
        self._send_command(robot_id, cmd)
