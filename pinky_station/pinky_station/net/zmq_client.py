import zmq
from PyQt6.QtCore import QThread, pyqtSignal, QObject
from pinky_station.protocol import pinky_pb2 as pb
import time

class ZmqReceiverThread(QThread):
    odom_received = pyqtSignal(object)
    lidar_received = pyqtSignal(object)
    battery_received = pyqtSignal(object)
    log_received = pyqtSignal(object)
    frame_received = pyqtSignal(bytes)  # emits jpeg bytes

    def __init__(self, host: str, port: int):
        super().__init__()
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
                            self.odom_received.emit(telemetry.odom)
                        elif telemetry.HasField("lidar_scan"):
                            self.lidar_received.emit(telemetry.lidar_scan)
                        elif telemetry.HasField("lidar_sectors"):
                            self.lidar_received.emit(telemetry.lidar_sectors)
                        elif telemetry.HasField("battery"):
                            self.battery_received.emit(telemetry.battery)
                        elif telemetry.HasField("log"):
                            self.log_received.emit(telemetry.log)
                            
                    elif topic == b"V":
                        video = pb.VideoStream()
                        video.ParseFromString(data)
                        if video.HasField("frame"):
                            self.frame_received.emit(video.frame.jpeg_data)

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
    odom_received = pyqtSignal(object)
    lidar_received = pyqtSignal(object)
    battery_received = pyqtSignal(object)
    log_received = pyqtSignal(object)
    frame_received = pyqtSignal(bytes)

    def __init__(self, pub_port=9200, req_port=9100):
        super().__init__()
        self.host = "127.0.0.1"
        self.pub_port = pub_port
        self.req_port = req_port
        self.ctx = zmq.Context.instance()
        self.req_socket = None
        self.receiver = None
        self.request_id = 0

    def connect_to(self, host: str):
        self.stop()
        self.host = host
        
        self.req_socket = self.ctx.socket(zmq.REQ)
        self.req_socket.connect(f"tcp://{self.host}:{self.req_port}")
        
        self.receiver = ZmqReceiverThread(self.host, self.pub_port)
        self.receiver.odom_received.connect(self.odom_received.emit)
        self.receiver.lidar_received.connect(self.lidar_received.emit)
        self.receiver.battery_received.connect(self.battery_received.emit)
        self.receiver.log_received.connect(self.log_received.emit)
        self.receiver.frame_received.connect(self.frame_received.emit)
        self.receiver.start()

    def stop(self):
        if self.receiver:
            self.receiver.stop()
            self.receiver = None
        if self.req_socket:
            self.req_socket.close()
            self.req_socket = None

    def _send_command(self, cmd: pb.ControlCommand):
        if not self.req_socket:
            return False
            
        self.request_id += 1
        cmd.request_id = self.request_id
        try:
            self.req_socket.send(cmd.SerializeToString(), zmq.NOBLOCK)
            poller = zmq.Poller()
            poller.register(self.req_socket, zmq.POLLIN)
            if poller.poll(500):
                ack_data = self.req_socket.recv()
                ack = pb.CommandAck()
                ack.ParseFromString(ack_data)
                return ack.success
            else:
                # Timeout: recreate socket
                self.req_socket.close()
                self.req_socket = self.ctx.socket(zmq.REQ)
                self.req_socket.connect(f"tcp://{self.host}:{self.req_port}")
                return False
        except Exception as e:
            print(f"Failed to send command: {e}")
            return False

    def send_nav_goal(self, x: float, y: float, theta: float):
        cmd = pb.ControlCommand()
        cmd.nav_goal.x = x
        cmd.nav_goal.y = y
        cmd.nav_goal.theta = theta
        self._send_command(cmd)

    def send_cmd_vel(self, linear: float, angular: float):
        cmd = pb.ControlCommand()
        cmd.cmd_vel.linear_x = linear
        cmd.cmd_vel.angular_z = angular
        self._send_command(cmd)

    def set_pose(self, x: float, y: float, theta: float):
        cmd = pb.ControlCommand()
        cmd.set_pose.x = x
        cmd.set_pose.y = y
        cmd.set_pose.theta = theta
        self._send_command(cmd)
