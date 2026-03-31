import struct

from PyQt6.QtCore import QThread, pyqtSignal
import queue
from pinky_station.protocol import message_types as mt

class CommandWorker(QThread):
    sig_log = pyqtSignal(str)

    def __init__(self, tcp_client, parent=None):
        super().__init__(parent)
        self.client = tcp_client
        self._queue = queue.Queue()
        self._running = False

    def send_cmd_vel(self, linear_x: float, angular_z: float):
        payload = struct.pack('<ff', linear_x, angular_z)
        self._queue.put((mt.MSG_CMD_VEL, payload))

    def send_nav_goal(self, x: float, y: float, theta: float):
        payload = struct.pack('<3f', x, y, theta)
        self._queue.put((mt.MSG_NAV_GOAL, payload))

    def set_pose(self, x: float, y: float, theta: float):
        payload = struct.pack('<3f', x, y, theta)
        self._queue.put((mt.MSG_SET_POSE, payload))

    def run(self):
        self._running = True
        while self._running:
            try:
                # blocks for 0.1s so thread can exit properly
                msg_type, payload = self._queue.get(timeout=0.1)
                
                if self.client and self.client._running:
                    res = self.client.send_message(msg_type, payload)
                    if not res:
                        self.sig_log.emit(f"Failed to send TCP command (type={msg_type})")
                else:
                    self.sig_log.emit("Cannot send TCP command: Client disconnected.")
                    
                self._queue.task_done()
            except queue.Empty:
                pass
            except Exception as e:
                self.sig_log.emit(f"Command worker error: {e}")

    def stop(self):
        self._running = False
