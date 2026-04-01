from PyQt6.QtCore import QThread, pyqtSignal
from pinky_station.protocol.serializer import ParsedMessage
from pinky_station.protocol import message_types as mt


class SensorWorker(QThread):
    sig_battery = pyqtSignal(object)
    sig_odom = pyqtSignal(object)
    sig_lidar = pyqtSignal(object)
    sig_imu = pyqtSignal(object)

    def __init__(self, udp_receiver, parent=None):
        super().__init__(parent)
        self.receiver = udp_receiver
        self._running = False
        # Install callback from main thread before worker starts,
        # avoiding cross-thread race with the receiver's recv loop.
        self._old_cb = self.receiver.on_message
        self.receiver.on_message = self._dispatch

    def _dispatch(self, msg: ParsedMessage):
        if msg.msg_type == mt.MSG_BATTERY:
            self.sig_battery.emit(msg)
        elif msg.msg_type == mt.MSG_ODOM:
            self.sig_odom.emit(msg)
        elif msg.msg_type == mt.MSG_LIDAR_24:
            self.sig_lidar.emit(msg)
        elif msg.msg_type == mt.MSG_IMU:
            self.sig_imu.emit(msg)
        if self._old_cb:
            self._old_cb(msg)

    def run(self):
        self._running = True
        while self._running and self.receiver._running:
            self.msleep(100)

    def stop(self):
        self._running = False
