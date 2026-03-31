from PyQt6.QtCore import QThread, pyqtSignal
from pinky_station.protocol.serializer import ParsedMessage

class SensorWorker(QThread):
    sig_battery = pyqtSignal(object)
    sig_odom = pyqtSignal(object)
    sig_lidar = pyqtSignal(object)
    sig_imu = pyqtSignal(object)

    def __init__(self, udp_receiver, parent=None):
        super().__init__(parent)
        self.receiver = udp_receiver
        self._running = False

    def run(self):
        self._running = True
        # Original udp receiver uses a callback, we'll intercept it
        old_cb = self.receiver.on_message
        
        def cb(msg: ParsedMessage):
            from pinky_station.protocol import message_types as mt
            if msg.msg_type == mt.MSG_BATTERY:
                self.sig_battery.emit(msg)
            elif msg.msg_type == mt.MSG_ODOM:
                self.sig_odom.emit(msg)
            elif msg.msg_type == mt.MSG_LIDAR_24:
                self.sig_lidar.emit(msg)
            elif msg.msg_type == mt.MSG_IMU:
                self.sig_imu.emit(msg)
            if old_cb:
                old_cb(msg)
                
        self.receiver.on_message = cb
        
        # Worker block loop
        while self._running and self.receiver._running:
            self.msleep(100)

    def stop(self):
        self._running = False
