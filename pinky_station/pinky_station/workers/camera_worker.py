from PyQt6.QtCore import QThread, pyqtSignal
from pinky_station.protocol.serializer import ParsedMessage

class CameraWorker(QThread):
    sig_frame = pyqtSignal(bytes)

    def __init__(self, tcp_client, parent=None):
        super().__init__(parent)
        self.client = tcp_client
        self._running = False

    def run(self):
        self._running = True
        old_cb = self.client.on_message
        
        def cb(msg: ParsedMessage):
            from pinky_station.protocol import message_types as mt
            if msg.msg_type == mt.MSG_CAMERA_FRAME:
                # payload = width(u16) + height(u16) + size(u32) + jpeg_bytes
                # Header is <HHI = 2+2+4 = 8 bytes
                jpeg_data = msg.payload[8:]
                self.sig_frame.emit(jpeg_data)
                
            if old_cb:
                old_cb(msg)

        self.client.on_message = cb

        while self._running and self.client._running:
            self.msleep(100)

    def stop(self):
        self._running = False
