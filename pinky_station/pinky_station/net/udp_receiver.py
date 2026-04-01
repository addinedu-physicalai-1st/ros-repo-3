import socket
import threading
from typing import Callable, Optional

from pinky_station.protocol import message_types as mt
from pinky_station.protocol.serializer import Deserializer, ParseResult, ParsedMessage

class UdpReceiver:
    def __init__(self, port: int = 9200):
        self.port = port
        self.sock: Optional[socket.socket] = None
        
        self._running = False
        self._recv_thread: Optional[threading.Thread] = None
        
        self.deserializer = Deserializer()
        
        # Callbacks
        self.on_message: Optional[Callable[[ParsedMessage], None]] = None

    def start(self) -> bool:
        if self._running:
            return True
            
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if hasattr(socket, 'SO_REUSEPORT'):
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            self.sock.bind(('', self.port))
            
            self._running = True
            self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
            self._recv_thread.start()
            return True
        except Exception as e:
            print(f"UdpReceiver start error: {e}")
            self.stop()
            return False

    def stop(self):
        self._running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            
        if self._recv_thread and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=1.0)

    def _recv_loop(self):
        # UDP datagrams are received whole.
        while self._running:
            try:
                data, addr = self.sock.recvfrom(65535)
                if not data:
                    continue
                
                # Try to parse the message
                res, msg, consumed = self.deserializer.parse(data)
                
                if res == ParseResult.OK and msg and self.on_message:
                    # UDP can drop or out-of-order, but CRC passed
                    self.on_message(msg)
                elif res != ParseResult.INCOMPLETE:
                    # Parse error in datagram -> discard
                    pass
                    
            except OSError:
                break
            except Exception as e:
                if self._running:
                    print(f"UdpReceiver recv exception: {e}")
                break
