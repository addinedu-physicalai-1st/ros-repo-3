import socket
import threading
import time
from typing import Callable, Optional

from pinky_station.protocol import message_types as mt
from pinky_station.protocol.serializer import Serializer, Deserializer, ParseResult, ParsedMessage

class TcpClient:
    def __init__(self, host: str, port: int = 9100):
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None
        
        self._running = False
        self._recv_thread: Optional[threading.Thread] = None
        self._ping_thread: Optional[threading.Thread] = None
        
        self.serializer = Serializer()
        self.deserializer = Deserializer()
        
        # Callbacks
        self.on_message: Optional[Callable[[ParsedMessage], None]] = None
        self.on_disconnect: Optional[Callable[[], None]] = None

    def connect(self) -> bool:
        if self._running:
            return True
            
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)  # Connect timeout
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(None)  # Blocking mode for recv thread
            
            self._running = True
            self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
            self._ping_thread = threading.Thread(target=self._ping_loop, daemon=True)
            
            self._recv_thread.start()
            self._ping_thread.start()
            return True
        except Exception as e:
            print(f"TcpClient connect error: {e}")
            self.disconnect()
            return False

    def disconnect(self):
        self._running = False
        if self.sock:
            try:
                # shutdown to interrupt recv
                self.sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            self.sock.close()
            self.sock = None
            
        if self._recv_thread and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=1.0)
        if self._ping_thread and self._ping_thread.is_alive():
            self._ping_thread.join(timeout=1.0)
            
        if self.on_disconnect:
            self.on_disconnect()

    def send_message(self, msg_type: int, payload: bytes) -> bool:
        if not self._running or not self.sock:
            return False
        
        frame = self.serializer.frame(msg_type, payload)
        try:
            self.sock.sendall(frame)
            return True
        except Exception as e:
            print(f"TcpClient send error: {e}")
            self.disconnect()
            return False

    def _recv_loop(self):
        buffer = bytearray()
        
        while self._running:
            try:
                data = self.sock.recv(4096)
                if not data:
                    break
                    
                buffer.extend(data)
                
                while len(buffer) >= 12: # Header size
                    res, msg, bytes_consumed = self.deserializer.parse(buffer)
                    if res == ParseResult.OK:
                        if self.on_message and msg:
                            # If it's a PING, respond with PONG immediately
                            if msg.msg_type == mt.MSG_PING:
                                try:
                                    import struct
                                    client_ts, = struct.unpack('<Q', msg.payload)
                                    server_ts = time.time_ns()
                                    pong_payload = struct.pack('<QQ', client_ts, server_ts)
                                    self.send_message(mt.MSG_PONG, pong_payload)
                                except Exception as e:
                                    print(f"Ping handle error: {e}")
                            else:
                                self.on_message(msg)
                        
                        buffer = buffer[bytes_consumed:]
                    elif res == ParseResult.INCOMPLETE:
                        break
                    else:
                        # Error parsing, drop 1 byte
                        buffer = buffer[1:]
                        
            except (socket.timeout, BlockingIOError):
                continue
            except Exception as e:
                if self._running:
                    print(f"TcpClient recv exception: {e}")
                break
                
        if self._running:
            self.disconnect()

    def _ping_loop(self):
        while self._running:
            try:
                # Send MSG_PING every 2 seconds
                ts = time.time_ns()
                payload = self.serializer.serialize_ping(ts)
                self.send_message(mt.MSG_PING, payload)
            except Exception:
                pass
            time.sleep(2.0)
