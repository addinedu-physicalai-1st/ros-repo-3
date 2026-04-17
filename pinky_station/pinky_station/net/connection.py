import threading
from enum import Enum
from typing import Callable, Optional

from pinky_station.protocol.serializer import ParsedMessage
from pinky_station.net.tcp_client import TcpClient
from pinky_station.net.udp_receiver import UdpReceiver

class ConnectionState(Enum):
    DISCONNECTED = 0
    CONNECTING = 1
    CONNECTED = 2

class ConnectionManager:
    def __init__(self, tcp_port: int = 9100, udp_port: int = 9200):
        self._state = ConnectionState.DISCONNECTED
        self._host = ""
        self._tcp_port = tcp_port
        self._udp_port = udp_port
        
        self.tcp: Optional[TcpClient] = None
        self.udp: Optional[UdpReceiver] = None
        
        # Callbacks
        self.on_state_change: Optional[Callable[[ConnectionState], None]] = None
        self.on_tcp_message: Optional[Callable[[ParsedMessage], None]] = None
        self.on_udp_message: Optional[Callable[[ParsedMessage], None]] = None

    def connect(self, host: str, tcp_port: int = 9100, udp_port: int = 9200):
        if self._state != ConnectionState.DISCONNECTED:
            return
            
        self._set_state(ConnectionState.CONNECTING)
        self._host = host
        self._tcp_port = tcp_port
        self._udp_port = udp_port
        
        # Start UDP listener
        self.udp = UdpReceiver(port=self._udp_port)
        self.udp.on_message = self._handle_udp_msg
        if not self.udp.start():
            self.disconnect()
            return
            
        # Connect TCP
        self.tcp = TcpClient(host=self._host, port=self._tcp_port)
        self.tcp.on_message = self._handle_tcp_msg
        self.tcp.on_disconnect = self._on_tcp_disconnect  
        if not self.tcp.connect():
            self.disconnect()
            return

        self._set_state(ConnectionState.CONNECTED)

    def disconnect(self):
        if self._state == ConnectionState.DISCONNECTED:
            return

        # Set state FIRST to prevent re-entrant calls from tcp.on_disconnect
        self._set_state(ConnectionState.DISCONNECTED)

        if self.tcp:
            # Clear callback before disconnect to break the recursion cycle
            self.tcp.on_disconnect = None
            self.tcp.disconnect()
            self.tcp = None

        if self.udp:
            self.udp.stop()
            self.udp = None

    def send_cmd_vel(self, linear_x: float, angular_z: float):
        if self.tcp and self._state == ConnectionState.CONNECTED:
            import struct
            payload = struct.pack('<ff', linear_x, angular_z)
            from pinky_station.protocol import message_types as mt
            self.tcp.send_message(mt.MSG_CMD_VEL, payload)

    def _set_state(self, new_state: ConnectionState):
        if self._state != new_state:
            self._state = new_state
            if self.on_state_change:
                self.on_state_change(self._state)

    def _on_tcp_disconnect(self):
        # Auto-disconnect completely if TCP fails
        self.disconnect()

    def _handle_tcp_msg(self, msg: ParsedMessage):
        if self.on_tcp_message:
            self.on_tcp_message(msg)

    def _handle_udp_msg(self, msg: ParsedMessage):
        if self.on_udp_message:
            self.on_udp_message(msg)
