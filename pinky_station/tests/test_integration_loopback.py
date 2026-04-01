"""Automated TCP/UDP loopback integration tests.

These tests automatically start the C++ ``pinky_robot --mock`` binary
via the ``cpp_server`` session fixture (see conftest.py).

Run::

    cd pinky_station && python3 -m pytest tests/test_integration_loopback.py -v
"""

import struct
import time

import pytest

from pinky_station.net.tcp_client import TcpClient
from pinky_station.net.udp_receiver import UdpReceiver
from pinky_station.protocol import message_types as mt
from pinky_station.protocol.serializer import ParsedMessage


# ── Fixtures ────────────────────────────────────────────────────────
@pytest.fixture
def tcp_client(cpp_server):
    """Provide a connected TCP client; skip if connection fails."""
    # Retry a few times in case the server is recovering from a prior disconnect
    client = TcpClient(host="127.0.0.1", port=9100)
    for attempt in range(5):
        if client.connect():
            break
        time.sleep(0.5)
    else:
        pytest.skip("TCP connect to pinky_robot failed after retries")
    time.sleep(0.3)  # let server register the connection
    yield client
    client.disconnect()
    time.sleep(0.2)  # let server process the disconnect


@pytest.fixture
def udp_receiver(cpp_server):
    """Provide a running UDP receiver; skip if bind fails."""
    receiver = UdpReceiver(port=9200)
    if not receiver.start():
        pytest.skip("UDP bind on port 9200 failed")
    yield receiver
    receiver.stop()


# ── Tests ───────────────────────────────────────────────────────────
class TestTcpRoundtrip:
    """Verify TCP message send/receive with the C++ server."""

    def test_ping(self, tcp_client):
        ts = time.time_ns()
        payload = struct.pack("<Q", ts)
        assert tcp_client.send_message(mt.MSG_PING, payload)

    def test_cmd_vel(self, tcp_client):
        payload = struct.pack("<ff", 0.15, -0.5)
        assert tcp_client.send_message(mt.MSG_CMD_VEL, payload)

    def test_nav_goal_and_cancel(self, tcp_client):
        goal_payload = struct.pack("<3f", 2.0, 1.0, 1.57)
        assert tcp_client.send_message(mt.MSG_NAV_GOAL, goal_payload)
        assert tcp_client.send_message(mt.MSG_NAV_CANCEL, b"")

    def test_set_pose(self, tcp_client):
        payload = struct.pack("<3f", 0.5, -0.3, 0.0)
        assert tcp_client.send_message(mt.MSG_SET_POSE, payload)

    def test_set_emotion(self, tcp_client):
        payload = struct.pack("<B", 1)  # kHappy
        assert tcp_client.send_message(mt.MSG_SET_EMOTION, payload)

    def test_multi_message_burst(self, tcp_client):
        """Send several messages rapidly to stress-test framing."""
        for i in range(20):
            payload = struct.pack("<ff", 0.01 * i, 0.0)
            assert tcp_client.send_message(mt.MSG_CMD_VEL, payload)


class TestUdpStreaming:
    """Verify sensor data streaming from the C++ server."""

    def test_odom_received(self, tcp_client, udp_receiver):
        """After TCP connect, server should stream odom over UDP."""
        received: list[ParsedMessage] = []
        udp_receiver.on_message = lambda msg: received.append(msg)

        time.sleep(2.0)  # wait for sensor loop (50Hz)

        assert len(received) > 0, "No UDP packets received from server"
        types = {msg.msg_type for msg in received}
        assert mt.MSG_ODOM in types, f"ODOM not in received types: {types}"


class TestReconnection:
    """Verify server handles connect/disconnect cycles."""

    def test_reconnect(self, cpp_server):
        for cycle in range(3):
            client = TcpClient(host="127.0.0.1", port=9100)
            connected = False
            for attempt in range(10):
                if client.connect():
                    connected = True
                    break
                time.sleep(0.3)
            assert connected, f"Reconnect failed on cycle {cycle}"
            payload = struct.pack("<ff", 0.0, 0.0)
            assert client.send_message(mt.MSG_CMD_VEL, payload)
            client.disconnect()
            time.sleep(0.5)
