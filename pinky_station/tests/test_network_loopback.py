"""
TCP/UDP loopback integration test.

이 통합 테스트는 C++ 실행파일 `./pinky_robot --mock` 이
같은 PC(또는 보드) 백그라운드에서 실행 중이라는 가정하에 동작합니다.

실행 방법:
  1) 터미널 A: cd pinky_core/build && ./pinky_robot --mock
  2) 터미널 B: cd pinky_station && python3 -m pytest tests/test_network_loopback.py -v
"""
import pytest
import time
import struct

from pinky_station.net.tcp_client import TcpClient
from pinky_station.net.udp_receiver import UdpReceiver
from pinky_station.protocol import message_types as mt
from pinky_station.protocol.serializer import ParsedMessage


@pytest.fixture
def network_clients():
    tcp = TcpClient(host="127.0.0.1", port=9100)
    udp = UdpReceiver(port=9200)

    tcp_ok = tcp.connect()
    udp_ok = udp.start()

    # 서버가 클라이언트 IP를 인식하고 UDP 타겟을 세팅할 시간 확보
    time.sleep(0.5)

    yield tcp, udp, tcp_ok, udp_ok

    tcp.disconnect()
    udp.stop()


def test_tcp_ping_pong_integration(network_clients):
    tcp, udp, tcp_ok, udp_ok = network_clients

    if not tcp_ok:
        pytest.skip("C++ 서버(pinky_robot)에 연결할 수 없습니다.")

    assert tcp._running is True

    # Ping 전송
    current_time_ns = time.time_ns()
    payload = struct.pack('<Q', current_time_ns)
    success = tcp.send_message(mt.MSG_PING, payload)
    assert success is True


def test_udp_streaming_integration(network_clients):
    tcp, udp, tcp_ok, udp_ok = network_clients

    if not tcp_ok:
        pytest.skip("C++ 서버(pinky_robot)에 연결할 수 없습니다.")
    if not udp_ok:
        pytest.skip("UDP 수신 소켓을 열 수 없습니다 (포트 점유).")

    received_msgs: list[ParsedMessage] = []

    # 콜백 등록 — UdpReceiver는 on_message 속성에 Callable[[ParsedMessage], None] 할당
    udp.on_message = lambda msg: received_msgs.append(msg)

    # C++ 로봇이 센서 데이터를 보낼 시간 확보
    time.sleep(2.0)

    assert len(received_msgs) > 0, (
        "C++ 서버로부터 수신된 패킷이 없습니다. "
        "pinky_robot이 실행 중인지 확인하세요."
    )

    # 수신된 메시지 타입 분류
    types_received = {msg.msg_type for msg in received_msgs}
    print(f"\n[수신된 메시지 타입]: {types_received}")
    print(f"[총 수신 패킷]: {len(received_msgs)}")

    # ODOM 수신 확인 (50Hz이므로 2초면 ~100개)
    assert mt.MSG_ODOM in types_received, (
        "ODOM 패킷 미수신. SerializeOdom/Frame 프레이밍을 확인하세요."
    )


def test_tcp_cmd_vel_roundtrip(network_clients):
    """CMD_VEL 전송이 오류 없이 완료되는지 확인."""
    tcp, udp, tcp_ok, udp_ok = network_clients

    if not tcp_ok:
        pytest.skip("C++ 서버(pinky_robot)에 연결할 수 없습니다.")

    payload = struct.pack('<ff', 0.1, 0.5)
    success = tcp.send_message(mt.MSG_CMD_VEL, payload)
    assert success is True


def test_tcp_nav_goal_roundtrip(network_clients):
    """NAV_GOAL 전송이 오류 없이 완료되는지 확인."""
    tcp, udp, tcp_ok, udp_ok = network_clients

    if not tcp_ok:
        pytest.skip("C++ 서버(pinky_robot)에 연결할 수 없습니다.")

    payload = struct.pack('<3f', 1.0, 2.0, 0.0)
    success = tcp.send_message(mt.MSG_NAV_GOAL, payload)
    assert success is True

    # Cancel
    success = tcp.send_message(mt.MSG_NAV_CANCEL, b'')
    assert success is True
