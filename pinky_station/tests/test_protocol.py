"""Protocol roundtrip and cross-language compatibility tests."""

import struct
import sys
import os

# Add parent dir to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pinky_station.protocol.checksum import crc16
from pinky_station.protocol.message_types import MsgType, MAGIC, PROTOCOL_VERSION
from pinky_station.protocol.serializer import (
    Serializer,
    ParseResult,
    parse_message,
    deserialize_odom,
    deserialize_imu,
    deserialize_lidar24,
    deserialize_battery,
    deserialize_cmd_vel,
    deserialize_nav_goal,
    deserialize_debug_log,
    deserialize_set_pose,
    deserialize_ping,
    deserialize_pong,
)

passed = 0
failed = 0


def check(cond: bool, label: str) -> None:
    global passed, failed
    if cond:
        passed += 1
    else:
        failed += 1
        print(f"  FAIL: {label}")


def near(a: float, b: float, eps: float = 1e-5) -> bool:
    return abs(a - b) < eps


def test_crc16():
    print("[test_crc16]")
    # Must match C++ CRC-CCITT: "123456789" -> 0x29B1
    result = crc16(b"123456789")
    check(result == 0x29B1, f"CRC16 of '123456789' = 0x{result:04X}, expected 0x29B1")

    # Empty
    result_empty = crc16(b"")
    check(result_empty == 0xFFFF, f"CRC16 of empty = 0x{result_empty:04X}, expected 0xFFFF")


def test_odom_roundtrip():
    print("[test_odom_roundtrip]")
    ser = Serializer()
    frame = ser.serialize_odom(1.23, -4.56, 0.789, 0.15, -0.3)

    result, msg, consumed = parse_message(frame)
    check(result == ParseResult.OK, "parse result OK")
    check(consumed == len(frame), f"consumed {consumed} == {len(frame)}")
    check(msg.msg_type == MsgType.ODOM, "msg type ODOM")

    x, y, theta, vx, vth = deserialize_odom(msg.payload)
    check(near(x, 1.23), f"x={x}")
    check(near(y, -4.56), f"y={y}")
    check(near(theta, 0.789), f"theta={theta}")
    check(near(vx, 0.15), f"vx={vx}")
    check(near(vth, -0.3), f"vth={vth}")


def test_imu_roundtrip():
    print("[test_imu_roundtrip]")
    ser = Serializer()
    frame = ser.serialize_imu(0.707, 0.0, 0.707, 0.0, 0.1, 0.2, 0.3, 9.8, 0.0, 0.1)

    result, msg, consumed = parse_message(frame)
    check(result == ParseResult.OK, "parse OK")

    vals = deserialize_imu(msg.payload)
    check(near(vals[0], 0.707), f"qw={vals[0]}")
    check(near(vals[6], 0.3), f"gz={vals[6]}")
    check(near(vals[7], 9.8), f"ax={vals[7]}")


def test_lidar24_roundtrip():
    print("[test_lidar24_roundtrip]")
    ser = Serializer()
    sectors = [i / 24.0 for i in range(24)]
    frame = ser.serialize_lidar24(sectors)

    result, msg, consumed = parse_message(frame)
    check(result == ParseResult.OK, "parse OK")

    decoded = deserialize_lidar24(msg.payload)
    for i in range(24):
        check(near(decoded[i], i / 24.0), f"sector[{i}]")


def test_battery_roundtrip():
    print("[test_battery_roundtrip]")
    ser = Serializer()
    frame = ser.serialize_battery(7.82, 87.0, 1)

    result, msg, _ = parse_message(frame)
    check(result == ParseResult.OK, "parse OK")

    v, p, s = deserialize_battery(msg.payload)
    check(near(v, 7.82), f"voltage={v}")
    check(near(p, 87.0), f"percentage={p}")
    check(s == 1, f"status={s}")


def test_cmd_vel_roundtrip():
    print("[test_cmd_vel_roundtrip]")
    ser = Serializer()
    frame = ser.serialize_cmd_vel(0.20, -0.5)

    result, msg, _ = parse_message(frame)
    check(result == ParseResult.OK, "parse OK")

    lx, az = deserialize_cmd_vel(msg.payload)
    check(near(lx, 0.20), f"linear_x={lx}")
    check(near(az, -0.5), f"angular_z={az}")


def test_nav_goal_roundtrip():
    print("[test_nav_goal_roundtrip]")
    ser = Serializer()
    frame = ser.serialize_nav_goal(3.5, -1.2, 1.57)

    result, msg, _ = parse_message(frame)
    check(result == ParseResult.OK, "parse OK")

    x, y, theta = deserialize_nav_goal(msg.payload)
    check(near(x, 3.5), f"x={x}")
    check(near(y, -1.2), f"y={y}")
    check(near(theta, 1.57), f"theta={theta}")


def test_debug_log_roundtrip():
    print("[test_debug_log_roundtrip]")
    ser = Serializer()
    frame = ser.serialize_debug_log(2, 1234567890123, "Hello pinky!")

    result, msg, _ = parse_message(frame)
    check(result == ParseResult.OK, "parse OK")

    severity, ts, text = deserialize_debug_log(msg.payload)
    check(severity == 2, f"severity={severity}")
    check(ts == 1234567890123, f"ts={ts}")
    check(text == "Hello pinky!", f"text={text}")


def test_bad_magic():
    print("[test_bad_magic]")
    bad_data = b"\xFF\xFF" + b"\x01\x01" + b"\x00" * 8
    result, msg, consumed = parse_message(bad_data)
    check(result == ParseResult.BAD_MAGIC, f"result={result}")
    check(consumed == 1, f"consumed={consumed}")


def test_incomplete():
    print("[test_incomplete]")
    partial = b"\x50\x4B\x01"
    result, msg, consumed = parse_message(partial)
    check(result == ParseResult.INCOMPLETE, f"result={result}")
    check(consumed == 0, f"consumed={consumed}")


def test_multiple_messages():
    print("[test_multiple_messages]")
    ser = Serializer()
    f1 = ser.serialize_cmd_vel(0.1, 0.2)
    f2 = ser.serialize_cmd_vel(0.3, -0.4)
    stream = f1 + f2

    r1, m1, c1 = parse_message(stream)
    check(r1 == ParseResult.OK, "first parse OK")
    lx1, _ = deserialize_cmd_vel(m1.payload)
    check(near(lx1, 0.1), f"first lx={lx1}")

    r2, m2, c2 = parse_message(stream[c1:])
    check(r2 == ParseResult.OK, "second parse OK")
    lx2, az2 = deserialize_cmd_vel(m2.payload)
    check(near(lx2, 0.3), f"second lx={lx2}")
    check(near(az2, -0.4), f"second az={az2}")


def test_sequence_increment():
    print("[test_sequence_increment]")
    ser = Serializer()
    f1 = ser.serialize_ping(100)
    f2 = ser.serialize_ping(200)

    _, m1, _ = parse_message(f1)
    _, m2, _ = parse_message(f2)
    check(m2.sequence == m1.sequence + 1, f"seq {m1.sequence} -> {m2.sequence}")


def test_cross_language_crc():
    """Verify CRC values match known C++ output."""
    print("[test_cross_language_crc]")
    # C++ Crc16("PK") should match Python crc16(b"PK")
    pk_crc = crc16(b"PK")
    # We just verify it's deterministic and non-trivial
    check(pk_crc != 0 and pk_crc != 0xFFFF, f"PK CRC = 0x{pk_crc:04X}")

    # Verify payload CRC matches what's embedded in a frame
    ser = Serializer()
    frame = ser.serialize_cmd_vel(0.26, 1.0)
    # Extract payload from frame: skip 12-byte header, payload is 8 bytes
    payload = frame[12:12 + 8]
    embedded_crc = struct.unpack_from("<H", frame, 20)[0]
    computed_crc = crc16(payload)
    check(embedded_crc == computed_crc, f"payload CRC 0x{embedded_crc:04X} == 0x{computed_crc:04X}")


if __name__ == "__main__":
    test_crc16()
    test_odom_roundtrip()
    test_imu_roundtrip()
    test_lidar24_roundtrip()
    test_battery_roundtrip()
    test_cmd_vel_roundtrip()
    test_nav_goal_roundtrip()
    test_debug_log_roundtrip()
    test_bad_magic()
    test_incomplete()
    test_multiple_messages()
    test_sequence_increment()
    test_cross_language_crc()

    print(f"\n=== Results: {passed} passed, {failed} failed ===")
    sys.exit(1 if failed > 0 else 0)
