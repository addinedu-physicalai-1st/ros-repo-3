"""Serialize / deserialize PK protocol messages — mirrors C++ serializer."""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional

from .checksum import crc16
from .message_types import (
    FRAME_OVERHEAD,
    HEADER_FORMAT,
    HEADER_SIZE,
    CRC_SIZE,
    MAGIC,
    PROTOCOL_VERSION,
    MsgType,
)


# ---------------------------------------------------------------------------
# Parse result
# ---------------------------------------------------------------------------
class ParseResult(IntEnum):
    OK = 0
    INCOMPLETE = 1
    BAD_MAGIC = 2
    BAD_VERSION = 3
    BAD_HEADER_CRC = 4
    BAD_PAYLOAD_CRC = 5


@dataclass
class ParsedMessage:
    msg_type: MsgType = MsgType.ODOM
    sequence: int = 0
    payload: bytes = b""


# ---------------------------------------------------------------------------
# Serializer (builds framed messages)
# ---------------------------------------------------------------------------
class Serializer:
    def __init__(self) -> None:
        self._sequence = 0

    def frame(self, msg_type: MsgType, payload: bytes) -> bytes:
        """Build a complete framed message: header + payload + payload_crc."""
        seq = self._sequence & 0xFFFF
        self._sequence += 1

        # Build header without CRC (first 10 bytes)
        partial = struct.pack(
            "<2sBBHI",
            MAGIC,
            PROTOCOL_VERSION,
            int(msg_type),
            seq,
            len(payload),
        )
        header_crc = crc16(partial)
        header = partial + struct.pack("<H", header_crc)

        payload_crc = struct.pack("<H", crc16(payload))
        return header + payload + payload_crc

    # -- Sensor data (Robot -> PC) -----------------------------------------

    def serialize_odom(
        self, x: float, y: float, theta: float, vx: float, vth: float
    ) -> bytes:
        return self.frame(
            MsgType.ODOM, struct.pack("<5f", x, y, theta, vx, vth)
        )

    def serialize_imu(
        self,
        qw: float, qx: float, qy: float, qz: float,
        gx: float, gy: float, gz: float,
        ax: float, ay: float, az: float,
    ) -> bytes:
        return self.frame(
            MsgType.IMU,
            struct.pack("<10f", qw, qx, qy, qz, gx, gy, gz, ax, ay, az),
        )

    def serialize_lidar24(self, sectors: list[float]) -> bytes:
        assert len(sectors) == 24
        return self.frame(
            MsgType.LIDAR_24, struct.pack("<24f", *sectors)
        )

    def serialize_battery(
        self, voltage: float, percentage: float, status: int
    ) -> bytes:
        return self.frame(
            MsgType.BATTERY, struct.pack("<ffB", voltage, percentage, status)
        )

    def serialize_joint_state(
        self, pos_l: float, pos_r: float, vel_l: float, vel_r: float
    ) -> bytes:
        return self.frame(
            MsgType.JOINT_STATE, struct.pack("<4f", pos_l, pos_r, vel_l, vel_r)
        )

    def serialize_camera_frame(
        self, width: int, height: int, jpeg_data: bytes
    ) -> bytes:
        header = struct.pack("<HHI", width, height, len(jpeg_data))
        return self.frame(MsgType.CAMERA_FRAME, header + jpeg_data)

    def serialize_debug_log(
        self, severity: int, timestamp_ns: int, text: str
    ) -> bytes:
        payload = struct.pack("<BQ", severity, timestamp_ns)
        payload += text.encode("utf-8") + b"\x00"
        return self.frame(MsgType.DEBUG_LOG, payload)

    def serialize_ir_sensor(self, v0: int, v1: int, v2: int) -> bytes:
        return self.frame(
            MsgType.IR_SENSOR, struct.pack("<3H", v0, v1, v2)
        )

    def serialize_us_sensor(self, range_m: float) -> bytes:
        return self.frame(MsgType.US_SENSOR, struct.pack("<f", range_m))

    def serialize_robot_status(self, flags: int) -> bytes:
        return self.frame(MsgType.ROBOT_STATUS, struct.pack("<I", flags))

    # -- Commands (PC -> Robot) --------------------------------------------

    def serialize_cmd_vel(self, linear_x: float, angular_z: float) -> bytes:
        return self.frame(
            MsgType.CMD_VEL, struct.pack("<ff", linear_x, angular_z)
        )

    def serialize_set_led(
        self, command: int, pixel_mask: int, r: int, g: int, b: int
    ) -> bytes:
        return self.frame(
            MsgType.SET_LED, struct.pack("<5B", command, pixel_mask, r, g, b)
        )

    def serialize_set_lamp(
        self, mode: int, r: int, g: int, b: int, time_ms: int
    ) -> bytes:
        return self.frame(
            MsgType.SET_LAMP, struct.pack("<4BH", mode, r, g, b, time_ms)
        )

    def serialize_set_emotion(self, emotion_id: int) -> bytes:
        return self.frame(MsgType.SET_EMOTION, struct.pack("<B", emotion_id))

    def serialize_set_brightness(self, brightness: int) -> bytes:
        return self.frame(
            MsgType.SET_BRIGHTNESS, struct.pack("<B", brightness)
        )

    def serialize_nav_goal(
        self, x: float, y: float, theta: float
    ) -> bytes:
        return self.frame(
            MsgType.NAV_GOAL, struct.pack("<3f", x, y, theta)
        )

    def serialize_nav_cancel(self) -> bytes:
        return self.frame(MsgType.NAV_CANCEL, b"")

    def serialize_set_pose(
        self, x: float, y: float, theta: float
    ) -> bytes:
        return self.frame(
            MsgType.SET_POSE, struct.pack("<3f", x, y, theta)
        )

    def serialize_ping(self, timestamp_ns: int) -> bytes:
        return self.frame(MsgType.PING, struct.pack("<Q", timestamp_ns))

    def serialize_pong(
        self, echo_ts: int, server_ts: int
    ) -> bytes:
        return self.frame(MsgType.PONG, struct.pack("<QQ", echo_ts, server_ts))

    def serialize_ack(
        self, ack_seq: int, status: int, message: str = ""
    ) -> bytes:
        payload = struct.pack("<HB", ack_seq, status)
        payload += message.encode("utf-8")
        return self.frame(MsgType.ACK, payload)

    def serialize_map_data(
        self,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        origin_theta: float,
        data: bytes,
    ) -> bytes:
        header = struct.pack(
            "<IIffff", width, height, resolution,
            origin_x, origin_y, origin_theta,
        )
        return self.frame(MsgType.MAP_DATA, header + data)


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------
def parse_message(
    data: bytes,
) -> tuple[ParseResult, Optional[ParsedMessage], int]:
    """Parse one framed message from *data*.

    Returns (result, parsed_msg_or_None, bytes_consumed).
    """
    length = len(data)

    if length < HEADER_SIZE:
        return ParseResult.INCOMPLETE, None, 0

    if data[0:2] != MAGIC:
        return ParseResult.BAD_MAGIC, None, 1

    if data[2] != PROTOCOL_VERSION:
        return ParseResult.BAD_VERSION, None, HEADER_SIZE

    # Verify header CRC (over first 10 bytes)
    computed_hcrc = crc16(data[:10])
    (received_hcrc,) = struct.unpack_from("<H", data, 10)
    if computed_hcrc != received_hcrc:
        return ParseResult.BAD_HEADER_CRC, None, 1

    # Unpack header
    _, _, msg_type_raw, seq, payload_len, _ = struct.unpack_from(
        HEADER_FORMAT, data, 0
    )
    total_size = HEADER_SIZE + payload_len + CRC_SIZE

    if length < total_size:
        return ParseResult.INCOMPLETE, None, 0

    # Verify payload CRC
    payload_bytes = data[HEADER_SIZE : HEADER_SIZE + payload_len]
    computed_pcrc = crc16(payload_bytes)
    (received_pcrc,) = struct.unpack_from(
        "<H", data, HEADER_SIZE + payload_len
    )
    if computed_pcrc != received_pcrc:
        return ParseResult.BAD_PAYLOAD_CRC, None, total_size

    msg = ParsedMessage(
        msg_type=MsgType(msg_type_raw),
        sequence=seq,
        payload=bytes(payload_bytes),
    )
    return ParseResult.OK, msg, total_size


# ---------------------------------------------------------------------------
# Payload deserializers
# ---------------------------------------------------------------------------
def deserialize_odom(
    payload: bytes,
) -> tuple[float, float, float, float, float]:
    return struct.unpack("<5f", payload)


def deserialize_imu(payload: bytes) -> tuple:
    return struct.unpack("<10f", payload)


def deserialize_lidar24(payload: bytes) -> list[float]:
    return list(struct.unpack("<24f", payload))


def deserialize_battery(payload: bytes) -> tuple[float, float, int]:
    v, p, s = struct.unpack("<ffB", payload)
    return v, p, s


def deserialize_joint_state(
    payload: bytes,
) -> tuple[float, float, float, float]:
    return struct.unpack("<4f", payload)


def deserialize_camera_frame(
    payload: bytes,
) -> tuple[int, int, bytes]:
    w, h, size = struct.unpack_from("<HHI", payload, 0)
    jpeg = payload[8 : 8 + size]
    return w, h, jpeg


def deserialize_debug_log(payload: bytes) -> tuple[int, int, str]:
    severity = payload[0]
    ts = struct.unpack_from("<Q", payload, 1)[0]
    text = payload[9:].split(b"\x00", 1)[0].decode("utf-8")
    return severity, ts, text


def deserialize_ir_sensor(payload: bytes) -> tuple[int, int, int]:
    return struct.unpack("<3H", payload)


def deserialize_us_sensor(payload: bytes) -> float:
    return struct.unpack("<f", payload)[0]


def deserialize_robot_status(payload: bytes) -> int:
    return struct.unpack("<I", payload)[0]


def deserialize_cmd_vel(payload: bytes) -> tuple[float, float]:
    return struct.unpack("<ff", payload)


def deserialize_led_command(
    payload: bytes,
) -> tuple[int, int, int, int, int]:
    return struct.unpack("<5B", payload)


def deserialize_lamp_command(
    payload: bytes,
) -> tuple[int, int, int, int, int]:
    mode, r, g, b = struct.unpack_from("<4B", payload, 0)
    (time_ms,) = struct.unpack_from("<H", payload, 4)
    return mode, r, g, b, time_ms


def deserialize_emotion(payload: bytes) -> int:
    return payload[0]


def deserialize_brightness(payload: bytes) -> int:
    return payload[0]


def deserialize_nav_goal(payload: bytes) -> tuple[float, float, float]:
    return struct.unpack("<3f", payload)


def deserialize_set_pose(payload: bytes) -> tuple[float, float, float]:
    return struct.unpack("<3f", payload)


def deserialize_ping(payload: bytes) -> int:
    return struct.unpack("<Q", payload)[0]


def deserialize_pong(payload: bytes) -> tuple[int, int]:
    return struct.unpack("<QQ", payload)


def deserialize_ack(payload: bytes) -> tuple[int, int, str]:
    seq, status = struct.unpack_from("<HB", payload, 0)
    message = payload[3:].decode("utf-8") if len(payload) > 3 else ""
    return seq, status, message


def deserialize_map_data(
    payload: bytes,
) -> tuple[int, int, float, float, float, float, bytes]:
    w, h, res, ox, oy, ot = struct.unpack_from("<IIffff", payload, 0)
    data = payload[24:]
    return w, h, res, ox, oy, ot, data
