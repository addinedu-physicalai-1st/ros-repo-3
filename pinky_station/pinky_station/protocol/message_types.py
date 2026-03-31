"""Binary protocol message types — mirrors C++ message_types.h exactly."""

import struct
from enum import IntEnum

# Protocol constants
MAGIC = b"\x50\x4B"  # "PK"
PROTOCOL_VERSION = 0x01
HEADER_SIZE = 12
CRC_SIZE = 2
FRAME_OVERHEAD = HEADER_SIZE + CRC_SIZE  # 14 bytes

HEADER_FORMAT = "<2sBBHIH"  # magic(2) + version(1) + type(1) + seq(2) + len(4) + hcrc(2)


class MsgType(IntEnum):
    """Message type IDs — must match C++ MsgType enum."""

    # Robot -> PC
    ODOM = 0x01
    IMU = 0x02
    LIDAR_SCAN = 0x03
    LIDAR_24 = 0x04
    BATTERY = 0x05
    JOINT_STATE = 0x06
    CAMERA_FRAME = 0x07
    DEBUG_LOG = 0x08
    IR_SENSOR = 0x09
    US_SENSOR = 0x0A
    ROBOT_STATUS = 0x0B

    # PC -> Robot
    CMD_VEL = 0x20
    SET_LED = 0x21
    SET_LAMP = 0x22
    SET_EMOTION = 0x23
    SET_BRIGHTNESS = 0x24
    NAV_GOAL = 0x25
    NAV_CANCEL = 0x26
    SET_POSE = 0x27
    PING = 0x28
    REQUEST_MAP = 0x29

    # Bidirectional
    PONG = 0x30
    ACK = 0x31
    MAP_DATA = 0x32
