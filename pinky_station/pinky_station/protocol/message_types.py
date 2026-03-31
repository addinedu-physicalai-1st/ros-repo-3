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


# ---------------------------------------------------------------------------
# Module-level aliases (used by net/ and workers/ code as mt.MSG_*)
# ---------------------------------------------------------------------------
MSG_ODOM = MsgType.ODOM
MSG_IMU = MsgType.IMU
MSG_LIDAR_SCAN = MsgType.LIDAR_SCAN
MSG_LIDAR_24 = MsgType.LIDAR_24
MSG_BATTERY = MsgType.BATTERY
MSG_JOINT_STATE = MsgType.JOINT_STATE
MSG_CAMERA_FRAME = MsgType.CAMERA_FRAME
MSG_DEBUG_LOG = MsgType.DEBUG_LOG
MSG_LOG = MsgType.DEBUG_LOG  # convenience alias
MSG_IR_SENSOR = MsgType.IR_SENSOR
MSG_US_SENSOR = MsgType.US_SENSOR
MSG_ROBOT_STATUS = MsgType.ROBOT_STATUS
MSG_CMD_VEL = MsgType.CMD_VEL
MSG_SET_LED = MsgType.SET_LED
MSG_SET_LAMP = MsgType.SET_LAMP
MSG_SET_EMOTION = MsgType.SET_EMOTION
MSG_SET_BRIGHTNESS = MsgType.SET_BRIGHTNESS
MSG_NAV_GOAL = MsgType.NAV_GOAL
MSG_NAV_CANCEL = MsgType.NAV_CANCEL
MSG_SET_POSE = MsgType.SET_POSE
MSG_PING = MsgType.PING
MSG_REQUEST_MAP = MsgType.REQUEST_MAP
MSG_PONG = MsgType.PONG
MSG_ACK = MsgType.ACK
MSG_MAP_DATA = MsgType.MAP_DATA
