#pragma once

#include <cstdint>
#include <cstddef>

namespace pinky {

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------
constexpr uint8_t kMagicByte0       = 0x50;  // 'P'
constexpr uint8_t kMagicByte1       = 0x4B;  // 'K'
constexpr uint8_t kProtocolVersion  = 0x01;
constexpr size_t  kHeaderSize       = 12;
constexpr size_t  kCrcSize          = 2;
constexpr size_t  kFrameOverhead    = kHeaderSize + kCrcSize;  // 14 bytes

// ---------------------------------------------------------------------------
// Message types
// ---------------------------------------------------------------------------
enum class MsgType : uint8_t {
  // Robot -> PC (sensor data, status)
  kOdom         = 0x01,
  kImu          = 0x02,
  kLidarScan    = 0x03,
  kLidar24      = 0x04,
  kBattery      = 0x05,
  kJointState   = 0x06,
  kCameraFrame  = 0x07,
  kDebugLog     = 0x08,
  kIrSensor     = 0x09,
  kUsSensor     = 0x0A,
  kRobotStatus  = 0x0B,

  // PC -> Robot (commands)
  kCmdVel       = 0x20,
  kSetLed       = 0x21,
  kSetLamp      = 0x22,
  kSetEmotion   = 0x23,
  kSetBrightness= 0x24,
  kNavGoal      = 0x25,
  kNavCancel    = 0x26,
  kSetPose      = 0x27,
  kPing         = 0x28,
  kRequestMap   = 0x29,

  // Bidirectional
  kPong         = 0x30,
  kAck          = 0x31,
  kMapData      = 0x32,
};

// ---------------------------------------------------------------------------
// Wire header (packed, 12 bytes)
// ---------------------------------------------------------------------------
#pragma pack(push, 1)
struct MessageHeader {
  uint8_t  magic[2];
  uint8_t  version;
  uint8_t  msg_type;
  uint16_t sequence;
  uint32_t payload_length;
  uint16_t header_crc;
};
#pragma pack(pop)

static_assert(sizeof(MessageHeader) == kHeaderSize,
              "MessageHeader must be exactly 12 bytes");

}  // namespace pinky
