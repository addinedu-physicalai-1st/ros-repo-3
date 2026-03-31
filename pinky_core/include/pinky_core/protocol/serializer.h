#pragma once

#include <cstdint>
#include <vector>

#include "pinky_core/common/types.h"
#include "pinky_core/protocol/message_types.h"

namespace pinky {

// ---------------------------------------------------------------------------
// Serializer: builds a framed binary message (header + payload + CRC)
// ---------------------------------------------------------------------------
class Serializer {
 public:
  Serializer();

  // Payload serialization — each returns payload bytes
  std::vector<uint8_t> SerializeOdom(const Odometry& odom);
  std::vector<uint8_t> SerializeImu(const ImuData& imu);
  std::vector<uint8_t> SerializeLidar24(const LidarSectors& sectors);
  std::vector<uint8_t> SerializeBattery(const BatteryState& battery);
  std::vector<uint8_t> SerializeJointState(const JointState& joints);
  std::vector<uint8_t> SerializeCameraFrame(const CameraFrame& frame);
  std::vector<uint8_t> SerializeDebugLog(const DebugLog& log);
  std::vector<uint8_t> SerializeIrSensor(const IrSensor& ir);
  std::vector<uint8_t> SerializeUsSensor(const UsSensor& us);
  std::vector<uint8_t> SerializeRobotStatus(const RobotStatus& status);
  std::vector<uint8_t> SerializeCmdVel(const CmdVel& cmd);
  std::vector<uint8_t> SerializeLedCommand(const LedCommand& led);
  std::vector<uint8_t> SerializeLampCommand(const LampCommand& lamp);
  std::vector<uint8_t> SerializeEmotion(uint8_t emotion_id);
  std::vector<uint8_t> SerializeBrightness(uint8_t brightness);
  std::vector<uint8_t> SerializeNavGoal(const NavGoal& goal);
  std::vector<uint8_t> SerializeNavCancel();
  std::vector<uint8_t> SerializeSetPose(const Pose2D& pose);
  std::vector<uint8_t> SerializePing(uint64_t timestamp_ns);
  std::vector<uint8_t> SerializePong(uint64_t echo_ts, uint64_t server_ts);
  std::vector<uint8_t> SerializeAck(uint16_t ack_seq, uint8_t status,
                                    const std::string& message);
  std::vector<uint8_t> SerializeMapData(const MapData& map);

  // Build a complete framed message: header + payload + payload_crc
  std::vector<uint8_t> Frame(MsgType type,
                             const std::vector<uint8_t>& payload);

 private:
  uint16_t sequence_{0};
};

// ---------------------------------------------------------------------------
// Deserializer: parse framed binary messages
// ---------------------------------------------------------------------------
struct ParsedMessage {
  MsgType msg_type;
  uint16_t sequence;
  std::vector<uint8_t> payload;
};

enum class ParseResult {
  kOk,
  kIncomplete,    // need more data
  kBadMagic,
  kBadVersion,
  kBadHeaderCrc,
  kBadPayloadCrc,
};

// Parse one message from a byte buffer.
// On success, fills `msg` and `bytes_consumed`.
// On kIncomplete, caller should accumulate more data.
ParseResult ParseMessage(const uint8_t* data, size_t length,
                         ParsedMessage& msg, size_t& bytes_consumed);

// Payload deserialization
Odometry     DeserializeOdom(const std::vector<uint8_t>& payload);
ImuData      DeserializeImu(const std::vector<uint8_t>& payload);
LidarSectors DeserializeLidar24(const std::vector<uint8_t>& payload);
BatteryState DeserializeBattery(const std::vector<uint8_t>& payload);
JointState   DeserializeJointState(const std::vector<uint8_t>& payload);
CameraFrame  DeserializeCameraFrame(const std::vector<uint8_t>& payload);
DebugLog     DeserializeDebugLog(const std::vector<uint8_t>& payload);
IrSensor     DeserializeIrSensor(const std::vector<uint8_t>& payload);
UsSensor     DeserializeUsSensor(const std::vector<uint8_t>& payload);
RobotStatus  DeserializeRobotStatus(const std::vector<uint8_t>& payload);
CmdVel       DeserializeCmdVel(const std::vector<uint8_t>& payload);
LedCommand   DeserializeLedCommand(const std::vector<uint8_t>& payload);
LampCommand  DeserializeLampCommand(const std::vector<uint8_t>& payload);
uint8_t      DeserializeEmotion(const std::vector<uint8_t>& payload);
uint8_t      DeserializeBrightness(const std::vector<uint8_t>& payload);
NavGoal      DeserializeNavGoal(const std::vector<uint8_t>& payload);
Pose2D       DeserializeSetPose(const std::vector<uint8_t>& payload);
uint64_t     DeserializePing(const std::vector<uint8_t>& payload);

struct PongData {
  uint64_t echo_ts;
  uint64_t server_ts;
};
PongData DeserializePong(const std::vector<uint8_t>& payload);

struct AckData {
  uint16_t ack_seq;
  uint8_t status;
  std::string message;
};
AckData DeserializeAck(const std::vector<uint8_t>& payload);

MapData DeserializeMapData(const std::vector<uint8_t>& payload);

}  // namespace pinky
