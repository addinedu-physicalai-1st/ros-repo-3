#include "pinky_core/protocol/serializer.h"

#include <cstring>
#include <algorithm>

#include "pinky_core/protocol/checksum.h"

namespace pinky {

// ---------------------------------------------------------------------------
// Helper: append POD to buffer
// ---------------------------------------------------------------------------
namespace {

template <typename T>
void Append(std::vector<uint8_t>& buf, T value) {
  const auto* ptr = reinterpret_cast<const uint8_t*>(&value);
  buf.insert(buf.end(), ptr, ptr + sizeof(T));
}

template <typename T>
T ReadAt(const uint8_t* data) {
  T value;
  std::memcpy(&value, data, sizeof(T));
  return value;
}

}  // namespace

// ---------------------------------------------------------------------------
// Serializer
// ---------------------------------------------------------------------------
Serializer::Serializer() = default;

std::vector<uint8_t> Serializer::Frame(MsgType type,
                                       const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> frame;
  frame.reserve(kFrameOverhead + payload.size());

  // Header fields (without CRC yet)
  MessageHeader header{};
  header.magic[0] = kMagicByte0;
  header.magic[1] = kMagicByte1;
  header.version = kProtocolVersion;
  header.msg_type = static_cast<uint8_t>(type);
  header.sequence = sequence_++;
  header.payload_length = static_cast<uint32_t>(payload.size());
  header.header_crc = 0;  // placeholder

  // Compute header CRC over first 10 bytes
  auto* raw = reinterpret_cast<const uint8_t*>(&header);
  header.header_crc = Crc16(raw, 10);

  // Write header
  raw = reinterpret_cast<const uint8_t*>(&header);
  frame.insert(frame.end(), raw, raw + kHeaderSize);

  // Write payload
  frame.insert(frame.end(), payload.begin(), payload.end());

  // Payload CRC
  uint16_t payload_crc = Crc16(payload.data(), payload.size());
  Append(frame, payload_crc);

  return frame;
}

// ---------------------------------------------------------------------------
// Serialize functions
// ---------------------------------------------------------------------------
std::vector<uint8_t> Serializer::SerializeOdom(const Odometry& odom) {
  std::vector<uint8_t> buf;
  buf.reserve(20);
  Append(buf, static_cast<float>(odom.x));
  Append(buf, static_cast<float>(odom.y));
  Append(buf, static_cast<float>(odom.theta));
  Append(buf, static_cast<float>(odom.vx));
  Append(buf, static_cast<float>(odom.vth));
  return buf;
}

std::vector<uint8_t> Serializer::SerializeImu(const ImuData& imu) {
  std::vector<uint8_t> buf;
  buf.reserve(40);
  Append(buf, static_cast<float>(imu.orientation.w));
  Append(buf, static_cast<float>(imu.orientation.x));
  Append(buf, static_cast<float>(imu.orientation.y));
  Append(buf, static_cast<float>(imu.orientation.z));
  Append(buf, static_cast<float>(imu.angular_velocity.x));
  Append(buf, static_cast<float>(imu.angular_velocity.y));
  Append(buf, static_cast<float>(imu.angular_velocity.z));
  Append(buf, static_cast<float>(imu.linear_acceleration.x));
  Append(buf, static_cast<float>(imu.linear_acceleration.y));
  Append(buf, static_cast<float>(imu.linear_acceleration.z));
  return buf;
}

std::vector<uint8_t> Serializer::SerializeLidar24(const LidarSectors& s) {
  std::vector<uint8_t> buf;
  buf.reserve(96);
  for (int i = 0; i < 24; ++i) {
    Append(buf, s.sectors[i]);
  }
  return buf;
}

std::vector<uint8_t> Serializer::SerializeBattery(const BatteryState& b) {
  std::vector<uint8_t> buf;
  buf.reserve(9);
  Append(buf, b.voltage);
  Append(buf, b.percentage);
  Append(buf, b.status);
  return buf;
}

std::vector<uint8_t> Serializer::SerializeJointState(const JointState& j) {
  std::vector<uint8_t> buf;
  buf.reserve(16);
  Append(buf, static_cast<float>(j.position[0]));
  Append(buf, static_cast<float>(j.position[1]));
  Append(buf, static_cast<float>(j.velocity[0]));
  Append(buf, static_cast<float>(j.velocity[1]));
  return buf;
}

std::vector<uint8_t> Serializer::SerializeCameraFrame(const CameraFrame& f) {
  std::vector<uint8_t> buf;
  buf.reserve(8 + f.jpeg_data.size());
  Append(buf, f.width);
  Append(buf, f.height);
  Append(buf, static_cast<uint32_t>(f.jpeg_data.size()));
  buf.insert(buf.end(), f.jpeg_data.begin(), f.jpeg_data.end());
  return buf;
}

std::vector<uint8_t> Serializer::SerializeDebugLog(const DebugLog& log) {
  std::vector<uint8_t> buf;
  buf.reserve(9 + log.text.size() + 1);
  Append(buf, log.severity);
  Append(buf, log.timestamp_ns);
  buf.insert(buf.end(), log.text.begin(), log.text.end());
  buf.push_back(0);  // null terminator
  return buf;
}

std::vector<uint8_t> Serializer::SerializeIrSensor(const IrSensor& ir) {
  std::vector<uint8_t> buf;
  buf.reserve(6);
  for (int i = 0; i < 3; ++i) {
    Append(buf, ir.values[i]);
  }
  return buf;
}

std::vector<uint8_t> Serializer::SerializeUsSensor(const UsSensor& us) {
  std::vector<uint8_t> buf;
  buf.reserve(4);
  Append(buf, us.range);
  return buf;
}

std::vector<uint8_t> Serializer::SerializeRobotStatus(const RobotStatus& s) {
  std::vector<uint8_t> buf;
  buf.reserve(4);
  Append(buf, s.flags);
  return buf;
}

std::vector<uint8_t> Serializer::SerializeCmdVel(const CmdVel& cmd) {
  std::vector<uint8_t> buf;
  buf.reserve(8);
  Append(buf, cmd.linear_x);
  Append(buf, cmd.angular_z);
  return buf;
}

std::vector<uint8_t> Serializer::SerializeLedCommand(const LedCommand& led) {
  std::vector<uint8_t> buf;
  buf.reserve(5);
  Append(buf, led.command);
  Append(buf, led.pixel_mask);
  Append(buf, led.r);
  Append(buf, led.g);
  Append(buf, led.b);
  return buf;
}

std::vector<uint8_t> Serializer::SerializeLampCommand(const LampCommand& l) {
  std::vector<uint8_t> buf;
  buf.reserve(6);
  Append(buf, l.mode);
  Append(buf, l.r);
  Append(buf, l.g);
  Append(buf, l.b);
  Append(buf, l.time_ms);
  return buf;
}

std::vector<uint8_t> Serializer::SerializeEmotion(uint8_t emotion_id) {
  return {emotion_id};
}

std::vector<uint8_t> Serializer::SerializeBrightness(uint8_t brightness) {
  return {brightness};
}

std::vector<uint8_t> Serializer::SerializeNavGoal(const NavGoal& goal) {
  std::vector<uint8_t> buf;
  buf.reserve(12);
  Append(buf, goal.x);
  Append(buf, goal.y);
  Append(buf, goal.theta);
  return buf;
}

std::vector<uint8_t> Serializer::SerializeNavCancel() {
  return {};
}

std::vector<uint8_t> Serializer::SerializeSetPose(const Pose2D& pose) {
  std::vector<uint8_t> buf;
  buf.reserve(12);
  Append(buf, pose.x);
  Append(buf, pose.y);
  Append(buf, pose.theta);
  return buf;
}

std::vector<uint8_t> Serializer::SerializePing(uint64_t timestamp_ns) {
  std::vector<uint8_t> buf;
  buf.reserve(8);
  Append(buf, timestamp_ns);
  return buf;
}

std::vector<uint8_t> Serializer::SerializePong(uint64_t echo_ts,
                                               uint64_t server_ts) {
  std::vector<uint8_t> buf;
  buf.reserve(16);
  Append(buf, echo_ts);
  Append(buf, server_ts);
  return buf;
}

std::vector<uint8_t> Serializer::SerializeAck(uint16_t ack_seq, uint8_t status,
                                              const std::string& message) {
  std::vector<uint8_t> buf;
  buf.reserve(3 + message.size());
  Append(buf, ack_seq);
  Append(buf, status);
  buf.insert(buf.end(), message.begin(), message.end());
  return buf;
}

std::vector<uint8_t> Serializer::SerializeMapData(const MapData& map) {
  std::vector<uint8_t> buf;
  buf.reserve(24 + map.data.size());
  Append(buf, map.width);
  Append(buf, map.height);
  Append(buf, map.resolution);
  Append(buf, map.origin_x);
  Append(buf, map.origin_y);
  Append(buf, map.origin_theta);
  buf.insert(buf.end(), map.data.begin(), map.data.end());
  return buf;
}

// ---------------------------------------------------------------------------
// ParseMessage
// ---------------------------------------------------------------------------
ParseResult ParseMessage(const uint8_t* data, size_t length,
                         ParsedMessage& msg, size_t& bytes_consumed) {
  bytes_consumed = 0;

  if (length < kHeaderSize) {
    return ParseResult::kIncomplete;
  }

  // Validate magic
  if (data[0] != kMagicByte0 || data[1] != kMagicByte1) {
    bytes_consumed = 1;  // skip one byte, try to resync
    return ParseResult::kBadMagic;
  }

  // Validate version
  if (data[2] != kProtocolVersion) {
    bytes_consumed = kHeaderSize;
    return ParseResult::kBadVersion;
  }

  // Validate header CRC
  uint16_t computed_hcrc = Crc16(data, 10);
  uint16_t received_hcrc = ReadAt<uint16_t>(data + 10);
  if (computed_hcrc != received_hcrc) {
    bytes_consumed = 1;
    return ParseResult::kBadHeaderCrc;
  }

  // Read header fields
  auto header = ReadAt<MessageHeader>(data);
  size_t total_size = kHeaderSize + header.payload_length + kCrcSize;

  if (length < total_size) {
    return ParseResult::kIncomplete;
  }

  // Validate payload CRC
  const uint8_t* payload_ptr = data + kHeaderSize;
  uint16_t computed_pcrc = Crc16(payload_ptr, header.payload_length);
  uint16_t received_pcrc =
      ReadAt<uint16_t>(data + kHeaderSize + header.payload_length);
  if (computed_pcrc != received_pcrc) {
    bytes_consumed = total_size;
    return ParseResult::kBadPayloadCrc;
  }

  // Success
  msg.msg_type = static_cast<MsgType>(header.msg_type);
  msg.sequence = header.sequence;
  msg.payload.assign(payload_ptr, payload_ptr + header.payload_length);
  bytes_consumed = total_size;
  return ParseResult::kOk;
}

// ---------------------------------------------------------------------------
// Deserialize functions
// ---------------------------------------------------------------------------
Odometry DeserializeOdom(const std::vector<uint8_t>& p) {
  Odometry o;
  o.x     = ReadAt<float>(p.data() + 0);
  o.y     = ReadAt<float>(p.data() + 4);
  o.theta = ReadAt<float>(p.data() + 8);
  o.vx    = ReadAt<float>(p.data() + 12);
  o.vth   = ReadAt<float>(p.data() + 16);
  return o;
}

ImuData DeserializeImu(const std::vector<uint8_t>& p) {
  ImuData d;
  d.orientation.w         = ReadAt<float>(p.data() + 0);
  d.orientation.x         = ReadAt<float>(p.data() + 4);
  d.orientation.y         = ReadAt<float>(p.data() + 8);
  d.orientation.z         = ReadAt<float>(p.data() + 12);
  d.angular_velocity.x    = ReadAt<float>(p.data() + 16);
  d.angular_velocity.y    = ReadAt<float>(p.data() + 20);
  d.angular_velocity.z    = ReadAt<float>(p.data() + 24);
  d.linear_acceleration.x = ReadAt<float>(p.data() + 28);
  d.linear_acceleration.y = ReadAt<float>(p.data() + 32);
  d.linear_acceleration.z = ReadAt<float>(p.data() + 36);
  return d;
}

LidarSectors DeserializeLidar24(const std::vector<uint8_t>& p) {
  LidarSectors s;
  for (int i = 0; i < 24; ++i) {
    s.sectors[i] = ReadAt<float>(p.data() + i * 4);
  }
  return s;
}

BatteryState DeserializeBattery(const std::vector<uint8_t>& p) {
  BatteryState b;
  b.voltage    = ReadAt<float>(p.data() + 0);
  b.percentage = ReadAt<float>(p.data() + 4);
  b.status     = ReadAt<uint8_t>(p.data() + 8);
  return b;
}

JointState DeserializeJointState(const std::vector<uint8_t>& p) {
  JointState j;
  j.position[0] = ReadAt<float>(p.data() + 0);
  j.position[1] = ReadAt<float>(p.data() + 4);
  j.velocity[0] = ReadAt<float>(p.data() + 8);
  j.velocity[1] = ReadAt<float>(p.data() + 12);
  return j;
}

CameraFrame DeserializeCameraFrame(const std::vector<uint8_t>& p) {
  CameraFrame f;
  f.width  = ReadAt<uint16_t>(p.data() + 0);
  f.height = ReadAt<uint16_t>(p.data() + 2);
  uint32_t jpeg_size = ReadAt<uint32_t>(p.data() + 4);
  f.jpeg_data.assign(p.data() + 8, p.data() + 8 + jpeg_size);
  return f;
}

DebugLog DeserializeDebugLog(const std::vector<uint8_t>& p) {
  DebugLog log;
  log.severity = ReadAt<uint8_t>(p.data() + 0);
  log.timestamp_ns = ReadAt<uint64_t>(p.data() + 1);
  // text starts at offset 9, null-terminated
  log.text = std::string(reinterpret_cast<const char*>(p.data() + 9));
  return log;
}

IrSensor DeserializeIrSensor(const std::vector<uint8_t>& p) {
  IrSensor ir;
  for (int i = 0; i < 3; ++i) {
    ir.values[i] = ReadAt<uint16_t>(p.data() + i * 2);
  }
  return ir;
}

UsSensor DeserializeUsSensor(const std::vector<uint8_t>& p) {
  UsSensor us;
  us.range = ReadAt<float>(p.data() + 0);
  return us;
}

RobotStatus DeserializeRobotStatus(const std::vector<uint8_t>& p) {
  RobotStatus s;
  s.flags = ReadAt<uint32_t>(p.data() + 0);
  return s;
}

CmdVel DeserializeCmdVel(const std::vector<uint8_t>& p) {
  CmdVel c;
  c.linear_x  = ReadAt<float>(p.data() + 0);
  c.angular_z = ReadAt<float>(p.data() + 4);
  return c;
}

LedCommand DeserializeLedCommand(const std::vector<uint8_t>& p) {
  LedCommand l;
  l.command    = p[0];
  l.pixel_mask = p[1];
  l.r = p[2];
  l.g = p[3];
  l.b = p[4];
  return l;
}

LampCommand DeserializeLampCommand(const std::vector<uint8_t>& p) {
  LampCommand l;
  l.mode = p[0];
  l.r = p[1];
  l.g = p[2];
  l.b = p[3];
  l.time_ms = ReadAt<uint16_t>(p.data() + 4);
  return l;
}

uint8_t DeserializeEmotion(const std::vector<uint8_t>& p) {
  return p[0];
}

uint8_t DeserializeBrightness(const std::vector<uint8_t>& p) {
  return p[0];
}

NavGoal DeserializeNavGoal(const std::vector<uint8_t>& p) {
  NavGoal g;
  g.x     = ReadAt<float>(p.data() + 0);
  g.y     = ReadAt<float>(p.data() + 4);
  g.theta = ReadAt<float>(p.data() + 8);
  return g;
}

Pose2D DeserializeSetPose(const std::vector<uint8_t>& p) {
  Pose2D pose;
  pose.x     = ReadAt<float>(p.data() + 0);
  pose.y     = ReadAt<float>(p.data() + 4);
  pose.theta = ReadAt<float>(p.data() + 8);
  return pose;
}

uint64_t DeserializePing(const std::vector<uint8_t>& p) {
  return ReadAt<uint64_t>(p.data());
}

PongData DeserializePong(const std::vector<uint8_t>& p) {
  PongData d;
  d.echo_ts   = ReadAt<uint64_t>(p.data() + 0);
  d.server_ts = ReadAt<uint64_t>(p.data() + 8);
  return d;
}

AckData DeserializeAck(const std::vector<uint8_t>& p) {
  AckData a;
  a.ack_seq = ReadAt<uint16_t>(p.data() + 0);
  a.status  = ReadAt<uint8_t>(p.data() + 2);
  if (p.size() > 3) {
    a.message.assign(p.begin() + 3, p.end());
  }
  return a;
}

MapData DeserializeMapData(const std::vector<uint8_t>& p) {
  MapData m;
  m.width        = ReadAt<uint32_t>(p.data() + 0);
  m.height       = ReadAt<uint32_t>(p.data() + 4);
  m.resolution   = ReadAt<float>(p.data() + 8);
  m.origin_x     = ReadAt<float>(p.data() + 12);
  m.origin_y     = ReadAt<float>(p.data() + 16);
  m.origin_theta = ReadAt<float>(p.data() + 20);
  if (p.size() > 24) {
    m.data.assign(p.begin() + 24, p.end());
  }
  return m;
}

}  // namespace pinky
