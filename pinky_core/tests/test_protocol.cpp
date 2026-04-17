#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "pinky_core/common/types.h"
#include "pinky_core/protocol/checksum.h"
#include "pinky_core/protocol/message_types.h"
#include "pinky_core/protocol/serializer.h"

using namespace pinky;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static constexpr float kEps = 1e-6f;

static bool Near(float a, float b) {
  return std::fabs(a - b) < kEps;
}

static int g_passed = 0;
static int g_failed = 0;

#define CHECK(cond)                                                         \
  do {                                                                      \
    if (!(cond)) {                                                          \
      std::printf("  FAIL: %s  (line %d)\n", #cond, __LINE__);             \
      ++g_failed;                                                           \
    } else {                                                                \
      ++g_passed;                                                           \
    }                                                                       \
  } while (0)

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
static void TestCrc16() {
  std::printf("[TestCrc16]\n");
  // Known test vector: "123456789" -> CRC-CCITT = 0x29B1
  const uint8_t data[] = "123456789";
  uint16_t crc = Crc16(data, 9);
  CHECK(crc == 0x29B1);

  // Empty input
  uint16_t empty_crc = Crc16(nullptr, 0);
  CHECK(empty_crc == 0xFFFF);
}

static void TestOdomRoundtrip() {
  std::printf("[TestOdomRoundtrip]\n");
  Serializer ser;
  Odometry odom;
  odom.x = 1.23f;
  odom.y = -4.56f;
  odom.theta = 0.789f;
  odom.vx = 0.15f;
  odom.vth = -0.3f;

  auto payload = ser.SerializeOdom(odom);
  auto frame = ser.Frame(MsgType::kOdom, payload);

  ParsedMessage msg;
  size_t consumed = 0;
  auto result = ParseMessage(frame.data(), frame.size(), msg, consumed);

  CHECK(result == ParseResult::kOk);
  CHECK(consumed == frame.size());
  CHECK(msg.msg_type == MsgType::kOdom);

  auto decoded = DeserializeOdom(msg.payload);
  CHECK(Near(static_cast<float>(decoded.x), 1.23f));
  CHECK(Near(static_cast<float>(decoded.y), -4.56f));
  CHECK(Near(static_cast<float>(decoded.theta), 0.789f));
  CHECK(Near(static_cast<float>(decoded.vx), 0.15f));
  CHECK(Near(static_cast<float>(decoded.vth), -0.3f));
}

static void TestImuRoundtrip() {
  std::printf("[TestImuRoundtrip]\n");
  Serializer ser;
  ImuData imu;
  imu.orientation = {0.707, 0.0, 0.707, 0.0};
  imu.angular_velocity = {0.1, 0.2, 0.3};
  imu.linear_acceleration = {9.8, 0.0, 0.1};

  auto payload = ser.SerializeImu(imu);
  auto frame = ser.Frame(MsgType::kImu, payload);

  ParsedMessage msg;
  size_t consumed = 0;
  auto result = ParseMessage(frame.data(), frame.size(), msg, consumed);
  CHECK(result == ParseResult::kOk);

  auto d = DeserializeImu(msg.payload);
  CHECK(Near(static_cast<float>(d.orientation.w), 0.707f));
  CHECK(Near(static_cast<float>(d.angular_velocity.z), 0.3f));
  CHECK(Near(static_cast<float>(d.linear_acceleration.x), 9.8f));
}

static void TestLidar24Roundtrip() {
  std::printf("[TestLidar24Roundtrip]\n");
  Serializer ser;
  LidarSectors sectors;
  for (int i = 0; i < 24; ++i) {
    sectors.sectors[i] = static_cast<float>(i) / 24.0f;
  }

  auto payload = ser.SerializeLidar24(sectors);
  auto frame = ser.Frame(MsgType::kLidar24, payload);

  ParsedMessage msg;
  size_t consumed = 0;
  auto result = ParseMessage(frame.data(), frame.size(), msg, consumed);
  CHECK(result == ParseResult::kOk);

  auto d = DeserializeLidar24(msg.payload);
  for (int i = 0; i < 24; ++i) {
    CHECK(Near(d.sectors[i], static_cast<float>(i) / 24.0f));
  }
}

static void TestBatteryRoundtrip() {
  std::printf("[TestBatteryRoundtrip]\n");
  Serializer ser;
  BatteryState batt;
  batt.voltage = 7.82f;
  batt.percentage = 87.0f;
  batt.status = 1;

  auto payload = ser.SerializeBattery(batt);
  auto frame = ser.Frame(MsgType::kBattery, payload);

  ParsedMessage msg;
  size_t consumed = 0;
  auto result = ParseMessage(frame.data(), frame.size(), msg, consumed);
  CHECK(result == ParseResult::kOk);

  auto d = DeserializeBattery(msg.payload);
  CHECK(Near(d.voltage, 7.82f));
  CHECK(Near(d.percentage, 87.0f));
  CHECK(d.status == 1);
}

static void TestCmdVelRoundtrip() {
  std::printf("[TestCmdVelRoundtrip]\n");
  Serializer ser;
  CmdVel cmd;
  cmd.linear_x = 0.20f;
  cmd.angular_z = -0.5f;

  auto payload = ser.SerializeCmdVel(cmd);
  auto frame = ser.Frame(MsgType::kCmdVel, payload);

  ParsedMessage msg;
  size_t consumed = 0;
  auto result = ParseMessage(frame.data(), frame.size(), msg, consumed);
  CHECK(result == ParseResult::kOk);

  auto d = DeserializeCmdVel(msg.payload);
  CHECK(Near(d.linear_x, 0.20f));
  CHECK(Near(d.angular_z, -0.5f));
}

static void TestNavGoalRoundtrip() {
  std::printf("[TestNavGoalRoundtrip]\n");
  Serializer ser;
  NavGoal goal;
  goal.x = 3.5f;
  goal.y = -1.2f;
  goal.theta = 1.57f;

  auto payload = ser.SerializeNavGoal(goal);
  auto frame = ser.Frame(MsgType::kNavGoal, payload);

  ParsedMessage msg;
  size_t consumed = 0;
  auto result = ParseMessage(frame.data(), frame.size(), msg, consumed);
  CHECK(result == ParseResult::kOk);

  auto d = DeserializeNavGoal(msg.payload);
  CHECK(Near(d.x, 3.5f));
  CHECK(Near(d.y, -1.2f));
  CHECK(Near(d.theta, 1.57f));
}

static void TestDebugLogRoundtrip() {
  std::printf("[TestDebugLogRoundtrip]\n");
  Serializer ser;
  DebugLog log;
  log.severity = 2;
  log.timestamp_ns = 1234567890123ULL;
  log.text = "Hello pinky!";

  auto payload = ser.SerializeDebugLog(log);
  auto frame = ser.Frame(MsgType::kDebugLog, payload);

  ParsedMessage msg;
  size_t consumed = 0;
  auto result = ParseMessage(frame.data(), frame.size(), msg, consumed);
  CHECK(result == ParseResult::kOk);

  auto d = DeserializeDebugLog(msg.payload);
  CHECK(d.severity == 2);
  CHECK(d.timestamp_ns == 1234567890123ULL);
  CHECK(d.text == "Hello pinky!");
}

static void TestBadMagic() {
  std::printf("[TestBadMagic]\n");
  uint8_t bad_data[] = {0xFF, 0xFF, 0x01, 0x01, 0, 0, 0, 0, 0, 0, 0, 0};
  ParsedMessage msg;
  size_t consumed = 0;
  auto result = ParseMessage(bad_data, sizeof(bad_data), msg, consumed);
  CHECK(result == ParseResult::kBadMagic);
  CHECK(consumed == 1);
}

static void TestIncompleteData() {
  std::printf("[TestIncompleteData]\n");
  uint8_t partial[] = {0x50, 0x4B, 0x01};
  ParsedMessage msg;
  size_t consumed = 0;
  auto result = ParseMessage(partial, sizeof(partial), msg, consumed);
  CHECK(result == ParseResult::kIncomplete);
  CHECK(consumed == 0);
}

static void TestMultipleMessagesInStream() {
  std::printf("[TestMultipleMessagesInStream]\n");
  Serializer ser;

  CmdVel cmd1{0.1f, 0.2f};
  CmdVel cmd2{0.3f, -0.4f};

  auto frame1 = ser.Frame(MsgType::kCmdVel, ser.SerializeCmdVel(cmd1));
  auto frame2 = ser.Frame(MsgType::kCmdVel, ser.SerializeCmdVel(cmd2));

  // Concatenate both frames
  std::vector<uint8_t> stream;
  stream.insert(stream.end(), frame1.begin(), frame1.end());
  stream.insert(stream.end(), frame2.begin(), frame2.end());

  // Parse first message
  ParsedMessage msg1;
  size_t consumed1 = 0;
  auto r1 = ParseMessage(stream.data(), stream.size(), msg1, consumed1);
  CHECK(r1 == ParseResult::kOk);

  auto d1 = DeserializeCmdVel(msg1.payload);
  CHECK(Near(d1.linear_x, 0.1f));

  // Parse second message from remainder
  ParsedMessage msg2;
  size_t consumed2 = 0;
  auto r2 = ParseMessage(stream.data() + consumed1,
                          stream.size() - consumed1, msg2, consumed2);
  CHECK(r2 == ParseResult::kOk);

  auto d2 = DeserializeCmdVel(msg2.payload);
  CHECK(Near(d2.linear_x, 0.3f));
  CHECK(Near(d2.angular_z, -0.4f));
}

static void TestSequenceIncrement() {
  std::printf("[TestSequenceIncrement]\n");
  Serializer ser;

  auto f1 = ser.Frame(MsgType::kPing, ser.SerializePing(100));
  auto f2 = ser.Frame(MsgType::kPing, ser.SerializePing(200));

  ParsedMessage m1, m2;
  size_t c1 = 0, c2 = 0;
  ParseMessage(f1.data(), f1.size(), m1, c1);
  ParseMessage(f2.data(), f2.size(), m2, c2);

  CHECK(m2.sequence == m1.sequence + 1);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main() {
  TestCrc16();
  TestOdomRoundtrip();
  TestImuRoundtrip();
  TestLidar24Roundtrip();
  TestBatteryRoundtrip();
  TestCmdVelRoundtrip();
  TestNavGoalRoundtrip();
  TestDebugLogRoundtrip();
  TestBadMagic();
  TestIncompleteData();
  TestMultipleMessagesInStream();
  TestSequenceIncrement();

  std::printf("\n=== Results: %d passed, %d failed ===\n", g_passed, g_failed);
  return g_failed > 0 ? 1 : 0;
}
