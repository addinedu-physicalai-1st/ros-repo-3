#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace pinky {

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------
struct Timestamp {
  int64_t nanoseconds{0};

  static Timestamp Now() {
    auto now = std::chrono::steady_clock::now();
    return {std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch())
                .count()};
  }
};

// ---------------------------------------------------------------------------
// Geometry primitives
// ---------------------------------------------------------------------------
struct Vec2 {
  double x{0.0};
  double y{0.0};
};

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// ---------------------------------------------------------------------------
// Sensor data
// ---------------------------------------------------------------------------
struct Odometry {
  Timestamp stamp;
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double vx{0.0};
  double vth{0.0};
};

struct ImuData {
  Timestamp stamp;
  Quaternion orientation;
  Vec3 angular_velocity;
  Vec3 linear_acceleration;
};

struct LidarScan {
  Timestamp stamp;
  float angle_min{0.0f};
  float angle_max{0.0f};
  float angle_increment{0.0f};
  float range_min{0.0f};
  float range_max{0.0f};
  std::vector<float> ranges;
};

struct LidarSectors {
  Timestamp stamp;
  std::array<float, 24> sectors{};
};

struct BatteryState {
  Timestamp stamp;
  float voltage{0.0f};
  float percentage{0.0f};
  uint8_t status{0};
};

struct JointState {
  Timestamp stamp;
  std::array<double, 2> position{};  // left, right (rad)
  std::array<double, 2> velocity{};  // left, right (rad/s)
};

struct IrSensor {
  std::array<uint16_t, 3> values{};
};

struct UsSensor {
  float range{0.0f};
};

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
struct CmdVel {
  float linear_x{0.0f};
  float angular_z{0.0f};
};

struct LedCommand {
  uint8_t command{0};     // 0=clear, 1=set_pixel, 2=fill
  uint8_t pixel_mask{0};
  uint8_t r{0};
  uint8_t g{0};
  uint8_t b{0};
};

struct LampCommand {
  uint8_t mode{0};  // 0=off, 1=on, 2=blink, 3=dim
  uint8_t r{0};
  uint8_t g{0};
  uint8_t b{0};
  uint16_t time_ms{0};
};

struct NavGoal {
  float x{0.0f};
  float y{0.0f};
  float theta{0.0f};
};

struct Pose2D {
  float x{0.0f};
  float y{0.0f};
  float theta{0.0f};
};

// ---------------------------------------------------------------------------
// Robot status bitmask
// ---------------------------------------------------------------------------
struct RobotStatus {
  uint32_t flags{0};

  static constexpr uint32_t kMotorOk  = 1u << 0;
  static constexpr uint32_t kImuOk    = 1u << 1;
  static constexpr uint32_t kLidarOk  = 1u << 2;
  static constexpr uint32_t kAdcOk    = 1u << 3;
  static constexpr uint32_t kLedOk    = 1u << 4;
  static constexpr uint32_t kLcdOk    = 1u << 5;
  static constexpr uint32_t kCameraOk = 1u << 6;

  bool IsOk(uint32_t flag) const { return (flags & flag) != 0; }
  void Set(uint32_t flag) { flags |= flag; }
  void Clear(uint32_t flag) { flags &= ~flag; }
};

// ---------------------------------------------------------------------------
// Debug log
// ---------------------------------------------------------------------------
struct DebugLog {
  uint8_t severity{0};  // 0=TRACE, 1=DEBUG, 2=INFO, 3=WARN, 4=ERROR
  uint64_t timestamp_ns{0};
  std::string text;
};

// ---------------------------------------------------------------------------
// Camera frame (JPEG compressed)
// ---------------------------------------------------------------------------
struct CameraFrame {
  uint16_t width{0};
  uint16_t height{0};
  std::vector<uint8_t> jpeg_data;
};

// ---------------------------------------------------------------------------
// Map data (occupancy grid)
// ---------------------------------------------------------------------------
struct MapData {
  uint32_t width{0};
  uint32_t height{0};
  float resolution{0.0f};
  float origin_x{0.0f};
  float origin_y{0.0f};
  float origin_theta{0.0f};
  std::vector<uint8_t> data;  // RLE-compressed occupancy
};

}  // namespace pinky
