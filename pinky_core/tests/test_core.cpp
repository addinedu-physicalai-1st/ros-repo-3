#include <cassert>
#include "pinky_core/common/constants.h"
#include <cstdio>
#include <vector>

#include "pinky_core/common/constants.h"
#include "pinky_core/common/types.h"
#include "pinky_core/core/battery_monitor.h"
#include "pinky_core/core/diff_drive.h"
#include "pinky_core/core/led_controller.h"
#include "pinky_core/core/lidar_processor.h"
#include "pinky_core/core/odometry.h"

using namespace pinky;

static constexpr double kEps = 1e-4;
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

static bool Near(double a, double b, double eps = kEps) {
  return std::fabs(a - b) < eps;
}

// ---------------------------------------------------------------------------
// DiffDrive tests
// ---------------------------------------------------------------------------
static void TestDiffDriveZero() {
  std::printf("[TestDiffDriveZero]\n");
  DiffDrive dd(kWheelRadius, kWheelBase);
  auto [rpm_l, rpm_r] = dd.VelocityToRpm(0.0, 0.0);
  CHECK(Near(rpm_l, 0.0));
  CHECK(Near(rpm_r, 0.0));
}

static void TestDiffDriveStraight() {
  std::printf("[TestDiffDriveStraight]\n");
  DiffDrive dd(kWheelRadius, kWheelBase);
  // Pure forward motion: both wheels same speed, right negated
  auto [rpm_l, rpm_r] = dd.VelocityToRpm(0.1, 0.0);
  CHECK(Near(rpm_l, -rpm_r));
  CHECK(rpm_l > 0.0);

  // Roundtrip: rpm -> velocity
  auto [vx, vth] = dd.RpmToVelocity(rpm_l, rpm_r);
  CHECK(Near(vx, 0.1));
  CHECK(Near(vth, 0.0, 1e-3));
}

static void TestDiffDriveRotation() {
  std::printf("[TestDiffDriveRotation]\n");
  DiffDrive dd(kWheelRadius, kWheelBase);
  // Pure rotation: v=0, w=1.0 rad/s
  auto [rpm_l, rpm_r] = dd.VelocityToRpm(0.0, 1.0);
  // Left wheel goes backward, right wheel goes forward (both negated convention)
  CHECK(rpm_l < 0.0);
  CHECK(rpm_r < 0.0);  // right is negated, so both negative means opposite physical dirs

  auto [vx, vth] = dd.RpmToVelocity(rpm_l, rpm_r);
  CHECK(Near(vx, 0.0, 1e-3));
  CHECK(Near(vth, 1.0, 1e-2));
}

static void TestDiffDriveRpmClamp() {
  std::printf("[TestDiffDriveRpmClamp]\n");
  DiffDrive dd(kWheelRadius, kWheelBase, 100.0);
  // Very high velocity that would exceed max RPM
  auto [rpm_l, rpm_r] = dd.VelocityToRpm(10.0, 0.0);
  CHECK(std::abs(rpm_l) <= 100.0 + kEps);
  CHECK(std::abs(rpm_r) <= 100.0 + kEps);
}

// ---------------------------------------------------------------------------
// Odometry tests
// ---------------------------------------------------------------------------
static void TestOdometryInit() {
  std::printf("[TestOdometryInit]\n");
  OdometryAccumulator odom(kWheelRadius, kWheelBase, kEncoderPpr);
  Timestamp t0{1000000000LL};  // 1 second
  auto result = odom.Update(0, 0, t0);
  CHECK(Near(result.x, 0.0));
  CHECK(Near(result.y, 0.0));
  CHECK(Near(result.vx, 0.0));
}

static void TestOdometryStraight() {
  std::printf("[TestOdometryStraight]\n");
  OdometryAccumulator odom(kWheelRadius, kWheelBase, kEncoderPpr);
  Timestamp t0{0};
  odom.Update(0, 0, t0);

  // Advance both encoders by 4096 ticks = 1 revolution
  // Distance per wheel = circumference = 2*pi*0.028 ≈ 0.17593 m
  // Right encoder is negated: delta_r = -(encoder_r - last_encoder_r_)
  // So to make right wheel go forward, encoder_r should decrease
  Timestamp t1{1000000000LL};
  auto result = odom.Update(4096, -4096, t1);

  double expected_dist = 2.0 * kPi * kWheelRadius;
  CHECK(Near(result.x, expected_dist, 1e-3));
  CHECK(Near(result.y, 0.0, 1e-3));
  CHECK(Near(result.vx, expected_dist, 1e-3));  // 1 second elapsed
}

static void TestOdometryReset() {
  std::printf("[TestOdometryReset]\n");
  OdometryAccumulator odom(kWheelRadius, kWheelBase, kEncoderPpr);
  Timestamp t0{0};
  odom.Update(0, 0, t0);
  odom.Update(1000, -1000, {100000000LL});

  odom.Reset();
  CHECK(Near(odom.x(), 0.0));
  CHECK(Near(odom.y(), 0.0));
  CHECK(Near(odom.theta(), 0.0));
}

// ---------------------------------------------------------------------------
// LidarProcessor tests
// ---------------------------------------------------------------------------
static void TestLidarProcessorUniform() {
  std::printf("[TestLidarProcessorUniform]\n");
  LidarProcessor proc(24, 3.5f);

  // All ranges = 1.75m -> normalized = 0.5
  LidarScan scan;
  scan.ranges.assign(640, 1.75f);

  auto sectors = proc.Process(scan);
  for (int i = 0; i < 24; ++i) {
    CHECK(Near(sectors.sectors[i], 0.5f, 1e-3));
  }
}

static void TestLidarProcessorNanInf() {
  std::printf("[TestLidarProcessorNanInf]\n");
  LidarProcessor proc(24, 3.5f);

  LidarScan scan;
  scan.ranges.assign(480, 2.0f);
  scan.ranges[0] = std::nanf("");     // NaN -> 0.0
  scan.ranges[1] = INFINITY;          // +Inf -> 3.5
  scan.ranges[2] = -INFINITY;         // -Inf -> 0.0

  auto sectors = proc.Process(scan);
  // After roll, the modified values move to sector ~12 area
  // Key check: no NaN/Inf in output
  for (int i = 0; i < 24; ++i) {
    CHECK(!std::isnan(sectors.sectors[i]));
    CHECK(!std::isinf(sectors.sectors[i]));
    CHECK(sectors.sectors[i] >= 0.0f);
    CHECK(sectors.sectors[i] <= 1.0f);
  }
}

static void TestLidarProcessorMinPool() {
  std::printf("[TestLidarProcessorMinPool]\n");
  LidarProcessor proc(24, 3.5f);

  // 240 points, all 3.5 except first sector has one 0.7 value
  LidarScan scan;
  scan.ranges.assign(240, 3.5f);
  // After roll(120), index 120 becomes index 0, so original index 0 becomes index 120
  // Sector size = 240/24 = 10. Index 120 is in sector 12.
  // To put a low value in sector 0 after roll: set index at (240 - 120) = 120 (pre-roll)
  // Actually, after roll by n/2=120: new[i] = old[(i - 120) mod 240]
  // We want sector 0 (indices 0-9 after roll) to have a low value.
  // new[0] = old[(0 - 120) mod 240] = old[120]
  scan.ranges[120] = 0.7f;

  auto sectors = proc.Process(scan);
  CHECK(Near(sectors.sectors[0], 0.7f / 3.5f, 1e-3));
  // Other sectors should be 1.0
  CHECK(Near(sectors.sectors[5], 1.0f, 1e-3));
}

// ---------------------------------------------------------------------------
// BatteryMonitor tests
// ---------------------------------------------------------------------------
static void TestBatteryMonitorFull() {
  std::printf("[TestBatteryMonitorFull]\n");
  BatteryMonitor monitor(6.0f, 8.4f, 6.8f);

  // ADC value that gives ~8.4V
  // V = (adc/4096) * 4.096 / (13.0/28.0)
  // 8.4 = (adc/4096) * 4.096 * (28.0/13.0)
  // adc = 8.4 / (4.096 * 28.0/13.0) * 4096
  float target_v = 8.4f;
  float ratio = 4.096f * (28.0f / 13.0f);
  uint16_t adc = static_cast<uint16_t>((target_v / ratio) * 4096.0f);

  auto state = monitor.Update(adc);
  CHECK(Near(state.voltage, 8.4f, 0.1f));
  CHECK(Near(state.percentage, 100.0f, 2.0f));
  CHECK(!monitor.IsLow(state.voltage));
}

static void TestBatteryMonitorLow() {
  std::printf("[TestBatteryMonitorLow]\n");
  BatteryMonitor monitor(6.0f, 8.4f, 6.8f);

  // ADC value for ~6.5V (below threshold)
  float target_v = 6.5f;
  float ratio = 4.096f * (28.0f / 13.0f);
  uint16_t adc = static_cast<uint16_t>((target_v / ratio) * 4096.0f);

  auto state = monitor.Update(adc);
  CHECK(state.voltage < 6.8f + 0.2f);
  CHECK(monitor.IsLow(state.voltage));
}

// ---------------------------------------------------------------------------
// LedController tests
// ---------------------------------------------------------------------------
static void TestLedOff() {
  std::printf("[TestLedOff]\n");
  LedController ctrl;
  ctrl.SetMode(0, 255, 0, 0, 1000);
  uint32_t color = ctrl.Tick();
  CHECK(color == 0xFF000000);
}

static void TestLedSolidOn() {
  std::printf("[TestLedSolidOn]\n");
  LedController ctrl;
  ctrl.SetMode(1, 255, 128, 64, 0);
  uint32_t color = ctrl.Tick();
  CHECK(color == 0xFF'FF'80'40);
}

static void TestLedBlink() {
  std::printf("[TestLedBlink]\n");
  LedController ctrl;
  ctrl.SetMode(2, 255, 0, 0, 100);  // 100ms interval
  uint32_t c1 = ctrl.Tick();
  CHECK(c1 == (0xFF000000 | (255u << 16)));  // ON (red)

  // Tick 11 times (110ms > 100ms), should toggle
  for (int i = 0; i < 11; ++i) ctrl.Tick();
  uint32_t c2 = ctrl.Tick();
  CHECK(c2 == 0xFF000000);  // OFF
}

static void TestHsvRoundtrip() {
  std::printf("[TestHsvRoundtrip]\n");
  auto [h, s, v] = LedController::RgbToHsv(255, 128, 64);
  auto [r, g, b] = LedController::HsvToRgb(h, s, v);
  CHECK(r == 255);
  CHECK(std::abs(static_cast<int>(g) - 128) <= 1);
  CHECK(std::abs(static_cast<int>(b) - 64) <= 1);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main() {
  TestDiffDriveZero();
  TestDiffDriveStraight();
  TestDiffDriveRotation();
  TestDiffDriveRpmClamp();

  TestOdometryInit();
  TestOdometryStraight();
  TestOdometryReset();

  TestLidarProcessorUniform();
  TestLidarProcessorNanInf();
  TestLidarProcessorMinPool();

  TestBatteryMonitorFull();
  TestBatteryMonitorLow();

  TestLedOff();
  TestLedSolidOn();
  TestLedBlink();
  TestHsvRoundtrip();

  std::printf("\n=== Core Tests: %d passed, %d failed ===\n",
              g_passed, g_failed);
  return g_failed > 0 ? 1 : 0;
}
