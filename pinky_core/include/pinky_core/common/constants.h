#pragma once

#include <cmath>
#include <cstdint>

namespace pinky {

// ---------------------------------------------------------------------------
// Physical robot parameters
// ---------------------------------------------------------------------------
constexpr double kWheelRadius     = 0.028;   // meters
constexpr double kWheelBase       = 0.0961;  // meters (wheel separation)
constexpr int    kEncoderPpr      = 4096;    // pulses per revolution
constexpr double kMaxRpm          = 100.0;
constexpr double kRpmToValueScale = 1.0 / 0.229;  // Dynamixel unit conversion

// ---------------------------------------------------------------------------
// Motor hardware
// ---------------------------------------------------------------------------
constexpr int    kMotorIdLeft     = 1;
constexpr int    kMotorIdRight    = 2;
constexpr int    kMotorBaudrate   = 1000000;  // 1MHz

// ---------------------------------------------------------------------------
// LiDAR
// ---------------------------------------------------------------------------
constexpr float  kMaxLidarDist    = 3.5f;    // meters (normalization ceiling)
constexpr int    kLidarSectors    = 24;

// ---------------------------------------------------------------------------
// RL inference
// ---------------------------------------------------------------------------
constexpr int    kStateDim        = 28;
constexpr int    kActionDim       = 2;
constexpr float  kVMin            = 0.0f;    // m/s
constexpr float  kVMax            = 0.26f;   // m/s
constexpr float  kWMax            = 1.0f;    // rad/s
constexpr float  kKpV             = 0.5f;    // PD linear proportional
constexpr float  kKdV             = 0.1f;    // PD linear derivative
constexpr float  kKpW             = 1.0f;    // PD angular proportional
constexpr float  kKdW             = 0.2f;    // PD angular derivative
constexpr int    kMaxSteps        = 750;
constexpr float  kGoalDistScale   = 5.0f;    // normalization divisor
constexpr float  kGoalTolerance   = 0.15f;   // meters
constexpr float  kLookaheadDist   = 0.5f;    // meters
constexpr double kControlPeriodMs = 50.0;    // 20Hz

// ---------------------------------------------------------------------------
// NavLoop defaults (overridden by rl_config.yaml at runtime)
// ---------------------------------------------------------------------------
constexpr float  kTurnFirstEnterRad = 1.833f;   // 105 deg
constexpr float  kTurnFirstExitRad  = 1.396f;   // 80 deg
constexpr float  kEmaAlpha          = 0.2f;
constexpr float  kAngularDeadzone   = 0.05f;
constexpr float  kSafetyStopDist    = 0.12f;    // meters
constexpr float  kSafetyScaleDist   = 0.25f;    // meters
constexpr float  kPCtrlVMax         = 0.12f;    // m/s
constexpr float  kPCtrlWMax         = 0.8f;     // rad/s
constexpr float  kLidarMinRange     = 0.15f;    // RPLIDAR S2 min valid range

// ---------------------------------------------------------------------------
// Battery
// ---------------------------------------------------------------------------
constexpr float  kBattVMin        = 6.0f;    // 0% voltage
constexpr float  kBattVMax        = 8.4f;    // 100% voltage
constexpr float  kBattLowThresh   = 6.8f;    // low battery warning

// ---------------------------------------------------------------------------
// Sensor rates (Hz)
// ---------------------------------------------------------------------------
constexpr double kMotorRate       = 50.0;
constexpr double kImuRate         = 100.0;
constexpr double kAdcRate         = 20.0;
constexpr double kLidarRate       = 10.0;
constexpr double kControlRate     = 20.0;    // RL inference rate

// ---------------------------------------------------------------------------
// Network
// ---------------------------------------------------------------------------
constexpr uint16_t kTcpPort       = 9100;
constexpr uint16_t kUdpPort       = 9200;

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------
constexpr double kPi              = 3.14159265358979323846;
constexpr double kTwoPi           = 2.0 * 3.14159265358979323846;

inline double NormalizeAngle(double angle) {
  while (angle > kPi)  angle -= kTwoPi;
  while (angle < -kPi) angle += kTwoPi;
  return angle;
}

template <typename T>
T Clamp(T value, T lo, T hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

}  // namespace pinky
