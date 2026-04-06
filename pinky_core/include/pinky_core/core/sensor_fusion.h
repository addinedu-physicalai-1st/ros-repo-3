#pragma once

#include "pinky_core/common/types.h"

namespace pinky {

// Extended Kalman Filter (EKF) for 2D robot localization.
//
// State vector: [x, y, theta, w]
//   x, y    — position (metres)
//   theta   — heading (radians, normalised to [-π, π])
//   w       — angular velocity (rad/s)
//
// Inputs:
//   Predict(v, w_enc, now)    — encoder: linear vel + angular vel (control + measurement)
//   UpdateImu(w_imu, now)     — IMU: angular velocity (measurement, lower noise)
//
// Both encoder angular velocity and IMU angular velocity are treated as
// independent measurements of state[3] (w), with different noise variances.
class SensorFusion {
 public:
  SensorFusion(double initial_x = 0.0,
               double initial_y = 0.0,
               double initial_theta = 0.0);

  // Propagate state with encoder data.
  // v      — linear velocity from encoders (m/s)
  // w_enc  — angular velocity from encoders (rad/s); also used as measurement
  // Returns true if state was updated.
  bool Predict(double v, double w_enc, Timestamp now);

  // Apply IMU angular velocity measurement (IMU is more accurate than encoders).
  void UpdateImu(double imu_yaw_rate, Timestamp now);

  void Reset(double x, double y, double theta);

  Odometry GetState(Timestamp stamp) const;

 private:
  // Apply a scalar angular-velocity measurement z with noise variance r.
  // Measurement model: H = [0, 0, 0, 1]
  void MeasureAngularVelocity(double z, double r);

  // state_[0]=x, state_[1]=y, state_[2]=theta, state_[3]=w
  double state_[4]{0.0, 0.0, 0.0, 0.0};

  // Covariance matrix P (4×4)
  double P_[4][4]{};

  // Process noise Q (diagonal, tuned empirically)
  static constexpr double kQX     = 1e-4;   // position uncertainty / step
  static constexpr double kQY     = 1e-4;
  static constexpr double kQTheta = 1e-3;   // heading drift
  static constexpr double kQW     = 1e-2;   // angular velocity random walk

  // Measurement noise variances
  static constexpr double kREnc = 0.10;   // encoder wheel slippage (rad/s)²
  static constexpr double kRImu = 0.02;   // IMU gyroscope noise (rad/s)²

  Timestamp last_predict_time_{0};
  bool initialized_{false};
};

}  // namespace pinky
