#include "pinky_core/core/sensor_fusion.h"

#include <cmath>
#include <cstring>

namespace pinky {

namespace {

// Multiply two 4×4 matrices: result = a * b
void Mat4Mul(const double a[4][4], const double b[4][4], double result[4][4]) {
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 4; ++k) {
        sum += a[i][k] * b[k][j];
      }
      result[i][j] = sum;
    }
  }
}

// Compute P = F * P * F^T + Q  (in-place, overwrites P)
void PropagateCovariance(double P[4][4], const double F[4][4],
                         const double Q[4][4]) {
  // FP = F * P
  double FP[4][4];
  Mat4Mul(F, P, FP);

  // FPFt = FP * F^T  (FPFt[i][j] = sum_k FP[i][k] * F[j][k])
  double FPFt[4][4];
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 4; ++k) {
        sum += FP[i][k] * F[j][k];
      }
      FPFt[i][j] = sum;
    }
  }

  // P = FPFt + Q
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      P[i][j] = FPFt[i][j] + Q[i][j];
    }
  }
}

}  // namespace

SensorFusion::SensorFusion(double initial_x,
                           double initial_y,
                           double initial_theta) {
  state_[0] = initial_x;
  state_[1] = initial_y;
  state_[2] = initial_theta;
  state_[3] = 0.0;

  // Initial covariance: confident in position, uncertain in heading/w
  std::memset(P_, 0, sizeof(P_));
  P_[2][2] = 0.1;   // theta uncertainty (rad²)
  P_[3][3] = 1.0;   // w uncertainty (rad/s)²
}

bool SensorFusion::Predict(double v, double w_enc, Timestamp now) {
  if (!initialized_) {
    last_predict_time_ = now;
    initialized_ = true;
    return false;
  }

  const double dt_ns =
      static_cast<double>(now.nanoseconds - last_predict_time_.nanoseconds);
  const double dt = dt_ns / 1e9;
  if (dt <= 0.0 || dt > 1.0) {
    // Skip degenerate steps (first call or clock jump)
    last_predict_time_ = now;
    return false;
  }

  const double theta = state_[2];
  const double w     = state_[3];

  // ── State propagation ────────────────────────────────────────────────
  state_[0] += v * std::cos(theta) * dt;
  state_[1] += v * std::sin(theta) * dt;
  state_[2] += w * dt;
  // state_[3] = w  (constant-velocity model — corrected by measurements)

  // Normalise theta to [-π, π]
  while (state_[2] >  M_PI) state_[2] -= 2.0 * M_PI;
  while (state_[2] < -M_PI) state_[2] += 2.0 * M_PI;

  // ── Covariance propagation ───────────────────────────────────────────
  // Jacobian of f w.r.t. state (linearised at current theta, w):
  // F = [[1, 0, -v*sin(theta)*dt,  0 ],
  //      [0, 1,  v*cos(theta)*dt,  0 ],
  //      [0, 0,  1,                dt],
  //      [0, 0,  0,                1 ]]
  double F[4][4] = {
    {1.0,  0.0, -v * std::sin(theta) * dt,  0.0},
    {0.0,  1.0,  v * std::cos(theta) * dt,  0.0},
    {0.0,  0.0,  1.0,                        dt },
    {0.0,  0.0,  0.0,                        1.0},
  };

  double Q[4][4] = {};
  Q[0][0] = kQX;
  Q[1][1] = kQY;
  Q[2][2] = kQTheta;
  Q[3][3] = kQW;

  PropagateCovariance(P_, F, Q);

  last_predict_time_ = now;

  // ── Encoder angular velocity measurement update ──────────────────────
  MeasureAngularVelocity(w_enc, kREnc);

  return true;
}

void SensorFusion::UpdateImu(double imu_yaw_rate, Timestamp /*now*/) {
  if (!initialized_) return;
  MeasureAngularVelocity(imu_yaw_rate, kRImu);
}

void SensorFusion::MeasureAngularVelocity(double z, double r) {
  // Measurement model: H = [0, 0, 0, 1]  →  z = state_[3] + noise
  // Innovation: y = z - state_[3]
  const double y = z - state_[3];

  // Innovation covariance: S = P[3][3] + R
  const double S = P_[3][3] + r;
  if (S < 1e-9) return;  // degenerate

  // Kalman gain: K = P[:,3] / S  (4-element column vector)
  double K[4];
  for (int i = 0; i < 4; ++i) {
    K[i] = P_[i][3] / S;
  }

  // State update: x += K * y
  for (int i = 0; i < 4; ++i) {
    state_[i] += K[i] * y;
  }

  // Covariance update: P = (I - K*H) * P
  // K*H is a 4×4 matrix where only column 3 is non-zero: (K*H)[i][j] = K[i]*H[j]
  // H = [0,0,0,1], so (K*H)[i][3] = K[i], all other columns zero.
  // (I - K*H)*P row i: P[i][j] - K[i]*P[3][j]
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      P_[i][j] -= K[i] * P_[3][j];
    }
  }

  // Normalise theta after update
  while (state_[2] >  M_PI) state_[2] -= 2.0 * M_PI;
  while (state_[2] < -M_PI) state_[2] += 2.0 * M_PI;
}

void SensorFusion::Reset(double x, double y, double theta) {
  state_[0] = x;
  state_[1] = y;
  state_[2] = theta;
  state_[3] = 0.0;

  std::memset(P_, 0, sizeof(P_));
  P_[2][2] = 0.1;
  P_[3][3] = 1.0;

  initialized_ = false;
}

Odometry SensorFusion::GetState(Timestamp stamp) const {
  return {stamp, state_[0], state_[1], state_[2], 0.0, state_[3]};
}

}  // namespace pinky
