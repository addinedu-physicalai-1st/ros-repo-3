#pragma once

#include <cstdint>

#include "pinky_core/common/types.h"

namespace pinky {

// Accumulates robot pose from encoder tick deltas.
class OdometryAccumulator {
 public:
  OdometryAccumulator(double wheel_radius, double wheel_base,
                      int pulses_per_rot);

  // Feed raw encoder positions. Returns updated odometry.
  // First call initializes internal state and returns zeros.
  Odometry Update(int32_t encoder_l, int32_t encoder_r, Timestamp now);

  void Reset();

  double x() const { return x_; }
  double y() const { return y_; }
  double theta() const { return theta_; }

 private:
  double wheel_radius_;
  double wheel_base_;
  int pulses_per_rot_;
  double circumference_;

  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};

  int32_t last_encoder_l_{0};
  int32_t last_encoder_r_{0};
  Timestamp last_time_{};
  bool initialized_{false};
};

}  // namespace pinky
