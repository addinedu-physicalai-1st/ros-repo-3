#pragma once

#include <cstdint>

#include "pinky_core/common/types.h"

namespace pinky {

// Accumulates robot pose from wheel rotation deltas (radians).
class OdometryAccumulator {
 public:
  OdometryAccumulator(double wheel_radius, double wheel_base);

  // Feed cumulative wheel positions in radians. Returns updated odometry.
  // First call initializes internal state and returns zeros.
  Odometry Update(double left_rad, double right_rad, Timestamp now);

  void Reset();
  void Reset(double x, double y, double theta);

  double x() const { return x_; }
  double y() const { return y_; }
  double theta() const { return theta_; }

 private:
  double wheel_radius_;
  double wheel_base_;

  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};

  double last_left_rad_{0.0};
  double last_right_rad_{0.0};
  Timestamp last_time_{};
  bool initialized_{false};
};

}  // namespace pinky
