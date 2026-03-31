#include "pinky_core/core/odometry.h"

#include "pinky_core/common/constants.h"

namespace pinky {

OdometryAccumulator::OdometryAccumulator(double wheel_radius,
                                         double wheel_base,
                                         int pulses_per_rot)
    : wheel_radius_(wheel_radius),
      wheel_base_(wheel_base),
      pulses_per_rot_(pulses_per_rot),
      circumference_(2.0 * kPi * wheel_radius) {}

Odometry OdometryAccumulator::Update(int32_t encoder_l, int32_t encoder_r,
                                     Timestamp now) {
  if (!initialized_) {
    last_encoder_l_ = encoder_l;
    last_encoder_r_ = encoder_r;
    last_time_ = now;
    initialized_ = true;
    return {now, x_, y_, theta_, 0.0, 0.0};
  }

  double dt_ns = static_cast<double>(now.nanoseconds - last_time_.nanoseconds);
  double dt = dt_ns / 1e9;
  if (dt <= 0.0) {
    return {now, x_, y_, theta_, 0.0, 0.0};
  }

  int32_t delta_l = encoder_l - last_encoder_l_;
  int32_t delta_r = -(encoder_r - last_encoder_r_);  // right motor negated

  last_encoder_l_ = encoder_l;
  last_encoder_r_ = encoder_r;

  double dist_l =
      (static_cast<double>(delta_l) / pulses_per_rot_) * circumference_;
  double dist_r =
      (static_cast<double>(delta_r) / pulses_per_rot_) * circumference_;

  double delta_distance = (dist_r + dist_l) / 2.0;
  double delta_theta = (dist_r - dist_l) / wheel_base_;

  theta_ += delta_theta;
  x_ += delta_distance * std::cos(theta_);
  y_ += delta_distance * std::sin(theta_);

  double vx = delta_distance / dt;
  double vth = delta_theta / dt;

  last_time_ = now;

  return {now, x_, y_, theta_, vx, vth};
}

void OdometryAccumulator::Reset() {
  x_ = y_ = theta_ = 0.0;
  initialized_ = false;
}

}  // namespace pinky
