#include "pinky_core/core/odometry.h"

#include <cmath>

namespace pinky {

OdometryAccumulator::OdometryAccumulator(double wheel_radius,
                                         double wheel_base)
    : wheel_radius_(wheel_radius),
      wheel_base_(wheel_base) {}

Odometry OdometryAccumulator::Update(double left_rad, double right_rad,
                                     Timestamp now) {
  if (!initialized_) {
    last_left_rad_ = left_rad;
    last_right_rad_ = right_rad;
    last_time_ = now;
    initialized_ = true;
    return {now, x_, y_, theta_, 0.0, 0.0};
  }

  double dt_ns = static_cast<double>(now.nanoseconds - last_time_.nanoseconds);
  double dt = dt_ns / 1e9;
  if (dt <= 0.0) {
    return {now, x_, y_, theta_, 0.0, 0.0};
  }

  double delta_l_rad = left_rad - last_left_rad_;
  double delta_r_rad = -(right_rad - last_right_rad_);  // right motor negated

  last_left_rad_ = left_rad;
  last_right_rad_ = right_rad;

  double dist_l = delta_l_rad * wheel_radius_;
  double dist_r = delta_r_rad * wheel_radius_;

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

void OdometryAccumulator::Reset(double x, double y, double theta) {
  x_ = x;
  y_ = y;
  theta_ = theta;
  initialized_ = false;
}

}  // namespace pinky
