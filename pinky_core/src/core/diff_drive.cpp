#include "pinky_core/core/diff_drive.h"

#include <algorithm>
#include <cmath>
#include "pinky_core/common/constants.h"

namespace pinky {

DiffDrive::DiffDrive(double wheel_radius, double wheel_base, double max_rpm,
                     double rpm_to_value_scale)
    : wheel_radius_(wheel_radius),
      wheel_base_(wheel_base),
      max_rpm_(max_rpm),
      rpm_to_value_scale_(rpm_to_value_scale),
      circumference_(2.0 * kPi * wheel_radius) {}

std::pair<double, double> DiffDrive::VelocityToRpm(double linear_x,
                                                   double angular_z) const {
  double v_l = linear_x - (angular_z * wheel_base_ / 2.0);
  double v_r = linear_x + (angular_z * wheel_base_ / 2.0);

  double rpm_l = (v_l / wheel_radius_) * 60.0 / (2.0 * kPi);
  double rpm_r = -(v_r / wheel_radius_) * 60.0 / (2.0 * kPi);

  // Limit RPM while preserving ratio
  double current_max = std::max(std::abs(rpm_l), std::abs(rpm_r));
  if (current_max > max_rpm_) {
    double scale = max_rpm_ / current_max;
    rpm_l *= scale;
    rpm_r *= scale;
  }

  return {rpm_l, rpm_r};
}

std::pair<double, double> DiffDrive::RpmToVelocity(double rpm_left,
                                                   double rpm_right) const {
  // rpm_right is negated on the wire, so negate back
  double v_l = rpm_left * (2.0 * kPi * wheel_radius_) / 60.0;
  double v_r = -rpm_right * (2.0 * kPi * wheel_radius_) / 60.0;

  double linear_x = (v_l + v_r) / 2.0;
  double angular_z = (v_r - v_l) / wheel_base_;

  return {linear_x, angular_z};
}

int32_t DiffDrive::RpmToValue(double rpm) const {
  return static_cast<int32_t>(std::round(rpm * rpm_to_value_scale_));
}

}  // namespace pinky
