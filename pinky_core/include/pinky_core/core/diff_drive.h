#pragma once

#include <cstdint>
#include <utility>

namespace pinky {

// Differential drive kinematics (pure math, no hardware dependency).
class DiffDrive {
 public:
  DiffDrive(double wheel_radius, double wheel_base, double max_rpm = 100.0,
            double rpm_to_value_scale = 1.0 / 0.229);

  // cmd_vel -> motor RPMs.
  // Returns (rpm_left, rpm_right). Right motor is negated for convention.
  std::pair<double, double> VelocityToRpm(double linear_x,
                                          double angular_z) const;

  // motor RPMs -> cmd_vel.
  // rpm_right should be the raw (negated) value from the motor.
  std::pair<double, double> RpmToVelocity(double rpm_left,
                                          double rpm_right) const;

  // Convert RPM to Dynamixel velocity value.
  int32_t RpmToValue(double rpm) const;

  double wheel_radius() const { return wheel_radius_; }
  double wheel_base() const { return wheel_base_; }
  double circumference() const { return circumference_; }

 private:
  double wheel_radius_;
  double wheel_base_;
  double max_rpm_;
  double rpm_to_value_scale_;
  double circumference_;
};

}  // namespace pinky
