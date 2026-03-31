#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "pinky_core/hal/interfaces.h"
#include "pinky_core/common/constants.h"

namespace pinky {

class DynamixelMotor : public IMotorDriver {
 public:
  struct Config {
    std::string port_name{"/dev/ttyAMA4"};
    int baudrate{1000000};
    std::vector<int> dxl_ids{1, 2}; // left=1, right=2
  };

  explicit DynamixelMotor(const Config& config);
  ~DynamixelMotor() override;

  bool Init() override;

  // Set motor velocities in rad/s
  void SetVelocityWait(double left_rad_s, double right_rad_s) override;

  // Read current joints state (position, velocity)
  JointState ReadJointState() override;

  // Also expose raw encoder counts for odometry calc if needed
  bool ReadFeedback(double& rpm_l, double& rpm_r, int32_t& pos_l, int32_t& pos_r);

 private:
  void InitializeMotors();
  bool SetDoubleRpm(double rpm_l, double rpm_r);

  Config config_;
  double rpm_to_value_scale_{1.0 / 0.229};
  double rpm_to_rads_{2.0 * kPi / 60.0};
  int pulses_per_rot_{4096};

  dynamixel::PortHandler* portHandler_{nullptr};
  dynamixel::PacketHandler* packetHandler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWrite_;
  std::unique_ptr<dynamixel::GroupBulkRead> groupBulkRead_;
};

}  // namespace pinky
