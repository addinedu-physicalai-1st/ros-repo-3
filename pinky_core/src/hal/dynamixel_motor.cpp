#include "pinky_core/hal/dynamixel_motor.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>
#include "pinky_core/common/constants.h"

#define ADDR_OPERATING_MODE    11
#define ADDR_TORQUE_ENABLE     64
#define ADDR_LED_RED           65
#define ADDR_GOAL_VELOCITY     104
#define ADDR_PROFILE_ACCEL     108
#define ADDR_PRESENT_VELOCITY  128
#define ADDR_PRESENT_POSITION  132

#define LEN_GOAL_VELOCITY      4
#define LEN_PRESENT_VELOCITY   4
#define LEN_PRESENT_POSITION   4

#define PROTOCOL_VERSION       2.0

namespace pinky {

using namespace std::chrono_literals;

DynamixelMotor::DynamixelMotor(const Config& config) : config_(config) {}

DynamixelMotor::~DynamixelMotor() {
  if (portHandler_ && packetHandler_) {
    // Disable torque
    SetDoubleRpm(0, 0);
    std::this_thread::sleep_for(100ms);
    for (int id : config_.dxl_ids) {
      packetHandler_->write1ByteTxRx(portHandler_, static_cast<uint8_t>(id), ADDR_TORQUE_ENABLE, 0);
      packetHandler_->write1ByteTxRx(portHandler_, static_cast<uint8_t>(id), ADDR_LED_RED, 0);
    }
  }
  if (portHandler_) {
    portHandler_->closePort();
  }
}

bool DynamixelMotor::Init() {
  portHandler_ = dynamixel::PortHandler::getPortHandler(config_.port_name.c_str());
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler_->openPort()) {
    std::cerr << "DynamixelMotor: Failed to open port " << config_.port_name << "\n";
    return false;
  }

  if (!portHandler_->setBaudRate(config_.baudrate)) {
    std::cerr << "DynamixelMotor: Failed to set baudrate " << config_.baudrate << "\n";
    return false;
  }

  groupSyncWrite_ = std::make_unique<dynamixel::GroupSyncWrite>(
      portHandler_, packetHandler_, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);
  groupBulkRead_ = std::make_unique<dynamixel::GroupBulkRead>(
      portHandler_, packetHandler_);

  InitializeMotors();
  return true;
}

void DynamixelMotor::InitializeMotors() {
  for (int id : config_.dxl_ids) {
    packetHandler_->reboot(portHandler_, static_cast<uint8_t>(id));
    std::this_thread::sleep_for(500ms);

    packetHandler_->write1ByteTxRx(portHandler_, static_cast<uint8_t>(id), ADDR_OPERATING_MODE, 1); // Velocity mode
    packetHandler_->write4ByteTxRx(portHandler_, static_cast<uint8_t>(id), ADDR_PROFILE_ACCEL, 200);
    packetHandler_->write1ByteTxRx(portHandler_, static_cast<uint8_t>(id), ADDR_TORQUE_ENABLE, 1);
    packetHandler_->write1ByteTxRx(portHandler_, static_cast<uint8_t>(id), ADDR_LED_RED, 1);
  }
}

void DynamixelMotor::SetVelocityWait(double left_rpm, double right_rpm) {
  SetDoubleRpm(left_rpm, right_rpm);
}

bool DynamixelMotor::SetDoubleRpm(double rpm_l, double rpm_r) {
  if (!groupSyncWrite_) return false;

  groupSyncWrite_->clearParam();
  
  int32_t val_l = static_cast<int32_t>(rpm_l * rpm_to_value_scale_);
  int32_t val_r = static_cast<int32_t>(rpm_r * rpm_to_value_scale_);

  uint8_t param_l[4], param_r[4];
  param_l[0] = DXL_LOBYTE(DXL_LOWORD(val_l));
  param_l[1] = DXL_HIBYTE(DXL_LOWORD(val_l));
  param_l[2] = DXL_LOBYTE(DXL_HIWORD(val_l));
  param_l[3] = DXL_HIBYTE(DXL_HIWORD(val_l));

  param_r[0] = DXL_LOBYTE(DXL_LOWORD(val_r));
  param_r[1] = DXL_HIBYTE(DXL_LOWORD(val_r));
  param_r[2] = DXL_LOBYTE(DXL_HIWORD(val_r));
  param_r[3] = DXL_HIBYTE(DXL_HIWORD(val_r));

  groupSyncWrite_->addParam(static_cast<uint8_t>(config_.dxl_ids[0]), param_l);
  groupSyncWrite_->addParam(static_cast<uint8_t>(config_.dxl_ids[1]), param_r);

  return groupSyncWrite_->txPacket() == COMM_SUCCESS;
}

bool DynamixelMotor::ReadFeedback(double& rpm_l, double& rpm_r, int32_t& pos_l, int32_t& pos_r) {
  if (!groupBulkRead_) return false;

  groupBulkRead_->clearParam();
  uint16_t read_len = LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION;

  for (int id : config_.dxl_ids) {
    groupBulkRead_->addParam(static_cast<uint8_t>(id), ADDR_PRESENT_VELOCITY, read_len);
  }

  if (groupBulkRead_->txRxPacket() != COMM_SUCCESS) return false;

  uint8_t id_l = static_cast<uint8_t>(config_.dxl_ids[0]);
  uint8_t id_r = static_cast<uint8_t>(config_.dxl_ids[1]);

  if (!groupBulkRead_->isAvailable(id_l, ADDR_PRESENT_VELOCITY, read_len) ||
      !groupBulkRead_->isAvailable(id_r, ADDR_PRESENT_VELOCITY, read_len)) {
    return false;
  }

  int32_t vel_raw_l = groupBulkRead_->getData(id_l, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
  pos_l = groupBulkRead_->getData(id_l, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
  int32_t vel_raw_r = groupBulkRead_->getData(id_r, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
  pos_r = groupBulkRead_->getData(id_r, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

  rpm_l = static_cast<double>(vel_raw_l) / rpm_to_value_scale_;
  rpm_r = static_cast<double>(vel_raw_r) / rpm_to_value_scale_;

  return true;
}

JointState DynamixelMotor::ReadJointState() {
  JointState js;
  js.stamp = Timestamp::Now();
  double rpm_l = 0, rpm_r = 0;
  int32_t pos_l = 0, pos_r = 0;

  if (ReadFeedback(rpm_l, rpm_r, pos_l, pos_r)) {
    js.position[0] = (static_cast<double>(pos_l) / pulses_per_rot_) * (2.0 * kPi);
    js.position[1] = (static_cast<double>(pos_r) / pulses_per_rot_) * (2.0 * kPi);
    js.velocity[0] = rpm_l * rpm_to_rads_;
    js.velocity[1] = rpm_r * rpm_to_rads_;
  }

  return js;
}

}  // namespace pinky
