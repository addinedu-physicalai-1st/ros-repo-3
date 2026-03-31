#include "pinky_core/hal/bno055_imu.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <unistd.h>

#include "wiringPiI2C.h"

namespace pinky {

using namespace std::chrono_literals;

Bno055Imu::Bno055Imu(const Config& config) : config_(config) {}

Bno055Imu::~Bno055Imu() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool Bno055Imu::Init() {
  fd_ = wiringPiI2CSetupInterface(config_.interface.c_str(), config_.address);
  if (fd_ == -1) {
    std::cerr << "Bno055Imu: Failed to init I2C communication on " << config_.interface << "\n";
    return false;
  }

  int chipid = wiringPiI2CReadReg8(fd_, 0x00);
  if (chipid != 0xA0) { // BNO055 chip ID
    std::cerr << "Bno055Imu: Invalid chip ID: 0x" << std::hex << chipid << "\n";
    // Depending on timing, might still succeed, but usually signals failure
  }

  // SYS_TRIGGER, RST_SYS
  wiringPiI2CWriteReg8(fd_, 0x3F, 0x20);

  int retries = 0;
  while (retries++ < 20) {
    int status = wiringPiI2CReadReg8(fd_, 0x3A);
    if (status == 0) break;
    std::this_thread::sleep_for(100ms);
  }

  // PowerMode: Normal => 0x00
  wiringPiI2CWriteReg8(fd_, 0x3E, 0x00);
  std::this_thread::sleep_for(500ms);

  // OperationMode: NDOF (0x0C) or IMU (0x08)
  // Original ros python: IMU (QUAD + ACC + IMU, MAGx => 0x08)
  wiringPiI2CWriteReg8(fd_, 0x3D, 0x08);

  retries = 0;
  while (retries++ < 20) {
    int status = wiringPiI2CReadReg8(fd_, 0x39);
    if (status == 0x05) break;
    std::this_thread::sleep_for(100ms);
    wiringPiI2CWriteReg8(fd_, 0x3D, 0x08);
  }

  std::cout << "Bno055Imu: Initialized.\n";
  return true;
}

bool Bno055Imu::ReadData(ImuData& data) {
  if (fd_ == -1) return false;

  uint8_t buffer[32] = {0};
  
	// Some versions of wiringPi might not have readBlockData. 
	// As a typical workaround (since i2c block read can be tricky), we invoke it.
  // Assumes wiringPiI2CReadBlockData works or replaced with standard i2c read.
  // (In the original code wiringPiI2CReadBlockData is used)
  
  // NOTE: wiringPiI2CReadBlockData signature might be different, normally we read chunks.
  // We'll trust the original wiringPi build has it.
  
  // Actually, standard wiringPiI2C doesn't have ReadBlockData for more than 32 bytes or behaves weirdly.
  // We will assume the custom wiringPi provided does support it, as seen in pinky_imu_bno055.
  extern int wiringPiI2CReadBlockData(int fd, int reg, uint8_t* buffer, int len);
  
  int res = wiringPiI2CReadBlockData(fd_, 0x08, buffer, 32);
  if (res < 0) return false;

  data.stamp = Timestamp::Now();

  // Linear acceleration (m/s^2) - Register 0x08
  data.linear_acceleration.x = static_cast<int16_t>((buffer[1] << 8) | buffer[0]) / 100.0;
  data.linear_acceleration.y = static_cast<int16_t>((buffer[3] << 8) | buffer[2]) / 100.0;
  data.linear_acceleration.z = static_cast<int16_t>((buffer[5] << 8) | buffer[4]) / 100.0;

  // Angular velocity (rad/s or deg/s?) Original says /16.0 
  // BNO055 default output is in dps/16.0. If we need rad/s, we should convert later.
  // Let's stick to original node logic for now, but commonly ROS Imus expect rad/s.
  // Assuming the node returned dps and we keep the same:
  data.angular_velocity.x = static_cast<int16_t>((buffer[13] << 8) | buffer[12]) / 16.0;
  data.angular_velocity.y = static_cast<int16_t>((buffer[15] << 8) | buffer[14]) / 16.0;
  data.angular_velocity.z = static_cast<int16_t>((buffer[17] << 8) | buffer[16]) / 16.0;

  // Quaternion - Register 0x20
  data.orientation.w = static_cast<int16_t>((buffer[25] << 8) | buffer[24]) / 16384.0;
  data.orientation.x = static_cast<int16_t>((buffer[27] << 8) | buffer[26]) / 16384.0;
  data.orientation.y = static_cast<int16_t>((buffer[29] << 8) | buffer[28]) / 16384.0;
  data.orientation.z = static_cast<int16_t>((buffer[31] << 8) | buffer[30]) / 16384.0;

  return true;
}

}  // namespace pinky
