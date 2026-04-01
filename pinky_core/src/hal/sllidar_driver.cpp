#include "pinky_core/hal/sllidar_driver.h"

#include <iostream>
#include <cmath>
#include "pinky_core/common/constants.h"

// Slamtec SDK headers (assumed available in include path)
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

namespace pinky {

using namespace sl;

SllidarDriver::SllidarDriver(const Config& config) : config_(config) {}

SllidarDriver::~SllidarDriver() {
  if (drv_) {
    drv_->stop();
    drv_->setMotorSpeed(0);
    delete drv_;
    drv_ = nullptr;
  }
}

bool SllidarDriver::Init() {
  IChannel* channel = *createSerialPortChannel(config_.port, config_.baudrate);
  drv_ = *createLidarDriver();
  
  if (!drv_ || !channel) {
    std::cerr << "SllidarDriver: Failed to create channel or driver\n";
    return false;
  }

  if (SL_IS_FAIL(drv_->connect(channel))) {
    std::cerr << "SllidarDriver: Failed to connect to LiDAR on " << config_.port << "\n";
    return false;
  }

  sl_lidar_response_device_info_t devinfo;
  sl_result op_result = drv_->getDeviceInfo(devinfo);
  if (SL_IS_FAIL(op_result)) {
    std::cerr << "SllidarDriver: Failed to get device info, code: 0x" << std::hex << op_result << std::dec << "\n";
    return false;
  }

  std::cout << "SLLiDAR connected successfully.\n";
  return true;
}

bool SllidarDriver::StartScan() {
  if (!drv_) return false;
  
  // DenseBoost scan mode
  LidarScanMode scanMode;
  if (SL_IS_FAIL(drv_->startScanExpress(0, 0, 0, &scanMode))) {
    std::cerr << "SllidarDriver: Failed to start scan\n";
    return false;
  }
  
  drv_->setMotorSpeed();  // Default speed
  return true;
}

void SllidarDriver::StopScan() {
  if (drv_) {
    drv_->stop();
    drv_->setMotorSpeed(0);
  }
}

bool SllidarDriver::GetScan(LidarScan& scan) {
  if (!drv_) return false;

  sl_lidar_response_measurement_node_hq_t nodes[8192];
  size_t count = sizeof(nodes) / sizeof(nodes[0]);

  // Grab one complete full 360 scan
  sl_result ans = drv_->grabScanDataHq(nodes, count);
  if (SL_IS_FAIL(ans)) {
    return false;
  }

  // Sort by angle ascending
  drv_->ascendScanData(nodes, count);

  scan.stamp = Timestamp::Now();
  scan.angle_min = 0.0f;
  scan.angle_max = 2.0f * kPi;
  scan.angle_increment = (2.0f * kPi) / static_cast<float>(count);
  scan.range_min = 0.15f;
  scan.range_max = 12.0f;

  scan.ranges.resize(count);
  for (size_t i = 0; i < count; ++i) {
    float range_m = nodes[i].dist_mm_q2 / 4000.0f; 
    scan.ranges[i] = range_m;
  }

  return true;
}

}  // namespace pinky
