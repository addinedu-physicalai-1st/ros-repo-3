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
    delete drv_;
    drv_ = nullptr;
    return false;
  }

  sl_lidar_response_device_info_t devinfo;
  sl_result op_result = drv_->getDeviceInfo(devinfo);
  if (SL_IS_FAIL(op_result)) {
    std::cerr << "SllidarDriver: Failed to get device info, code: 0x"
              << std::hex << op_result << std::dec
              << " (check USB/UART connection and power)\n";
    delete drv_;
    drv_ = nullptr;
    return false;
  }

  std::cout << "SLLiDAR connected successfully."
            << " Model: " << static_cast<int>(devinfo.model)
            << " FW: " << (devinfo.firmware_version >> 8)
            << "." << (devinfo.firmware_version & 0xFF)
            << " HW: " << static_cast<int>(devinfo.hardware_version) << "\n";

  // Health check (matching ROS2 sllidar_node)
  sl_lidar_response_device_health_t healthinfo;
  op_result = drv_->getHealth(healthinfo);
  if (SL_IS_OK(op_result)) {
    std::cout << "SLLiDAR health status: " << static_cast<int>(healthinfo.status);
    if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
      std::cerr << " — ERROR state (code: " << healthinfo.error_code
                << "). Rebooting LiDAR...\n";
      drv_->reset();
      return false;
    }
    std::cout << " (OK)\n";
  } else {
    std::cerr << "SLLiDAR: getHealth failed: 0x"
              << std::hex << op_result << std::dec << "\n";
  }

  return true;
}

bool SllidarDriver::StartScan() {
  if (!drv_) return false;

  // 1. Start motor FIRST (matching ROS2 sllidar_node order)
  sl_result motor_result = drv_->setMotorSpeed();
  if (SL_IS_FAIL(motor_result)) {
    std::cerr << "SllidarDriver: setMotorSpeed failed: 0x"
              << std::hex << motor_result << std::dec << "\n";
    return false;
  }

  // 2. Then start scan (typical mode = express/DenseBoost auto-select)
  LidarScanMode scanMode;
  sl_result scan_result = drv_->startScan(false, true, 0, &scanMode);
  if (SL_IS_FAIL(scan_result)) {
    std::cerr << "SllidarDriver: startScan failed: 0x"
              << std::hex << scan_result << std::dec << "\n";
    drv_->setMotorSpeed(0);
    return false;
  }

  std::cout << "SLLiDAR scan mode: " << scanMode.scan_mode
            << " (sample_duration=" << scanMode.us_per_sample
            << "us, max_dist=" << scanMode.max_distance << "m)\n";
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

  // Grab one complete full 360 scan (5s timeout for motor spin-up)
  sl_result ans = drv_->grabScanDataHq(nodes, count, 5000);
  if (SL_IS_FAIL(ans)) {
    return false;
  }

  // Sort by angle ascending (SLLiDAR native: 0→360° clockwise)
  drv_->ascendScanData(nodes, count);

  scan.stamp = Timestamp::Now();
  scan.angle_min = 0.0f;
  scan.angle_max = 2.0f * kPi;
  scan.angle_increment = (2.0f * kPi) / static_cast<float>(count);
  scan.range_min = 0.15f;
  scan.range_max = 12.0f;

  // Convert CW → CCW (matching ROS2 LaserScan convention used in training).
  // SLLiDAR: index 0=front, increasing index=clockwise
  // ROS/training: index 0=front, increasing index=counter-clockwise
  // Mapping: CCW index i ← CW index (count - i) % count
  scan.ranges.resize(count);
  for (size_t i = 0; i < count; ++i) {
    size_t src_idx = (count - i) % count;
    float range_m = nodes[src_idx].dist_mm_q2 / 4000.0f;
    scan.ranges[i] = (range_m == 0.0f) ? 0.0f : range_m;
  }

  return true;
}

}  // namespace pinky
