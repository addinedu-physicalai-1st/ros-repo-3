#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include "pinky_core/hal/interfaces.h"
#include "pinky_core/net/tcp_server.h"
#include "pinky_core/net/udp_server.h"
#include "pinky_core/net/connection_manager.h"
#include "pinky_core/net/frame_sender.h"
#include "pinky_core/protocol/serializer.h"
#include "pinky_core/core/diff_drive.h"
#include "pinky_core/core/odometry.h"
#include "pinky_core/core/battery_monitor.h"
#include "pinky_core/core/led_controller.h"
#include "pinky_core/core/lidar_processor.h"
#include "pinky_core/inference/onnx_actor.h"
#include "pinky_core/inference/observation_builder.h"
#include "pinky_core/inference/rl_controller.h"

namespace pinky {

struct RobotConfig {
  uint16_t tcp_port{9100};
  uint16_t udp_port{9200};
  bool enable_hal{true};  // False on PC for mock
  std::string onnx_model_path{"models/sac_actor.onnx"};
};

class RobotApp {
 public:
  explicit RobotApp(const RobotConfig& config);
  ~RobotApp();

  bool Init();
  void Run();
  void Stop();

 private:
  // Thread loops
  void MotorOdomLoop();
  void ImuLoop();
  void AdcLoop();
  void LidarLoop();
  
  // Handlers
  void OnTcpMessage(int fd, const ParsedMessage& msg);

  RobotConfig config_;
  std::atomic<bool> running_{false};

  // Hardware Drivers
  std::unique_ptr<IMotorDriver> motor_;
  std::unique_ptr<ILidarDriver> lidar_;
  std::unique_ptr<IImuDriver> imu_;
  std::unique_ptr<IAdcDriver> adc_;
  std::unique_ptr<ILedDriver> led_;
  std::unique_ptr<ILcdDriver> lcd_;

  // Network
  std::shared_ptr<TcpServer> tcp_;
  std::shared_ptr<UdpServer> udp_;
  std::shared_ptr<ConnectionManager> conn_mgr_;
  std::shared_ptr<Serializer> serializer_;
  std::unique_ptr<FrameSender> frame_sender_;

  // Core & Inference
  DiffDrive diff_drive_;
  OdometryCalculator odom_calc_;
  BatteryMonitor battery_monitor_;
  LidarProcessor lidar_processor_;
  std::unique_ptr<OnnxActor> onnx_actor_;
  ObservationBuilder obs_builder_;
  RlController rl_controller_;

  // State integration
  std::mutex state_mutex_;
  Odometry current_odom_;
  CmdVel target_cmd_vel_; 
  bool rl_navigation_active_{false};
  NavGoal current_goal_;
  int rl_step_count_{0};

  // Threads
  std::thread motor_thread_;
  std::thread imu_thread_;
  std::thread adc_thread_;
  std::thread lidar_thread_;
};

}  // namespace pinky
