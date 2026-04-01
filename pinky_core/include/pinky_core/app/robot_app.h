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
#ifdef PINKY_HAS_ONNXRUNTIME
#include "pinky_core/inference/onnx_actor.h"
#endif
#include "pinky_core/inference/observation_builder.h"
#include "pinky_core/inference/rl_controller.h"

namespace pinky {

// RL inference configuration (defaults from constants.h)
struct RlConfig {
  std::string model_path{"models/sac_actor.onnx"};
  std::string input_name{"state"};
  std::string output_name{"action"};

  // Action mapping
  float v_min{kVMin};
  float v_max{kVMax};
  float w_max{kWMax};

  // PD control
  float kp_v{kKpV};
  float kd_v{kKdV};
  float kp_w{kKpW};
  float kd_w{kKdW};

  // Observation
  float goal_dist_scale{kGoalDistScale};
  int max_steps{kMaxSteps};

  // Navigation
  float goal_tolerance{kGoalTolerance};
  float lookahead_dist{kLookaheadDist};
  double control_period_ms{kControlPeriodMs};
};

struct RobotConfig {
  uint16_t tcp_port{9100};
  uint16_t udp_port{9200};
  bool enable_hal{true};  // False on PC for mock

  // Robot physics (overridable via YAML, defaults from constants.h)
  double wheel_radius{kWheelRadius};
  double wheel_base{kWheelBase};
  double max_rpm{kMaxRpm};

  // RL inference config
  RlConfig rl;
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
  void CameraLoop();
  
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
  std::unique_ptr<ICameraDriver> camera_;

  // Network
  std::shared_ptr<TcpServer> tcp_;
  std::shared_ptr<UdpServer> udp_;
  std::shared_ptr<ConnectionManager> conn_mgr_;
  std::shared_ptr<Serializer> serializer_;
  std::unique_ptr<FrameSender> frame_sender_;

  // Core & Inference (initialized from config_ in constructor body)
  DiffDrive diff_drive_;
  OdometryAccumulator odom_calc_;
  BatteryMonitor battery_monitor_{kBattVMin, kBattVMax, kBattLowThresh};
  LidarProcessor lidar_processor_{kLidarSectors, kMaxLidarDist};
#ifdef PINKY_HAS_ONNXRUNTIME
  std::unique_ptr<OnnxActor> onnx_actor_;
#endif
  ObservationBuilder obs_builder_{kGoalDistScale, kMaxSteps};
  RlController rl_controller_{kKpV, kKdV, kKpW, kKdW, kVMin, kVMax, kWMax};

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
  std::thread camera_thread_;
};

}  // namespace pinky
