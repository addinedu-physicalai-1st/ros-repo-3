#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include "pinky_core/hal/interfaces.h"
#include "pinky_core/net/zmq_server.h"
#include "pinky_core/core/diff_drive.h"
#include "pinky_core/core/odometry.h"
#include "pinky_core/core/sensor_fusion.h"
#include "pinky_core/core/battery_monitor.h"
#include "pinky_core/core/lidar_processor.h"
#include "pinky_core/core/led_controller.h"
#include "pinky_core/core/emotion_renderer.h"
#ifdef PINKY_HAS_ONNXRUNTIME
#include "pinky_core/inference/onnx_actor.h"
#endif
#include "pinky_core/inference/observation_builder.h"
#include "pinky_core/inference/rl_controller.h"

namespace pinky {

struct RlConfig {
  std::string model_path{"../models/sac_actor.onnx"};
  std::string input_name{"state"};
  std::string output_name{"action"};

  float v_min{kVMin};
  float v_max{kVMax};
  float w_max{kWMax};

  float kp_v{kKpV};
  float kd_v{kKdV};
  float kp_w{kKpW};
  float kd_w{kKdW};

  float goal_dist_scale{kGoalDistScale};
  int max_steps{kMaxSteps};

  float goal_tolerance{kGoalTolerance};
  float lookahead_dist{kLookaheadDist};
  double control_period_ms{kControlPeriodMs};

  std::string emotion_dir{"emotion"};
};

struct RobotConfig {
  uint16_t rep_port{9100};
  uint16_t pub_port{9200};
  bool enable_hal{true};

  std::string lidar_device{"/dev/ttyUSB0"};
  uint32_t lidar_baudrate{460800};

  double wheel_radius{kWheelRadius};
  double wheel_base{kWheelBase};
  double max_rpm{kMaxRpm};
  std::string robot_id{"7"};

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
  void MotorOdomLoop();
  void ImuLoop();
  void AdcLoop();
  void LidarLoop();
  void CameraLoop();
  void LcdLoop();
  
  void OnCommand(const proto::ControlCommand& cmd, proto::CommandAck& ack);

  RobotConfig config_;
  std::atomic<bool> running_{false};

  std::unique_ptr<IMotorDriver> motor_;
  std::unique_ptr<ILidarDriver> lidar_;
  std::unique_ptr<IImuDriver> imu_;
  std::unique_ptr<IAdcDriver> adc_;
  std::unique_ptr<ILedDriver> led_;
  std::unique_ptr<ILcdDriver> lcd_;
  std::unique_ptr<ICameraDriver> camera_;

  std::unique_ptr<ZmqServer> zmq_server_;

  DiffDrive diff_drive_;
  OdometryAccumulator odom_calc_;
  SensorFusion sensor_fusion_;
  BatteryMonitor battery_monitor_{kBattVMin, kBattVMax, kBattLowThresh};
  LidarProcessor lidar_processor_{kLidarSectors, kMaxLidarDist};
#ifdef PINKY_HAS_ONNXRUNTIME
  std::unique_ptr<OnnxActor> onnx_actor_;
#endif
  ObservationBuilder obs_builder_{kGoalDistScale, kMaxSteps};
  RlController rl_controller_{kKpV, kKdV, kKpW, kKdW, kVMin, kVMax, kWMax};

  std::mutex state_mutex_;
  Odometry current_odom_;
  CmdVel target_cmd_vel_; 
  bool rl_navigation_active_{false};
  NavGoal current_goal_;
  int rl_step_count_{0};
  
  EmotionId current_emotion_{EmotionId::kNeutral};

  std::thread motor_thread_;
  std::thread imu_thread_;
  std::thread adc_thread_;
  std::thread lidar_thread_;
  std::thread camera_thread_;
  std::thread lcd_thread_;
};

}  // namespace pinky
