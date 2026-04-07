#include "pinky_core/app/robot_app.h"

#include <iostream>
#include <algorithm>
#include <chrono>
#include <cmath>
#include "pinky_core/common/constants.h"
#include "pinky_core/core/emotion_renderer.h"

#ifdef BUILD_HAL
#include "pinky_core/hal/dynamixel_motor.h"
#include "pinky_core/hal/sllidar_driver.h"
#include "pinky_core/hal/bno055_imu.h"
#include "pinky_core/hal/adc_sensor.h"
#include "pinky_core/hal/ws2811_led.h"
#include "pinky_core/hal/ili9341_lcd.h"
#include "pinky_core/hal/rpicam_capture.h"
#endif

namespace pinky {

using namespace std::chrono_literals;

RobotApp::RobotApp(const RobotConfig& config)
    : config_(config),
      diff_drive_(config.wheel_radius, config.wheel_base, config.max_rpm),
      odom_calc_(config.wheel_radius, config.wheel_base),
      obs_builder_(config.rl.goal_dist_scale, config.rl.max_steps),
      rl_controller_(config.rl.kp_v, config.rl.kd_v,
                     config.rl.kp_w, config.rl.kd_w,
                     config.rl.v_min, config.rl.v_max, config.rl.w_max) {
  zmq_server_ = std::make_unique<ZmqServer>(config_.rep_port, config_.pub_port);
}

RobotApp::~RobotApp() {
  Stop();
}

bool RobotApp::Init() {
  if (config_.enable_hal) {
#ifdef BUILD_HAL
    motor_ = std::make_unique<DynamixelMotor>(DynamixelMotor::Config{});
    if (!motor_->Init()) {
      std::cerr << "Motor init failed (Check power)\n";
      motor_.reset();
    }

    lidar_ = std::make_unique<SllidarDriver>(SllidarDriver::Config{
    config_.lidar_device, 
    static_cast<int>(config_.lidar_baudrate) 
    });
    if (!lidar_->Init()) {
      std::cerr << "Lidar init failed (Check connection/power)\n";
      lidar_.reset();
    }

    imu_ = std::make_unique<Bno055Imu>(Bno055Imu::Config{});
    if (!imu_->Init()) {
      std::cerr << "IMU init failed\n";
      imu_.reset();
    }

    adc_ = std::make_unique<AdcSensor>(AdcSensor::Config{});
    if (!adc_->Init()) {
      std::cerr << "ADC init failed\n";
      adc_.reset();
    }

    led_ = std::make_unique<Ws2811Led>(Ws2811Led::Config{});
    if (!led_->Init()) {
      std::cerr << "LED init failed\n";
      led_.reset();
    }

    lcd_ = std::make_unique<Ili9341Lcd>(Ili9341Lcd::Config{});
    if (!lcd_->Init()) {
      std::cerr << "LCD init failed\n";
      lcd_.reset();
    }
#else
    std::cerr << "Cannot enable HAL: Project built without BUILD_HAL.\n";

    return false;
#endif
  } else {
    std::cout << "HAL disabled (Mock/PC mode).\n";
  }

  try {
    onnx_actor_ = std::make_unique<OnnxActor>(config_.rl.model_path);
  } catch (const std::exception& e) {
    std::cerr << "OnnxActor load failed: " << e.what() << "\n";
  }

#ifdef BUILD_HAL
  camera_ = std::make_unique<RpicamCapture>();
  if (!camera_->Init()) {
    std::cerr << "RpicamCapture init failed\n";
    camera_.reset();
  }
#endif

  // Hook network callbacks
  zmq_server_->SetCommandCallback([this](const proto::ControlCommand& cmd, proto::CommandAck& ack) {
    this->OnCommand(cmd, ack);
  });

  return true;
}

void RobotApp::Run() {
  std::cout << "Starting RobotApp...\n";
  running_.store(true);

  zmq_server_->Start();

  if (lidar_) {
    if (lidar_->StartScan()) {
      std::cout << "LiDAR scan started.\n";
    } else {
      std::cerr << "LiDAR scan start FAILED — P-control only.\n";
    }
  }

  motor_thread_ = std::thread(&RobotApp::MotorOdomLoop, this);
  imu_thread_ = std::thread(&RobotApp::ImuLoop, this);
  adc_thread_ = std::thread(&RobotApp::AdcLoop, this);
  lidar_thread_ = std::thread(&RobotApp::LidarLoop, this);
  nav_thread_ = std::thread(&RobotApp::NavLoop, this);
  if (camera_) camera_thread_ = std::thread(&RobotApp::CameraLoop, this);
  if (lcd_) lcd_thread_ = std::thread(&RobotApp::LcdLoop, this);

  while (running_.load()) {
    std::this_thread::sleep_for(1s);
  }
}

void RobotApp::Stop() {
  if (!running_.exchange(false)) return;

  if (zmq_server_) zmq_server_->Stop();

  if (lidar_) lidar_->StopScan();
  if (motor_) motor_->SetVelocityWait(0.0, 0.0);
  if (led_) led_->Clear();

  if (motor_thread_.joinable()) motor_thread_.join();
  if (imu_thread_.joinable()) imu_thread_.join();
  if (adc_thread_.joinable()) adc_thread_.join();
  if (lidar_thread_.joinable()) lidar_thread_.join();
  if (nav_thread_.joinable()) nav_thread_.join();
  if (camera_thread_.joinable()) camera_thread_.join();
  if (lcd_thread_.joinable()) lcd_thread_.join();
}

void RobotApp::OnCommand(const proto::ControlCommand& cmd, proto::CommandAck& ack) {
  ack.set_robot_id(cmd.robot_id());
  ack.set_request_id(cmd.request_id());
  ack.set_success(true);

  if (!cmd.robot_id().empty() && cmd.robot_id() != config_.robot_id) {
    ack.set_success(false);
    ack.set_message("Robot ID mismatch: expected " + config_.robot_id + ", got " + cmd.robot_id());
    return;
  }

  if (cmd.has_cmd_vel()) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    target_cmd_vel_.linear_x = cmd.cmd_vel().linear_x();
    target_cmd_vel_.angular_z = cmd.cmd_vel().angular_z();
    rl_navigation_active_ = false; // Override RL on manual override
    ack.set_message("CmdVel applied");
  } else if (cmd.has_nav_goal()) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_goal_.x = cmd.nav_goal().x();
    current_goal_.y = cmd.nav_goal().y();
    current_goal_.theta = cmd.nav_goal().theta();
    rl_navigation_active_ = true;
    rl_step_count_ = 0;
    smoothed_rl_action_ = {0.0f, 0.0f};
    std::cout << "[NAV] Goal received: (" << current_goal_.x
              << ", " << current_goal_.y << ", " << current_goal_.theta << ")\n";
    ack.set_message("NavGoal started");
  } else if (cmd.has_nav_cancel()) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    rl_navigation_active_ = false;
    target_cmd_vel_ = {0.0f, 0.0f};
    std::cout << "[NAV] Navigation canceled\n";
    ack.set_message("Nav canceled");
  } else if (cmd.has_set_pose()) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    double px = cmd.set_pose().x();
    double py = cmd.set_pose().y();
    double pt = cmd.set_pose().theta();
    odom_calc_.Reset(px, py, pt);
    sensor_fusion_.Reset(px, py, pt);
    current_odom_.x = px;
    current_odom_.y = py;
    current_odom_.theta = pt;
    current_odom_.vx = 0.0;
    current_odom_.vth = 0.0;
    ack.set_message("Pose reset");
  } else {
    ack.set_success(false);
    ack.set_message("Unknown or unsupported command");
  }
}

void RobotApp::LcdLoop() {
  EmotionId loaded_emotion = static_cast<EmotionId>(-1);
  AnimatedEmotion anim;
  int current_frame = 0;

  static const char* kEmotionFiles[] = {
    "basic.gif", "happy.gif", "sad.gif",
    "angry.gif", "basic.gif", "bored.gif",
  };

  while (running_.load()) {
    auto start_time = std::chrono::steady_clock::now();
    EmotionId target_emotion;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      target_emotion = current_emotion_;
    }

    if (target_emotion != loaded_emotion) {
      loaded_emotion = target_emotion;
      int lcd_w = lcd_->Width();
      int lcd_h = lcd_->Height();
      uint8_t eid = static_cast<uint8_t>(target_emotion);
      std::string gif_path = config_.rl.emotion_dir + "/" + kEmotionFiles[eid < 6 ? eid : 0];
      
      anim = LoadAnimatedEmotion(gif_path, lcd_w, lcd_h);
      if (anim.frames.empty()) {
        std::cerr << "LCD: Failed to load " << gif_path << " — using shape fallback\n";
        GifFrame fallback;
        fallback.pixels = RenderEmotion(target_emotion, lcd_w, lcd_h);
        fallback.delay_ms = 100;
        anim.frames.push_back(std::move(fallback));
      }
      current_frame = 0;
    }

    int delay_ms = 100;
    if (!anim.frames.empty() && lcd_) {
      const auto& frame = anim.frames[current_frame];
      lcd_->DrawFrameRgb565(frame.pixels.data(), frame.pixels.size());
      delay_ms = frame.delay_ms;
      current_frame = (current_frame + 1) % anim.frames.size();
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time;
    auto target_duration = std::chrono::milliseconds(delay_ms);
    if (elapsed < target_duration) {
      std::this_thread::sleep_for(target_duration - elapsed);
    }
  }
}

void RobotApp::MotorOdomLoop() {
  const auto period = std::chrono::milliseconds(20); // 50Hz
  
  while(running_.load()) {
    auto start_time = std::chrono::steady_clock::now();

    if (motor_) {
      JointState js = motor_->ReadJointState();
      
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        // 1. Get raw wheel odometry
        Odometry raw_odom = odom_calc_.Update(
            js.position[0], js.position[1], js.stamp);
        
        // 2. Fuse with IMU data
        sensor_fusion_.Predict(raw_odom.vx, raw_odom.vth, js.stamp);
        current_odom_ = sensor_fusion_.GetState(js.stamp);
        
        // Broadcast fused Odom via ZmqServer
        proto::SensorTelemetry t;
        t.set_robot_id(config_.robot_id);
        auto* proto_odom = t.mutable_odom();
        proto_odom->mutable_stamp()->set_nanoseconds(js.stamp.nanoseconds);
        proto_odom->set_x(current_odom_.x);
        proto_odom->set_y(current_odom_.y);
        proto_odom->set_theta(current_odom_.theta);
        proto_odom->set_vx(current_odom_.vx);
        proto_odom->set_vth(current_odom_.vth);
        zmq_server_->PublishTelemetry(t);

        // Apply commands to motor
        auto [rpm_l, rpm_r] = diff_drive_.VelocityToRpm(
            target_cmd_vel_.linear_x, target_cmd_vel_.angular_z);
        motor_->SetVelocityWait(rpm_l, rpm_r);
      }
    } else {
      // Mock mode: broadcast current_odom_ continuously
      std::lock_guard<std::mutex> lock(state_mutex_);
      proto::SensorTelemetry t;
      t.set_robot_id(config_.robot_id);
      auto* proto_odom = t.mutable_odom();
      proto_odom->mutable_stamp()->set_nanoseconds(0); // Mock timestamp
      proto_odom->set_x(current_odom_.x);
      proto_odom->set_y(current_odom_.y);
      proto_odom->set_theta(current_odom_.theta);
      proto_odom->set_vx(current_odom_.vx);
      proto_odom->set_vth(current_odom_.vth);
      zmq_server_->PublishTelemetry(t);
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed < period) std::this_thread::sleep_for(period - elapsed);
  }
}

void RobotApp::ImuLoop() {
  const auto period = std::chrono::milliseconds(10); // 100Hz
  while(running_.load()) {
    auto start_time = std::chrono::steady_clock::now();
    
    if (imu_) {
      ImuData data;
      if (imu_->ReadData(data)) {
        // Feed IMU yaw rate into sensor fusion (angular_velocity.z = yaw rate)
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          sensor_fusion_.UpdateImu(data.angular_velocity.z, data.stamp);
        }

        proto::SensorTelemetry t;
        t.set_robot_id(config_.robot_id);
        auto* proto_imu = t.mutable_imu();
        proto_imu->mutable_stamp()->set_nanoseconds(data.stamp.nanoseconds);
        proto_imu->mutable_orientation()->set_w(data.orientation.w);
        proto_imu->mutable_orientation()->set_x(data.orientation.x);
        proto_imu->mutable_orientation()->set_y(data.orientation.y);
        proto_imu->mutable_orientation()->set_z(data.orientation.z);
        proto_imu->mutable_angular_velocity()->set_x(data.angular_velocity.x);
        proto_imu->mutable_angular_velocity()->set_y(data.angular_velocity.y);
        proto_imu->mutable_angular_velocity()->set_z(data.angular_velocity.z);
        proto_imu->mutable_linear_acceleration()->set_x(data.linear_acceleration.x);
        proto_imu->mutable_linear_acceleration()->set_y(data.linear_acceleration.y);
        proto_imu->mutable_linear_acceleration()->set_z(data.linear_acceleration.z);
        zmq_server_->PublishTelemetry(t);
      }
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed < period) std::this_thread::sleep_for(period - elapsed);
  }
}

void RobotApp::AdcLoop() {
  const auto period = std::chrono::milliseconds(50); // 20Hz
  while(running_.load()) {
    auto start_time = std::chrono::steady_clock::now();

    if (adc_) {
      uint16_t c0, c1, c2, c3, c4;
      if (adc_->ReadAll(c0, c1, c2, c3, c4)) {
        BatteryState batt = battery_monitor_.Update(c4);
        
        proto::SensorTelemetry t;
        t.set_robot_id(config_.robot_id);
        auto* proto_batt = t.mutable_battery();
        proto_batt->mutable_stamp()->set_nanoseconds(batt.stamp.nanoseconds);
        proto_batt->set_voltage(batt.voltage);
        proto_batt->set_percentage(batt.percentage);
        proto_batt->set_status(batt.status);
        zmq_server_->PublishTelemetry(t);
        
        // Can also publish IR/US sensors here...
      }
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed < period) std::this_thread::sleep_for(period - elapsed);
  }
}

void RobotApp::LidarLoop() {
  int scan_count = 0;
  int fail_count = 0;
  while (running_.load()) {
    if (lidar_) {
      LidarScan scan;
      if (lidar_->GetScan(scan)) {
        if (scan_count == 0) {
          std::cout << "[LIDAR] First scan received (" << scan.ranges.size()
                    << " points, after " << fail_count << " failures)\n";
        }
        fail_count = 0;
        ++scan_count;

        LidarSectors sectors = lidar_processor_.Process(scan);

        // Store latest sectors for NavLoop (RL inference)
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          latest_sectors_ = sectors;
          has_lidar_sectors_ = true;
        }

        // Stream sectors (compressed) to PC
        proto::SensorTelemetry t;
        t.set_robot_id(config_.robot_id);
        auto* proto_sectors = t.mutable_lidar_sectors();
        proto_sectors->mutable_stamp()->set_nanoseconds(sectors.stamp.nanoseconds);
        for (float s : sectors.sectors) {
          proto_sectors->add_sectors(s);
        }
        zmq_server_->PublishTelemetry(t);

        // Stream full scan (for map building) to PC
        proto::SensorTelemetry scan_t;
        scan_t.set_robot_id(config_.robot_id);
        auto* proto_scan = scan_t.mutable_lidar_scan();
        proto_scan->mutable_stamp()->set_nanoseconds(scan.stamp.nanoseconds);
        proto_scan->set_angle_min(scan.angle_min);
        proto_scan->set_angle_max(scan.angle_max);
        proto_scan->set_angle_increment(scan.angle_increment);
        proto_scan->set_range_min(scan.range_min);
        proto_scan->set_range_max(scan.range_max);
        for (float r : scan.ranges) {
          proto_scan->add_ranges(r);
        }
        zmq_server_->PublishTelemetry(scan_t);
      } else {
        ++fail_count;
        if (fail_count == 1 || fail_count == 10 || fail_count == 100 ||
            (fail_count % 500 == 0)) {
          std::cerr << "[LIDAR] GetScan failed (count=" << fail_count << ")\n";
        }
        std::this_thread::sleep_for(50ms);
      }
    } else {
      std::this_thread::sleep_for(100ms);
    }
  }
}

void RobotApp::NavLoop() {
  const auto period = std::chrono::milliseconds(
      static_cast<int>(config_.rl.control_period_ms));  // 50ms = 20Hz

  // Mode tracking for transition logs (persists across loop iterations)
  enum class NavMode { kIdle, kTurnFirst, kRl, kPCtrl };
  NavMode prev_mode = NavMode::kIdle;
  constexpr float kRadToDeg = 180.0f / static_cast<float>(M_PI);

  while (running_.load()) {
    auto start_time = std::chrono::steady_clock::now();

    NavGoal goal;
    Odometry odom;
    int step = 0;
    bool has_sectors = false;
    LidarSectors sectors;
    bool is_active = false;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      is_active = rl_navigation_active_;
      if (is_active) {
        goal = current_goal_;
        odom = current_odom_;
        step = rl_step_count_++;
        has_sectors = has_lidar_sectors_;
        if (has_sectors) {
          sectors = latest_sectors_;
        }
      }
    }

    if (is_active) {
      if (step == 0) {
        std::cout << "[NAV] Navigation active"
                  << " goal=(" << goal.x << "," << goal.y << ")"
                  << " has_lidar=" << has_sectors
                  << " has_onnx=" << (onnx_actor_ != nullptr) << "\n";
        prev_mode = NavMode::kIdle;
      }

      float dx = static_cast<float>(goal.x - odom.x);
      float dy = static_cast<float>(goal.y - odom.y);
      float dist = std::sqrt(dx * dx + dy * dy);

      // Pre-compute angle_diff (needed for turn-first check and P-control)
      float goal_angle_world = std::atan2(dy, dx);
      float angle_diff = goal_angle_world - static_cast<float>(odom.theta);
      while (angle_diff > static_cast<float>(M_PI))
        angle_diff -= 2.0f * static_cast<float>(M_PI);
      while (angle_diff < -static_cast<float>(M_PI))
        angle_diff += 2.0f * static_cast<float>(M_PI);

      // Threshold: goal is more than 100° behind → rotate in place first
      // This overrides the RL model which may blindly go forward.
      constexpr float kTurnFirstThresholdRad = 1.745f;  // 100°
      constexpr float kPCtrlVMax = 0.12f;
      constexpr float kPCtrlWMax = 0.8f;

      if (dist < 0.30f) {
        // ── Goal reached ──────────────────────────────────────────────
        std::cout << "[NAV] Goal reached! dist=" << dist << "\n";
        std::lock_guard<std::mutex> lock(state_mutex_);
        rl_navigation_active_ = false;
        target_cmd_vel_ = {0.0f, 0.0f};
        prev_mode = NavMode::kIdle;

      } else if (std::abs(angle_diff) > kTurnFirstThresholdRad) {
        // ── Turn-first mode ───────────────────────────────────────────
        // Goal is > 100° behind: rotate in place before driving.
        NavMode cur_mode = NavMode::kTurnFirst;
        if (cur_mode != prev_mode) {
          std::cout << "[NAV] >> TURN-FIRST"
                    << " angle=" << angle_diff * kRadToDeg << "deg"
                    << " dist=" << dist << "m\n";
          prev_mode = cur_mode;
        }
        CmdVel cmd{0.0f, std::copysign(kWMax * 0.8f, angle_diff)};
        if (step % 20 == 0) {
          std::cout << "[NAV] Turn-first: step=" << step
                    << " angle=" << angle_diff * kRadToDeg << "deg"
                    << " cmd=(0," << cmd.angular_z << ")\n";
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        target_cmd_vel_ = cmd;

      } else if (onnx_actor_ && has_sectors) {
        // ── RL controller ─────────────────────────────────────────────
        NavMode cur_mode = NavMode::kRl;
        if (cur_mode != prev_mode) {
          std::cout << "[NAV] >> RL"
                    << " dist=" << dist << "m"
                    << " angle=" << angle_diff * kRadToDeg << "deg\n";
          prev_mode = cur_mode;
        }

        obs_builder_.SetGoal(goal.x, goal.y);
        std::array<float, 28> obs = obs_builder_.Build(sectors, odom, step);

        // Log full 28D observation vector at nav start and every 100 steps.
        // Format: lidar[0..23] | dist_norm | cos_angle | sin_angle | step_progress
        if (step == 0 || step % 100 == 0) {
          std::cout << "[OBS] step=" << step << " lidar=[";
          for (int i = 0; i < 24; ++i) {
            std::cout << obs[i];
            if (i < 23) std::cout << ",";
          }
          std::cout << "]"
                    << " dist_n=" << obs[24]
                    << " cos=" << obs[25]
                    << " sin=" << obs[26]
                    << " prog=" << obs[27] << "\n";
        }

        std::array<float, 2> raw_action = onnx_actor_->Infer(obs);

        // EMA smoothing: reduces chattering without adding latency at low freq.
        // alpha=0.4 → 40% new value, 60% carry-over at 20 Hz.
        constexpr float kEmaAlpha = 0.4f;
        smoothed_rl_action_[0] =
            kEmaAlpha * raw_action[0] + (1.0f - kEmaAlpha) * smoothed_rl_action_[0];
        smoothed_rl_action_[1] =
            kEmaAlpha * raw_action[1] + (1.0f - kEmaAlpha) * smoothed_rl_action_[1];

        // Angular velocity deadzone: suppress micro-corrections when driving straight.
        constexpr float kAngularDeadzone = 0.05f;  // normalised action units
        if (std::abs(smoothed_rl_action_[1]) < kAngularDeadzone) {
          smoothed_rl_action_[1] = 0.0f;
        }

        // Capture pre-PD target for log comparison.
        CmdVel pd_target = rl_controller_.ActionToTarget(smoothed_rl_action_);

        CmdVel cmd = rl_controller_.Compute(
            smoothed_rl_action_,
            static_cast<float>(odom.vx),
            static_cast<float>(odom.vth));

        // Safety layer: limit forward speed when front sectors are close.
        // Sectors[10..14] cover the front ±30° arc (sector[12] = direct front).
        float front_min = 1.0f;
        for (int s = 10; s <= 14; ++s) {
          front_min = std::min(front_min, sectors.sectors[s]);
        }
        float front_dist = front_min * kMaxLidarDist;  // metres
        if (cmd.linear_x > 0.0f) {
          if (front_dist < 0.20f) {
            cmd.linear_x = 0.0f;  // emergency stop
          } else if (front_dist < 0.40f) {
            float scale = (front_dist - 0.20f) / 0.20f;
            cmd.linear_x *= scale;
          }
        }

        if (step % 20 == 0) {
          std::cout << "[RL] step=" << step
                    << " dist=" << dist << "m"
                    << " front=" << front_dist << "m"
                    << " raw=[" << raw_action[0] << "," << raw_action[1] << "]"
                    << " smooth=[" << smoothed_rl_action_[0] << ","
                    << smoothed_rl_action_[1] << "]"
                    << " target=(" << pd_target.linear_x << ","
                    << pd_target.angular_z << ")"
                    << " cmd=(" << cmd.linear_x << "," << cmd.angular_z << ")"
                    << " pd_d=(" << (cmd.linear_x - pd_target.linear_x) << ","
                    << (cmd.angular_z - pd_target.angular_z) << ")\n";
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        target_cmd_vel_ = cmd;

      } else {
        // ── P-control fallback ────────────────────────────────────────
        // Works without LiDAR — reduced speed, no obstacle avoidance.
        NavMode cur_mode = NavMode::kPCtrl;
        if (cur_mode != prev_mode) {
          std::cout << "[NAV] >> P-CTRL fallback"
                    << " no_lidar=" << !has_sectors
                    << " no_onnx=" << (onnx_actor_ == nullptr) << "\n";
          prev_mode = cur_mode;
        }
        float abs_angle = std::abs(angle_diff);
        float turn_scale = std::max(0.0f, 1.0f - abs_angle / (0.5f * static_cast<float>(M_PI)));
        float speed = std::min(0.25f * dist, kPCtrlVMax) * turn_scale;
        CmdVel cmd{
          std::max(speed, 0.0f),
          std::clamp(1.5f * angle_diff, -kPCtrlWMax, kPCtrlWMax)
        };

        if (step % 20 == 0) {
          std::cout << "[P-CTRL] step=" << step
                    << " dist=" << dist << "m"
                    << " angle=" << angle_diff * kRadToDeg << "deg"
                    << " cmd=(" << cmd.linear_x << "," << cmd.angular_z << ")\n";
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        target_cmd_vel_ = cmd;
      }
    } else {
      prev_mode = NavMode::kIdle;
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed < period) std::this_thread::sleep_for(period - elapsed);
  }
}

void RobotApp::CameraLoop() {
  const auto period = std::chrono::milliseconds(100);  // 10 fps
  while (running_.load()) {
    auto start_time = std::chrono::steady_clock::now();

    if (camera_) {
      std::vector<uint8_t> jpeg;
      uint16_t width = 0;
      uint16_t height = 0;
      if (camera_->CaptureJpeg(jpeg, width, height)) {
        proto::VideoStream v;
        v.set_robot_id(config_.robot_id);
        auto* frame = v.mutable_frame();
        frame->set_width(width);
        frame->set_height(height);
        frame->set_jpeg_data(jpeg.data(), jpeg.size());
        
        zmq_server_->PublishVideo(v);
      }
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed < period) std::this_thread::sleep_for(period - elapsed);
  }
}

}  // namespace pinky
// Force recompile to fix ABI mismatch from ws2811_led.h
