#include "pinky_core/app/robot_app.h"

#include <iostream>
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
  tcp_ = std::make_shared<TcpServer>(config_.tcp_port);
  udp_ = std::make_shared<UdpServer>(config_.udp_port);
  conn_mgr_ = std::make_shared<ConnectionManager>(tcp_, udp_);
  serializer_ = std::make_shared<Serializer>();
  frame_sender_ = std::make_unique<FrameSender>(tcp_, serializer_);
}

RobotApp::~RobotApp() {
  Stop();
}

bool RobotApp::Init() {
  if (config_.enable_hal) {
#ifdef BUILD_HAL
    motor_ = std::make_unique<DynamixelMotor>(DynamixelMotor::Config{});
    if (!motor_->Init()) std::cerr << "Motor init failed (Check power)\n";

    lidar_ = std::make_unique<SllidarDriver>(SllidarDriver::Config{});
    if (!lidar_->Init()) std::cerr << "Lidar init failed\n";

    imu_ = std::make_unique<Bno055Imu>(Bno055Imu::Config{});
    if (!imu_->Init()) std::cerr << "IMU init failed\n";

    adc_ = std::make_unique<AdcSensor>(AdcSensor::Config{});
    if (!adc_->Init()) std::cerr << "ADC init failed\n";

    led_ = std::make_unique<Ws2811Led>(Ws2811Led::Config{});
    if (!led_->Init()) std::cerr << "LED init failed\n";

    lcd_ = std::make_unique<Ili9341Lcd>(Ili9341Lcd::Config{});
    if (!lcd_->Init()) std::cerr << "LCD init failed\n";
#else
    std::cerr << "Cannot enable HAL: Project built without BUILD_HAL.\n";
    return false;
#endif
  } else {
    std::cout << "HAL disabled (Mock/PC mode).\n";
  }

#ifdef PINKY_HAS_ONNXRUNTIME
  try {
    onnx_actor_ = std::make_unique<OnnxActor>(config_.rl.model_path);
  } catch (const std::exception& e) {
    std::cerr << "OnnxActor load failed: " << e.what() << "\n";
  }
#endif

  // Hook network callbacks
  tcp_->SetMessageCallback([this](int fd, const ParsedMessage& msg) {
    this->OnTcpMessage(fd, msg);
  });
  tcp_->SetConnectionCallback([this](int fd, bool connected, const std::string& ip) {
    if (connected) {
      conn_mgr_->OnClientConnected(fd, ip);
    } else {
      conn_mgr_->OnClientDisconnected(fd);
    }
  });

  return true;
}

void RobotApp::Run() {
  std::cout << "Starting RobotApp...\n";
  running_.store(true);

  tcp_->Start();
  udp_->Start();
  conn_mgr_->Start();

  if (lidar_) lidar_->StartScan();

  motor_thread_ = std::thread(&RobotApp::MotorOdomLoop, this);
  imu_thread_ = std::thread(&RobotApp::ImuLoop, this);
  adc_thread_ = std::thread(&RobotApp::AdcLoop, this);
  lidar_thread_ = std::thread(&RobotApp::LidarLoop, this);
  if (camera_) camera_thread_ = std::thread(&RobotApp::CameraLoop, this);

  while (running_.load()) {
    std::this_thread::sleep_for(1s);
  }
}

void RobotApp::Stop() {
  if (!running_.exchange(false)) return;

  if (tcp_) tcp_->Stop();
  if (udp_) udp_->Stop();
  if (conn_mgr_) conn_mgr_->Stop();

  if (lidar_) lidar_->StopScan();
  if (motor_) motor_->SetVelocityWait(0.0, 0.0);
  if (led_) led_->Clear();

  if (motor_thread_.joinable()) motor_thread_.join();
  if (imu_thread_.joinable()) imu_thread_.join();
  if (adc_thread_.joinable()) adc_thread_.join();
  if (lidar_thread_.joinable()) lidar_thread_.join();
  if (camera_thread_.joinable()) camera_thread_.join();
}

void RobotApp::OnTcpMessage(int fd, const ParsedMessage& msg) {
  if (msg.msg_type == MsgType::kPing) {
    uint64_t ts = DeserializePing(msg.payload);
    conn_mgr_->OnPingReceived(fd, ts);
    return;
  }

  if (msg.msg_type == MsgType::kCmdVel) {
    CmdVel cmd = DeserializeCmdVel(msg.payload);
    std::lock_guard<std::mutex> lock(state_mutex_);
    target_cmd_vel_ = cmd;
    rl_navigation_active_ = false; // Override RL on manual override
    return;
  }

  if (msg.msg_type == MsgType::kNavGoal) {
    NavGoal goal = DeserializeNavGoal(msg.payload);
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_goal_ = goal;
    rl_navigation_active_ = true;
    rl_step_count_ = 0;
    return;
  }

  if (msg.msg_type == MsgType::kNavCancel) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    rl_navigation_active_ = false;
    target_cmd_vel_ = {0.0f, 0.0f};
    return;
  }

  if (msg.msg_type == MsgType::kSetEmotion) {
    uint8_t eid = DeserializeEmotion(msg.payload);
    if (lcd_) {
      auto fb = RenderEmotion(static_cast<EmotionId>(eid));
      lcd_->DrawFrame(fb.data(), fb.size());
    }
    return;
  }

  if (msg.msg_type == MsgType::kSetPose) {
    Pose2D pose = DeserializeSetPose(msg.payload);
    std::lock_guard<std::mutex> lock(state_mutex_);
    odom_calc_.Reset(static_cast<double>(pose.x),
                     static_cast<double>(pose.y),
                     static_cast<double>(pose.theta));
    return;
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
        // Update odometry
        current_odom_ = odom_calc_.Update(
            js.position[0], js.position[1], js.stamp);
        
        // Broadcast Odom via UDP
        std::vector<uint8_t> odom_payload = serializer_->SerializeOdom(current_odom_);
        std::vector<uint8_t> udp_pkt = serializer_->Frame(MsgType::kOdom, odom_payload);
        udp_->Send(udp_pkt);

        // Apply commands to motor
        // Both manual and RL paths write to target_cmd_vel_;
        // RL path applies PD control in LidarLoop at inference rate.
        auto [rpm_l, rpm_r] = diff_drive_.VelocityToRpm(
            target_cmd_vel_.linear_x, target_cmd_vel_.angular_z);
        motor_->SetVelocityWait(rpm_l, rpm_r);
      }
    } else {
      // Mock mode: broadcast current_odom_ continuously
      std::lock_guard<std::mutex> lock(state_mutex_);
      std::vector<uint8_t> odom_payload = serializer_->SerializeOdom(current_odom_);
      std::vector<uint8_t> udp_pkt = serializer_->Frame(MsgType::kOdom, odom_payload);
      udp_->Send(udp_pkt);
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
        std::vector<uint8_t> payload = serializer_->SerializeImu(data);
        std::vector<uint8_t> udp_pkt = serializer_->Frame(MsgType::kImu, payload);
        udp_->Send(udp_pkt);
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
        std::vector<uint8_t> b_payload = serializer_->SerializeBattery(batt);
        std::vector<uint8_t> udp_pkt = serializer_->Frame(MsgType::kBattery, b_payload);
        udp_->Send(udp_pkt);
        
        // Can also publish IR/US sensors here...
      }
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed < period) std::this_thread::sleep_for(period - elapsed);
  }
}

void RobotApp::LidarLoop() {
  while(running_.load()) {
    if (lidar_) {
      LidarScan scan;
      if (lidar_->GetScan(scan)) {
        LidarSectors sectors = lidar_processor_.Process(scan);
        
        // Stream back to PC
        std::vector<uint8_t> sec_payload = serializer_->SerializeLidar24(sectors);
        std::vector<uint8_t> udp_pkt = serializer_->Frame(MsgType::kLidar24, sec_payload);
        udp_->Send(udp_pkt);

        // RL Inference if active
        bool is_active;
        NavGoal goal;
        Odometry odom;
        int step;
        
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          is_active = rl_navigation_active_;
          goal = current_goal_;
          odom = current_odom_;
          step = rl_step_count_++;
        }

#ifdef PINKY_HAS_ONNXRUNTIME
        if (is_active && onnx_actor_) {
          obs_builder_.SetGoal(goal.x, goal.y);
          std::array<float, 28> obs = obs_builder_.Build(sectors, odom, step);
          std::array<float, 2> action = onnx_actor_->Infer(obs);
          CmdVel cmd = rl_controller_.Compute(
              action,
              static_cast<float>(odom.vx),
              static_cast<float>(odom.vth));

          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            target_cmd_vel_ = cmd;
          }
        }
#endif
      } else {
        std::this_thread::sleep_for(50ms);
      }
    } else {
      std::this_thread::sleep_for(100ms);
    }
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
        CameraFrame frame;
        frame.width = width;
        frame.height = height;
        frame.jpeg_data = std::move(jpeg);

        std::vector<uint8_t> payload = serializer_->SerializeCameraFrame(frame);
        std::vector<uint8_t> tcp_pkt = serializer_->Frame(MsgType::kCameraFrame, payload);
        tcp_->Broadcast(tcp_pkt);
      }
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed < period) std::this_thread::sleep_for(period - elapsed);
  }
}

}  // namespace pinky
