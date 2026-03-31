#include "pinky_core/app/robot_app.h"

#include <iostream>
#include <chrono>
#include <cmath>
#include "pinky_core/common/constants.h"

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

RobotApp::RobotApp(const RobotConfig& config) : config_(config) {
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

  try {
    onnx_actor_ = std::make_unique<OnnxActor>(config_.onnx_model_path);
  } catch (const std::exception& e) {
    std::cerr << "OnnxActor load failed: " << e.what() << "\n";
  }

  // Hook network callbacks
  tcp_->SetMessageCallback([this](int fd, const ParsedMessage& msg) {
    this->OnTcpMessage(fd, msg);
  });
  tcp_->SetConnectionCallback([this](int fd, bool connected) {
    if (connected) {
      // Dummy IP since TcpServer doesn't supply it right now; 
      // in production fetch via getpeername
      conn_mgr_->OnClientConnected(fd, "127.0.0.1"); 
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

  if (motor_thread_.joinable()) motor_thread_.join();
  if (imu_thread_.joinable()) imu_thread_.join();
  if (adc_thread_.joinable()) adc_thread_.join();
  if (lidar_thread_.joinable()) lidar_thread_.join();
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

  if (msg.msg_type == MsgType::kSetPose) {
    Pose2D pose = DeserializeSetPose(msg.payload);
    std::lock_guard<std::mutex> lock(state_mutex_);
    odom_calc_.Reset(pose.x, pose.y, pose.theta);
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
        current_odom_ = odom_calc_.Update(js.position[0], js.position[1], Timestamp::Now().nanoseconds);
        
        // Broadcast Odom via UDP
        std::vector<uint8_t> odom_payload = serializer_->SerializeOdom(current_odom_);
        std::vector<uint8_t> udp_pkt = serializer_->Frame(MsgType::kOdom, odom_payload);
        udp_->Send(udp_pkt);

        // Apply commands to motor
        if (!rl_navigation_active_) {
          // Manual Command
          auto rpms = diff_drive_.VelocityToRpm(target_cmd_vel_.linear_x, target_cmd_vel_.angular_z);
          // Convert RPM to rad/s for SetVelocityWait
          double left_rad_s = rpms.first * (2.0 * kPi / 60.0);
          double right_rad_s = rpms.second * (2.0 * kPi / 60.0);
          motor_->SetVelocityWait(left_rad_s, right_rad_s);
        } else {
          // RL Command handled here or in Lidar loop. 
          // Usually RL step runs at Lidar rate (10-20Hz) and outputs target v, w.
          // Motor loop runs at 50Hz and applies PD tracking to reach target.
          CmdVel cmd = rl_controller_.Compute(target_cmd_vel_, current_odom_.vx, current_odom_.vth);
          auto rpms = diff_drive_.VelocityToRpm(cmd.linear_x, cmd.angular_z);
          double left_rad_s = rpms.first * (2.0 * kPi / 60.0);
          double right_rad_s = rpms.second * (2.0 * kPi / 60.0);
          motor_->SetVelocityWait(left_rad_s, right_rad_s);
        }
      }
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

        if (is_active && onnx_actor_) {
          std::array<float, 28> obs = obs_builder_.Build(sectors, odom, goal.x, goal.y, step);
          std::array<float, 2> action = onnx_actor_->Infer(obs);
          
          float v = (action[0] + 1.0f) / 2.0f * 0.26f;
          float w = action[1] * 1.0f;
          
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // Target v, w sent to motor loop for PD control
            target_cmd_vel_.linear_x = v;
            target_cmd_vel_.angular_z = w;
          }
        }
      } else {
        std::this_thread::sleep_for(50ms);
      }
    } else {
      std::this_thread::sleep_for(100ms);
    }
  }
}

}  // namespace pinky
