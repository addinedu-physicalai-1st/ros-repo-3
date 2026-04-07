#include "pinky_core/app/config_loader.h"

#include <iostream>

#include <yaml-cpp/yaml.h>

namespace pinky {

bool LoadConfig(const std::string& yaml_path, RobotConfig& config) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    std::cerr << "Config load error: " << e.what() << "\n";
    return false;
  }

  // network
  if (auto net = root["network"]) {
    if (net["tcp_port"]) config.rep_port = net["tcp_port"].as<uint16_t>();
    if (net["udp_port"]) config.pub_port = net["udp_port"].as<uint16_t>();
  }

  // hal
  if (auto hal = root["hal"]) {
    if (hal["enable"]) config.enable_hal = hal["enable"].as<bool>();
    if (auto lidar = hal["lidar"]) {
      if (lidar["device"]) config.lidar_device = lidar["device"].as<std::string>();
      if (lidar["baudrate"]) config.lidar_baudrate = lidar["baudrate"].as<uint32_t>();
    }
  }

  // robot physics — overrides compiled-in constants when present
  if (auto robot = root["robot"]) {
    if (robot["id"])            config.robot_id      = robot["id"].as<std::string>();
    if (robot["wheel_radius"])  config.wheel_radius  = robot["wheel_radius"].as<double>();
    if (robot["wheel_base"])    config.wheel_base    = robot["wheel_base"].as<double>();
    if (robot["max_rpm"])       config.max_rpm       = robot["max_rpm"].as<double>();
  }

  // RL model path from embedded section (backward compat)
  if (auto model = root["model"]) {
    if (model["path"]) config.rl.model_path = model["path"].as<std::string>();
  }

  return true;
}

bool LoadRlConfig(const std::string& yaml_path, RlConfig& config) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    std::cerr << "RL config load error: " << e.what() << "\n";
    return false;
  }

  // model
  if (auto m = root["model"]) {
    if (m["path"])        config.model_path  = m["path"].as<std::string>();
    if (m["input_name"])  config.input_name  = m["input_name"].as<std::string>();
    if (m["output_name"]) config.output_name = m["output_name"].as<std::string>();
  }

  // observation
  if (auto obs = root["observation"]) {
    if (obs["goal_dist_scale"]) config.goal_dist_scale = obs["goal_dist_scale"].as<float>();
    if (obs["max_steps"])       config.max_steps       = obs["max_steps"].as<int>();
  }

  // action_mapping
  if (auto act = root["action_mapping"]) {
    if (act["v_min"]) config.v_min = act["v_min"].as<float>();
    if (act["v_max"]) config.v_max = act["v_max"].as<float>();
    if (act["w_max"]) config.w_max = act["w_max"].as<float>();
  }

  // pd_control
  if (auto pd = root["pd_control"]) {
    if (pd["kp_v"]) config.kp_v = pd["kp_v"].as<float>();
    if (pd["kd_v"]) config.kd_v = pd["kd_v"].as<float>();
    if (pd["kp_w"]) config.kp_w = pd["kp_w"].as<float>();
    if (pd["kd_w"]) config.kd_w = pd["kd_w"].as<float>();
  }

  // navigation
  if (auto nav = root["navigation"]) {
    if (nav["goal_tolerance"])    config.goal_tolerance    = nav["goal_tolerance"].as<float>();
    if (nav["lookahead_dist"])    config.lookahead_dist    = nav["lookahead_dist"].as<float>();
    if (nav["control_period_ms"]) config.control_period_ms = nav["control_period_ms"].as<double>();

    constexpr float kDegToRad = static_cast<float>(kPi) / 180.0f;
    if (nav["turn_first_enter_deg"])
      config.turn_first_enter_rad = nav["turn_first_enter_deg"].as<float>() * kDegToRad;
    if (nav["turn_first_exit_deg"])
      config.turn_first_exit_rad = nav["turn_first_exit_deg"].as<float>() * kDegToRad;
    if (nav["ema_alpha"])        config.ema_alpha        = nav["ema_alpha"].as<float>();
    if (nav["angular_deadzone"]) config.angular_deadzone = nav["angular_deadzone"].as<float>();
    if (nav["safety_stop_dist"]) config.safety_stop_dist = nav["safety_stop_dist"].as<float>();
    if (nav["safety_scale_dist"]) config.safety_scale_dist = nav["safety_scale_dist"].as<float>();
    if (nav["pctrl_v_max"])      config.pctrl_v_max      = nav["pctrl_v_max"].as<float>();
    if (nav["pctrl_w_max"])      config.pctrl_w_max      = nav["pctrl_w_max"].as<float>();
  }

  // lidar
  if (auto lid = root["lidar"]) {
    if (lid["min_range"]) config.lidar_min_range = lid["min_range"].as<float>();
  }

  // emotion
  if (auto emo = root["emotion"]) {
    if (emo["dir"]) config.emotion_dir = emo["dir"].as<std::string>();
  }

  return true;
}

}  // namespace pinky
