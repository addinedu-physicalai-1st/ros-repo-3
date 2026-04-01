#include <iostream>
#include <signal.h>
#include <atomic>
#include <string>

#include "pinky_core/app/robot_app.h"
#include "pinky_core/app/config_loader.h"

std::atomic<bool> g_quit{false};

void SigintHandler(int sig) {
  g_quit.store(true);
}

void PrintUsage(const char* prog) {
  std::cout << "Usage: " << prog << " [options]\n"
            << "  --mock              Disable HAL (PC mode)\n"
            << "  --config <path>     Load robot YAML config file\n"
            << "  --rl-config <path>  Load RL inference YAML config file\n";
}

int main(int argc, char** argv) {
  signal(SIGINT, SigintHandler);

  pinky::RobotConfig config;
  std::string config_path;
  std::string rl_config_path;
  bool mock_flag = false;

  // Parse args
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "--mock") {
      mock_flag = true;
    } else if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--rl-config" && i + 1 < argc) {
      rl_config_path = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0]);
      return 0;
    }
  }

  // Load YAML first, then apply CLI overrides
  if (!config_path.empty()) {
    if (!pinky::LoadConfig(config_path, config)) {
      std::cerr << "Failed to load config: " << config_path << "\n";
      return 1;
    }
    std::cout << "Config loaded: " << config_path << "\n";
  }
  if (!rl_config_path.empty()) {
    if (!pinky::LoadRlConfig(rl_config_path, config.rl)) {
      std::cerr << "Failed to load RL config: " << rl_config_path << "\n";
      return 1;
    }
    std::cout << "RL config loaded: " << rl_config_path << "\n";
  }
  if (mock_flag) {
    config.enable_hal = false;
  }

  std::cout << "Initializing Pinky Core App...\n";
  std::cout << "  TCP port: " << config.tcp_port
            << ", UDP port: " << config.udp_port
            << ", HAL: " << (config.enable_hal ? "ON" : "OFF") << "\n";

  pinky::RobotApp app(config);
  if (!app.Init()) {
    std::cerr << "Initialization failed. Exiting.\n";
    return 1;
  }

  std::thread app_thread([&app]() { app.Run(); });

  while (!g_quit.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  std::cout << "\nStopping Pinky Core App...\n";
  app.Stop();

  if (app_thread.joinable()) {
    app_thread.join();
  }

  std::cout << "Exited gracefully.\n";
  return 0;
}
