#include <iostream>
#include <signal.h>
#include <atomic>

#include "pinky_core/app/robot_app.h"

std::atomic<bool> g_quit{false};

void SigintHandler(int sig) {
  g_quit.store(true);
}

int main(int argc, char** argv) {
  signal(SIGINT, SigintHandler);
  
  pinky::RobotConfig config;
  
  // Minimal config parsing, could be expanded to use YAML-cpp if needed.
  if (argc > 1) {
    if (std::string(argv[1]) == "--mock") {
      config.enable_hal = false;
    }
  }

  std::cout << "Initializing Pinky Core App...\n";
  
  pinky::RobotApp app(config);
  if (!app.Init()) {
    std::cerr << "Initialization failed. Exiting.\n";
    return 1;
  }

  // Run asynchronously or block, since Run() handles its own loops
  // Assuming Run() spawns threads and blocks on a sleep loop
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
