#pragma once

#include <string>

#include "pinky_core/app/robot_app.h"

namespace pinky {

#ifdef PINKY_HAS_YAMLCPP
// Load RobotConfig from a YAML file. Fields not present in YAML keep defaults.
// Returns false on file open/parse error.
bool LoadConfig(const std::string& yaml_path, RobotConfig& config);

// Load RlConfig from rl_config.yaml. Fields not present keep defaults.
bool LoadRlConfig(const std::string& yaml_path, RlConfig& config);
#else
inline bool LoadConfig(const std::string& /*yaml_path*/, RobotConfig& /*config*/) {
  return false;
}
inline bool LoadRlConfig(const std::string& /*yaml_path*/, RlConfig& /*config*/) {
  return false;
}
#endif

}  // namespace pinky
