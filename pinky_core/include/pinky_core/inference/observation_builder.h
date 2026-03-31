#pragma once

#include <array>

#include "pinky_core/common/constants.h"
#include "pinky_core/common/types.h"

namespace pinky {

// Constructs the 28D observation vector from sensor data.
//   [0:24]  = 24 normalized LiDAR sectors
//   [24]    = clip(goal_distance / 5.0, 0, 1)
//   [25]    = cos(angle_to_goal)
//   [26]    = sin(angle_to_goal)
//   [27]    = current_step / max_steps
class ObservationBuilder {
 public:
  void SetGoal(float goal_x, float goal_y);

  std::array<float, kStateDim> Build(const LidarSectors& lidar,
                                     const Odometry& odom,
                                     int current_step) const;

  float goal_x() const { return goal_x_; }
  float goal_y() const { return goal_y_; }

 private:
  float goal_x_{0.0f};
  float goal_y_{0.0f};
};

}  // namespace pinky
