#include "pinky_core/inference/observation_builder.h"

#include <cmath>

namespace pinky {

void ObservationBuilder::SetGoal(float goal_x, float goal_y) {
  goal_x_ = goal_x;
  goal_y_ = goal_y;
}

std::array<float, kStateDim> ObservationBuilder::Build(
    const LidarSectors& lidar, const Odometry& odom, int current_step) const {
  std::array<float, kStateDim> obs{};

  // [0:24] LiDAR sectors (already normalized by LidarProcessor)
  for (int i = 0; i < kLidarSectors; ++i) {
    obs[i] = lidar.sectors[i];
  }

  // Goal distance and angle (robot-relative)
  double dx = goal_x_ - odom.x;
  double dy = goal_y_ - odom.y;
  double distance = std::hypot(dx, dy);
  double goal_angle = NormalizeAngle(std::atan2(dy, dx) - odom.theta);

  // [24] normalized distance
  obs[24] = Clamp(static_cast<float>(distance / kGoalDistScale), 0.0f, 1.0f);
  // [25] cos(goal_angle)
  obs[25] = static_cast<float>(std::cos(goal_angle));
  // [26] sin(goal_angle)
  obs[26] = static_cast<float>(std::sin(goal_angle));
  // [27] progress
  obs[27] = static_cast<float>(current_step) / static_cast<float>(kMaxSteps);

  return obs;
}

}  // namespace pinky
