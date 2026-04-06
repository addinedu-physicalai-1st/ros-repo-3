#include "pinky_core/inference/observation_builder.h"

#include <cmath>

namespace pinky {

ObservationBuilder::ObservationBuilder(float goal_dist_scale, int max_steps)
    : goal_dist_scale_(goal_dist_scale), max_steps_(max_steps) {}

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
  obs[24] = Clamp(static_cast<float>(distance / goal_dist_scale_), 0.0f, 1.0f);
  // [25] angle / pi
  obs[25] = static_cast<float>(goal_angle / M_PI);
  // [26] normalized linear velocity (v / 0.5)
  obs[26] = static_cast<float>(odom.vx / 0.5);
  // [27] normalized angular velocity (w / 1.0)
  obs[27] = static_cast<float>(odom.vth / 1.0);

  return obs;
}

}  // namespace pinky
