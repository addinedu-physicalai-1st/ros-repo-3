#include "pinky_core/inference/rl_controller.h"

namespace pinky {

RlController::RlController(float kp_v, float kd_v, float kp_w, float kd_w,
                           float v_min, float v_max, float w_max)
    : kp_v_(kp_v), kd_v_(kd_v), kp_w_(kp_w), kd_w_(kd_w),
      v_min_(v_min), v_max_(v_max), w_max_(w_max) {}

CmdVel RlController::ActionToTarget(
    const std::array<float, kActionDim>& action) const {
  // action[0] in [-1, 1] -> v in [v_min_, v_max_]
  float target_v = v_min_ + (action[0] + 1.0f) / 2.0f * (v_max_ - v_min_);
  // action[1] in [-1, 1] -> w in [-w_max_, w_max_]
  float target_w = action[1] * w_max_;
  return {target_v, target_w};
}

CmdVel RlController::Compute(const std::array<float, kActionDim>& action,
                              float current_v, float current_w) {
  CmdVel target = ActionToTarget(action);

  float error_v = target.linear_x - current_v;
  float error_w = target.angular_z - current_w;

  float cmd_v = target.linear_x + kp_v_ * error_v +
                kd_v_ * (error_v - prev_error_v_);
  float cmd_w = target.angular_z + kp_w_ * error_w +
                kd_w_ * (error_w - prev_error_w_);

  prev_error_v_ = error_v;
  prev_error_w_ = error_w;

  cmd_v = Clamp(cmd_v, v_min_, v_max_);
  cmd_w = Clamp(cmd_w, -w_max_, w_max_);

  return {cmd_v, cmd_w};
}

void RlController::Reset() {
  prev_error_v_ = 0.0f;
  prev_error_w_ = 0.0f;
}

}  // namespace pinky
