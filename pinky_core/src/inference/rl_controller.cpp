#include "pinky_core/inference/rl_controller.h"

namespace pinky {

RlController::RlController(float kp_v, float kd_v, float kp_w, float kd_w)
    : kp_v_(kp_v), kd_v_(kd_v), kp_w_(kp_w), kd_w_(kd_w) {}

CmdVel RlController::ActionToTarget(
    const std::array<float, kActionDim>& action) {
  // action[0] in [-1, 1] -> v in [V_MIN, V_MAX]
  float target_v = kVMin + (action[0] + 1.0f) / 2.0f * (kVMax - kVMin);
  // action[1] in [-1, 1] -> w in [-W_MAX, W_MAX]
  float target_w = action[1] * kWMax;
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

  cmd_v = Clamp(cmd_v, kVMin, kVMax);
  cmd_w = Clamp(cmd_w, -kWMax, kWMax);

  return {cmd_v, cmd_w};
}

void RlController::Reset() {
  prev_error_v_ = 0.0f;
  prev_error_w_ = 0.0f;
}

}  // namespace pinky
