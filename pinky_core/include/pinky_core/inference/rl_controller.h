#pragma once

#include <array>

#include "pinky_core/common/constants.h"
#include "pinky_core/common/types.h"

namespace pinky {

// Wraps RL action output with PD control.
// Does NOT own the OnnxActor — caller is responsible for inference.
class RlController {
 public:
  RlController(float kp_v = kKpV, float kd_v = kKdV, float kp_w = kKpW,
               float kd_w = kKdW, float v_min = kVMin, float v_max = kVMax,
               float w_max = kWMax);

  // Convert raw RL action [-1,1] to cmd_vel, applying PD control.
  // current_v/current_w: current measured velocities from odometry.
  CmdVel Compute(const std::array<float, kActionDim>& action, float current_v,
                 float current_w);

  // Map raw action to target velocities (no PD).
  CmdVel ActionToTarget(const std::array<float, kActionDim>& action) const;

  void Reset();

 private:
  float kp_v_;
  float kd_v_;
  float kp_w_;
  float kd_w_;
  float v_min_;
  float v_max_;
  float w_max_;
  float prev_error_v_{0.0f};
  float prev_error_w_{0.0f};
};

}  // namespace pinky
