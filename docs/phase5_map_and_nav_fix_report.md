# Phase 5: Map Building and Navigation Fix Report

## 1. Overview
In this phase, we addressed two major issues reported during the autonomous navigation and map building tests:
1. **Map Building Issue:** The SLAM map was only registering obstacles (black lines) without drawing free space (white areas). The map was also failing to remember previously explored areas once the robot moved away.
2. **Navigation/Obstacle Avoidance Issue:** The robot was driving straight into obstacles during autonomous navigation, failing to use its learned SAC (Soft Actor-Critic) obstacle avoidance behaviors despite working fine in simulation.

## 2. Root Cause Analysis & Fixes

### 2.1. Occupancy Grid Map Building Fix
*   **Root Cause:** The `occupancy_grid.py` logic was completely ignoring LiDAR beams that returned `0.0` or `infinity` (which `sllidar_driver` outputs for out-of-range or invalid readings). Because these beams were `continue`d, the raycasting algorithm (`_trace_ray`) never executed for empty space. This resulted in the map failing to clear out free space between the robot and its max range.
*   **Resolution:** Modified `OccupancyGridBuilder.update` in `pinky_station/core/occupancy_grid.py`:
    *   If a ray distance is `0.0`, infinite, or greater than `range_max`, it is now clamped to `range_max`.
    *   Bresenham's raycasting (`_trace_ray`) is now executed up to `range_max` to properly clear out free space.
    *   The `is_valid_hit` flag ensures we do not place an obstacle (black wall) at the max range limit.

### 2.2. SAC Navigation Model Observation Fix
*   **Root Cause:** There was a severe mismatch between the observation state array structure used during Python training (`pinky_env.py`) and the C++ inference implementation (`observation_builder.cpp`).
    *   **Python Training State:** `[25] angle / π`, `[26] normalized linear velocity (v / 0.5)`, `[27] normalized angular velocity (w / 1.0)`.
    *   **C++ Inference State:** `[25] cos(goal_angle)`, `[26] sin(goal_angle)`, `[27] progress (current_step / max_steps)`.
    *   Because the neural network received completely different features than what it was trained on, its spatial awareness and velocity tracking failed, leading to catastrophic obstacle collisions.
*   **Resolution:** Updated `ObservationBuilder::Build` in `pinky_core/src/inference/observation_builder.cpp` to perfectly match the Python environment's normalization:
    *   `obs[25] = goal_angle / M_PI;`
    *   `obs[26] = odom.vx / 0.5;`
    *   `obs[27] = odom.vth / 1.0;`

## 3. Results
*   **Mapping:** The robot now correctly raytraces free space, yielding a persistent, gray/white/black occupancy grid that correctly remembers explored areas.
*   **Navigation:** The C++ inference engine now supplies the correct observation tensor to the ONNX model. The robot successfully executes its trained obstacle-avoidance and path-finding maneuvers.