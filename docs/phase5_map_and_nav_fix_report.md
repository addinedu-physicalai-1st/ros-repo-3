# Phase 5: Map Building and Navigation Fix Report

## 1. Overview
In this phase, we addressed three major issues reported during the autonomous navigation and map building tests:
1. **Map Building Issue:** The SLAM map was only registering obstacles (black lines) without drawing free space (white areas). Additionally, the map was highly distorted (mirrored) because of incorrect LiDAR angle mapping.
2. **Navigation/Obstacle Avoidance Issue:** The robot was driving erratically ("like a drunkard") and crashing straight into obstacles.
3. **Execution Failure:** The `./pinky_robot` executable would exit immediately with no output.

## 2. Root Cause Analysis & Fixes

### 2.1. Occupancy Grid Map Building Fix
*   **Root Cause:** The `occupancy_grid.py` logic was completely ignoring LiDAR beams that returned `0.0` or `infinity`. Because these beams were `continue`d, the raycasting algorithm (`_trace_ray`) never executed for empty space.
*   **Resolution:** Modified `OccupancyGridBuilder.update` in `pinky_station/core/occupancy_grid.py`:
    *   If a ray distance is `0.0`, infinite, or greater than `range_max`, it is now clamped to `range_max`.
    *   Bresenham's raycasting (`_trace_ray`) is now executed up to `range_max` to properly clear out free space.
    *   The `is_valid_hit` flag ensures we do not place an obstacle (black wall) at the max range limit.

### 2.2. SAC Navigation Model & Constants Fix
*   **Root Cause 1 (Observation Mismatch):** A previous attempt to fix the observation state array structure matched the wrong environment. The `sac_actor.onnx` model was trained with `pinky_sac_env.py`, which uses `cos(angle)`, `sin(angle)`, and `progress`, not normalized velocity.
*   **Root Cause 2 (Action Space Mismatch):** Another AI update incorrectly changed the maximum linear and angular velocities in `constants.h` (`kVMax` to 0.5, `kWMax` to 1.5). The SAC model was trained with limits of `0.26` m/s and `1.0` rad/s, causing erratic, over-amplified control outputs.
*   **Resolution:** 
    *   Reverted `ObservationBuilder::Build` in `pinky_core/src/inference/observation_builder.cpp` to correctly use `cos(goal_angle)`, `sin(goal_angle)`, and `current_step / max_steps`.
    *   Reverted the speed constants in `pinky_core/include/pinky_core/common/constants.h` back to `kVMax = 0.26f` and `kWMax = 1.0f`.

### 2.3. LiDAR CW to CCW Array Mapping Fix
*   **Root Cause:** The SLLiDAR outputs data clockwise (CW) starting from its physical front (0°). A previous fix attempted to convert this to counter-clockwise (CCW) but failed to account for the physical 180-degree rotation of the LiDAR mounted on the robot (as defined by `rpy="0 0 pi"` in the URDF). This caused both the map to be mirrored and the obstacle avoidance sectors to be flipped left-to-right.
*   **Resolution:** Updated the array mapping in `pinky_core/src/hal/sllidar_driver.cpp` to correctly align the 0-index with the front of the `base_link` and increment CCW:
    *   `size_t src_idx = (count / 2 - i + count) % count;`

### 2.5. Map Building Logic Optimization (Ref. Standard Compliance)
*   **Root Cause:** The `free_thresh` and PGM output values did not perfectly align with the ROS 2 `map_server` and the reference PDF standards, leading to inconsistent visualization (off-white instead of pure white).
*   **Resolution:**
    *   **Threshold Update:** Adjusted `free_thresh` from `0.196` to **`0.25`** in `OccupancyGridBuilder` to match the reference parameters.
    *   **Logic Correction:** Fixed the probability comparison in `to_pgm_array` to `prob < free_thresh` (previously using an inverse logic) to ensure correct free-space detection.
    *   **Color Value:** Changed the PGM output for free space from `254` to **`255` (pure white)** to strictly follow the ROS 2 occupancy grid convention shown in the reference material.

### 2.6. ROS 2 Bridge (Nav2/SLAM) Integration
*   **Requirement:** The system needed to bridge raw LiDAR data to ROS 2 for external SLAM tools (like `slam_toolbox`) as highlighted in the "Core is slam toolbox" section of the reference PDF.
*   **Resolution:**
    *   **NavWorker Update:** Added `publish_scan` method to `RosBridgeNode` and `on_lidar_scan_received` handler to `NavWorker` in `pinky_station`.
    *   **Signal Connection:** In `MainWindow`, connected the `ZmqClient.lidar_scan_received` signal to the `NavWorker` bridge. 
    *   **Result:** Real-time LiDAR data received from the robot via ZeroMQ is now automatically published to the `/scan` ROS 2 topic, enabling seamless integration with `slam_toolbox` and `rviz2`.

## 3. Results
*   **Mapping:** The robot now correctly raytraces free space without distortion, yielding a persistent, accurate occupancy grid that appears pure white in free areas, matching the professional SLAM standards.
*   **Interoperability:** The PC-side GUI now acts as a complete bridge, allowing standard ROS 2 navigation and mapping stacks to utilize the robot's hardware data transparently.
*   **Navigation:** The C++ inference engine now supplies correct observations and respects the trained velocity boundaries, enabling smooth, accurate obstacle-avoidance maneuvers.
*   **Stability:** The application now launches successfully without immediate crashes.