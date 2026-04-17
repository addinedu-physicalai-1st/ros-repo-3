# Pinky Pro: Pure C++ Core + PyQt6 Control Station Implementation Plan

## Context

Pinky Pro is an educational robot (RPi5 ARM64, ROS2 Jazzy) that currently uses ROS2 nodes for all hardware control and communication. This project transforms the architecture into:

1. **Robot side**: Pure C++ core (de-ROSing) with direct hardware control and TCP/UDP binary socket server
2. **PC side**: PyQt6 GUI control station (camera, map, teleop, battery, terminal, navigation)
3. **RL deployment**: ONNX Runtime SAC model for local obstacle avoidance (290KB, 28D->2D)

Existing ROS2 code (`pinky_pro/`) is preserved untouched. New code lives in `pinky_core/` (robot) and `pinky_station/` (PC).

---

## Phase 1: Protocol + Common Types

**Goal**: Binary communication protocol (C++ & Python) + ROS-free data types.

### 1-1. Directory scaffold

```
pinky_core/                          # Robot C++ project
  CMakeLists.txt
  config/
    robot_config.yaml
    rl_config.yaml
  models/
    sac_actor.onnx                   # Copy from RL_Book/models/
  include/pinky_core/
    common/  protocol/  hal/  core/  inference/  net/  app/
  src/
    common/  protocol/  hal/  core/  inference/  net/  app/
  tests/

pinky_station/                       # PC PyQt6 project
  requirements.txt
  config/station_config.yaml
  pinky_station/
    __init__.py  __main__.py  app.py
    protocol/  net/  gui/  workers/
  resources/
    icons/  styles/dark.qss
```

### 1-2. Binary protocol frame

```
Offset  Size  Field            Description
0       2     MAGIC            0x50 0x4B ("PK")
2       1     VERSION          0x01
3       1     MSG_TYPE         Message type enum (uint8)
4       2     SEQUENCE         uint16 LE
6       4     PAYLOAD_LENGTH   uint32 LE
10      2     HEADER_CRC       CRC16-CCITT over bytes 0..9
12      N     PAYLOAD          Type-specific data
12+N    2     PAYLOAD_CRC      CRC16-CCITT over payload
```

Header: 12 bytes fixed. Total: 14 + N bytes.

### 1-3. Message types

| ID | Name | Direction | Payload | Transport |
|----|------|-----------|---------|-----------|
| 0x01 | ODOM | Robot->PC | x,y,theta,vx,vth (5*f32=20B) | UDP |
| 0x02 | IMU | Robot->PC | quat(4)+gyro(3)+accel(3) (10*f32=40B) | UDP |
| 0x03 | LIDAR_SCAN | Robot->PC | num(u16)+[angle,range]*N | UDP |
| 0x04 | LIDAR_24 | Robot->PC | 24*f32=96B (preprocessed) | UDP |
| 0x05 | BATTERY | Robot->PC | voltage(f32)+pct(f32)+status(u8)=9B | UDP |
| 0x06 | JOINT_STATE | Robot->PC | pos_l,pos_r,vel_l,vel_r (4*f32=16B) | UDP |
| 0x07 | CAMERA_FRAME | Robot->PC | w(u16)+h(u16)+size(u32)+jpeg(N) | TCP |
| 0x08 | DEBUG_LOG | Robot->PC | severity(u8)+ts(u64)+text | TCP |
| 0x09 | IR_SENSOR | Robot->PC | 3*u16=6B | UDP |
| 0x0A | US_SENSOR | Robot->PC | range(f32)=4B | UDP |
| 0x0B | ROBOT_STATUS | Robot->PC | bitmask(u32) | UDP |
| 0x20 | CMD_VEL | PC->Robot | linear_x(f32)+angular_z(f32)=8B | TCP |
| 0x21 | SET_LED | PC->Robot | cmd(u8)+mask(u8)+r,g,b=5B | TCP |
| 0x22 | SET_LAMP | PC->Robot | mode(u8)+r,g,b+time_ms(u16)=6B | TCP |
| 0x23 | SET_EMOTION | PC->Robot | emotion_id(u8)=1B | TCP |
| 0x24 | SET_BRIGHTNESS | PC->Robot | brightness(u8)=1B | TCP |
| 0x25 | NAV_GOAL | PC->Robot | x,y,theta (3*f32=12B) | TCP |
| 0x26 | NAV_CANCEL | PC->Robot | empty | TCP |
| 0x27 | SET_POSE | PC->Robot | x,y,theta (3*f32=12B) | TCP |
| 0x28 | PING | PC->Robot | ts(u64)=8B | TCP |
| 0x30 | PONG | Bidirectional | echo_ts+server_ts (2*u64=16B) | TCP |
| 0x31 | ACK | Bidirectional | ack_seq(u16)+status(u8)+msg | TCP |
| 0x32 | MAP_DATA | Robot->PC | w,h(u32)+res(f32)+origin(3*f32)+data(RLE) | TCP |

Transport: TCP port 9100 (reliable: commands, camera, map, logs), UDP port 9200 (fast: sensors).

### 1-4. Common types (`types.h`)

ROS-free C++ structs: `Vec2`, `Vec3`, `Quaternion`, `Timestamp`, `Odometry`, `ImuData`, `LidarScan`, `LidarSectors`, `BatteryState`, `JointState`, `CmdVel`, `NavGoal`, etc.

### 1-5. Files to create

**C++ (pinky_core/)**:
- `include/pinky_core/common/types.h` - All data structs
- `include/pinky_core/common/constants.h` - Physical constants
- `include/pinky_core/common/logger.h` + `src/common/logger.cpp` - spdlog wrapper
- `include/pinky_core/protocol/message_types.h` - MsgType enum + MessageHeader
- `include/pinky_core/protocol/serializer.h` + `src/protocol/serializer.cpp`
- `include/pinky_core/protocol/checksum.h` + `src/protocol/checksum.cpp` - CRC16
- `tests/test_protocol.cpp` - Roundtrip serialize/deserialize

**Python (pinky_station/)**:
- `pinky_station/protocol/message_types.py` - Mirror C++ enums
- `pinky_station/protocol/serializer.py` - struct module pack/unpack
- `pinky_station/protocol/checksum.py` - CRC16 matching C++

### 1-6. Reference files (read-only)
- `pinky_pro/src/pinky_pro/pinky_interfaces/` - Service definitions for message type design

---

## Phase 2: Core Logic Library

**Goal**: ROS-independent C++ algorithm classes.

### 2-1. DiffDrive (`core/diff_drive.h/.cpp`)
- `VelocityToRpm(linear_x, angular_z) -> (rpm_l, rpm_r)`
- `RpmToVelocity(rpm_l, rpm_r) -> (vx, vth)`
- Constants: wheel_radius=0.028, wheel_base=0.0961, max_rpm=100
- Reference: `pinky_bringup_cpp/src/main_node.cpp:188-208`

### 2-2. Odometry (`core/odometry.h/.cpp`)
- `Update(encoder_l, encoder_r, timestamp) -> Odometry`
- Encoder delta integration, x/y/theta accumulation
- Reference: `pinky_bringup_cpp/src/main_node.cpp:218-255`

### 2-3. LidarProcessor (`core/lidar_processor.h/.cpp`)
- `Process(LidarScan) -> LidarSectors`
- Pipeline: NaN/Inf clean -> roll(n/2) -> 24-sector min-pool -> /3.5 normalize
- Reference: `RL_Book/envs/pinky_env.py:37-57`

### 2-4. BatteryMonitor (`core/battery_monitor.h/.cpp`)
- `Update(adc_value) -> BatteryState`
- Formula: `V = (adc/4096) * 4.096 / (13.0/28.0)`
- Range: 6.0V (0%) ~ 8.4V (100%), low threshold 6.8V
- Reference: `pinky_sensor_adc/src/main_node.cpp:80-95`

### 2-5. LedController (`core/led_controller.h/.cpp`)
- State machine: off/on/blink/dim modes
- HSV/RGB conversion for dimming
- Reference: `pinky_lamp_control/src/main_node.cpp:106-229`

---

## Phase 3: ONNX Inference Engine

**Goal**: SAC RL model inference in pure C++.

### 3-1. OnnxActor (`inference/onnx_actor.h/.cpp`)
- Load `sac_actor.onnx` via ONNX Runtime C++ API
- `Infer(array<float,28>) -> array<float,2>` (tanh output in [-1,1])
- Input tensor name: "state", Output: "action"
- SessionOptions: `intra_op_num_threads = 1`

### 3-2. ObservationBuilder (`inference/observation_builder.h/.cpp`)
- `Build(LidarSectors, Odometry, goal_x, goal_y, step_count) -> array<float,28>`
- [0:24] = lidar sectors, [24] = clip(dist/5.0, 0, 1), [25] = cos(angle), [26] = sin(angle), [27] = step/750

### 3-3. RlController (`inference/rl_controller.h/.cpp`)
- Action mapping: `v = (a[0]+1)/2 * 0.26`, `w = a[1] * 1.0`
- PD control: KP_V=1.5, KD_V=0.3, KP_W=1.5, KD_W=0.3
- `Compute(observation, current_v, current_w) -> CmdVel`
- Reference: `RL_Book/envs/pinky_sac_env.py:250-268`

### 3-4. ONNX Runtime setup
- RPi5: Download `onnxruntime-linux-aarch64-1.17.1.tgz`
- CMake: `find_path(ONNXRUNTIME_INCLUDE_DIR)`, `find_library(ONNXRUNTIME_LIBRARY)`

---

## Phase 4: Network Layer

**Goal**: TCP/UDP server (robot) and client (PC).

### 4-1. TCP Server (`net/tcp_server.h/.cpp`)
- `epoll`-based async I/O
- Accept connections, message framing (header parse -> payload read -> CRC verify)
- Connection heartbeat via PING/PONG
- Port 9100

### 4-2. UDP Server (`net/udp_server.h/.cpp`)
- Non-blocking `sendto()` for sensor broadcast
- Unicast to connected PC address
- Port 9200

### 4-3. ConnectionManager (`net/connection_manager.h/.cpp`)
- Track connected clients, lifecycle management
- Auto-disconnect on heartbeat timeout

### 4-4. FrameSender (`net/frame_sender.h/.cpp`)
- Camera JPEG compression (OpenCV `imencode`) + MSG_CAMERA_FRAME send over TCP

### 4-5. Python client (pinky_station/)
- `net/tcp_client.py` - Persistent TCP connection, send commands, receive responses
- `net/udp_receiver.py` - UDP listener thread for sensor data
- `net/connection.py` - Connection state machine (disconnected/connecting/connected)

---

## Phase 5: HAL Layer (ARM64 only)

**Goal**: Direct hardware control, no ROS2.

Each HAL class implements a pure virtual interface (`I*Driver`) for mock testing on PC.

### 5-1. DynamixelMotor (`hal/dynamixel_motor.h/.cpp`)
- Wraps DynamixelSDK directly (already pure C++)
- Port: `/dev/ttyAMA4`, baudrate: 1MHz, motor IDs: 1(L), 2(R)
- Velocity mode, GroupSyncWrite/GroupBulkRead
- Reference: `pinky_bringup_cpp/src/main_node.cpp:68-186`

### 5-2. SllidarDriver (`hal/sllidar_driver.h/.cpp`)
- Wraps Slamtec SDK from `sllidar_ros2/sdk/` (pure C++)
- Serial: `/dev/ttyAMA0`, 1MHz, DenseBoost mode
- `GetScan() -> LidarScan`

### 5-3. Bno055Imu (`hal/bno055_imu.h/.cpp`)
- I2C: `/dev/i2c-0`, address 0x28
- Init: reset -> power normal -> IMU mode -> wait status 0x05
- Read 32B from register 0x08: accel(/100), gyro(/16), quat(/16384)
- Reference: `pinky_imu_bno055/src/main_node.cpp:25-88`

### 5-4. AdcSensor (`hal/adc_sensor.h/.cpp`)
- I2C: `/dev/i2c-1`, address 0x08
- 5 registers: {0x88, 0xC8, 0x98, 0xD8, 0xF8}, 12-bit assembly
- Reference: `pinky_sensor_adc/src/main_node.cpp:42-57`

### 5-5. Ws2811Led/Ws2811Lamp (`hal/ws2811_led.h/.cpp`, `hal/ws2811_lamp.h/.cpp`)
- LED: GPIO 18, GRB, 8 LEDs
- Lamp: GPIO 19, GBR, 8 LEDs
- Uses librpi_ws281x

### 5-6. Ili9341Lcd (`hal/ili9341_lcd.h/.cpp`)
- SPI bus 0 @ 80MHz, RST=GPIO27, DC=GPIO25, BL=GPIO18
- RGB565 conversion, ILI9341 init sequence
- Reference: `pinky_emotion_cpp/src/main_node.cpp:30-125`

### 5-7. External dependencies

| Library | Source | Link |
|---------|--------|------|
| DynamixelSDK | `/opt/ros/jazzy/lib/libdynamixel_sdk.so` | `-ldynamixel_sdk` |
| WiringPi | `pinky_devices/WiringPi/` | Build from source |
| rpi_ws281x | `pinky_devices/rpi_ws281x/` | Build from source |
| SLLiDAR SDK | `sllidar_ros2/sdk/` | Compile into project |
| OpenCV | System | `find_package(OpenCV)` |
| ONNX Runtime | Microsoft release | `-lonnxruntime` |
| spdlog | System/vendored | Logging |

---

## Phase 6: Robot Application

**Goal**: Single executable integrating all components.

### 6-1. RobotApp (`app/robot_app.h/.cpp`)
- Initialize all HAL drivers from config
- Sensor loops at respective rates:
  - Motor + Odometry: 50Hz
  - IMU: 100Hz
  - ADC (battery/sensors): 20Hz
  - LiDAR: ~10Hz
- UDP broadcast sensor data to connected PC
- TCP command listener -> route to HAL/core
- RL navigation loop: sensors -> ObservationBuilder -> OnnxActor -> RlController -> DynamixelMotor

### 6-2. Threading model
- **Main thread**: TCP accept + command dispatch
- **Sensor thread**: High-priority (motors + IMU + odom @ 50Hz)
- **LiDAR thread**: Dedicated scan acquisition
- **LED/LCD thread**: Lower priority visual updates
- **Network thread**: UDP sensor broadcast

### 6-3. Entry point (`app/main.cpp`)
- Parse YAML config
- Create RobotApp
- Run until SIGINT

### 6-4. CMakeLists.txt
```cmake
option(BUILD_HAL "Build HAL (requires ARM64)" ON)
option(BUILD_TESTS "Build unit tests" ON)

if(NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(BUILD_HAL OFF)
    message(STATUS "Non-ARM64: HAL disabled, core + tests only")
endif()
```

---

## Phase 7: PyQt6 GUI Control Station

**Goal**: Full-featured PC GUI.

### 7-1. Main window layout

```
+-----------------------------------------------------------+
| [Toolbar] [Connect IP:__] [Port:__] [Connect] | [2D Pose] | [DomainID:__]
+----------------------------+--------------------------+
| Camera View (JPEG stream)  | Map Widget               |
| 640x480                    | (occupancy grid,         |
|                            |  robot dot + DomainId,   |
| [Snapshot]                 |  click-to-set-goal,      |
|                            |  path overlay)           |
+----------------------------+--------------------------+
| Teleop Widget              | Battery Widget           |
| [  ] [UP] [  ]             | [======== 87%]           |
| [LT] [ST] [RT]             | Voltage: 7.82V           |
| [  ] [DN] [  ]             |                          |
| Speed: [slider] 0.20 m/s   | Robot Status: OK/ERR     |
+----------------------------+--------------------------+
| Terminal (monospace, filtered, color-coded)             |
| [Filter: All|cmd_vel|odom|battery|errors] [Clear]      |
+--------------------------------------------------------+
```

### 7-2. Widget classes

| Widget | File | Key features |
|--------|------|-------------|
| MainWindow | `gui/main_window.py` | Layout, owns TCP/UDP clients, signal routing |
| CameraWidget | `gui/camera_widget.py` | QLabel + QImage.fromData(jpeg), snapshot |
| MapWidget | `gui/map_widget.py` | QPainter occupancy grid, robot circle+arrow+DomainId, click goal, pan/zoom, path polyline |
| TeleopWidget | `gui/teleop_widget.py` | 3x3 QPushButton grid, QSlider speed, WASD/arrow keys, auto-stop on release |
| TerminalWidget | `gui/terminal_widget.py` | QPlainTextEdit read-only, 5000 line buffer, filter combo, color-coded |
| BatteryWidget | `gui/battery_widget.py` | QProgressBar, voltage text, green/yellow/red thresholds |
| Toolbar | `gui/toolbar.py` | Connection controls, 2D Pose Estimate button |

### 7-3. Worker threads

| Worker | Purpose |
|--------|---------|
| SensorWorker | QThread: UDP recv -> emit typed Qt signals |
| CameraWorker | QThread: TCP recv CAMERA_FRAME -> emit QImage |
| CommandWorker | QThread: Queue -> TCP send commands |
| NavWorker | QThread: optional rclpy Nav2 bridge, fallback to TCP NAV_GOAL |

### 7-4. 2D Pose Estimate
- MapWidget에서 마우스 드래그로 위치+방향 지정
- MSG_SET_POSE (x, y, theta) 전송 -> 로봇의 odometry reset
- 프로그램 시작 시 자동 실행 옵션

### 7-5. Navigation flow
1. User clicks map -> MapWidget emits goal(x, y, theta)
2. MainWindow sends MSG_NAV_GOAL via TCP
3. Robot: Nav2 global planner (if bridge active) -> global path
4. Robot: RL local planner reads LiDAR + goal -> ONNX inference -> cmd_vel
5. Robot streams ODOM back -> MapWidget updates robot position

---

## Phase 8: Integration & Debug

### 8-1. Debug output (robot side)
- All sensor data logged with timestamps via spdlog
- MSG_DEBUG_LOG sent to PC for remote monitoring
- 28D observation vector logging for RL verification (compare with Python output)
- cmd_vel logging at each control step

### 8-2. Verification checklist
- [ ] Protocol: C++ serialize -> Python deserialize roundtrip (and vice versa)
- [ ] LidarProcessor: Same 24D output as Python `pinky_env.py:37-57` for identical input
- [ ] OnnxActor: Same 2D output as PyTorch model for identical 28D input
- [ ] DiffDrive: RPM values match existing ROS2 node for identical cmd_vel
- [ ] Battery: Voltage/percentage match ADC node for identical ADC values
- [ ] TCP/UDP: Sensor data flows from robot to PC GUI correctly
- [ ] Teleop: PC arrow keys control robot motors
- [ ] Camera: JPEG stream displays in GUI at 15+ fps
- [ ] RL navigation: Robot avoids obstacles while following global path

---

## Implementation Order Summary

| Phase | Content | Depends on | Buildable on PC? |
|-------|---------|------------|-------------------|
| 1 | Protocol + Common types | None | Yes |
| 2 | Core logic (DiffDrive, Odometry, LiDAR, Battery, LED) | Phase 1 | Yes |
| 3 | ONNX inference (OnnxActor, ObsBuilder, RLController) | Phase 2 | Yes |
| 4 | Network layer (TCP/UDP server + Python client) | Phase 1 | Yes (loopback) |
| 5 | HAL layer (all hardware drivers) | Phase 2 | ARM64 only |
| 6 | Robot application (orchestrator + threading) | Phase 3,4,5 | ARM64 only |
| 7 | PyQt6 GUI (all widgets + workers) | Phase 4 | Yes |
| 8 | Integration & debug | All | Robot + PC |

Phase 7 can run in parallel with Phase 5-6 since GUI depends only on the network client (Phase 4).

---

## Critical Reference Files (read-only, never modify)

| Component | File | Lines |
|-----------|------|-------|
| Motor/Odometry | `pinky_bringup_cpp/src/main_node.cpp` | 68-255 |
| IMU | `pinky_imu_bno055/src/main_node.cpp` | 25-88 |
| Battery/ADC | `pinky_sensor_adc/src/main_node.cpp` | 42-95 |
| LED | `pinky_led_cpp/src/main_node.cpp` | 38-155 |
| Lamp | `pinky_lamp_control/src/main_node.cpp` | 12-229 |
| LCD | `pinky_emotion_cpp/src/main_node.cpp` | 30-198 |
| LiDAR SDK | `sllidar_ros2/sdk/` | Entire SDK |
| RL observation | `RL_Book/envs/pinky_sac_env.py` | 152-176 |
| RL action/PD | `RL_Book/envs/pinky_sac_env.py` | 250-268 |
| LiDAR preprocess | `RL_Book/envs/pinky_env.py` | 37-57 |
| ONNX export | `RL_Book/export_onnx.py` | 56-124 |
| SAC network | `RL_Book/agents/sac/sac_network.py` | 36-84 |
| Nav2 config | `pinky_navigation/params/` | All |
