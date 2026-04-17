# Phase 4-8 코드 리뷰 및 수정 보고서

> 작성일: 2026-03-31
> 대상: 다른 AI가 작성한 Phase 4(네트워크) ~ Phase 8(통합) 코드
> Phase 1-3은 직접 작성 완료 (프로토콜, 코어 로직, ONNX 추론)

---

## 1. 빌드 결과

### C++ (pinky_core/)
```
cmake --build . -j$(nproc)  →  BUILD OK

[100%] Built target pinky_core      (정적 라이브러리)
[100%] Built target test_protocol   (프로토콜 테스트)
[100%] Built target test_core       (코어 로직 테스트)
[100%] Built target pinky_robot     (메인 실행파일)

test_protocol: 64 passed, 0 failed
test_core:    155 passed, 0 failed
```

- 플랫폼: x86_64 (PC), `BUILD_HAL=OFF` (ARM64 전용 HAL 비활성)
- ONNX Runtime 미설치 → 추론 모듈 조건부 컴파일로 스킵
- HAL 소스(hal/*.cpp)는 ARM64에서만 빌드됨 → PC에서 검증 불가

### Python (pinky_station/)
```
pytest tests/ -v  →  13 passed, 0 failed

test_crc16, test_odom_roundtrip, test_imu_roundtrip,
test_lidar24_roundtrip, test_battery_roundtrip, test_cmd_vel_roundtrip,
test_nav_goal_roundtrip, test_debug_log_roundtrip, test_bad_magic,
test_incomplete, test_multiple_messages, test_sequence_increment,
test_cross_language_crc
```

- PyQt6 GUI 위젯은 런타임 테스트 필요 (단위 테스트 미작성)

---

## 2. 수정 완료 내역 (총 16건)

### CRITICAL (8건)

| # | 파일 | 문제 | 수정 |
|---|------|------|------|
| 1 | `pinky_station/protocol/serializer.py` | `Deserializer` 클래스 미존재 (tcp_client, udp_receiver가 import) | `parse_message()` 래핑하는 `Deserializer` 클래스 추가 |
| 2 | `pinky_station/protocol/message_types.py` | `mt.MSG_PING` 등 모듈 레벨 상수 미존재 (`MsgType.PING`만 있음) | 24개 `MSG_*` 별칭 추가 |
| 3 | `pinky_station/net/tcp_client.py` | `serialize_ping()` → 이미 프레임된 데이터를 `send_message()`가 다시 프레임 (이중 프레이밍) | raw `struct.pack('<Q', ts)` 전달로 변경 |
| 4 | `pinky_station/net/connection.py` | `serialize_cmd_vel()` 이중 프레이밍 동일 문제 | raw `struct.pack('<ff', ...)` 전달로 변경 |
| 5 | `pinky_station/workers/command_worker.py` | `send_cmd_vel`, `send_nav_goal`, `set_pose` 3개 메서드 모두 이중 프레이밍 | 모두 raw struct.pack 전달로 변경 |
| 6 | `pinky_station/gui/widgets/battery_widget.py` | `'<QffB'` 형식 (17바이트) 사용 — C++은 `'<ffB'` (9바이트, 타임스탬프 없음) | `'<ffB'`로 수정 |
| 7 | `pinky_station/gui/widgets/map_widget.py` | `'<Qddddd'` 형식 (48바이트, double) — C++은 `'<5f'` (20바이트, float) | `'<5f'`로 수정 |
| 8 | `pinky_core/src/net/tcp_server.cpp` | 클라이언트 IP를 `"127.0.0.1"` 하드코딩 → UDP가 localhost로만 전송 | `accept()` 후 `inet_ntop()` 사용, 콜백 시그니처에 IP 추가 |

### MAJOR (7건)

| # | 파일 | 문제 | 수정 |
|---|------|------|------|
| 9 | `pinky_station/workers/camera_worker.py` | JPEG 오프셋 `payload[4:]` — 실제 카메라 헤더 `<HHI` = 8바이트 | `payload[8:]`로 수정 |
| 10 | `pinky_station/gui/main_window.py` | `mt.MSG_LOG` 참조 — 존재하지 않는 이름 | `mt.MSG_DEBUG_LOG`로 수정 |
| 11 | `pinky_station/gui/widgets/terminal_widget.py` | 디버그 로그 파싱: `str_len` 필드 파싱 시도 — C++은 null-terminated 문자열 | `payload[9:].split(b"\x00", 1)[0]`으로 수정 |
| 12 | `pinky_core/include/.../app/robot_app.h` | `OdometryCalculator` (미존재), 생성자 인자 누락, OnnxActor 가드 없음 | `OdometryAccumulator`로 변경, 상수 기본값 추가, `#ifdef` 가드 |
| 13 | `pinky_core/src/app/robot_app.cpp` | Build() 인자 5→3, Reset() 인자 3→0, Timestamp 타입, RlController API 불일치 등 7개 | 전부 수정. RL 제어 흐름: LidarLoop에서 PD 적용, MotorLoop은 단순 적용 |
| 14 | `pinky_core/src/hal/bno055_imu.cpp` | 소멸자에서 I2C fd 미해제 (리소스 누수) | `close(fd_)` 추가 |
| 15 | `pinky_core/src/hal/adc_sensor.cpp` | 동일 — I2C fd 미해제 | `close(fd_)` 추가 |

### MEDIUM/MINOR (2건)

| # | 파일 | 문제 | 수정 |
|---|------|------|------|
| 16 | `pinky_core/src/net/connection_manager.cpp` | ping timeout 시 `OnClientDisconnected()` 직접 호출 → TcpServer와 상태 불일치 | `TcpServer::ForceDisconnect()` 추가, ConnectionManager가 이를 호출 |
| 17 | `pinky_core/src/net/udp_server.cpp` | sendto 실패 시 에러 로그 없음 | `strerror(errno)` 로깅 추가 |

### 추가 생성 파일

| 파일 | 내용 |
|------|------|
| `pinky_core/config/robot_config.yaml` | 하드웨어 포트, 센서 주소, 물리 상수, 센서 주기 |
| `pinky_core/config/rl_config.yaml` | ONNX 모델 경로, 관측/행동 매핑, PD 제어 상수 |
| `pinky_station/config/station_config.yaml` | 연결 설정, GUI 기본값, 텔레옵 속도 |

---

## 3. 아직 확인/진행 필요한 사항

### 3-1. 런타임 통합 테스트 (최우선)

- [ ] **TCP/UDP 루프백 테스트**: PC에서 TcpServer + UdpServer 기동 → Python 클라이언트로 연결 → 메시지 왕복 검증
  - 현재 네트워크 코드는 컴파일만 됨, 실제 소켓 통신 미검증
  - `test_network_loopback.py` 또는 `test_network_loopback.cpp` 작성 필요

- [ ] **PyQt6 GUI 실행 테스트**: `python -m pinky_station` 실행하여 윈도우 뜨는지, 위젯 배치 확인
  - PyQt6가 uv 가상환경에 설치되어 있는지 확인 필요
  - `__main__.py`, `app.py` 진입점 검증

- [ ] **C++ ↔ Python 프로토콜 통합 테스트**: C++ Serializer로 만든 프레임 → Python Deserializer로 파싱 (양방향)
  - 특히 battery(9B), odom(20B), camera_frame, debug_log 메시지

### 3-2. C++ 코드 미검증 영역

- [ ] **HAL 드라이버 (ARM64 전용)**
  - `dynamixel_motor.cpp` — DynamixelSDK 연동, 속도 모드 설정/읽기
  - `sllidar_driver.cpp` — RPLiDAR SDK 래핑, DenseBoost 스캔
  - `ws2811_led.cpp` — WS2811 LED/Lamp 제어
  - `ili9341_lcd.cpp` — SPI LCD 드라이버
  - → RPi5에서만 빌드/테스트 가능, 각 HAL 헤더 파일의 인터페이스와 실제 구현의 일치 여부 검증 필요

- [ ] **robot_app.cpp 런타임 흐름**
  - `MotorOdomLoop`: JointState의 `position`이 int32 엔코더 틱인지 double 라디안인지 — HAL 구현에 따라 `static_cast<int32_t>` 변환이 올바른지 확인 필요
  - `ImuLoop`, `AdcLoop`: Serializer가 `SerializeImu(ImuData)` 등을 호출하는데, 이 반환값이 프레임된 메시지인지 raw payload인지 확인 필요 (현재 코드는 `SerializeOdom()` → `Frame()` 순서로 이중 프레이밍 가능성 있음)
  - `LidarLoop`: OnnxActor 추론 후 `RlController::Compute()` 호출 흐름 검증

- [ ] **robot_app.cpp의 Serializer 사용 패턴 검증**
  - Line 173-175: `serializer_->SerializeOdom()` 후 `serializer_->Frame()` 호출 — C++ Serializer의 `SerializeOdom()`이 raw payload를 반환하는지, 이미 프레임된 데이터를 반환하는지 확인 필요
  - `src/protocol/serializer.cpp`의 `SerializeOdom()` 구현 읽고 확인할 것

- [ ] **OdometryAccumulator Reset with Pose**: 현재 `Reset()`은 인자 없이 (0,0,0) 초기화. 2D Pose Estimate 기능을 위해 `Reset(x, y, theta)` 오버로드 필요

### 3-3. Python 코드 미검증 영역

- [ ] **GUI 위젯 시그널/슬롯 연결**
  - `TeleopWidget.sig_cmd_vel` — 시그니처가 `(float, float)`인지 확인
  - `MapWidget.sig_set_goal`, `sig_set_pose` — 연결 대상 `CommandWorker` 메서드와 시그니처 일치 확인
  - `ToolbarWidget.sig_connect_toggled`, `sig_set_pose_mode` — 시그니처 확인

- [ ] **Worker 스레드 안전성**
  - `SensorWorker`, `CameraWorker`: UDP/TCP on_message 콜백을 QThread에서 덮어쓰는 패턴 — 원래 콜백과의 레이스 조건 가능성
  - `CommandWorker`: queue 기반이라 비교적 안전하나, 클라이언트 연결 해제 시 큐 잔여 메시지 처리 필요

- [ ] **LidarViewWidget**: pyqtgraph 의존성 확인, 데이터 연결 누락 (sensor_worker의 sig_lidar가 연결 안 됨)

- [ ] **VideoViewWidget**: JPEG → QImage 변환 로직 확인

- [ ] **2D Pose Estimate 기능**: `main_window._on_pose_mode_changed()` 구현이 `pass` — MapWidget과의 연동 미구현

### 3-4. 의존성/환경

- [ ] uv 가상환경에 PyQt6, pyqtgraph 설치 확인
- [ ] RPi5에서 빌드 시 필요한 라이브러리 설치 확인:
  - DynamixelSDK (`/opt/ros/jazzy/lib/libdynamixel_sdk.so`)
  - WiringPi (`pinky_devices/WiringPi/`)
  - rpi_ws281x (`pinky_devices/rpi_ws281x/`)
  - RPLiDAR SDK (`pinky_devices/rplidar_sdk/`)
  - ONNX Runtime aarch64 (`/opt/onnxruntime/`)

### 3-5. 기능 미구현 목록

| 기능 | 상태 | 비고 |
|------|------|------|
| TCP/UDP 루프백 통합 테스트 | 미작성 | |
| YAML 설정 파싱 (C++) | 미구현 | `main.cpp`는 `--mock` 플래그만 처리 |
| 2D Pose Estimate (GUI) | 미구현 | `_on_pose_mode_changed()` = pass |
| MapWidget 경로 오버레이 | 미구현 | 계획서에는 있으나 코드 없음 |
| Nav2 브리지 (선택) | 미구현 | NavWorker 파일 자체 미존재 |
| 카메라 스트리밍 (로봇) | 미구현 | FrameSender는 있으나 카메라 캡처 루프 없음 |
| LCD 감정 표시 | 미구현 | HAL만 있고 이미지 렌더링 로직 없음 |
| GUI 다크 테마 스타일시트 | 미적용 | `resources/styles/dark.qss` 미존재 |

---

## 4. 파일 구조 현황

```
pinky_core/
  CMakeLists.txt                     ✅ 빌드 OK
  config/
    robot_config.yaml                ✅ 신규 생성
    rl_config.yaml                   ✅ 신규 생성
  include/pinky_core/
    common/  types.h, constants.h, logger.h
    protocol/  message_types.h, serializer.h, checksum.h
    core/  diff_drive.h, odometry.h, lidar_processor.h, battery_monitor.h, led_controller.h
    inference/  onnx_actor.h, observation_builder.h, rl_controller.h
    net/  tcp_server.h, udp_server.h, connection_manager.h, frame_sender.h
    hal/  interfaces.h, dynamixel_motor.h, sllidar_driver.h, bno055_imu.h, adc_sensor.h, ws2811_led.h, ili9341_lcd.h
    app/  robot_app.h
  src/  (모든 .cpp 구현)
  tests/
    test_protocol.cpp                ✅ 64 pass
    test_core.cpp                    ✅ 155 pass

pinky_station/
  config/
    station_config.yaml              ✅ 신규 생성
  pinky_station/
    protocol/  message_types.py, serializer.py, checksum.py
    net/  tcp_client.py, udp_receiver.py, connection.py
    gui/
      main_window.py
      widgets/  toolbar, lidar_view, video_view, teleop_widget, battery_widget, terminal_widget, map_widget
    workers/  sensor_worker.py, camera_worker.py, command_worker.py
  tests/
    test_protocol.py                 ✅ 13 pass
```

---

## 5. 다음 세션에서 권장 진행 순서

1. **TCP/UDP 루프백 통합 테스트 작성** — PC에서 서버 기동 + Python 클라이언트 연결하여 양방향 메시지 검증
2. **robot_app.cpp Serializer 이중 프레이밍 확인** — `SerializeOdom()` 반환값이 raw payload인지 확인 후 필요 시 수정
3. **OdometryAccumulator::Reset(x, y, theta)** 오버로드 추가
4. **PyQt6 GUI 실행 테스트** — uv 환경에서 실제 윈도우 띄워보기
5. **2D Pose Estimate 구현** — MapWidget 마우스 드래그 → SET_POSE 전송
6. **YAML 설정 파싱** — yaml-cpp 연동 또는 간단한 파서
7. **ARM64 빌드 테스트** — RPi5에서 HAL 포함 전체 빌드
