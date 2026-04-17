# Phase 4-8 2차 코드 리뷰 및 수정 보고서

> 작성일: 2026-04-01
> 대상: 1차 리뷰(2026-03-31) 이후 남은 미검증 영역 재검토
> 전제: 1차 리뷰에서 16건 수정 완료 상태 (docs/phase4_8_review_report.md 참조)

---

## 1. 검증 결과 요약

### 빌드/테스트 상태 (수정 후)

```
C++:   155 passed (test_core), 64 passed (test_protocol) — 0 failed
Python: 13 passed (test_protocol) — 0 failed
        2 failed (test_network_loopback) — 기존 API 불일치, 이번 수정과 무관
```

### 1차 수정 16건 재검증: 전부 정상 적용 확인

- Python 이중 프레이밍 수정 (#3,4,5): raw struct.pack 사용 확인
- struct 포맷 수정 (#6,7): `<ffB`, `<5f` 올바름
- TCP IP 추출 (#8): `inet_ntop()` 사용 확인
- HAL fd 해제 (#14,15): `close(fd_)` 있음
- 기타 전부 올바르게 반영됨

---

## 2. 신규 발견 및 수정 내역 (총 8건)

### CRITICAL (1건)

#### #1. OdometryAccumulator 라디안/틱 이중변환 — 오도메트리 완전 고장

**파일:** `odometry.h`, `odometry.cpp`, `robot_app.h`, `robot_app.cpp`, `test_core.cpp`

**문제:**
`DynamixelMotor::ReadJointState()`는 position을 **라디안**(double)으로 반환:
```cpp
// dynamixel_motor.cpp:149
js.position[0] = (static_cast<double>(pos_l) / pulses_per_rot_) * (2.0 * kPi);
```

그런데 `MotorOdomLoop`이 `static_cast<int32_t>`로 캐스팅하여 전달:
```cpp
// robot_app.cpp (수정 전)
current_odom_ = odom_calc_.Update(
    static_cast<int32_t>(js.position[0]),  // 0.17 rad → 0 !!
    static_cast<int32_t>(js.position[1]),
    Timestamp::Now());
```

`OdometryAccumulator::Update()`는 raw 엔코더 틱(int32_t)을 기대하여 내부에서 `/ pulses_per_rot * circumference` 변환을 한번 더 수행. 결과적으로 라디안 값(0.0~6.28)이 0 또는 1로 잘려서 **오도메트리가 사실상 동작하지 않음**.

**수정:**
- `OdometryAccumulator` 인터페이스를 `int32_t` tick → `double` radian으로 변경
- 내부 거리 계산: `dist = delta_rad * wheel_radius` (틱 변환 제거)
- `pulses_per_rot` 매개변수 제거
- `robot_app.cpp`에서 `static_cast<int32_t>` 제거, `js.stamp` 사용
- 테스트를 라디안 기반으로 업데이트 (4096 ticks → 2π rad)

```cpp
// odometry.h (수정 후)
Odometry Update(double left_rad, double right_rad, Timestamp now);

// robot_app.cpp (수정 후)
current_odom_ = odom_calc_.Update(
    js.position[0], js.position[1], js.stamp);
```

---

### MAJOR (2건)

#### #2. TcpServer ReadClient use-after-free

**파일:** `tcp_server.cpp`

**문제:**
`ReadClient()`가 `clients_mutex_` 해제 후 `recv_buffer`의 raw pointer를 계속 사용.
`ConnectionManager::WatcherLoop()`에서 `ForceDisconnect()` 호출 시 다른 스레드에서
`clients_` map entry가 삭제되어 **dangling pointer 접근** 가능.

```cpp
// 수정 전 — use-after-free 위험
std::vector<uint8_t>* client_buffer = nullptr;
{
    std::lock_guard<std::mutex> lock(clients_mutex_);
    client_buffer = &it->second.recv_buffer;  // raw pointer 획득
}
// mutex 해제됨 — ForceDisconnect가 entry 삭제 가능
client_buffer->insert(...);  // DANGER: dangling pointer
```

**수정:** 3-phase 패턴으로 구조 변경:
1. **Phase 1** (lock 없이): `read()` → 로컬 `incoming` 버퍼에 수집
2. **Phase 2** (lock 잡고): `incoming` → `recv_buffer` 추가 + 파싱 → `messages` 벡터 수집
3. **Phase 3** (lock 없이): 콜백 dispatch (deadlock 방지)

---

#### #3. TcpServer Send/Broadcast mutex 장시간 점유

**파일:** `tcp_server.cpp`

**문제:**
`Send()`와 `Broadcast()`가 `clients_mutex_`를 잡은 상태로 blocking `send()` + `usleep(1000)` 재시도 루프 실행. epoll 스레드가 stall되어 전체 서버 응답성 저하.

**수정:**
- `Send()`: lock 안에서 존재 확인만 하고, lock 해제 후 send 수행. EAGAIN 시 `usleep` 재시도 대신 즉시 실패 반환.
- `Broadcast()`: lock 안에서 fd 목록만 복사하고, lock 해제 후 각 fd에 send.

---

### MEDIUM (5건)

#### #4. BNO055 각속도 단위 오류 (dps → rad/s)

**파일:** `bno055_imu.cpp`

**문제:**
BNO055 센서는 1 LSB = 1/16 dps를 반환. `/ 16.0`으로 dps 변환만 되어 있었고, 시스템 나머지(RL 컨트롤러, Odom 등)는 rad/s를 기대.

**수정:** `* (π / 180.0)` 변환 추가.
```cpp
constexpr double kDpsToRads = 3.14159265358979323846 / 180.0;
data.angular_velocity.x = ... / 16.0 * kDpsToRads;
```

---

#### #5. OdometryAccumulator::Reset(x, y, theta) 오버로드 추가

**파일:** `odometry.h`, `odometry.cpp`, `robot_app.cpp`

**문제:**
기존 `Reset()`은 (0,0,0)으로만 초기화. 2D Pose Estimate 기능을 위해 임의 위치로 리셋 필요.

**수정:**
- `Reset(double x, double y, double theta)` 오버로드 추가
- `robot_app.cpp` kSetPose 핸들러에서 `DeserializeSetPose()` → `Reset(x, y, theta)` 호출

---

#### #6. LidarViewWidget 시그널 미연결

**파일:** `main_window.py`

**문제:**
`sensor_worker.sig_lidar`가 emit되지만 `LidarViewWidget`에 연결되지 않아 라이다 시각화 미동작.

**수정:**
- `_on_lidar_data()` 핸들러 추가: 24개 정규화 섹터를 언팩하고 `max_range(3.5m)` 곱하여 실제 거리로 변환
- `sig_lidar` → `_on_lidar_data` → `lidar_view.update_scan()` 연결

---

#### #7. SensorWorker 스레드 안전성

**파일:** `sensor_worker.py`

**문제:**
`run()` 메서드(QThread) 안에서 `self.receiver.on_message` 콜백을 교체. UDP receiver의 `_recv_loop` 스레드와 경합 가능.

**수정:**
- 콜백 교체를 `__init__()` (main thread)로 이동, `_dispatch()` 메서드로 분리
- import를 모듈 상단으로 이동하여 매 콜백마다 lazy import 제거

---

#### #8. MapWidget 로봇 위치 Y좌표 부호 오류 + 2D Pose 모드

**파일:** `map_widget.py`, `main_window.py`

**문제:**
로봇 위치 `ry_screen = cy + offset_y * scale`로 계산 — Y축 방향이 반전되어 위로 패닝 시 로봇이 아래로 이동.

**수정:**
- `ry_screen = cy - offset_y * scale`로 부호 수정
- `set_pose_mode()` 메서드 추가: pose 모드에서 좌클릭 시 `sig_set_pose` 발행
- `main_window._on_pose_mode_changed()`를 `map_view.set_pose_mode(active)` 호출로 구현

---

## 3. 변경 파일 목록

| 파일 | 변경 유형 | 관련 수정 |
|------|-----------|-----------|
| `pinky_core/include/pinky_core/core/odometry.h` | 인터페이스 변경 | #1, #5 |
| `pinky_core/src/core/odometry.cpp` | 구현 변경 | #1, #5 |
| `pinky_core/include/pinky_core/app/robot_app.h` | 생성자 인자 변경 | #1 |
| `pinky_core/src/app/robot_app.cpp` | Update/Reset 호출 수정 | #1, #5 |
| `pinky_core/tests/test_core.cpp` | 테스트 라디안 기반으로 변환 | #1 |
| `pinky_core/src/net/tcp_server.cpp` | ReadClient/Send/Broadcast 구조 변경 | #2, #3 |
| `pinky_core/src/hal/bno055_imu.cpp` | 각속도 단위 변환 추가 | #4 |
| `pinky_station/.../main_window.py` | 라이다 연결, pose mode 연동 | #6, #8 |
| `pinky_station/.../map_widget.py` | Y좌표 부호, pose mode 추가 | #8 |
| `pinky_station/.../sensor_worker.py` | 콜백 main thread 설치 | #7 |

---

## 4. 남은 미구현/미검증 항목

> 1차 보고서 섹션 3 기준, 이번 수정으로 해결된 항목은 제외

### 런타임 통합 테스트
- [ ] TCP/UDP 루프백 테스트 (`test_network_loopback.py` API 불일치 수정 필요)
- [ ] PyQt6 GUI 실행 테스트
- [ ] C++ ↔ Python 프로토콜 통합 테스트

### HAL (ARM64 전용)
- [ ] DynamixelMotor: `JointState.position`이 실제 하드웨어에서 올바른 라디안 값인지 검증
- [ ] SllidarDriver, Ws2811Led, Ili9341Lcd 드라이버 런타임 검증

### 기능 미구현
- [ ] YAML 설정 파싱 (C++ yaml-cpp 연동)
- [ ] MapWidget 경로 오버레이
- [ ] 카메라 스트리밍 (로봇 측 캡처 루프)
- [ ] LCD 감정 표시 (이미지 렌더링 로직)
- [ ] GUI 다크 테마 스타일시트
- [ ] Nav2 브리지 (NavWorker)
