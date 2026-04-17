# Phase 5: EKF 센서퓨전 구현 및 맵빌딩 제거 보고서

**작성일:** 2026-04-06
**브랜치:** dev_ZeroMQ
**상태:** 수정 완료

---

## 1. 개요

이번 세션에서는 세 가지 주요 작업을 수행하였다.

1. **맵빌딩 기능 제거** — Station GUI의 자체 Occupancy Grid 빌더를 전부 제거.  
   `pinky_pro`의 ROS SLAM(slam_toolbox)으로 맵을 생성한 후 Load Map으로 불러오는 방식으로 전환.
2. **EKF 센서퓨전 구현** — Complementary Filter를 제거하고 4-상태 Extended Kalman Filter로 교체.  
   IMU와 엔코더 각속도를 독립적인 측정값으로 융합.
3. **자율주행 개선** — RL 액션에 EMA 스무딩과 각속도 데드존 추가.

---

## 2. 맵빌딩 제거 (SLAM 마이그레이션)

### 2.1 배경 및 원인

커스텀 Occupancy Grid 빌더는 로봇이 회전할 때 원형 패턴만 생성되는 왜곡 문제가 있었다.  
근본 원인은 SLAM 없이 Dead-reckoning만으로 맵을 생성하는 구조적 한계:

- ZeroMQ odom/scan 타임스탬프 비동기 → 포즈·스캔 불일치
- theta 누적 드리프트 → 스캔 정합 실패
- 스캔 매칭(루프 클로저) 없음

**결론**: 신뢰할 수 있는 맵 생성에는 SLAM이 필수. `pinky_pro`의 slam_toolbox를 그대로 활용.

### 2.2 제거된 코드

| 파일 | 제거 항목 |
|------|----------|
| `toolbar.py` | `btn_map_build`, `btn_map_save`, `sig_map_build_toggle`, `sig_map_save`, `_on_map_build_toggled` |
| `main_window.py` | `OccupancyGridBuilder` import, `og_builder`, `map_building_active`, `_og_update_counter`, `_on_map_build_toggle`, `_on_map_save`, `_on_lidar_scan` 핸들러, 관련 signal 연결 |
| `map_widget.py` | `og_image`, `og_resolution`, `og_origin` 속성, `set_occupancy_grid()`, `clear_occupancy_grid()`, paintEvent OG 렌더링 블록 |

### 2.3 보존된 기능

- `btn_load_map` / `sig_load_map` / `_on_load_map` — 완성된 맵 파일(.yaml/.pgm) 로드
- `zmq_client.lidar_scan_received` → `nav_worker.on_lidar_scan_received` — ROS 2 브리지용 스캔 퍼블리시

---

## 3. EKF 센서퓨전 구현

### 3.1 기존 구현 문제

```
// 기존 Complementary Filter
current_w = alpha_ * imu_w + (1 - alpha_) * enc_w  // alpha=0.9 고정
theta_ += current_w * dt
x_ += v * dt * cos(theta_)
y_ += v * dt * sin(theta_)
```

문제점:
- 고정 alpha=0.9 (90% IMU, 10% 엔코더): 상황에 무관한 단순 혼합
- 공분산 추적 없음 → 불확실성 증가 감지 불가
- 바이어스 추정 없음
- 위치 오차 교정 불가

### 3.2 EKF 설계

**상태 벡터**: `s = [x, y, theta, w]`

| 인덱스 | 상태 | 단위 |
|--------|------|------|
| 0 | x (위치) | m |
| 1 | y (위치) | m |
| 2 | theta (방향각) | rad |
| 3 | w (각속도) | rad/s |

**예측 단계** (`Predict(v, w_enc, now)`)

운동 모델:
```
x_new     = x + v·cos(θ)·dt
y_new     = y + v·sin(θ)·dt
theta_new = theta + w·dt
w_new     = w          (constant-velocity model)
```

선형화 야코비안 F:
```
F = [[1, 0, -v·sin(θ)·dt,  0 ],
     [0, 1,  v·cos(θ)·dt,  0 ],
     [0, 0,  1,             dt],
     [0, 0,  0,             1 ]]
```

공분산 전파:
```
P = F·P·F^T + Q
```

**측정 업데이트** (각속도 측정 공통 모델)

측정 모델: `H = [0, 0, 0, 1]` (각속도만 관측)

칼만 이득:
```
S = P[3][3] + R
K = P[:,3] / S      // 4×1 벡터
x += K · (z - w)
P = (I - K·H) · P
```

| 측정 소스 | 메서드 | 노이즈 분산 R |
|----------|--------|------------|
| 엔코더 각속도 | `Predict()` 내부 | `R_enc = 0.10 (rad/s)²` |
| IMU 각속도 | `UpdateImu()` | `R_imu = 0.02 (rad/s)²` — 약 5배 신뢰도 높음 |

**프로세스 노이즈 Q (대각)**

| 항목 | 값 |
|------|-----|
| Q_x, Q_y | 1e-4 |
| Q_theta  | 1e-3 |
| Q_w      | 1e-2 |

### 3.3 퍼블릭 인터페이스 (변경 없음)

```cpp
// sensor_fusion.h — 인터페이스 유지, 내부 구현만 교체
bool Predict(double v, double w_enc, Timestamp now);
void UpdateImu(double imu_yaw_rate, Timestamp now);
void Reset(double x, double y, double theta);
Odometry GetState(Timestamp stamp) const;
```

`robot_app.cpp`의 호출부 수정 없음.

### 3.4 수정 파일

- `pinky_core/include/pinky_core/core/sensor_fusion.h` — 전면 재작성
- `pinky_core/src/core/sensor_fusion.cpp` — 전면 재작성 (Eigen 의존 없음, 4×4 행렬 직접 구현)

---

## 4. 자율주행 개선

### 4.1 EMA 스무딩 (RL 액션 채터링 억제)

SAC 모델은 매 스텝 독립적으로 추론하므로 프레임 간 급격한 제어값 변화(채터링)가 발생한다.  
RL 훈련 시 smoothness penalty가 없어 물리적으로 연속적이지 않은 출력이 나오는 것이 근본 원인.

**해결**: EMA(지수 이동 평균) 스무딩 적용 (α=0.4)

```cpp
constexpr float kEmaAlpha = 0.4f;
smoothed_rl_action_[0] = kEmaAlpha * raw[0] + (1.0f - kEmaAlpha) * smoothed_rl_action_[0];
smoothed_rl_action_[1] = kEmaAlpha * raw[1] + (1.0f - kEmaAlpha) * smoothed_rl_action_[1];
```

α=0.4 → 20Hz 기준 약 60% 이전 값 보존. 반응성과 부드러움의 균형.  
새 goal 수신 시 `smoothed_rl_action_ = {0, 0}` 으로 초기화.

### 4.2 각속도 데드존 (직진 시 미세 조향 억제)

직선 주행 중 RL이 소폭의 각속도를 계속 출력해 로봇이 좌우로 흔들리는 문제.

```cpp
constexpr float kAngularDeadzone = 0.05f;  // normalised action units
if (std::abs(smoothed_rl_action_[1]) < kAngularDeadzone) {
  smoothed_rl_action_[1] = 0.0f;
}
```

### 4.3 수정 파일

- `pinky_core/include/pinky_core/app/robot_app.h` — `smoothed_rl_action_[2]` 멤버 추가
- `pinky_core/src/app/robot_app.cpp` — EMA 스무딩 + 데드존, goal 수신 시 초기화

---

## 5. nav_worker.py 버그 수정

**증상**: ROS 2 미설치 환경에서 `NameError: name 'Twist' is not defined`  
**원인**: `RosBridgeNode` 클래스 정의 시점에 `Twist` 등의 타입 어노테이션이 즉시 평가됨  
**수정**: `from __future__ import annotations` 추가 → 모든 어노테이션 평가를 런타임으로 지연

---

## 6. 빌드 및 테스트 결과

```
cmake --build pinky_core/build
→ libpinky_core.a, pinky_robot, test_core: 성공

test_core
→ 155 passed, 0 failed
```

Python import 체인 검증:
```
from pinky_station.gui.main_window import PinkyStationWindow  → OK
from pinky_station.gui.widgets.toolbar import ToolbarWidget   → OK
from pinky_station.gui.widgets.map_widget import MapWidget    → OK
```

---

## 7. 변경 파일 목록

| 파일 | 변경 유형 |
|------|---------|
| `pinky_core/include/pinky_core/core/sensor_fusion.h` | EKF 재작성 |
| `pinky_core/src/core/sensor_fusion.cpp` | EKF 재작성 |
| `pinky_core/include/pinky_core/app/robot_app.h` | `smoothed_rl_action_` 추가 |
| `pinky_core/src/app/robot_app.cpp` | EMA 스무딩, 데드존, goal 초기화 |
| `pinky_core/src/inference/observation_builder.cpp` | obs[27] Clamp 추가 (이전 세션) |
| `pinky_station/pinky_station/gui/widgets/toolbar.py` | 맵빌딩 UI 제거 |
| `pinky_station/pinky_station/gui/main_window.py` | OG 빌더 및 핸들러 제거 |
| `pinky_station/pinky_station/gui/widgets/map_widget.py` | OG 오버레이 제거 |
| `pinky_station/pinky_station/core/occupancy_grid.py` | 미사용 (RGBA 변환만 남음, 차후 삭제 가능) |
| `pinky_station/pinky_station/workers/nav_worker.py` | `__future__ annotations` 버그 수정 |
