# SAC RL 모델 ONNX 배포 가이드

> 작성: 2026-03-31
> 대상 모델: `results/pinky_sac/models/sac_PinkyPro-v0_2026-03-30_15-55-22/225280/network.th`
> 상태: ONNX 내보내기 완료, 검증 통과

---

## 1. 개요

학습된 SAC 강화학습 모델(PyTorch)을 C++ 로봇(Pinky Pro, RPi5 ARM64)에서
실행하기 위해 ONNX 포맷으로 변환하고, 로봇 측에서 필요한 코드 구조를 정의한다.

**핵심 철학**: RL 모델 자체는 map-less.
LiDAR + 상대적 목표 정보만 사용. Nav2/TF2는 "어디로 갈지"만 알려주고,
"어떻게 갈지"는 RL이 결정.

---

## 2. 전체 아키텍처

```
[PC 관제탑 - PyQt6 GUI]
  │
  │  ROS 2 통신
  ▼
[로봇 본체 - RPi5 ARM64, 순수 C++]
  │
  ├── Nav2 Global Planner ──(/plan)──┐
  │                                   ▼
  ├── /scan (LiDAR C1) ──────> RlInferenceNode ──(/cmd_vel)──> Dynamixel 모터
  │                                   │
  └── TF2 (map→base_link) ──> 상대 거리/각도 계산
                                      │
                                 ONNX Runtime
                                 (SAC Actor)
                                      │
                                28D obs → 2D action
```

---

## 3. ONNX 내보내기 (완료)

### 3-1. 실행 결과

```bash
.venv/bin/python RL_Book/export_onnx.py \
  --checkpoint RL_Book/results/pinky_sac/models/sac_PinkyPro-v0_2026-03-30_15-55-22/225280/network.th \
  --output RL_Book/models/sac_actor.onnx
```

| 항목 | 결과 |
|------|------|
| 출력 파일 | `RL_Book/models/sac_actor.onnx` |
| 파일 크기 | 290KB (단일 파일, 가중치 내장) |
| 총 파라미터 | 74,244 |
| 입력 shape | `(1, 28)` float32 (이름: `state`) |
| 출력 shape | `(1, 2)` float32 (이름: `action`) |
| ONNX 구조 검증 | 통과 |
| PyTorch vs ORT 수치 비교 | 100회 통과 (최대 오차: 3.40e-06) |

### 3-2. 재내보내기 (다른 체크포인트 사용 시)

```bash
.venv/bin/python RL_Book/export_onnx.py \
  --checkpoint <새 체크포인트의 network.th 경로> \
  --output <출력 경로.onnx> \
  --state-dim 28 --action-dim 2 --hidden-dim 256
```

필요 패키지: `requirements.txt` 참조 (torch, onnx, onnxruntime, onnxscript)

---

## 4. 모델 사양 (로봇 구현 필수 참조)

### 4-1. 네트워크 구조

```
Input: state (28D float32)
  │
  ├── Linear(28, 256) + ReLU
  ├── Linear(256, 256) + ReLU
  └── mean_head: Linear(256, 2) → tanh()

Output: action (2D float32) ∈ [-1, 1]
```

추론 시 Critic, Target Q, log_std_head는 모두 불필요. ONNX 모델에 포함되지 않음.

### 4-2. 관측 벡터 (28D)

| 인덱스 | 내용 | 범위 | 계산 |
|--------|------|------|------|
| 0~23 | LiDAR 24섹터 | [0, 1] | 구간별 min / 3.5m |
| 24 | 목표 거리 | [0, 1] | clip(distance / 5.0, 0, 1) |
| 25 | 목표 방향 cos | [-1, 1] | cos(goal_angle) |
| 26 | 목표 방향 sin | [-1, 1] | sin(goal_angle) |
| 27 | 진행률 | [0, 1] | current_step / 750 |

### 4-3. 액션 매핑

| 모델 출력 | 매핑 공식 | 물리 범위 |
|-----------|-----------|-----------|
| action[0] ∈ [-1, 1] | `v = (a[0]+1)/2 * 0.26` | [0.0, 0.26] m/s |
| action[1] ∈ [-1, 1] | `w = a[1] * 1.0` | [-1.0, 1.0] rad/s |

### 4-4. PD 제어 게인

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| KP_V | 1.5 | 선속도 비례 게인 |
| KD_V | 0.3 | 선속도 미분 게인 |
| KP_W | 1.5 | 각속도 비례 게인 |
| KD_W | 0.3 | 각속도 미분 게인 |
| 제어 주기 | 50ms (20Hz) | 학습 시 time.sleep(0.05) |

---

## 5. LiDAR 전처리 (반드시 Python과 동일해야 함)

원본 코드: `envs/pinky_env.py:37-57`

### 5-1. 처리 단계

```
1. Clean
   - NaN → 0.0
   - +Inf → 3.5 (MAX_LIDAR_DIST)
   - -Inf → 0.0

2. Roll (전방 중앙 정렬)
   - 배열을 n/2만큼 우측 이동
   - Python: np.roll(ranges, n // 2)
   - C++:    std::rotate(v.begin(), v.end() - n/2, v.end())

3. Sector binning (24구간)
   - sector_size = n / 24  (정수 나눗셈)
   - 각 구간의 최솟값(min) 추출
   - 마지막 섹터(i=23)는 남은 요소 전부 포함
   - C++ 의사코드:
     for (int i = 0; i < 24; i++) {
       int start = i * sector_size;
       int end = (i < 23) ? start + sector_size : n;
       sectors[i] = *std::min_element(v.begin() + start, v.begin() + end);
     }

4. Normalize
   - 각 값을 3.5로 나눔 → [0, 1] 범위
```

### 5-2. 주의사항

- 전처리 수치가 학습 환경과 한 자리라도 다르면 로봇이 엉뚱하게 움직인다
- RPLiDAR C1의 실제 스캔 포인트 수(n)는 Gazebo(640)와 다를 수 있음
  → sector_size = n / 24 로 동적 계산해야 함
- 디버그 시: 28D 관측 벡터를 로그로 출력하여 Python 환경과 비교 검증

---

## 6. 로봇에 필요한 파일/라이브러리 목록

### 6-1. 모델 파일 (PC에서 복사)

| 파일 | 위치 (이 레포 기준) | 크기 |
|------|---------------------|------|
| `sac_actor.onnx` | `RL_Book/models/sac_actor.onnx` | 290KB |

### 6-2. ONNX Runtime (ARM64용)

```bash
# RPi5 (aarch64)에 설치
wget https://github.com/microsoft/onnxruntime/releases/download/v1.17.1/onnxruntime-linux-aarch64-1.17.1.tgz
tar -xzf onnxruntime-linux-aarch64-1.17.1.tgz
```

| 항목 | 경로 |
|------|------|
| 헤더 | `include/onnxruntime_cxx_api.h` |
| 라이브러리 | `lib/libonnxruntime.so` |

CMakeLists.txt 연동 예시:
```cmake
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS /opt/onnxruntime/include)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  PATHS /opt/onnxruntime/lib)

target_include_directories(rl_inference_node PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
target_link_libraries(rl_inference_node ${ONNXRUNTIME_LIBRARY})
```

### 6-3. C++ 코드 구성 요소

| 컴포넌트 | 역할 | ROS 의존성 |
|----------|------|------------|
| `OnnxActor` | ONNX 모델 로드 + 추론 | onnxruntime만 |
| `LidarPreprocessor` | LaserScan → 24D 정규화 | 없음 (순수 C++) |
| `RlInferenceNode` | ROS 2 노드: 센서 구독, 추론, cmd_vel 발행 | rclcpp, sensor_msgs, nav_msgs, geometry_msgs, tf2_ros |

### 6-4. ROS 2 토픽 인터페이스

**Subscribe**:

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/scan` | sensor_msgs::msg::LaserScan | LiDAR 원시 데이터 |
| `/plan` | nav_msgs::msg::Path | Nav2 글로벌 경로 |

**Publish**:

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/cmd_vel` | geometry_msgs::msg::Twist | 모터 속도 명령 |

**TF2**: `map → base_link` 변환 조회 (로봇 월드 좌표 + yaw)

---

## 7. C++ OnnxActor 구현 가이드

### 7-1. 핵심 API

```cpp
#include <onnxruntime_cxx_api.h>
#include <array>
#include <string>

class OnnxActor {
 public:
  explicit OnnxActor(const std::string& model_path)
    : env_(ORT_LOGGING_LEVEL_WARNING, "RlInference"),
      session_(env_, model_path.c_str(), Ort::SessionOptions()) {
    // SessionOptions: intra_op_num_threads = 1 (소형 네트워크)
    // 생성 시 입출력 shape 검증: input {1,28}, output {1,2}
  }

  std::array<float, 2> Infer(const std::array<float, 28>& state) {
    // 1. 입력 텐서 생성 (zero-copy)
    auto memory_info = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator, OrtMemTypeDefault);
    std::array<int64_t, 2> input_shape = {1, 28};
    auto input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, const_cast<float*>(state.data()), 28,
        input_shape.data(), 2);

    // 2. 추론 실행
    const char* input_names[] = {"state"};
    const char* output_names[] = {"action"};
    auto output = session_.Run(
        Ort::RunOptions{nullptr},
        input_names, &input_tensor, 1,
        output_names, 1);

    // 3. 결과 복사
    const float* data = output[0].GetTensorData<float>();
    return {data[0], data[1]};
  }

 private:
  Ort::Env env_;
  Ort::Session session_;
};
```

### 7-2. 성능 참고

- 네트워크 크기: 74,244 파라미터 (매우 작음)
- RPi5에서 추론 시간: 약 0.1~0.5ms 예상 (제어 주기 50ms 대비 여유)
- `intra_op_num_threads = 1`로 설정해도 충분

---

## 8. 제어 루프 의사코드 (로봇 구현 참조)

```
매 50ms (20Hz):
  1. if (scan 미수신 or plan 미수신):
       cmd_vel = {0, 0}  // 정지
       return

  2. (x, y, yaw) = TF2.lookup("map" → "base_link")

  3. (goal_x, goal_y) = plan에서 lookahead 지점 추출
     - 로봇에서 가장 가까운 경로 지점 찾기
     - 거기서 lookahead_dist(0.5m)만큼 전방의 경로 지점 선택

  4. distance = sqrt((goal_x-x)^2 + (goal_y-y)^2)
     goal_angle = normalize(atan2(goal_y-y, goal_x-x) - yaw)
     // normalize: (angle + pi) % (2*pi) - pi → [-pi, pi]

  5. observation[28] 구성:
     [0:24]  = LidarPreprocessor.Process(scan.ranges)
     [24]    = clip(distance / 5.0, 0, 1)
     [25]    = cos(goal_angle)
     [26]    = sin(goal_angle)
     [27]    = current_step / 750

  6. action[2] = OnnxActor.Infer(observation)

  7. target_v = (action[0] + 1) / 2 * 0.26
     target_w = action[1] * 1.0

  8. PD 제어:
     error_v = target_v - current_v    // current_v: /odom에서 얻은 현재 선속도
     error_w = target_w - current_w    // current_w: /odom에서 얻은 현재 각속도
     cmd_v = target_v + 1.5*error_v + 0.3*(error_v - prev_error_v)
     cmd_w = target_w + 1.5*error_w + 0.3*(error_w - prev_error_w)
     prev_error_v = error_v
     prev_error_w = error_w
     cmd_v = clip(cmd_v, 0.0, 0.26)
     cmd_w = clip(cmd_w, -1.0, 1.0)

  9. publish cmd_vel = {linear.x: cmd_v, angular.z: cmd_w}

  10. current_step++
      if distance < 0.05: 목표 도달 → 정지, 다음 경로 구간으로
      if current_step >= 750: 타임아웃 → 정지
```

---

## 9. 파라미터 요약

```yaml
rl_inference_node:
  ros__parameters:
    model_path: "/path/to/sac_actor.onnx"
    control_rate: 20.0      # Hz (50ms 주기)
    max_steps: 750           # 에피소드 최대 스텝 (학습과 동일)
    v_min: 0.0               # 최소 선속도 (m/s)
    v_max: 0.26              # 최대 선속도 (m/s)
    w_max: 1.0               # 최대 각속도 (rad/s)
    kp_v: 1.5                # 선속도 비례 게인
    kd_v: 0.3                # 선속도 미분 게인
    kp_w: 1.5                # 각속도 비례 게인
    kd_w: 0.3                # 각속도 미분 게인
    lookahead_dist: 0.5      # Nav2 경로에서 전방 추출 거리 (m)
    goal_tolerance: 0.05     # 목표 도달 판정 거리 (m)
    max_lidar_dist: 3.5      # LiDAR 최대 거리 (정규화 기준)
    goal_distance_scale: 5.0 # 목표 거리 정규화 분모
```

---

## 10. 핵심 참조 파일 (이 레포 기준)

| 파일 | 용도 |
|------|------|
| `RL_Book/export_onnx.py` | ONNX 내보내기 스크립트 |
| `RL_Book/models/sac_actor.onnx` | 내보낸 ONNX 모델 (290KB) |
| `RL_Book/agents/sac/sac_network.py` | Actor 네트워크 구조, state_dict 키 |
| `RL_Book/envs/pinky_sac_env.py:152-176` | 관측 벡터 구성 (28D) |
| `RL_Book/envs/pinky_sac_env.py:250-268` | 액션 매핑 + PD 제어 |
| `RL_Book/envs/pinky_env.py:37-57` | LiDAR 전처리 로직 |
| `RL_Book/config/agents/sac/PinkyPro-v0.yaml` | 학습 하이퍼파라미터 |

---

## 11. 로봇 하드웨어 참고

| 항목 | 사양 |
|------|------|
| 컴퓨터 | Raspberry Pi 5 (16GB) |
| OS | Ubuntu 24.04, ROS 2 Jazzy |
| LiDAR | RPLiDAR C1 (DenseBoost 모드, /dev/ttyAMA0) |
| 모터 | Dynamixel (프로토콜 2.0, /dev/ttyAMA4) |
| 바퀴 반경 | 0.028m |
| 바퀴 간격 | 0.0961m |
| 최대 RPM | 100 |
