# Pinky Robot Navigation System (C++)

Pinky Robot의 자율주행 및 원격 제어를 위한 핵심 C++ 코어 시스템과 Python GUI 스테이션 애플리케이션입니다. 강화학습(RL) 기반의 회피 주행 모델, EKF 기반 센서 퓨전(Sensor Fusion), 그리고 ZeroMQ와 Protobuf를 이용한 고성능 통신 아키텍처를 특징으로 합니다.

---

## 1. Architecture & Directory Structure

시스템은 로봇 하드웨어를 직접 제어하는 **코어(Core)**와 PC에서 원격으로 관제하는 **스테이션(Station)**으로 분리되어 동작합니다.

* **pinky_core (C++)**: 로봇 하드웨어 모터 제어, 다양한 센서(LiDAR, IMU, Camera 등) 데이터 수집 및 **EKF(Extended Kalman Filter) 기반 센서 퓨전**, ONNX Runtime을 이용한 RL 모델(SAC) 추론, 로컬 주행 제어 루프(NavLoop)를 담당합니다.
* **pinky_station (Python)**: PC에서 실행되는 GUI 애플리케이션입니다. 로봇 원격 관제, 2D 맵 기반의 글로벌 경로(Waypoint) 지정, ZeroMQ를 통한 `NAV_GOAL`/`NAV_CANCEL` 명령 전송 및 로봇 텔레메트리 수신을 담당합니다.
* **Protocol (Protobuf)**: 코어와 스테이션 간의 모든 데이터 교환(명령, 상태, 센서 데이터, 텔레메트리 등)은 **Protocol Buffers (Protobuf)**로 직렬화되어 ZeroMQ를 통해 고속으로 전송됩니다.

```text
pinky_cpp/
├── pinky_core/           # 로봇 제어 핵심 (C++17)
│   ├── config/           # 로봇 및 RL 주행 파라미터 (YAML)
│   ├── include/          # EKF 및 핵심 코어 헤더 파일
│   ├── models/           # ONNX 강화학습 모델 (sac_actor.onnx)
│   ├── proto/            # Protobuf 메시지 정의 (pinky.proto)
│   ├── src/              # 코어 소스코드 (app, core, hal, net)
│   └── CMakeLists.txt
├── pinky_station/        # PC 관제 스테이션 GUI (Python, PyQt5)
│   ├── gui/              # 메인 윈도우, 맵 위젯 등
│   ├── protocol/         # Protobuf 컴파일된 Python 코드
│   ├── workers/          # 백그라운드 통신 및 ROS2 Nav2 연동 워커
│   └── net/              # ZeroMQ 클라이언트
└── docs/                 # 주행 알고리즘 및 업데이트 문서
```

---

## 2. Dependencies & Installation

### [PC - pinky_station]
PC 환경은 Python 기반의 GUI 애플리케이션으로 구성됩니다.
* Python 3.x
* [uv](https://github.com/astral-sh/uv) (빠른 파이썬 패키지 관리자)
* 패키지 설치: 
  ```bash
  uv pip install -r pinky_station/requirements.txt
  ```
  *(PyQt5, PyZMQ, **protobuf**, numpy 등의 필수 패키지가 포함되어 있습니다.)*

### [Robot - pinky_core]
라즈베리파이 등 로봇 환경은 C++ 기반으로 하드웨어와 직접 통신합니다.
* C++17 호환 컴파일러 (GCC/Clang)
* CMake
* **ONNX Runtime (C++ API)**: 강화학습 모델 추론용
* **ZeroMQ**: `libzmq3-dev`, `cppzmq-dev`
* **Protobuf**: `protobuf-compiler`, `libprotobuf-dev` (메시지 직렬화/역직렬화)
* **로봇 센서 라이브러리**: SLLiDAR SDK, libcamera, wiringPi (GPIO 제어) 등 하드웨어 의존성 패키지

---

## 3. Execution Guide

### 1) PC 스테이션 실행 (pinky_station)
```bash
cd ~/ros2_ws/pinky_cpp
# 가상환경 활성화 (예: source .venv/bin/activate)
cd pinky_station
python3 -m pinky_station.main
```

### 2) 로봇 코어 빌드 및 실행 (pinky_core)
최초 실행이거나 소스코드를 클론한 직후라면 CMake 빌드가 필요합니다. `CMakeLists.txt` 내에 Protobuf 컴파일 구문이 포함되어 있으므로 `cmake` 과정에서 `.proto` 파일이 자동으로 C++ 코드로 변환됩니다.
```bash
cd ~/ros2_ws/pinky_cpp/pinky_core
mkdir -p build && cd build

# 기존 빌드 캐시 삭제 (빌드 오류 시 안전한 클린 빌드를 위해 권장)
rm -rf * 

# 빌드 및 실행
cmake ..
make -j4
./pinky_robot
```

---

## 4. GUI Usage & Communication

1. **통신 연결**: PC 스테이션 GUI 우측 상단 네트워크 패널에서 로봇의 IP 주소(예: `192.168.0.40`)를 입력하고 **Connect** 버튼을 누릅니다. (ZeroMQ 통신은 기본적으로 REP 9100, PUB 9200 포트를 사용합니다.)
2. **목표 지정**: 중앙의 2D Map 화면에서 마우스로 클릭하여 웨이포인트(목표 지점)들을 맵 상에 추가합니다.
3. **주행 시작**: 상단 툴바의 **Start Navigation** (재생 버튼)을 누르면 첫 번째 웨이포인트 좌표가 `NAV_GOAL` 메시지로 직렬화되어 로봇에 전송됩니다.
4. **연속 주행**: 로봇은 자체 `NavLoop`에 따라 자율 주행을 시작하며, 해당 지점에 도착하면 스테이션이 자동으로 다음 웨이포인트를 전송하여 연속 주행을 이어나갑니다. 주행 중 취소/정지 버튼으로 즉각적인 멈춤이 가능합니다.

---

## 5. Navigation & Sensor Algorithm

로봇 내부(`pinky_core`)는 확장 칼만 필터(EKF)를 이용한 정밀 측위와 3가지 동적 모드로 구성된 자율 주행 알고리즘을 사용합니다.

### 1) Sensor Fusion (EKF)
* **확장 칼만 필터 (Extended Kalman Filter)**: 로봇의 양륜 엔코더에서 나오는 추측 항법(Odometry) 데이터와 IMU(BNO055 등)의 방위각 및 가속도 데이터를 융합합니다. 
* 이를 통해 바퀴 슬립이나 센서 노이즈가 보정된 훨씬 정밀한 2D 로컬라이제이션(x, y, theta)을 수행하며, 정확한 현재 위치와 헤딩이 계산되어야 RL 기반 회피 제어가 정상적으로 이루어집니다.

### 2) 주행 제어 모드 (NavLoop 20Hz)
로봇은 목표를 향해 상황에 따라 다음 3가지 모드를 동적으로 자동 전환합니다.

1. **Turn-first 모드 (제자리 회전)**
   * 목표 방향과 로봇 헤딩의 각도 차이가 클 때 제자리 회전을 우선 수행합니다. 
   * 경계 부근에서 회전/전진을 반복하는 채터링을 방지하기 위해 **히스테리시스**가 적용되어 있습니다 (진입 105°, 탈출 80°).
2. **RL Controller 모드 (강화학습 메인 자율주행)**
   * **28차원 관측 벡터**: 라이다 24섹터 압축 값 + 거리/각도/진행도 정보를 구성하여 `sac_actor.onnx` 모델에 입력, 회피 및 주행 타겟 속도를 추론합니다.
   * **EMA Smoothing (지수이동평균)**: 이전 출력값과 현재값을 8:2 비율로 혼합하여 급발진과 진동을 억제합니다.
   * **PD Control**: RL 모델의 타겟 속도를 실제 모터가 부드럽게 추종할 수 있도록 최적화된 게인을 적용합니다.
   * **Safety Layer**: 전방 LiDAR 측정값을 기반으로 장애물이 0.25m 이내면 감속, 0.12m 이내면 비상 정지합니다.
3. **P-Ctrl Fallback 모드**
   * 센서 오류나 RL 모델 미적재 시 동작하는 비상/폴백 모드로, 단순 비례 제어를 통한 목표 지점 추종을 수행합니다.

---

## 6. Execution Log Example

다음은 로봇 구동 시 출력되는 실제 정상 동작 로그 예시입니다. 각도에 따른 Turn-first 전환, RL 모델의 평활화(smooth) 적용, PD 보정(pd_d), 그리고 도착 판정(Goal reached)이 명확히 확인됩니다.

```text
[NAV] Goal received: (-0.205963, -1.79437, 0)
[NAV] Navigation active goal=(-0.205963,-1.79437) has_lidar=1 has_onnx=1

# 1. 각도 차이가 107.4도로 커서 Turn-first 모드로 제자리 회전 돌입
[NAV] >> TURN-FIRST angle=107.484deg dist=0.544283m
[NAV] Turn-first: step=0 angle=107.484deg cmd=(0,0.8)

# 2. 회전하여 각도 차이가 79.8도로 줄어들자 RL 모드로 즉시 전환 (히스테리시스 정상 적용)
[NAV] >> RL dist=0.545094m angle=79.846deg
[RL] step=20 dist=0.541796m front=0.505m raw=[-0.493397,0.91345] smooth=[-0.27315,0.598617] target=(0.0944905,0.598617) cmd=(0.14102,0.695908) pd_d=(0.0465295,0.0972904)
[RL] step=40 dist=0.487322m front=0.281m raw=[-0.808744,0.492185] smooth=[-0.563929,0.626168] target=(0.0566892,0.626168) cmd=(0.0842382,0.570707) pd_d=(0.027549,-0.0554607)

# 3. 목표 도달 판정 (허용 오차 0.15m 이내 진입 시 자동 정지)
[NAV] Goal reached! dist=0.147766
[NAV] Navigation canceled
```