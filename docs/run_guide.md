# Pinky Robot & Station 실행 가이드

> **작성일:** 2026-04-01
> **목적:** 구현된 C++ 로봇 코어 프로그램(`pinky_robot`)과 파이썬 관제탑 GUI(`pinky_station`)의 실행 방법 및 옵션 안내

---

## 1. 사전 준비 (환경 설정)

프로그램 및 테스트를 실행하기 전, 파이썬 가상환경(uv)과 ROS 2 환경을 활성화해야 합니다.

### 1.1. 파이썬 가상환경 활성화
`pinky_station` 및 테스트 스크립트는 가상환경 내의 패키지(`PyQt6`, `pytest` 등)를 사용하므로 반드시 활성화해야 합니다.
```bash
cd ~/ros2_ws/pinky_cpp
source .venv/bin/activate
```

### 1.2. ROS 2 환경 로드 (선택 사항)
`pinky_station`의 `NavWorker` (ROS 2 Nav2 브릿지) 기능을 사용하려면 ROS 2 환경이 소싱되어 있어야 합니다. 소싱되지 않은 경우 해당 기능만 비활성화되고 GUI 자체는 정상적으로 작동합니다.
```bash
source /opt/ros/jazzy/setup.bash
```

---

## 2. C++ 로봇 코어 실행 (`pinky_robot`)

로봇 코어 프로그램은 센서를 읽고, 모터를 제어하며, 강화학습 추론을 수행하는 C++ 메인 프로세스입니다.

### 2.1. 빌드 확인
실행 전 빌드가 완료되어 있는지 확인합니다.
```bash
cd ~/ros2_ws/pinky_cpp/pinky_core/build
cmake --build . -j$(nproc)
```

### 2.2. 실행 명령어 및 옵션
```bash
./pinky_robot [옵션]
```

**주요 옵션:**
* `--mock`: 하드웨어(HAL) 기능을 비활성화하고 가상 모드(PC 모드)로 실행합니다. 라즈베리파이 등 실제 하드웨어가 연결되지 않은 PC 환경에서는 이 옵션을 반드시 사용해야 합니다.
* `--config <path>`: 로봇 하드웨어 및 네트워크 설정이 담긴 YAML 파일 경로를 지정합니다. (예: `../config/robot_config.yaml`)
* `--rl-config <path>`: 강화학습 모델 및 PD 제어 설정이 담긴 YAML 파일 경로를 지정합니다. (예: `../config/rl_config.yaml`)

**실행 예시 (PC 환경 가상 테스트):**
```bash
cd ~/ros2_ws/pinky_cpp/pinky_core/build
./pinky_robot --config ../config/robot_config.yaml --rl-config ../config/rl_config.yaml --mock
```

---

## 3. 파이썬 관제탑 GUI 실행 (`pinky_station`)

로봇과 통신하여 원격 제어(Teleop), 센서 데이터 시각화(Map, Lidar, Camera), 디버그 로그 및 배터리 상태를 모니터링하는 PyQt6 기반 GUI 프로그램입니다.

### 3.1. GUI 실행 명령어
가상환경이 활성화된 상태에서 아래 명령어를 실행합니다.
```bash
cd ~/ros2_ws/pinky_cpp
python3 -m pinky_station.pinky_station.main
```
> **참고:** C++ 로봇 코어가 먼저 실행되어 있어야 `Connect` 버튼을 눌렀을 때 정상적으로 연결됩니다.

### 3.2. 설정 파일 적용 (`station_config.yaml`)
실행 시 프로그램은 자동으로 프로젝트 내부(`config/station_config.yaml`) 또는 사용자 홈(`~/.pinky/station_config.yaml`)에서 설정을 로드합니다. 명시적으로 지정하려면 `--config` 인자를 사용합니다.
```bash
python3 -m pinky_station.pinky_station.main --config config/station_config.yaml
```

---

## 4. 통합 연동 테스트 (로컬 통신)

개발 PC 한 대에서 로봇과 관제탑을 동시에 띄우고 통신하는 방법입니다.

1. **터미널 1 (로봇 코어 가동):**
   ```bash
   cd ~/ros2_ws/pinky_cpp/pinky_core/build
   ./pinky_robot --mock
   ```
2. **터미널 2 (관제탑 GUI 가동):**
   ```bash
   cd ~/ros2_ws/pinky_cpp
   source .venv/bin/activate
   python3 -m pinky_station.pinky_station.main
   ```
3. **연결:**
   GUI 상단 툴바에 기본값(`127.0.0.1`)이 입력되어 있습니다. `Connect` 버튼을 누르면:
   * 상태 텍스트가 붉은색 `DISCONNECTED`에서 녹색 `CONNECTED`로 변경됩니다.
   * 로봇(터미널 1) 측에서 클라이언트 연결 성공 로그가 출력됩니다.
   * 하단 터미널 위젯에 연결 완료 메시지가 표시되며 주기적으로 Odom 등의 센서 값이 업데이트되기 시작합니다.

---

## 5. 자동화 테스트 실행 (Unit & Integration Tests)

코드 수정 후 로직의 결함 유무를 확인하려면 `pytest`를 이용해 자동화 테스트를 실행할 수 있습니다.

### 5.1. C++ 코어 테스트
프로토콜 파서 및 핵심 데이터 구조를 검증합니다.
```bash
cd ~/ros2_ws/pinky_cpp/pinky_core/build
ctest --output-on-failure
```

### 5.2. 파이썬 GUI & 네트워크 통합 테스트
TCP/UDP 소켓 통신 무결성 및 GUI 위젯의 시그널/UI 업데이트 로직을 검증합니다. (이 과정에서 C++ 프로세스가 자동으로 띄워집니다.)

```bash
cd ~/ros2_ws/pinky_cpp/pinky_station
source ../.venv/bin/activate
QT_QPA_PLATFORM=offscreen python3 -m pytest tests/ -v
```
*(주의: `QT_QPA_PLATFORM=offscreen` 옵션은 실제 윈도우 팝업을 띄우지 않고 백그라운드에서 GUI 테스트를 빠르게 진행하기 위한 필수 옵션입니다.)*