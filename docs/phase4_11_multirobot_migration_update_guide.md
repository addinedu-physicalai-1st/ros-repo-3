# Pinky Station & Core Migration Update Guide

## 1. 주요 변경 내역 요약 (Changelog)

기존 TCP/UDP 및 커스텀 직렬화 기반의 통신을 **ZeroMQ + Protobuf**로 전면 개편하고, 라즈베리파이 5의 **카메라 스트리밍 충돌(Crash) 문제**를 해결하였으며, 관제탑 GUI에서 **자율주행 맵 연동**을 개선했습니다.

### 1-1. 통신 인프라 개편 (Phase 1)
*   **Protobuf 스키마 도입 (`pinky_core/proto/pinky.proto`)**: 센서 데이터(`SensorTelemetry`), 비디오 스트림(`VideoStream`), 제어 명령(`ControlCommand`) 등을 정의.
*   **ZeroMQ 서버 (`pinky_core/src/net/zmq_server.cpp`)**: 기존 `tcp_server` 및 `udp_server`를 삭제하고 ZMQ 기반 PUB/REP 패턴으로 통합.
    *   **Port 9200 (PUB)**: 로봇에서 PC로 센서, 로그, 카메라 프레임 전송 (멀티파트 메시지로 `T`, `V` 토픽 구분).
    *   **Port 9100 (REP)**: PC에서 로봇으로 제어 명령 수신.
*   **ZeroMQ 클라이언트 (`pinky_station/pinky_station/net/zmq_client.py`)**: PyQt6 QThread 백그라운드에서 ZMQ 메시지를 수신하여 GUI로 pyqtSignal 발생.

### 1-2. 카메라 스트리밍 크래시 해결 (Phase 2)
*   **OpenCV V4L2 직접 제어 (`pinky_core/src/hal/opencv_camera.cpp`)**: 
    *   Pi 5 환경에서 `libcamera` GStreamer 파이프라인 이용 시 발생하는 `validate() failed in ControlInfoMap` 블랙스크린 이슈 우회.
    *   `cv::VideoCapture` 백엔드로 `cv::CAP_V4L2` 직접 지정 (장치: `/dev/video0`).
    *   안정적인 스트리밍을 위해 640x480 해상도 및 15 FPS로 프레임 캡처 제한.

### 1-3. 관제탑 GUI 및 네비게이션 연동 (Phase 3)
*   **MapWidget 개선 (`pinky_station/pinky_station/gui/widgets/map_widget.py`)**:
    *   `yaml` + `pgm` 형태의 ROS 2 정적 맵 로드 지원. 맵이 없는 경우에도 빈 그리드에서 정상 작동.
    *   마우스 클릭(클릭 이벤트)으로 목표 좌표(Goal) 설정 시 `sig_set_goal` 발생.
*   **MainWindow 파이프라인 통합**:
    *   ZMQ 클라이언트에서 수신된 Odometry, Lidar 데이터가 맵 및 Lidar 뷰어에 실시간으로 업데이트.
    *   텔레옵(조이스틱)명령 및 맵 Goal 클릭이 ZMQ `ControlCommand`로 직렬화되어 로봇(Core)의 `CmdVel` 또는 `NavGoal`로 전달.
    *   카메라 영상 데이터 또한 `frame_received` 시그널을 거쳐 GUI 비디오 패널에 출력.

---

## 2. 사용 가이드 (How to run)

### 2-1. (PC) 관제탑 GUI 실행
현재 환경은 `uv` 가상환경 기반입니다. 의존성(protobuf, pyzmq, PyQt6 등)은 설치가 완료된 상태입니다.

```bash
cd /home/hajun/ros2_ws/pinky_cpp
uv run python3 -m pinky_station.main
```

**GUI 연결 방법:**
1. GUI 상단 툴바에서 로봇의 IP 주소 입력.
2. `[Connect]` 버튼 클릭 (ZMQ 소켓 9100, 9200 연결 수립).
3. 하단 터미널(Terminal Log)에서 "Connected ZMQ sockets" 확인 후 텔레옵(Teleop)이나 맵 Goal 마우스 클릭 조작.

### 2-2. (Robot) C++ Core 빌드 및 실행
PC 환경의 Mock 모드나 실제 라즈베리파이(ARM64)에서 실행하는 방법입니다. ZMQ 및 Protobuf 의존성이 추가되었으므로 CMake 빌드를 다시 해야 합니다.

```bash
cd /home/hajun/ros2_ws/pinky_cpp/pinky_core/build
cmake ..
make -j4
./pinky_robot
```
> *(참고: 로봇 보드(ARM64)로 코드를 옮긴 후, 로봇 장비에서도 동일하게 `cmake` 빌드를 진행해야 HAL 계층(하드웨어 제어)이 활성화되어 실제 모터/카메라가 작동합니다.)*

---

## 3. 시스템 의존성 설치 노트 (System Dependencies)

PC에서는 필요한 라이브러리(`protobuf`, `pyzmq`)가 Python `uv`를 통해 설치되어 작동하지만, **라즈베리파이(로봇)** 환경이나 **처음 세팅하는 리눅스 PC** 환경에서는 C++ 컴파일을 위해 OS 패키지 관리자 권한(`sudo`)이 필요합니다. 

이 프로젝트를 새로운 환경이나 로봇 보드에 배포할 때는 아래 시스템 패키지 설치를 먼저 진행해 주세요.

```bash
# 로봇 보드(Raspberry Pi OS / Ubuntu) 또는 새 PC 세팅 시
sudo apt-get update
sudo apt-get install -y protobuf-compiler libprotobuf-dev libzmq3-dev pkg-config
```
*   `libprotobuf-dev`, `protobuf-compiler`: C++ 프로토버프 메시지 파싱 및 `protoc` 실행을 위해 필요.
*   `libzmq3-dev`: C++ `pinky_core` 내부 ZMQ 통신 계층 컴파일을 위해 필요.