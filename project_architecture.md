# Pinky Robot Project Architecture

본 문서는 `pinky_cpp` 프로젝트 패키지의 디렉토리 트리 구조와 전체 소프트웨어 아키텍처에 대해 요약한 문서입니다.

## 1. 디렉토리 구조 트리 (Directory Tree)
아래는 프로젝트 내 핵심 소스코드 계층을 중심으로 필터링하여 시각화한 구조입니다.

```text
pinky_cpp
├── pinky_core/                 # [C++] 고성능 실시간 제어, 하드웨어 제어 및 통신을 담당하는 코어 모듈
│   ├── config/                 # 로봇 및 RL 모델 설정 파일 (yaml)
│   ├── emotion/                # 로봇 표정/감정 표현을 위한 GIF 에셋 리소스
│   ├── include/pinky_core/     # C++ 헤더 파일 (app, core, hal, inference, net, protocol)
│   ├── models/                 # 강화학습 모델 (ONNX 포맷 등)
│   ├── src/                    # C++ 소스 파일 및 라이브러리 구현부
│   └── tests/                  # 모듈(core, protocol) 테스트 코드
│
├── pinky_pro/                  # [ROS 2] 로봇 자율주행, 시뮬레이션, 메타 로직을 다루는 ROS 패키지 모음
│   └── src/
│       ├── pinky_bringup/      # 로봇 구동을 위한 최상위 Launch 패키지 (C++/Python)
│       ├── pinky_description/  # 로봇 3D 우분투, URDF 및 RViz 관련 모델링
│       ├── pinky_emotion/      # 감정 표현 노드 (ROS)
│       ├── pinky_gz_sim/       # Gazebo 기반 시뮬레이션 환경 패키지 
│       ├── pinky_imu_bno055/   # IMU 센서 노드
│       ├── pinky_interfaces/   # Custrom ROS Message, Service, Action 정의 패키지
│       ├── pinky_lamp_control/ # 램프(LED 등) 제어 모듈
│       ├── pinky_navigation/   # Nav2 기반 SLAM 및 네비게이션 자율주행 설정
│       ├── pinky_sensor_adc/   # ADC 기반 데이터 읽기 노드 
│       └── sllidar_ros2/       # 라이다(SLLidar) 공식/커스텀 ROS 2 드라이버
│
├── pinky_station/              # [Python] PC 웹/GUI 관제 모니터링 스테이션 어플리케이션
│   ├── config/                 # 스테이션 연결(IP, Port 등) 설정 파일
│   ├── pinky_station/          # GUI 메인 로직 
│   │   ├── gui/                # PySide/PyQt 기반 UI 위젯 (배터리, 맵, 라이다, 카메라 뷰 등)
│   │   ├── net/                # TCP/UDP 소켓 통신 클라이언트 레이어 구현
│   │   ├── protocol/           # 데이터 시리얼라이저 및 체크섬 검증용 프로토콜
│   │   └── workers/            # GUI 블로킹을 막는 Background 스레드 워커들 (센서, 카메라, 커맨드 등)
│   ├── requirements.txt
│   └── tests/                  # 관제앱 네트워크 루프백, UI 테스트
│
├── pinkylib/                   # [Python] 순수 파이썬 센서 및 하드웨어 추상화(HAL) 라이브러리 패키지
│   ├── lcd/                    # LCD 제어 라이브러리 
│   └── sensor/                 # 모터, 배터리, 부저, IMU, yolo 등 단일 센서 래퍼 (Python)
│
├── pinky_devices/              # [3rd-party/Device] 하드웨어 드라이버 및 외부 모듈
│   ├── rpi-ws281x-python/      # 네오픽셀 LED 등 제어용 서드파티 모듈
│   ├── rplidar_sdk/            # RPLIDAR C++ SDK
│   └── WiringPi/               # 라즈베리파이 I/O 제어용 라이브러리
│
└── docs/                       # 각종 설계/버그수정 관련 시스템 문서 및 가이드 
```

---

## 2. 소프트웨어 아키텍처 (Software Architecture)
본 프로젝트는 **하드웨어 기반 제어(C++)**, **ROS 2 생태계(Python/C++)**, **관제 스테이션(Python GUI)**의 3계층으로 구분되어 유기적으로 동작합니다.

### 2.1 하위 계층: Hardware & Core Control (`pinky_core`, `pinkylib`)
- **언어 및 특징**: C++ 및 Python 혼용, 실시간성에 초점을 맞춘 데몬/백그라운드 프로세스
- **역할**:
  - `pinky_core/hal`: Dynamixel 모터, Lidar, ADC 핀, LCD 제어 등의 최하단 하드웨어 추상화 계층(HAL)을 구현
  - `pinky_core/net & protocol`: 관제 스테이션 및 타 서비스와의 TCP/UDP 통신을 보장, 직렬화(Serialization) 및 체크섬 처리 수행
  - `pinky_core/inference`: 로봇의 지능적 행동 판단을 지원하기 위해 포함된 ONNX 모델을 로드하여 고속 엣지 추론(Inference) 실행
  - `pinkylib`: 센서를 직접적으로 파이썬 환경에서 접근하기 위한 독립적 제어 컴포넌트 래퍼를 제공하여 간단한 스크립트 작성 용이

### 2.2 중간 계층: ROS 2 System (`pinky_pro` Workspace)
- **플랫폼**: ROS 2 (Humble / Iron 등), Gazebo
- **역할**:
  - **자율 주행 (Autonomous Navigation)**: `pinky_navigation`과 Nav2 스택을 활용하여 SLAM 및 경로 탐색, 회피를 담당
  - **시뮬레이션 (Gazebo)**: `pinky_gz_sim`과 `pinky_description`을 통해 현실 하드웨어 투입 전, 소프트웨어 런타임을 점검하는 가상 환경 인프라
  - **센서/액추에이터 인터페이스**: `pinky_bringup`을 통해 ROS 노드화 된 각종 하드웨어들(`.pinky_sensor_adc`, `.pinky_emotion` 등)을 한 번에 실행하고 토픽을 주고받음

### 2.3 상위 계층: Ground Control Station (`pinky_station`)
- **구현**: Python, PySide6 / PyQt 
- **역할**:
  - **GUI 모니터링 체계**: 라이다 포인트 클라우드 시각화(`lidar_view`), 실시간 스트리밍된 카메라 화면(`video_view`), 텔레메트리/배터리 모니터링 창 지원
  - **조향 및 명령 지시**: `teleop_widget`, `terminal_widget`을 통해 원격에서 로봇에게 직접 명령 하달 및 실시간 상태 진단
  - **비동기 스레딩 뷰**: PySide Worker(`workers/`)를 통해 메인 GUI 병목 없이 Network I/O 및 디바이스 상태값을 안정적으로 구독 및 출력

---

## 3. 핵심 데이터 플로우 (Data Flow)
1. **Sensor (Lidar, Camera, IMU) Data**
   `Hardware` $\rightarrow$ `pinky_core` / `pinky_pro(ROS 2 Drivers)` $\rightarrow$ `Local Control Loop(Navigation, Inference)` $\rightarrow$ `pinky_station(GUI)`
2. **Command / Teleop Flow** 
   `User in pinky_station` $\rightarrow$ `TCP/UDP Message` $\rightarrow$ `pinky_core(protocol/serializer)` $\rightarrow$ `HAL(motor/actuator)` 혹은 `pinky_pro(ROS 2 Topic/Service)`
3. **AI Inference Flow**
   `pinky_core/inference` $\rightarrow$ `ONNX Runtime` $\rightarrow$ `Hardware Action (LED, diff_drive)`
