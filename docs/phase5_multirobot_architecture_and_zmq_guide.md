# Pinky Multi-Robot Architecture & ZeroMQ Communication Guide

현재 진행 중인 마이그레이션 프로젝트는 ROS 2(DDS 기반)에 의존하던 구조를 탈피하여, **ZeroMQ(ZMQ)와 Protocol Buffers(Protobuf)** 를 이용한 가볍고 빠른 순수 C++ 아키텍처로 전환하는 과정입니다.

본 문서는 3대의 로봇(ID: 7번 포함)을 동시에 제어하기 위해 적용된 시스템 아키텍처와, 구현된 통신 방식(ZeroMQ)이 과거 ROS 2의 통신 방식(Topic, Service, Action)과 어떻게 대응되는지 정리한 가이드입니다.

---

## 1. 현재 통신 방식 (ZeroMQ + Protobuf) 이해하기

### 1-1. ZeroMQ란?
ZeroMQ(ZMQ)는 단순한 소켓(TCP/UDP) 통신을 넘어, **메시지 지향(Message-oriented)** 통신을 매우 쉽고 빠르며 안정적으로 구현할 수 있게 해주는 고성능 비동기 메시징 라이브러리입니다. 브로커(중앙 서버)가 필요 없는 P2P 방식이 기본이며, ROS 2의 DDS(Data Distribution Service)를 훨씬 가볍게 대체할 수 있습니다.

### 1-2. Protocol Buffers (Protobuf) 란?
구글이 개발한 데이터 직렬화 포맷입니다. ZMQ는 "바이트 배열"만 전송하므로, 이 바이트 배열 안에 어떤 데이터(속도, 배터리, 이미지 등)가 들어있는지 정의하고 압축/해제하는 역할이 필요합니다. 이를 위해 `pinky.proto` 파일을 정의하여 C++과 Python 양쪽에서 동일한 구조로 데이터를 주고받습니다.

### 1-3. 현재 구현된 프로토콜 (포트 및 패턴)
각 로봇은 고유의 IP에서 2개의 포트와 ZMQ 통신 패턴을 조합하여 사용합니다.

1. **포트 9200 (PUB / SUB 패턴) - 텔레메트리 및 데이터 스트리밍**
   *   **로봇(PUB)**: 센서 데이터(오도메트리, IMU, 배터리)와 카메라 프레임(`VideoStream`)을 생성하여 9200 포트로 일방적으로 흩뿌립니다(Publish).
   *   **관제탑(SUB)**: 로봇의 IP:9200에 접속하여 필요한 토픽(`T` 텔레메트리, `V` 비디오)만 골라서 수신합니다(Subscribe).
   *   *(ROS 2의 `Publisher / Subscriber`와 완벽히 동일합니다.)*

2. **포트 9100 (REQ / REP 패턴) - 제어 명령 및 상태 확인**
   *   **관제탑(REQ)**: 로봇을 움직이기 위한 명령(`CmdVel`, `NavGoal`)을 특정 로봇의 9100 포트로 보냅니다(Request).
   *   **로봇(REP)**: 명령을 수신하고, 정상적으로 처리되었는지 응답(`CommandAck`)을 반환합니다(Reply).
   *   *(ROS 2의 `Service`와 유사하며, 제어의 신뢰성을 보장합니다.)*

---

## 2. ROS 2 개념과 ZeroMQ 대응 (Mapping)

과거 ROS 2에서 사용하던 주요 통신 방식들은 현재 ZMQ 아키텍처에서 다음과 같이 구현/대체됩니다.

| ROS 2 개념 | ZeroMQ + Protobuf 대응 방식 | 설명 |
| :--- | :--- | :--- |
| **Topic (토픽)** | **PUB / SUB** (Port 9200) | `SensorTelemetry` 구조체 안에 `odom`, `battery` 등을 넣어 PUB 전송. 수신자는 SUB 소켓으로 구독. |
| **Service (서비스)** | **REQ / REP** (Port 9100) | 관제탑이 `ControlCommand`를 REQ로 전송하면, 로봇이 즉각 `CommandAck`를 REP로 반환. |
| **Action (액션)** | **REQ/REP + 상태 폴링(PUB/SUB)** | 목표(Goal) 설정은 REQ/REP로 전송하고, 주행 중 진행률(Feedback) 및 결과(Result)는 PUB/SUB의 `RobotStatus` 토픽을 통해 관제탑이 지속적으로 확인. |
| **Parameter (파라미터)** | **설정 파일 (`.yaml`) + REQ/REP** | 로봇 구동 시 `robot_config.yaml`을 읽어 초기화하며, 실행 중 변경이 필요한 값은 REQ/REP 명령을 새로 정의하여 처리. |

---

## 3. 다중 로봇 (Multi-Robot) 확장 및 자율주행 고도화 현황 (최신 업데이트)

현재 시스템은 1:N 구조 지원과 함께 자율주행 UX 및 정밀도가 대폭 개선되었습니다.

### 3-1. 통신 및 식별자 (Robot ID)
*   **로봇 코어 (`pinky_core`)**: `robot_id` 설정을 통해 부팅 시 `Robot ID: 7`과 같이 식별자를 출력하며, 모든 송신 데이터에 해당 ID를 포함합니다.
*   **보안 강화**: 이제 로봇 ID를 입력하지 않으면 로봇을 추가(Add)할 수 없으며, 중복 추가가 방지됩니다.

### 3-2. 관제탑 GUI 및 자율주행 UX (구현 완료)
1.  **연결 해제 기능 (Disconnect)**: 툴바에서 현재 활성화된 로봇을 개별적으로 연결 해제하고 리스트에서 제거할 수 있습니다.
2.  **경유지(Waypoint) 네비게이션**:
    *   맵 위를 클릭하여 여러 개의 경유지를 설정할 수 있습니다.
    *   **[Add Waypoint]** 버튼으로 정식 경로를 구성하며, 맵 상에 `GOAL 1`, `GOAL 2` 등 라벨이 표시됩니다.
    *   **Start/Stop/Resume/Reset**: 주행 시작, 일시 정지(Stop 시 Resume으로 변경), 목표 유지 후 재개, 그리고 전체 초기화 기능을 지원합니다.
    *   **자동 주행**: 현재 목표에 도달(20cm 이내)하면 자동으로 다음 경유지로 명령을 전송합니다.
3.  **정밀 맵 인터페이스 (`QTransform`)**:
    *   수학적 행렬 변환을 통해 줌/팬 상태에서도 마우스 클릭 지점과 맵 좌표가 완벽히 일치합니다.
4.  **2D Pose Estimate (RViz 스타일)**:
    *   마우스 왼쪽 버튼 클릭 후 드래그하여 로봇의 위치와 방향(Heading)을 동시에 설정할 수 있는 직관적인 인터페이스를 적용했습니다.

### 3-3. 디버깅 및 자가 진단
*   **카메라 수신 확인**: 관제탑 터미널 로그에 `[DEBUG] Frame received for [ID]` 메시지를 추가하여, 로봇으로부터 비디오 데이터가 정상 도착하는지 실시간 확인이 가능합니다.

---

## 4. 다중 로봇 시스템 사용 가이드

1.  **로봇(단말) 설정**:
    *   각 로봇의 `pinky_core/config/robot_config.yaml`의 `id:` 값을 다르게 설정 후 실행합니다.
2.  **관제탑 실행 및 로봇 추가**:
    *   `uv run python3 -m pinky_station.main` 실행 후 ID와 IP를 입력하여 로봇들을 추가합니다.
3.  **위치 초기화 및 경로 설정**:
    *   **[2D Pose Estimate]**를 활성화하고 맵에서 클릭-드래그하여 로봇의 실제 위치를 잡아줍니다.
    *   맵을 클릭하고 **[Add Waypoint]**를 눌러 원하는 주행 경로를 만듭니다.
4.  **자율주행 실행**:
    *   **[Start]**를 눌러 주행을 시작하고, 필요시 **[Stop/Resume]** 또는 **[Reset]** 기능을 사용합니다.
