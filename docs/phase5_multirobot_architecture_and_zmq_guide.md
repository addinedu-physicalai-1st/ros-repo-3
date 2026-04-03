# Pinky Multi-Robot Architecture & ZeroMQ Communication Guide

현재 진행 중인 마이그레이션 프로젝트는 ROS 2(DDS 기반)에 의존하던 구조를 탈피하여, **ZeroMQ(ZMQ)와 Protocol Buffers(Protobuf)** 를 이용한 가볍고 빠른 순수 C++ 아키텍처로 전환하는 과정입니다.

본 문서는 앞으로 3대의 로봇(ID: 7번 포함)을 동시에 제어하기 위해 미리 준비해야 할 사항과, 현재 구현된 통신 방식(ZeroMQ)이 과거 ROS 2의 통신 방식(Topic, Service, Action)과 어떻게 대응되는지 정리한 가이드입니다.

---

## 1. 현재 통신 방식 (ZeroMQ + Protobuf) 이해하기

### 1-1. ZeroMQ란?
ZeroMQ(ZMQ)는 단순한 소켓(TCP/UDP) 통신을 넘어, **메시지 지향(Message-oriented)** 통신을 매우 쉽고 빠르며 안정적으로 구현할 수 있게 해주는 고성능 비동기 메시징 라이브러리입니다. 브로커(중앙 서버)가 필요 없는 P2P 방식이 기본이며, ROS 2의 DDS(Data Distribution Service)를 훨씬 가볍게 대체할 수 있습니다.

### 1-2. Protocol Buffers (Protobuf) 란?
구글이 개발한 데이터 직렬화 포맷입니다. ZMQ는 "바이트 배열"만 전송하므로, 이 바이트 배열 안에 어떤 데이터(속도, 배터리, 이미지 등)가 들어있는지 정의하고 압축/해제하는 역할이 필요합니다. 이를 위해 `pinky.proto` 파일을 정의하여 C++과 Python 양쪽에서 동일한 구조로 데이터를 주고받습니다.

### 1-3. 현재 구현된 프로토콜 (포트 및 패턴)
현재 단일 로봇 구조에서는 2개의 포트와 ZMQ 통신 패턴을 조합하여 사용합니다.

1. **포트 9200 (PUB / SUB 패턴) - 텔레메트리 및 데이터 스트리밍**
   *   **로봇(PUB)**: 센서 데이터(오도메트리, IMU, 배터리)와 카메라 프레임(`VideoStream`)을 생성하여 9200 포트로 일방적으로 흩뿌립니다(Publish).
   *   **관제탑(SUB)**: 로봇의 IP:9200에 접속하여 필요한 토픽(`T` 텔레메트리, `V` 비디오)만 골라서 수신합니다(Subscribe).
   *   *(ROS 2의 `Publisher / Subscriber`와 완벽히 동일합니다.)*

2. **포트 9100 (REQ / REP 패턴) - 제어 명령 및 상태 확인**
   *   **관제탑(REQ)**: 로봇을 움직이기 위한 명령(`CmdVel`, `NavGoal`)을 9100 포트로 보냅니다(Request).
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

## 3. 다중 로봇 (Multi-Robot) 확장을 위한 향후 계획

현재 1:1 연결 구조(하나의 IP 입력 후 Connect)를 1:N 구조(관제탑 1대 - 로봇 3대)로 확장하려면 아키텍처의 일부 변경과 GUI 개편이 필요합니다. 사용자님이 지정하신 **ID: 7 로봇**을 포함하여 다음과 같이 설계해야 합니다.

### 3-1. 통신 및 식별자 (Robot ID) 확장
이미 `pinky.proto` 구조체 설계 시, 각 메시지에 `robot_id` 필드를 미리 포함해 두었습니다.
*   **PUB/SUB (데이터 수신)**: 3대의 로봇이 각각 자신의 IP에서 9200 포트로 데이터를 PUB합니다. 관제탑의 ZMQ 클라이언트는 3개의 IP 모두에 SUB 소켓을 연결(Connect)합니다. 수신된 `SensorTelemetry` 메시지 내부의 `robot_id = "7"`을 확인하여 어떤 로봇의 데이터인지 식별합니다.
*   **REQ/REP (명령 송신)**: 현재의 동기식 REQ/REP 패턴은 1:1에 적합합니다. 3대의 로봇에게 비동기적으로 동시에 명령을 내리거나 특정 로봇에게만 명령을 내리기 위해, 관제탑에서 로봇마다 별도의 REQ 소켓을 유지하거나, **ROUTER / DEALER 패턴**으로 전환하여 비동기 라우팅을 구현해야 합니다.

### 3-2. 관제탑 GUI (Pinky Station) 개편 방향
현재 단일 뷰로 구성된 화면을 다중 로봇 관제가 가능하도록 변경해야 합니다.

1.  **동적 연결 관리 (Connection Pool)**:
    *   툴바의 단일 IP 입력창 대신, **[Add Robot]** 버튼을 통해 여러 로봇의 IP와 ID(예: ID:7, IP:192.168.0.40)를 리스트 형태로 등록하고 개별/전체 Connect 할 수 있는 `ConnectionManager` 위젯이 필요합니다.
2.  **멀티 뷰포트 (Multi-Viewport)**:
    *   카메라 영상, 배터리, 터미널 로그 등이 3대의 로봇 각각의 탭(Tab) 또는 분할 화면(Grid)으로 나뉘어 표시되어야 합니다.
3.  **다중 로봇 맵 네비게이션**:
    *   `MapWidget`은 3대의 로봇(ID별 다른 색상의 원/아이콘)을 동시에 맵 위에 그려야 합니다.
    *   목표(Goal) 설정 시, 맵 위에서 목적지를 클릭한 후 팝업이나 단축키를 통해 "이 목표를 7번 로봇에게 할당할지, 8번 로봇에게 할당할지" 선택하는 상호작용(UX)이 추가되어야 합니다.

### 3-3. 로봇 코어 (`pinky_core`) 변경 사항
*   **설정 파일 업데이트**: `robot_config.yaml`에 `robot_id: "7"` 이라는 식별자를 부여합니다.
*   **식별자 주입**: `RobotApp`에서 모든 `SensorTelemetry` 및 `VideoStream`을 PUB할 때, 메시지 객체에 `msg.set_robot_id(config_.robot_id)`를 필수적으로 채워넣어 관제탑이 출처를 알 수 있게 합니다.

---

## 4. 당장 다음 단계로 준비해야 할 것 (Next Action Items)

1.  **프로토콜 확인 로직 추가 (C++)**: 로봇에서 메시지를 보낼 때 Protobuf의 `robot_id` 필드에 "7"이라는 ID를 하드코딩 또는 설정 파일에서 읽어와서 삽입하도록 `robot_app.cpp` 코드를 수정합니다.
2.  **관제탑 GUI 멀티 채널 기반 마련 (Python)**: `zmq_client.py`가 하나의 IP만 바라보는 것이 아니라, 딕셔너리(`Dict[str, ZmqReceiverThread]`) 형태로 여러 IP 연결을 관리하도록 리팩토링합니다.
3.  **로봇 제어 테스트**: 다중 로봇 확장에 앞서, 현재 구현된 ID 7번 로봇이 ZMQ 통신을 통해 전진/후진 제어(`CmdVel`) 및 네비게이션 제어(`NavGoal`)가 단일 환경에서 완벽하게 동작하는지 (네트워크 끊김이나 병목은 없는지) 최종 검증합니다.