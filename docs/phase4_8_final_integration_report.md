# Phase 4-8 최종 통합 및 트러블슈팅 보고서

> **작성일:** 2026-04-01
> **목적:** Phase 4-8의 잔여 미구현 항목(통합 테스트, GUI 테스트, Nav2 브릿지, YAML 파싱) 완료 및 발생한 이슈 해결 내역 정리

---

## 1. 주요 구현 내용

### 1.1. 설정 파일(YAML) 파싱 적용
- **Python (Station):** `station_config.yaml` 파일을 파싱하여 GUI 위젯(Teleop, Terminal, Map) 및 네트워크(ConnectionManager)의 초기값을 동적으로 설정하는 `StationConfig` 데이터 클래스 및 로더(`config.py`)를 구현했습니다. `main.py` 실행 시 자동 로드되거나 `--config` 플래그로 지정 가능합니다.
- **C++ (Robot Core):** 기존 `robot_config.yaml` 파싱에 이어, 강화학습 추론 전용 설정인 `rl_config.yaml`을 파싱하기 위해 `config_loader.cpp`에 `LoadRlConfig()`를 추가하고 `RlConfig` 구조체를 도입했습니다. `--rl-config` 플래그를 통해 실행 시 반영됩니다.

### 1.2. TCP/UDP 루프백 통합 테스트 (`test_integration_loopback.py`)
- C++ `pinky_robot --mock` 프로세스를 테스트 픽스처(`conftest.py`)를 통해 백그라운드로 띄운 뒤, 파이썬 클라이언트가 로컬호스트(127.0.0.1)로 접속하여 실제 소켓 레벨의 TCP(명령, 설정) 및 UDP(센서 스트리밍) 통신이 성공적으로 이루어지는지 검증하는 E2E 테스트를 완성했습니다.

### 1.3. GUI 위젯 단위 테스트 (`test_gui_widgets.py`)
- `pytest-qt`를 활용하여 PyQt6 위젯에 대한 단위 테스트를 작성했습니다.
- 가상의 마우스 클릭(Teleop), 값 업데이트(Battery), 필터링 및 텍스트 렌더링(Terminal), Odometry 수신에 따른 내부 변수 변환(Map) 등이 정상 동작함을 검증했습니다.
- 헤드리스 환경(CI/터미널)에서도 테스트가 돌 수 있도록 `QT_QPA_PLATFORM=offscreen` 환경변수 세팅을 도입했습니다.

### 1.4. ROS2 Nav2 브릿지 (`NavWorker`)
- `pinky_station`과 ROS2 Navigation2 스택을 연결하는 `NavWorker` 및 `RosBridgeNode`(`nav_worker.py`)를 신규 구현했습니다.
- **동작 흐름:** 로봇에서 UDP로 올라온 ODOM 데이터를 `tf2`(`odom` -> `base_link`)와 `nav_msgs/Odometry`로 ROS2 네트워크에 발행(Publish)합니다.
- 동시에 ROS2 환경(RViz 등)에서 내려오는 `cmd_vel` 및 `goal_pose`를 구독(Subscribe)하여 `pinky_station`의 TCP 프로토콜 포맷으로 변환해 로봇에 전송합니다.
- ROS2(`rclpy`)가 설치되지 않은 일반 PC에서도 GUI가 크래시 나지 않도록 방어적인 예외 처리(Graceful degradation)를 적용했습니다.

---

## 2. 핵심 트러블슈팅 내역

### 2.1. 50분 통합 테스트 정체(Hang / Deadlock) 현상 해결
- **문제 원인:** 이전 테스트 시도 중 C++ 서버가 클라이언트 연결 종료를 제대로 처리하지 못해 먹통이 되었는데, 파이썬 `TcpClient`의 소켓 통신 모드가 타임아웃 없는 완전 블로킹(`settimeout(None)`) 상태로 설정되어 있어 응답 없는 서버를 영원히 기다리는 데드락(Deadlock)이 발생했습니다.
- **해결책:** 파이썬 `TcpClient`의 `recv()` 스레드에 1.0초의 하드 타임아웃(`self.sock.settimeout(1.0)`)을 강제하여, 무한 대기를 방지하고 프로그램이 정상적으로 오류를 뱉고 다음 플로우로 넘어가도록 방어적 프로그래밍을 적용했습니다.

### 2.2. UDP 포트 충돌 및 Mock 모드 브로드캐스트 침묵 해결
- **문제 원인 1 (포트 충돌):** 로컬 루프백 테스트 시 하나의 PC 안에서 C++ 로봇 코어와 파이썬 테스트 클라이언트가 동시에 `9200`번 포트를 선점하려다 `Address already in use` 에러가 발생했습니다.
- **해결책 1:** C++의 `UdpServer`와 파이썬의 `UdpReceiver` 소켓 모두에 `SO_REUSEADDR` 및 `SO_REUSEPORT` 소켓 옵션을 부여하여 포트 자원을 원활히 공유하도록 수정했습니다.
- **문제 원인 2 (UDP 침묵):** C++ 로봇 앱이 `--mock` (가상 모드)로 실행될 때, 모터 하드웨어 객체가 없다는 이유로 가상의 Odometry 데이터조차 송신(Send)하지 않도록 분기가 막혀 있었습니다.
- **해결책 2:** `robot_app.cpp`의 분기문(`else`)을 수정하여, Mock 모드일 때도 일정한 주기로 임의의 Odom 패킷이 브로드캐스트 되도록 고쳤습니다.

### 2.3. 파이썬 문법 오류 및 테스트 의존성 충돌
- **동적 상속 문법 오류:** `NavWorker` 작성 시 `class RosBridgeNode(Node) if HAS_ROS2 else object:` 와 같은 잘못된 파이썬 동적 상속 문법으로 인해 컴파일 에러(SyntaxError)가 발생했습니다. 이를 별도의 조건문으로 부모 클래스를 결정하는 방식(`Node_Base = Node if HAS_ROS2 else object; class RosBridgeNode(Node_Base):`)으로 깔끔하게 우회했습니다.
- **의존성 충돌:** uv 가상환경 내에 ROS2의 `launch_testing` 플러그인이 엮이면서 `lark` 모듈을 찾지 못하거나 `pytest-qt`가 없는 문제가 발생했습니다. `uv pip`를 통해 누락된 패키지들을 수동으로 추가 설치하여 해결했습니다.

---

## 3. 실행 및 설치 가이드 (새로운 환경 세팅 시)

### 필요 의존성 설치
통합 테스트와 GUI 단위 테스트를 실행하기 위해 파이썬 가상환경(uv)에 아래 라이브러리들이 설치되어 있어야 합니다.

```bash
# 가상환경 활성화 (프로젝트 루트)
source .venv/bin/activate

# 필수 패키지 설치
uv pip install pytest pytest-qt PyQt6 lark
```

### 테스트 실행 명령어

**GUI 단위 테스트 실행 (UI 팝업 없이 터미널 내부에서 검증)**
```bash
cd pinky_station
QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_gui_widgets.py -v
```

**TCP/UDP 통신 루프백 통합 테스트 실행**
```bash
cd pinky_station
python3 -m pytest tests/test_integration_loopback.py -v
```

**전체 테스트 일괄 실행**
```bash
cd pinky_station
QT_QPA_PLATFORM=offscreen python3 -m pytest tests/ -v
```
*(현재 기준 통합 테스트 8건, GUI 위젯 테스트 5건 총 13건 모두 PASSED 상태입니다.)*

---

## 4. 추가 수정 내역 (2026-04-01)
사용자 피드백을 반영하여 하드웨어 런타임 초기화 이슈 및 디스플레이 전력/뷰 문제를 해결했습니다.

### 4.1. Lidar 초기화 및 ONNX 모델 경로 기본값 수정
- **문제:** 사용자가 `--config` 옵션 없이 `./pinky_robot`을 실행할 때, 구버전의 하드코딩된 기본값이 적용되어 `SllidarDriver` 초기화 실패(1MHz vs 256kHz 통신속도 불일치) 및 ONNX 모델 경로(`models/` vs `../models/`) 탐색 실패 발생.
- **수정:** 
  - `robot_config.yaml` 및 `sllidar_driver.h`의 기본 `baudrate`를 S1 라이다 규격인 `256000`으로 수정.
  - `rl_config.yaml` 및 `robot_app.h`의 기본 모델 경로를 빌드 폴더(`build/`) 기준 정상 위치인 `../models/sac_actor.onnx`로 수정.

### 4.2. LCD 렌더링 방식 변경 (전력 소모 최적화 및 GIF 표시)
- **문제:** 기존 `EmotionRenderer`가 원/선 등 기본 도형을 흰색/푸른색 배경 위에 그려 전력 소모가 심하고, 기획된 `basic.gif` 파일이 아닌 하드코딩된 도형이 노출됨.
- **수정:**
  - 이미지 디코딩을 위해 `stb_image.h` 단일 헤더 라이브러리를 프로젝트에 통합.
  - LCD 렌더링 백그라운드를 `0x0000`(완전 검은색)으로 변경하여 디스플레이 픽셀 전력 소모 최소화.
  - 감정 렌더링 시 기존 하드코딩 도형 대신 `pinky_emotion/emotion/*.gif` 파일들(`basic.gif`, `happy.gif`, `sad.gif` 등)을 로드하여 240x240 RGB565 규격으로 화면 중앙에 렌더링하도록 `emotion_renderer.cpp`를 전면 개편. (해당 파일이 없을 경우에만 기존 도형으로 Fallback)

### 4.3. Python 관제탑 실행 경로 안내
- **문제:** `pinky_cpp` 최상위 폴더에서 `python3 -m pinky_station.pinky_station.main` 실행 시 파이썬 패키지 경로 탐색 충돌로 `ImportError` 발생.
- **가이드 반영:** `run_guide.md`에 반드시 `cd pinky_station`으로 서브 폴더 진입 후 `python3 -m pinky_station.main`으로 실행해야 함을 명시함.

### 4.4. 로봇 앱 초기 구동 시 GIF 출력 보장
- **문제:** LCD의 `Init()` 직후 아무런 감정 초기화 명령을 내리지 않아, 프로그램 시작 시 화면이 검은색 캔버스만 띄우고 있는 현상 발견.
- **수정:** `RobotApp::Init()` 내에서 LCD 초기화가 성공하자마자 `RenderEmotion(EmotionId::kNeutral)`을 강제로 한 번 호출하여, `basic.gif`를 즉시 렌더링하도록 흐름을 수정. 이를 통해 로봇이 켜지면 곧바로 눈을 깜빡이게 됨.

### 4.5. ONNX 모델 IR 버전 다운그레이드
- **문제:** ONNX C++ 런타임 환경에서 최대 지원 IR 버전이 9인데 모델이 10으로 빌드되어 로드 실패( `Unsupported model IR version: 10`).
- **수정:** 파이썬 `onnx` 패키지를 사용하여 `sac_actor.onnx`의 `ir_version` 메타데이터를 9로 수동 다운그레이드한 후 재저장.

### 4.6. Lidar `getDeviceInfo` 에러 로깅 구체화
- **문제:** `SllidarDriver`가 `Init()` 과정 중 실패할 때 구체적인 원인(타임아웃 등)을 파악하기 힘듦.
- **수정:** 에러 로깅 부분에 SLLiDAR SDK의 `op_result` 에러 코드를 16진수(`std::hex`)로 함께 출력하도록 수정하여 하드웨어 트러블슈팅을 용이하게 함.

---

## 5. 추가 수정 내역 — LCD 렌더링 치명적 버그 3건 수정 (2026-04-01)

### 5.1. DrawFrame RGB 포맷 불일치 (Silent Failure)
- **문제:** `ILcdDriver::DrawFrame()`은 RGB888 버퍼(W×H×3 bytes)만 허용하는데, `RenderEmotion()`은 RGB565(W×H×2 bytes)를 반환. 크기 불일치 시 `DrawFrame`이 아무 에러 없이 `return`하여 LCD에 아무것도 그려지지 않음.
- **수정:**
  - `ILcdDriver` 인터페이스에 `DrawFrameRgb565()` 메서드 추가 (RGB565 버퍼 직접 SPI 전송)
  - `Width()`, `Height()` 가상 메서드 추가 (런타임 해상도 조회)
  - `Ili9341Lcd`에 해당 구현 추가, 크기 불일치 시 stderr 경고 출력
  - `robot_app.cpp`에서 `DrawFrame()` → `DrawFrameRgb565()` 호출로 변경

**변경 파일:**
- `include/pinky_core/hal/interfaces.h` — `DrawFrameRgb565`, `Width`, `Height` 가상 메서드 추가
- `include/pinky_core/hal/ili9341_lcd.h` — override 구현 선언
- `src/hal/ili9341_lcd.cpp` — `DrawFrameRgb565()` 구현 추가
- `src/app/robot_app.cpp` — Init(), kSetEmotion 핸들러 호출 변경

### 5.2. EmotionRenderer 해상도 불일치 (240×240 vs 320×240)
- **문제:** `emotion_renderer.cpp`의 `kWidth=240, kHeight=240` 고정값이 ILI9341 LCD의 실제 해상도(320×240)와 다름. 결과적으로 생성된 프레임버퍼 크기가 LCD와 맞지 않아 표시 불가.
- **수정:**
  - `RenderEmotion(EmotionId, int width, int height)` 파라미터화 (기본값 320×240)
  - 모든 도형 프리미티브(`SetPixel`, `FillCircle`, `FillRect`, `DrawArc`, `Fill`)가 `canvas_w`, `canvas_h` 인자를 받도록 변경
  - 얼굴 요소 좌표를 캔버스 중앙(`width/2`, `height/2`) 기준 상대 배치로 변경

**변경 파일:**
- `include/pinky_core/core/emotion_renderer.h` — 시그니처 변경, `LoadEmotionImage()` 추가
- `src/core/emotion_renderer.cpp` — 전면 재작성

### 5.3. GIF 이미지 경로 및 리사이즈
- **문제:** 이전 수정에서 하드코딩된 상대경로 `../../pinky_pro/src/...`는 로봇의 실제 빌드 디렉토리에서 해당 위치에 파일이 존재하지 않아 항상 실패.
- **수정:**
  - `RlConfig`에 `emotion_dir` 필드 추가 (기본값: `/home/hajun/ros2_ws/ros_test/src/pinky_pro/pinky_emotion/emotion`)
  - `rl_config.yaml`에 `emotion.dir` 섹션 추가, `config_loader.cpp`에서 파싱
  - `LoadEmotionImage()`: stb_image로 GIF 첫 프레임 로드 → RGBA 4채널 디코딩 → Nearest-neighbor 리사이즈(1000×750 → 320×240) → 알파 블렌딩(투명 배경 = 검은색) → RGB565 변환
  - `robot_app.cpp`: Init()에서 `LoadEmotionImage(emotion_dir + "/basic.gif")` 시도 → 실패 시 도형 fallback
  - `kSetEmotion` 핸들러: EmotionId → GIF 파일명 매핑 테이블 사용

**변경 파일:**
- `include/pinky_core/app/robot_app.h` — `RlConfig::emotion_dir` 추가
- `src/app/config_loader.cpp` — `emotion.dir` 파싱 추가
- `pinky_core/config/rl_config.yaml` — `emotion` 섹션 추가
- `src/app/robot_app.cpp` — Init(), kSetEmotion 핸들러 수정

---

## 6. 변경 파일 종합 (섹션 5 관련, 8개 파일)

| 파일 | 변경 유형 | 관련 항목 |
|------|-----------|-----------|
| `hal/interfaces.h` | 수정 | DrawFrameRgb565, Width, Height 추가 |
| `hal/ili9341_lcd.h` | 수정 | override 구현 |
| `hal/ili9341_lcd.cpp` | 수정 | DrawFrameRgb565 구현 |
| `core/emotion_renderer.h` | 수정 | 파라미터화, LoadEmotionImage 추가 |
| `core/emotion_renderer.cpp` | **전면 재작성** | 320×240, 리사이즈, 알파 블렌딩 |
| `app/robot_app.h` | 수정 | RlConfig::emotion_dir |
| `app/robot_app.cpp` | 수정 | DrawFrameRgb565, GIF 로드 로직 |
| `app/config_loader.cpp` | 수정 | emotion 섹션 파싱 |

---

## 7. 런타임 테스트 후 버그 수정 3건 (2026-04-01)

실제 로봇(RPi5, user `pinky`)과 PC에서 통합 실행 후 발견된 3가지 런타임 이슈를 수정했습니다.

### 7.1. PC GUI 무한 재귀 크래시 (RecursionError)

- **문제:** 로봇과의 TCP 연결이 끊어지면 `TcpClient.disconnect()` → `on_disconnect` 콜백 → `ConnectionManager.disconnect()` → `self.tcp.disconnect()` 순환 호출이 발생하여 파이썬 재귀 한도(RecursionError)에 도달하며 GUI가 크래시됨.
- **수정 (`pinky_station/pinky_station/net/connection.py`):**
  - `disconnect()` 진입 시 **상태를 먼저** `DISCONNECTED`로 설정하여 재진입 차단
  - `self.tcp.on_disconnect = None`으로 콜백 제거 후 `tcp.disconnect()` 호출하여 순환 참조 근본 차단
  - 이미 `DISCONNECTED` 상태이면 즉시 `return`하는 가드 조건 추가

### 7.2. 감정 GIF 경로 불일치 (로봇에서 shape fallback만 표시)

- **문제:** `RlConfig::emotion_dir`의 기본값이 `/home/hajun/ros2_ws/ros_test/src/pinky_pro/pinky_emotion/emotion`으로 하드코딩되어 있어, 로봇(user `pinky`, home `/home/pinky/`)에서는 해당 경로가 존재하지 않음. `LoadEmotionImage()`가 항상 실패하여 도형 기반 fallback만 LCD에 표시됨.
- **수정:**
  - `RlConfig::emotion_dir` 기본값을 `"emotion"` (상대경로)으로 변경 — 빌드 디렉토리 기준 `emotion/` 폴더에서 GIF 탐색
  - `rl_config.yaml`의 `emotion.dir` 값도 동일하게 상대경로로 변경
  - GIF 로드 실패 시 시도한 전체 경로를 stderr로 출력하여 디버깅 용이하게 개선
  - `main.cpp`에서 시작 시 `Emotion dir:` 경로를 로그 출력

- **로봇 배포 시 사용법:**
  ```bash
  # 방법 1: 빌드 폴더 옆에 emotion 디렉토리 배치
  cp -r /path/to/emotion ~/pinky_core/build/emotion/

  # 방법 2: --rl-config로 절대경로가 포함된 설정 파일 지정
  ./pinky_robot --rl-config /path/to/rl_config.yaml
  ```

### 7.3. HAL 드라이버 Init 실패 시 안전하지 않은 동작

- **문제:** `SllidarDriver::Init()`가 `0x80008002`(디바이스 정보 조회 실패)로 실패해도 `lidar_` unique_ptr이 유효한 상태로 남아, `Run()` 단계에서 `StartScan()` 및 `LidarLoop()`가 깨진 드라이버 객체에 접근. 다른 HAL 드라이버(Motor, IMU, ADC, LED)도 동일한 문제 구조.
- **수정:**
  - **`robot_app.cpp`:** 모든 HAL 드라이버 `Init()` 실패 시 `.reset()` 호출하여 포인터를 nullptr로 해제. 이후 `Run()`의 null 체크에 의해 관련 스레드/루프가 자연스럽게 스킵됨.
  - **`sllidar_driver.cpp`:** `connect()` 실패 및 `getDeviceInfo()` 실패 시 `drv_`를 `delete` 후 nullptr로 설정하여 소멸자에서의 이중 접근 방지. 에러 메시지에 `"check USB/UART connection and power"` 안내 추가.
  - **참고:** `0x80008002` 에러 자체는 하드웨어 연결/전원 문제이므로 코드 수정만으로는 해결 불가. USB/UART 케이블 및 라이다 전원 공급 확인 필요.

---

## 8. 변경 파일 종합 (섹션 7 관련, 6개 파일)

| 파일 | 변경 유형 | 관련 항목 |
|------|-----------|-----------|
| `net/connection.py` (Python) | 수정 | disconnect 재귀 차단 |
| `app/robot_app.h` | 수정 | emotion_dir 기본값 상대경로 |
| `app/robot_app.cpp` | 수정 | HAL init 실패 시 reset, GIF 실패 로그 |
| `app/main.cpp` | 수정 | emotion_dir 로그 출력 |
| `hal/sllidar_driver.cpp` | 수정 | Init 실패 시 drv_ 정리, 에러 메시지 개선 |
| `config/rl_config.yaml` | 수정 | emotion.dir 상대경로 |
