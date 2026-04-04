# Phase 5: 버그 수정 보고서

**작성일:** 2026-04-04
**브랜치:** dev_ZeroMQ
**상태:** 수정 완료

---

## 1. 개요

로봇(pinky_core) 실행 시 발생한 크래시, 카메라 미표시, 자율주행 미동작, GUI 이슈들을 진단하고 수정하였다.

---

## 2. 수정된 버그

### Bug 1: `double free or corruption` — ZmqServer 스레드 안전성 결여

**증상:**
```
Starting RobotApp...
double free or corruption (out)
Aborted (core dumped)
```

**원인:**  
`ZmqServer::pub_sock_` (ZMQ PUB 소켓)이 motor/imu/adc/lidar/camera 총 5개 스레드에서 mutex 없이 동시에 `send()` 호출됨. ZMQ 소켓은 스레드 안전하지 않으므로 힙 손상 발생.

**수정 파일:**
- `pinky_core/include/pinky_core/net/zmq_server.h` — `std::mutex pub_mutex_` 멤버 추가
- `pinky_core/src/net/zmq_server.cpp` — `PublishTelemetry`, `PublishVideo`에 `lock_guard` 적용

---

### Bug 2: `ModuleNotFoundError: No module named 'zmq'` — 카메라 서버 Python 환경 누락

**증상:**
```
Traceback (most recent call last):
  File ".../pinky_camera_server.py", line 1, in <module>
    import zmq
ModuleNotFoundError: No module named 'zmq'
```

**원인:**  
`robot_app.cpp`에서 `uv run ../src/hal/pinky_camera_server.py`로 카메라 서버를 실행하는데, 로봇의 `pinky_core/` 디렉토리에 `pyproject.toml`이 없어 `uv`가 `zmq`가 있는 가상환경을 찾지 못함.

**수정 파일:**
- `pinky_core/pyproject.toml` (**신규**) — `pyzmq`, `opencv-python`, `numpy` 의존성 정의
- `pinky_core/src/app/robot_app.cpp` — 실행 커맨드를 `uv run --project ..` 로 변경, `python3` fallback 추가

**로봇 최초 실행 전 1회 필요:**
```bash
cd ~/pinky_core
uv sync
```

---

### Bug 3: PC 터미널 `[DEBUG]` 스팸

**증상:**  
관제탑 실행 PC 터미널에서 `[DEBUG] Received multipart topic: b'T'` 메시지가 매우 빠르게(~50Hz) 계속 출력됨.

**원인:**  
`zmq_client.py`의 수신 루프 내 debug print 문이 모든 ZMQ 메시지마다 실행됨.

**수정 파일:**
- `pinky_station/pinky_station/net/zmq_client.py` — debug print 2건 제거

---

### Bug 4: 자율주행 미동작 — `sig_set_goal` 미발신 + Nav2 fallback 없음

**증상:**  
맵 클릭 시 로봇이 반응하지 않음.

**원인:**
1. `map_widget.py`에서 `sig_set_goal` 시그널이 정의되어 있으나 한 번도 `emit()` 되지 않음.
2. `main_window.py`의 `_on_set_goal`은 Nav2(`rclpy`)가 실행 중일 때만 동작하며, Nav2 없는 환경에서의 fallback이 없음.

**수정 파일:**
- `pinky_station/pinky_station/gui/widgets/map_widget.py`
  - 좌클릭(non-pose 모드) 시 `sig_set_goal.emit(x, y, 0.0)` 추가
- `pinky_station/pinky_station/gui/main_window.py`
  - `_on_set_goal`: Nav2 사용 가능하면 전역 경로 요청, 불가능하면 `send_nav_goal` 직접 전송

**동작 흐름 (Nav2 없는 환경):**
```
맵 좌클릭 → sig_set_goal → _on_set_goal → zmq_client.send_nav_goal(x, y)
                                         → 로봇 RL 컨트롤러 활성화
```

---

### Bug 5: GUI LiDAR 뷰 미표시 — `lidar_received` 시그널 미연결

**증상:**  
`ZmqClient.lidar_received` 시그널이 발생하지만 GUI에 아무것도 연결되어 있지 않아 LiDAR 데이터가 표시되지 않음.

**수정 파일:**
- `pinky_station/pinky_station/gui/main_window.py`
  - `LidarViewWidget` import 및 center 패널 하단에 추가
  - `lidar_received` 시그널 → `_on_lidar` 핸들러 연결
- `pinky_station/pinky_station/gui/widgets/lidar_view.py`
  - `update_sectors(sectors: list)` 메서드 추가 (robot이 전송하는 24-sector 정규화 데이터 처리)

---

### Bug 6: ZMQ 커맨드 실패 시 무음 처리

**증상:**  
로봇이 응답하지 않을 때(timeout) GUI에 아무 피드백이 없고, stdout에만 출력됨.

**수정 파일:**
- `pinky_station/pinky_station/net/zmq_client.py`
  - `sig_command_failed = pyqtSignal(str, str)` 시그널 추가
  - timeout / exception 발생 시 `sig_command_failed` emit
- `pinky_station/pinky_station/gui/main_window.py`
  - `sig_command_failed` → `_on_command_failed` 연결 (터미널 위젯에 오류 표시)

---

## 3. 변경 파일 목록

| 파일 | 변경 유형 | 내용 |
|------|-----------|------|
| `pinky_core/pyproject.toml` | **신규** | 카메라 서버용 Python 의존성 (pyzmq, opencv, numpy) |
| `pinky_core/include/pinky_core/net/zmq_server.h` | 수정 | `pub_mutex_` 추가 |
| `pinky_core/src/net/zmq_server.cpp` | 수정 | PublishTelemetry/PublishVideo mutex 보호 |
| `pinky_core/src/app/robot_app.cpp` | 수정 | 카메라 서버 launch 커맨드 개선 |
| `pinky_station/pinky_station/net/zmq_client.py` | 수정 | debug print 제거, `sig_command_failed` 추가 |
| `pinky_station/pinky_station/gui/widgets/map_widget.py` | 수정 | 좌클릭 시 `sig_set_goal` emit |
| `pinky_station/pinky_station/gui/widgets/lidar_view.py` | 수정 | `update_sectors()` 메서드 추가 |
| `pinky_station/pinky_station/gui/main_window.py` | 수정 | LiDAR 뷰 추가, nav fallback, 오류 피드백 |

---

## 4. 남은 작업 (계획서 기준 미구현 항목)

계획서(`bubbly-waddling-whistle.md`) 대비 아직 구현되지 않은 주요 항목:

- **Map Data 스트리밍** (`MSG_MAP_DATA`): 로봇이 점유 격자 맵을 PC로 전송하는 기능 미구현.
- **LED/Lamp 제어** (`SET_LED`, `SET_LAMP`): GUI에서 LED 색상 제어 UI 없음.
- **Emotion 제어** (`SET_EMOTION`): GUI에서 LCD 감정 표현 변경 UI 없음.
- **Battery/IR/US 센서 표시**: ADC 센서값 중 배터리 외 IR/US 센서 GUI 미표시.
- **Snapshot 기능**: 카메라 화면 스냅샷 저장 버튼 미구현.
- **통합 테스트**: `pytest` 기반 네트워크 루프백 테스트 일부 미검증.
