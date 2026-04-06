# Phase 5: 네비게이션 아키텍처 개선 및 UI 버그 수정 보고서

**작성일:** 2026-04-06
**브랜치:** dev_ZeroMQ
**커밋:** 60b5d2b, (이번 세션 추가 변경)
**상태:** 수정 완료

---

## 1. 개요

자율주행(goal_pose) 미동작, 카메라 좌우반전, 경로 시각화 품질 저하, Start 버튼 상태 미관리 등
실기 테스트에서 발견된 문제들을 진단하고 수정하였다.

---

## 2. 진단 결과 요약

| 항목 | 증상 | 원인 |
|------|------|------|
| 자율주행 미동작 | `[NAV] Goal received` 이후 `[RL]`/`[P-CTRL]` 로그 없음 | RL/P-control이 `LidarLoop` 내부에 있어 LiDAR GetScan 실패 시 전혀 실행 불가 |
| 카메라 좌우반전 | 배경(소화기 위치 등)이 실제 반대 방향으로 표시 | 이전 세션에서 추가된 `image.mirrored(True, False)` 가 `rpicam-vid --hflip`과 이중 반전 초래 |
| 경로 시각화 | waypoint 간 직선 연결, 로봇 이동과 무관한 정적 표시 | 로봇 현재 위치 기준 실시간 경로 갱신 없음, 곡선 처리 없음 |
| Start 버튼 | 주행 중 재클릭 가능, 중복 goal 전송 위험 | 클릭 시 비활성화 처리 누락 |

---

## 3. 수정 내용

### 3.1. 네비게이션 아키텍처 분리 — NavLoop 독립 (C++)

**이전 구조:**
```
LidarLoop:
  GetScan() 성공 시에만
    → RL 추론 or P-control 실행
```

**문제:** LiDAR `grabScanDataHq()` 실패/블로킹 시 네비게이션 완전 중단.
P-control은 odom과 목표 좌표만 있으면 동작 가능하지만 실행 기회조차 없었음.

**신규 구조:**
```
LidarLoop (기존, ~7Hz):
  GetScan() 성공 시
    → sectors를 latest_sectors_에 저장
    → PC로 telemetry 발행

NavLoop (신규, 20Hz — LiDAR와 독립):
  rl_navigation_active_ == true 시:
    거리 < 0.3m → 목표 도달, 정지
    onnx_actor_ && has_lidar_sectors_ → RL 추론
    else → P-control fallback (odom + goal만으로 주행)
```

**효과:** LiDAR 스캔이 없어도 P-control로 정상 주행.
LiDAR 복구 시 자동으로 RL로 전환.

**수정 파일:**
- `pinky_core/include/pinky_core/app/robot_app.h`
  - `NavLoop()` 메서드 선언 추가
  - `latest_sectors_`, `has_lidar_sectors_`, `nav_thread_` 멤버 추가
- `pinky_core/src/app/robot_app.cpp`
  - `LidarLoop()`: sectors를 공유 변수에 저장하는 로직으로 단순화
  - `NavLoop()` 신규 구현 (20Hz 독립 루프)
  - `Run()`: `nav_thread_` 시작 추가
  - `Stop()`: `nav_thread_` join 추가

**진단 로그 추가:**
- `[LIDAR] First scan received (N points)` — LiDAR 첫 스캔 확인
- `[NAV] Navigation active (has_lidar=N, has_onnx=N)` — 사용 컨트롤러 확인
- `[RL] step=N dist=N ...` — RL 추론 결과 (50스텝마다)
- `[P-CTRL] step=N dist=N ...` — P-control 상태 (50스텝마다)
- `[NAV] Goal reached! dist=N` — 목표 도달 확인

---

### 3.2. 카메라 좌우반전 수정 (Python)

**원인:** 이전 세션에서 추가된 `image.mirrored(True, False)`가
`rpicam-vid --hflip --vflip`(물리적 180도 회전 보정)과 이중 반전을 일으킴.

**수정:** `video_view.py`의 `image.mirrored(True, False)` 제거.

**참고:** `rotate_180=true` 설정의 `--hflip --vflip`은 카메라가 물리적으로
뒤집혀 장착된 경우의 180도 회전 보정이며, 배경이 정확하게 나오는 것이 올바른 상태.
카메라 앞에서 오른손을 흔들면 화면 왼쪽에 나오는 것은 카메라의 정상 동작.

**수정 파일:**
- `pinky_station/pinky_station/gui/widgets/video_view.py`

---

### 3.3. 실시간 네비게이션 경로 시각화 개선 (Python)

**이전:** waypoint 간 직선 연결선만 표시. 로봇 이동과 무관한 정적 그래픽.

**변경:**
1. **live path (실시간 경로):** 로봇 현재 위치 → 현재 목표 waypoint → 남은 waypoints를
   **Catmull-Rom 스플라인 곡선**으로 연결하여 표시.
   - odom 수신마다 자동 갱신 (로봇이 움직일수록 경로 곡선 변화)
   - Nav2의 `plan` 토픽 시각화와 유사한 부드러운 곡선 형태
   - 색상: 시안(#00C8FF), 굵기 3px
2. **static path (정적 waypoints 연결):** 기존 직선 연결을 연한 노란 점선으로 유지 (계획 경로 참조용)

**Catmull-Rom 스플라인 구현:**
- `MapWidget._catmull_rom_chain()` 정적 메서드 추가
- 제어점 확장(mirroring)으로 시작/끝 waypoint도 곡선이 통과하도록 처리
- segments=12 (각 구간 12분할)로 부드러운 곡선 품질 확보

**수정 파일:**
- `pinky_station/pinky_station/gui/widgets/map_widget.py`

---

### 3.4. Start 버튼 상태 관리 (Python)

**문제:** 주행 중 Start 버튼 재클릭 시 중복 nav_goal 전송 위험.

**수정:**
- Start 클릭 시 → `btn_start` 비활성화, `btn_add_waypoint` 비활성화
- Reset 클릭 시 → `btn_start` 재활성화
- 최종 목표 도달 시 → `btn_start` 자동 재활성화
- Disconnect 시 → 경로 상태 초기화

**수정 파일:**
- `pinky_station/pinky_station/gui/widgets/toolbar.py`
- `pinky_station/pinky_station/gui/main_window.py`

---

### 3.5. P-control 안전 속도 감소 및 LiDAR 실패 진단 (C++)

**배경:** 실기 테스트에서 `has_lidar=0` 상태로 P-control이 kVMax(0.26 m/s)로 주행하여
장애물에 충돌하는 문제 발생.

**P-control 속도 조정:**

| 항목 | 이전 | 이후 |
|------|------|------|
| 최대 전진 속도 | 0.26 m/s (kVMax) | 0.12 m/s |
| 최대 회전 속도 | 1.0 rad/s | 0.8 rad/s |
| 전진 게인 | `0.4 * dist` | `0.25 * dist` |
| 목표 감속 | 없음 | 거리에 비례해 자동 감속 |
| 회전 임계 | 각도 크면 turn_scale 감소 | 90도 이상 시 거의 제자리 회전 |

**LiDAR 진단 로그 추가:**
- `Run()`: `lidar_->StartScan()` 반환값 확인 — `"LiDAR scan started."` 또는
  `"LiDAR scan start FAILED — P-control only."` 출력
- `LidarLoop()`: `GetScan()` 실패 횟수 누적, 1/10/100/500 단위로 경고 출력
  ```
  [LIDAR] GetScan failed (count=1)
  [LIDAR] First scan received (N points, after M failures)
  ```

**수정 파일:**
- `pinky_core/src/app/robot_app.cpp`

---

### 3.6. SLAM 스타일 맵 그리기 기능 추가 (C++ + Python)

**배경:** ROS2 SLAM Toolbox처럼 LiDAR 스캔 + 오도메트리로 Occupancy Grid Map을 실시간 구축하고,
`.pgm` + `.yaml` 형식으로 저장하는 기능 추가.

**구현 방식:**

```
[Robot] LidarLoop
  → LidarSectors (24개, 기존 RL용)  PUB
  → LidarScan (전체 ranges[], 신규) PUB

[Station] ZmqClient
  lidar_scan_received 시그널 (LidarScan 전용, sectors와 분리)

[Station] OccupancyGridBuilder
  update(scan, x, y, θ) — Log-odds + Bresenham raycasting
  to_qimage()           — MapWidget 렌더링용 QImage 변환
  save(path)            — .pgm + .yaml 저장

[Station] MapWidget
  OG 그리드를 반투명(85%) 오버레이로 렌더링 (기존 정적 맵 위에)

[Station] Toolbar
  "Map Build" 토글 버튼 + "Save Map" 버튼
```

**Occupancy Grid 파라미터:**

| 항목 | 값 |
|------|----|
| 해상도 | 0.05 m/cell |
| 그리드 크기 | 800×800 (40m×40m) |
| 원점 | (-20, -20) |
| 최대 LiDAR 범위 | 12 m |
| occupied 임계값 | 0.65 (probability) |
| free 임계값 | 0.196 |

**맵 색상 (ROS2 map_server 호환):**
- 255 (흰색): free
- 0 (검정): occupied
- 205 (회색): unknown

**사용 흐름:**
1. Map Build 버튼 ON → OG 그리드 초기화
2. 로봇 주행/조종 → LiDAR 스캔 수신마다 그리드 자동 갱신 (3스캔마다 화면 갱신)
3. Save Map → 파일 다이얼로그 → `.pgm` + `.yaml` 저장

**추가/수정 파일:**
- `pinky_core/src/app/robot_app.cpp`: `LidarLoop()`에 전체 `LidarScan` PUB 추가
- `pinky_station/pinky_station/net/zmq_client.py`: `lidar_scan_received` 시그널 분리
- `pinky_station/pinky_station/core/occupancy_grid.py`: `OccupancyGridBuilder` 신규 구현
- `pinky_station/pinky_station/gui/widgets/toolbar.py`: Map Build / Save Map 버튼 추가
- `pinky_station/pinky_station/gui/widgets/map_widget.py`: OG 오버레이 렌더링 추가
- `pinky_station/pinky_station/gui/main_window.py`: 맵 빌드 시그널 연결 및 저장 처리

---

## 4. 변경 파일 목록

| 파일 | 변경 내용 |
|------|-----------|
| `pinky_core/include/pinky_core/app/robot_app.h` | `NavLoop()`, `nav_thread_`, `latest_sectors_`, `has_lidar_sectors_` 추가 |
| `pinky_core/src/app/robot_app.cpp` | `LidarLoop` 단순화 및 실패 진단, `NavLoop` 신규 구현, P-control 안전 속도 적용 |
| `pinky_station/pinky_station/gui/widgets/video_view.py` | 이중 hflip 제거 |
| `pinky_station/pinky_station/gui/widgets/map_widget.py` | Catmull-Rom 스플라인 경로, `_catmull_rom_chain()` 추가 |
| `pinky_station/pinky_station/gui/widgets/toolbar.py` | Start 버튼 상태 관리 |
| `pinky_station/pinky_station/gui/main_window.py` | `current_waypoint_idx` map_view 동기화, goal 도달 시 Start 복원, 맵 빌드 시그널 연결 |
| `pinky_station/pinky_station/net/zmq_client.py` | `lidar_scan_received` 시그널 추가 (full scan / sectors 분리) |
| `pinky_station/pinky_station/core/occupancy_grid.py` | `OccupancyGridBuilder` 신규 — log-odds raycasting, .pgm/.yaml 저장 |

---

---

### 3.7. LiDAR GetScan 실패 근본 원인 수정 (C++)

**증상:** `LiDAR scan started.` 이후 `[LIDAR] GetScan failed (count=1/10/...)` 무한 반복.

**근본 원인: 모터 시작 순서 반전**

| 항목 | 기존 (잘못됨) | 수정 후 (ROS2 sllidar_node 동일) |
|------|--------------|----------------------------------|
| StartScan 순서 | `startScan()` → `setMotorSpeed()` | `setMotorSpeed()` → `startScan()` |

`grabScanDataHq()`는 모터가 1회전을 완료할 때까지 대기. 모터가 아직 안 돌고 있으면 항상 타임아웃.

**추가 수정:**
- `Init()`: `getHealth()` 체크 추가 — ERROR 상태면 `reset()` 후 실패 반환. 모델/FW/HW 버전 출력
- `StartScan()`: `setMotorSpeed()` 반환값 체크, 에러 코드 로깅, 선택된 스캔 모드 출력
- `GetScan()`: `grabScanDataHq` 타임아웃 2000ms → **5000ms** (모터 스핀업 대기 여유 확보)

**기대 출력 (수정 후):**
```
SLLiDAR connected successfully. Model: XX FW: X.X HW: X
SLLiDAR health status: 0 (OK)
SLLiDAR scan mode: DenseBoost (sample_duration=...us, max_dist=...m)
LiDAR scan started.
[LIDAR] First scan received (XXX points, after 0 failures)
```

**수정 파일:**
- `pinky_core/src/hal/sllidar_driver.cpp`

---

### 3.8. SAC 관측값 구조 불일치 수정 (C++) — by other AI

**원인:** Python 훈련 환경과 C++ 추론의 obs[25~27] 구조 불일치.

| 인덱스 | 기존 C++ (잘못됨) | Python 훈련 / 수정 후 |
|--------|------------------|----------------------|
| [25] | `cos(goal_angle)` | `goal_angle / π` |
| [26] | `sin(goal_angle)` | `odom.vx / 0.5` |
| [27] | `step / max_steps` | `odom.vth / 1.0` |

**수정 파일:**
- `pinky_core/src/inference/observation_builder.cpp`

---

### 3.9. OccupancyGrid free-space raycasting 수정 (Python) — by other AI

**원인:** 범위 밖(0.0, inf) 빔을 `continue`로 건너뛰어 free space가 전혀 그려지지 않음.

**수정:** 유효하지 않은 빔은 `range_max`로 클램프 후 raycasting 실행 (`is_valid_hit=False`로 occupied 마킹만 제외).

**수정 파일:**
- `pinky_station/pinky_station/core/occupancy_grid.py`

---

## 5. 테스트 체크리스트

- [x] nav_goal 전송 후 `[NAV] Navigation active` 로그 확인
- [x] P-control 모드(`has_lidar=0`)에서 로봇 물리적 이동 확인
- [ ] 빌드 후 `LiDAR scan started.` 또는 `FAILED` 로그로 StartScan 상태 확인
- [ ] `[LIDAR] GetScan failed` 카운트 확인 → LiDAR 스캔 실패 원인 파악
- [ ] `[LIDAR] First scan received` 확인 후 `has_lidar=1` 전환 및 RL 주행 확인
- [ ] 맵 위젯에서 곡선 경로가 로봇 이동에 따라 실시간 갱신 확인
- [ ] 카메라 배경이 실제 환경과 좌우 일치 확인
- [ ] Start → 주행 중 Start 버튼 비활성화 확인
- [ ] Reset 또는 목표 도달 후 Start 버튼 재활성화 확인
- [ ] 빌드 후 `SLLiDAR health status: 0 (OK)` 및 `SLLiDAR scan mode:` 로그 확인
- [ ] `[LIDAR] First scan received` 로그로 LiDAR 스캔 정상 수신 확인
- [ ] `[NAV] Navigation active (has_lidar=1, has_onnx=1)` → RL 주행 전환 확인
- [ ] Map Build ON → 로봇 주행하면서 OG 맵 실시간 갱신 확인
- [ ] Save Map → .pgm/.yaml 저장 후 Load Map으로 로드 확인
