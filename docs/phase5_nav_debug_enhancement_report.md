# Phase 5: NavLoop 디버깅 로직 강화 보고서

**작성일:** 2026-04-07  
**브랜치:** dev_ZeroMQ  
**수정 파일:** `pinky_core/src/app/robot_app.cpp`

---

## 1. 배경

다른 AI의 정적 코드 분석(`docs/phase5_hybrid_nav_optimization_guide.md`)을 검토한 결과,
NavLoop의 디버깅 투명성 부족이 가장 시급한 문제로 확인됨.
실제 코드와 문서 분석의 차이를 비교한 뒤 아래 항목만 우선 수정.

**분석에서 확인한 주요 문제들 (이번에 수정하지 않은 항목):**

| 항목 | 내용 | 비고 |
|------|------|------|
| PD 게인 과다 | `kKpV/kKpW = 1.5` → 정지 상태에서 2.8× 증폭 | 파라미터 튜닝 단계에서 조정 |
| EMA alpha 약함 | `kEmaAlpha = 0.4` → 진동 억제 미흡 | 실기 테스트 후 조정 |
| Safety Layer 미흡 | 전방 장애물 시 `v=0`만, `w` 회피 없음 | 별도 이슈로 관리 |
| `kLookaheadDist` 미사용 | `constants.h`에 정의되어 있으나 NavLoop에서 미사용 — 죽은 상수 | 추후 정리 |

---

## 2. 수정 내용

### 2.1 모드 전환 로그 (신규)

**문제:** 제어 모드(Turn-first / RL / P-Ctrl)가 전환될 때 아무 로그가 없어 어떤 모드로 진입했는지 추적 불가.

**수정:** `NavMode` 열거형으로 이전 모드를 추적, 모드 전환 시 항상 즉시 출력.

```
[NAV] >> RL dist=1.23m angle=12.5deg
[NAV] >> TURN-FIRST angle=105.3deg dist=0.80m
[NAV] >> P-CTRL fallback no_lidar=1 no_onnx=0
```

### 2.2 28D 관측 벡터 로그 (신규)

**문제:** `ObservationBuilder::Build()`가 생성하는 28개 수치가 단 한 번도 로그에 출력되지 않아,
시뮬레이터 학습 데이터와 실기 관측값의 차이를 비교할 수 없었음.

**수정:** 내비게이션 시작 시(`step==0`)와 100스텝마다 전체 obs 벡터 출력.

```
[OBS] step=0 lidar=[0.91,0.88,...,0.73] dist_n=0.42 cos=0.87 sin=0.49 prog=0.00
```

출력 구조:
- `lidar[0..23]` — 24개 정규화 섹터 (0=장애물, 1=공간)
- `dist_n` — `clip(goal_dist / 5.0, 0, 1)`
- `cos`, `sin` — 목표 방향 각도 성분
- `prog` — `current_step / max_steps`

### 2.3 PD 델타 로그 (신규)

**문제:** RL 출력(`smooth_action`)이 PD 컨트롤러에 의해 얼마나 증폭/변형되는지 볼 수 없었음.
진동의 원인이 RL 모델인지 PD 레이어인지 구분 불가.

**수정:** `ActionToTarget()`으로 PD 적용 전 목표값을 별도 추출, 최종 cmd와 함께 출력.

```
[RL] step=20 dist=1.10m front=2.80m raw=[0.62,-0.11] smooth=[0.58,-0.09]
     target=(0.21,-0.09) cmd=(0.26,-0.14) pd_d=(0.05,-0.05)
```

- `target` — PD 적용 전 RL 타겟 속도
- `cmd` — PD + clamp 후 실제 명령
- `pd_d` — `cmd - target` (PD가 추가한 delta)

### 2.4 로그 출력 간격 조정

**문제:** `step % 50` → 20Hz 기준 2.5초 간격. 빠른 동작 분석 불가.

**수정:** `step % 20` → 1초 간격으로 단축.

### 2.5 각도 단위를 라디안 → 도(°)로 통일

모든 각도 로그를 `×(180/π)` 변환하여 직관적으로 읽을 수 있도록 변경.

---

## 3. 변경되지 않은 동작

로깅만 추가/수정했으며 제어 로직은 일절 변경하지 않음:
- EMA alpha, kAngularDeadzone, kTurnFirstThresholdRad 파라미터 유지
- Safety layer 로직 유지
- PD 게인(kKpV, kKpW) 유지

---

## 4. 실기 테스트 후 추가 수정 (2026-04-07)

실기 테스트 결과 아래 3건의 치명적 버그가 확인되어 추가 수정:

### 4.1 Goal tolerance 과다 (0.30m → config 값 사용, 기본 0.15m)

**증상:** 목표(-0.77, -1.29)가 로봇(-0.94, -1.27)에서 0.17m 거리 → 이동 없이 도착 판정.

**원인:** C++ `robot_app.cpp`에서 `dist < 0.30f` 하드코딩, GUI `main_window.py`에서도 `dist < 0.3` 하드코딩.

**수정:**
- `constants.h`: `kGoalTolerance` 0.05 → 0.15m
- `robot_app.cpp`: 하드코딩 제거, `config_.rl.goal_tolerance` 사용
- `main_window.py`: 0.3 → 0.15

### 4.2 LiDAR 최소거리 필터 미흡 + Safety Layer 과도

**증상:** 로봇이 전혀 전진 불가. 1060스텝 동안 `cmd.linear_x = 0` 지속.

**원인:**
1. LiDAR 최소거리 필터 0.05m → RPLIDAR 최소 측정 범위(~0.15m) 미만의 노이즈가 min-pooling에 혼입
2. 안전계층 emergency stop 임계값 0.20m → 전방 노이즈 0.15m에 항상 걸림

**수정:**
- `lidar_processor.cpp`: 최소거리 필터 0.05 → 0.10m
- `robot_app.cpp`: emergency stop 0.20 → 0.12m, 감속 구간 0.40 → 0.25m

### 4.3 로봇 TF 표시 오류

**증상:** 맵에서 로봇의 실제 방향(heading)이 보이지 않음. 세계 원점 축(0,0)이 로봇의 TF처럼 보여서 혼동.

**수정:**
- `map_widget.py`: 로봇 위치에 TF 프레임 표시 (빨강=전방/X, 초록=좌측/Y)
- 세계 원점 축 반투명(alpha=60) + 짧게(0.5m) 축소
- 2D Pose Estimate 시 자동 센터링 (`center_on`) 추가

---

## 5. 개선 전후 비교

| 항목 | 이전 | 이후 |
|------|------|------|
| 로그 간격 | 50스텝 (2.5초) | 20스텝 (1초) |
| 28D obs 로그 | 없음 | 시작 + 100스텝마다 |
| 모드 전환 로그 | 없음 | 전환 시 항상 즉시 출력 |
| PD 영향 확인 | 불가 | `target` vs `cmd` vs `pd_d` 비교 |
| 각도 단위 | 라디안 (혼재) | 도(°) 통일 |
| Goal tolerance | 0.30m 하드코딩 | 0.15m (config) |
| LiDAR 노이즈 필터 | 0.05m | 0.10m |
| Safety emergency | 0.20m | 0.12m |
| Safety 감속 | 0.40m | 0.25m |
| 로봇 TF 표시 | 원점 축만 | 로봇 위치에 TF 프레임 |
| Pose Estimate | 수동 패닝 | 자동 센터링 |
