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

## 4. 개선 전후 비교

| 항목 | 이전 | 이후 |
|------|------|------|
| 로그 간격 | 50스텝 (2.5초) | 20스텝 (1초) |
| 28D obs 로그 | 없음 | 시작 + 100스텝마다 |
| 모드 전환 로그 | 없음 | 전환 시 항상 즉시 출력 |
| PD 영향 확인 | 불가 | `target` vs `cmd` vs `pd_d` 비교 |
| 각도 단위 | 라디안 (혼재) | 도(°) 통일 |
