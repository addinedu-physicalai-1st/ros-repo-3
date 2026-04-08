# Phase 5: A* 경로 시각화 및 Reset 개선

**날짜**: 2026-04-08
**커밋**: da3f125

## 배경

실제 로봇 테스트에서 GUI 경로 표시 문제 확인:
- 대시 라인(waypoint 연결선)과 스플라인(live 경로)이 맵의 장애물(검은 벽)을 직선으로 관통
- Reset 버튼이 waypoint만 삭제하고 odometry trail(초록 이동 경로)은 남아 있음

## 변경 사항

### 1. A* 경로 계획기 추가 (`pinky_station/core/path_planner.py`, 신규)

맵 이미지(QImage) 기반 A* 알고리즘으로 장애물 회피 경로 계산.

| 기능 | 설명 |
|------|------|
| `_build_grid()` | QImage → binary grid (free/obstacle) |
| `_inflate()` | 장애물 팽창 (기본 0.12m) — 로봇 폭 감안 |
| `plan(start, goal)` | 단일 구간 A* 경로 반환 |
| `plan_through(points)` | 다중 waypoint 통과 경로 (구간별 A* 연결) |
| `_simplify()` | 경로 다운샘플 (0.15m 간격) |

### 2. MapWidget 통합 (`pinky_station/gui/widgets/map_widget.py`)

**맵 로드 시**: `PathPlanner` 인스턴스 자동 생성.

**waypoint 추가 시**: `_replan_path()` 자동 호출 → A* 계획 경로를 `self.planned_path`에 저장.

**경로 시각화 변경**:
- 대시 라인: A* 계획 경로(`planned_path`) 우선 표시. A* 실패 시 직선 fallback
- 스플라인(live): 네비게이션 중 로봇 현재 위치 → 남은 waypoints 경로도 실시간 A* 재계산

**Reset 시 trail 삭제**:
- `clear_waypoints()`에 `self.trail.clear()` 추가
- Reset 버튼 → waypoint + 이동 경로 표시 동시 삭제

### 3. 로봇 제어 코드 미변경

이번 수정은 GUI 시각화 전용. `pinky_core`(C++) 코드는 변경 없음.

## 수정 파일

| 파일 | 변경 내용 |
|------|----------|
| `pinky_station/core/path_planner.py` | **신규** — A* 경로 계획기 |
| `pinky_station/gui/widgets/map_widget.py` | PathPlanner 통합, 경로 표시 개선, Reset trail 삭제 |
