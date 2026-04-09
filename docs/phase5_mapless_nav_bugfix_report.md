# Phase 5: Map-less Navigation Bug Fix Report

## 1. 개요 (Overview)
Map less 환경에서 로봇이 좁은 통로나 장애물을 마주했을 때 
비정상적인 주행(갈팡질팡하거나, 벽에 비비면서 고속 전진하는 현상)을 보이는 문제를 해결하기 위한 디버깅 및 수정 리포트입니다.

## 2. 문제 원인 분석 (Root Cause Analysis)

### 2.1. 초근접 장애물 무시 현상 (15cm Blind Spot Bug)
* **증상**: 로봇이 왼쪽 또는 오른쪽 벽에 완전히 충돌한 상태에서, 벽을 피하지 못하고 계속 비비면서 빠른 속도로 직진하려고 함.
* **원인**: C++의 `lidar_processor.cpp`에는 전처리 시 `kLidarMinRange (0.15f)` 필터가 적용되어 있었습니다. 거리가 15cm 미만으로 가까워지면 센서값을 "노이즈"로 판단하여 삭제(Ignore)해버렸습니다.
* **파급 효과**: 15cm 미만의 장애물 데이터가 삭제되면, 해당 섹터는 최대 거리인 `3.5m (1.0)` 공간으로 채워집니다. RL 로컬 플래너는 코앞에 있는 벽을 "3.5m 뻥 뚫린 빈 공간"으로 착각(Hallucination)하여 그 방향으로 풀악셀을 밟게 되는 치명적인 버그가 발생했습니다.

### 2.2. 과도한 전방 안전 감속 (Safety Layer Overreaction)
* **증상**: 정면이 뚫려있음에도 좁은 폭을 지날 때 우왕좌왕하며 매우 천천히 전진함.
* **원인**: `robot_app.cpp`에 구현된 Safety Layer가 로봇의 실제 폭(약 10cm)에 비해 지나치게 넓은 전방 범위(섹터 10 ~ 14, 좌우 30도 이상)를 감시하고 있었습니다.
* **파급 효과**: 실제로는 부딪히지 않고 지나갈 수 있는 좌우의 벽(15cm 거리)조차 정면의 위험 장애물로 인식하여 강제로 전진 속도를 감속시켜 RL 모델의 정상적인 전진 제어를 방해했습니다.

## 3. 수정 사항 (Modifications)

### 3.1. `constants.h` 수정
* **변경 전**: `constexpr float kLidarMinRange = 0.15f;`
* **변경 후**: `constexpr float kLidarMinRange = 0.01f;`
* **효과**: 15cm 이내의 벽이나 장애물 데이터도 버려지지 않고 RL 모델에 정확히 전달되어, 모델이 벽을 명확히 인식하고 회피할 수 있게 되었습니다. (Python 학습 환경과의 일치화)

### 3.2. `robot_app.cpp` 수정
* **변경 전**:
```cpp
for (int s = 10; s <= 14; ++s) {
  front_min = std::min(front_min, sectors.sectors[s]);
}
```
* **변경 후**:
```cpp
for (int s = 11; s <= 12; ++s) {
  front_min = std::min(front_min, sectors.sectors[s]);
}
```
* **효과**: Safety Layer의 감시 범위를 정면(섹터 11~12)으로 좁혀, 측면 장애물로 인한 불필요한 감속 개입을 차단했습니다.

## 4. 결론 (Conclusion)
위 두 가지 수정으로 인해 로봇은 좁은 환경에서도 측면 노이즈를 핑계로 감속하지 않게 되었으며, 벽에 초근접했을 때도 벽을 빈 공간으로 착각하는 일이 사라졌습니다. 결과적으로 Map less 상황의 순수 로컬 플래너(RL) 주행 성능이 대폭 향상되었고, Nav2 (Costmap) 기반 주행에서의 안정성 또한 극대화될 것으로 기대됩니다.
