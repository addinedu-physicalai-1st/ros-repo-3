# EnvironmentSpec 클래스
`EnvironmentSpec`는 환경의 MDP 정보를 표준화된 형태로 제공하는 클래스이다.

![환경 클래스의 구성도](img/class_diagram.png)

## EnvironmentSpec
### 속성
* **연속 행동 여부(`b_continuous_action`)**: 행동이 연속인지 여부를 나타낸다.
* **행동 스펙(`action_spec`)**: 행동의 모양, 데이터 타입, 행동 공간의 상한과 하한을 제공한다.
* **행동 크기(`action_size`)**: 행동을 벡터로 나타내면 행동 벡터의 크기를 나타낸다.
* **상태 스펙(`state_spec`)**: 상태의 모양, 데이터 타입을 제공한다.

### 메서드
**`__init__:`** 전달받은 환경 정보를 일부는 그대로 저장하고 일부는 데이터의 타입, 모양, 상한, 하한을 표현하는 `Array` 또는 `BoundedArray` 객체로 변환해 제공한다.