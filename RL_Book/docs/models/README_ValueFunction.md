# [정책](README.md) | 가치 함수
가치 함수는 정책을 평가하는 함수이다.

## 1. 디렉토리 구성
| 파일 이름                  | 설명                                                                                                                                                           |
|:-----------------------|:-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `models.py`	          | 가치 함수 클래스인 `ValueFunction`, `StateValueFunction`, `ActionValueFunction`, `ValueFunctionMLP`, `QFunctionMLP`, `QFunctionMLPDQN와` 순방향 신경망 클래스인 `MLP`가 정의돼 있다.| 


##  2. 클래스
가치 함수의 클래스 구성도는 다음과같다. 
가장 상위에는 가치 함수의 베이스 클래스인 `ValueFunction`이 있고 
하위에 상속받는 클래스는 ➊ 상태 기반의 가치 함수인지, 
행동 기반의 가치 함수인지 ➋ Q-가치 함수가 하나의 행동에 대해 Q-가치를 출력하는지, 
모든 이산 행동에 대해 Q-가치를 한꺼번에 출력하는지 ➌ 상태 데이터의 종류에 따라 구분된다.
단, 현재는 상태 데이터가 벡터인 경우만 확장하고 있다.

![가치 함수 클래스의 구성도](img/valuefunction_class_diagram.png)

가치 함수 클래스는 `ValueFunction`이라는 베이스 클래스에서 시작한다.
* [`ValueFunction`](ValueFunction.md): 가치 함수 클래스의 최상위 클래스

가치 함수는 상태 기반의 가치 함수 `StateValueFunction`과 행동 기반의 가치 함수 `ActionValueFunction`으로 확장된다.
* [`StateValueFunction`](StateValueFunction.md): 상태 기반 가치 함수
* [`ActionValueFunction`](ActionValueFunction.md): 행동 기반 가치 함수

상태 데이터가 벡터인 경우 가치 함수를 순방향 신경망 모델로 정의하기 위해 `MLP` 클래스에서 상속을 받는다.
* [`MLP`](MLP.md): 순방향 신경망 클래스

최종적으로 사용하게 될 상태 기반의 가치 함수는 다음과 같다.
* [`ValueFunctionMLP`](ValueFunctionMLP.md): 상태 기반의 가치 함수 클래스(A2C, PPO에서 사용)

Q-가치 함수는 하나의 행동에 대해 Q-가치를 출력하는 `QFunctionMLP`와 모든 이산 행동에 대해 Q-가치를 한꺼번에 출력하는 `QFunctionMLPDQN`으로 확장된다.
* [`QFunctionMLP`](QFunctionMLP.md): 상태와 행동을 입력 받아서 Q-가치를 출력하는 Q-가치 함수 클래스
* [`QFunctionMLPDQN`](QFunctionMLPDQN.md): 상태를 입력 받아서 모든 이산 행동에 대한 Q-가치를 한꺼번에 출력하는 Q-가치 함수 클래스(DQN, 더블 DQN에서 사용)