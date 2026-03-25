# QFunctionMLPDQN 클래스
`QFunctionMLPDQN`은 상태를 입력 받아서 모든 이산 행동에 대한 Q-가치를 한꺼번에 출력하는 MLP로 구현된 행동기반 가치 함수 클래스이다.

![가치 함수 클래스의 구성도](img/valuefunction_class_diagram.png)

## QFunctionMLP

### 메서드
* **`__init__`**: 모델 정보를 입력 받아서 MLP를 구성한다.
* **`forward`**: 상태를 입력 받아서 모든 이산 행동의 Q-가치를 출력한다.