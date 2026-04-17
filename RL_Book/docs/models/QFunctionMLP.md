# QFunctionMLP 클래스
`QFunctionMLP`는 상태와 행동을 벡터로 입력 받아서 가치를 출력 `MLP`로 구현된 행동기반 가치 함수 클래스이다.

![가치 함수 클래스의 구성도](img/valuefunction_class_diagram.png)

## QFunctionMLP

### 메서드
* **`__init__`**: 모델 정보를 입력 받아서 MLP를 구성한다.
* **`forward`**: 상태와 행동을 입력 받아서 Q-가치를 출력한다.