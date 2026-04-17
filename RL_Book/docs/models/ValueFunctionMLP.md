# ValueFunctionMLP 클래스
`ValueFunctionMLP`은 상태 벡터를 입력 받아서 가치를 출력하는 `MLP`로 구현된 상태 기반 가치 함수 클래스이다.

![가치 함수 클래스의 구성도](img/valuefunction_class_diagram.png)

## ValueFunctionMLP

### 메서드
* **`__init__`**: 모델 정보를 입력 받아서 MLP를 구성한다.
* **`forward`**: 상태를 입력 받아서 가치를 출력한다.