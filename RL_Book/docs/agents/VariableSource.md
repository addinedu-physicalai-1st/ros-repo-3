# VariableSource 클래스
`VariableSource`는 네트워크의 상태(파라미터와 버퍼)를 제공하는 인터페이스를 정의하는 추상 클래스이다.

![Agent 클래스 구성도](img/class_diagram.png)

## VariableSource
### 메서드
* **`get_variables`**: 네트워크의 상태(파라미터와 버퍼)를 반환하는 추상 메서드이다.