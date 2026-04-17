# PPONetwork 클래스
`PPONetwork`는 PPO 알고리즘의 학습자 클래스로 `Network`를 상속한다.

![PPO 알고리즘의 에이전트, 학습자, 네트워크 클래스](img/class_diagram.png)


## PPONetwork
### 메서드
* **`__init__`**: 부모 클래스인 `A2CNetwork`의 초기화 함수를 호출해서 네트워크를 초기화하고 정책과 가치 함수를 생성한다.