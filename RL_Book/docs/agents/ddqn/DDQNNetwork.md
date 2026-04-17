# DDQNNetwork 클래스
`DDQNNetwork`는 더블 DQN 알고리즘의 학습자 클래스로 `DQNNetwork`를 상속한다.

![더블 DQN 알고리즘의 에이전트, 학습자, 네트워크 클래스](img/class_diagram.png)


## DDQNNetwork

### 속성
* **가치 함수 모델(`critic`)**: Q-가치 함수를 나타내는 딥러닝 모델로 `QFunctionMLPDQN`로 생성된다.
* **타깃 가치 함수 모델(`target_critic`)**: 타깃 Q-가치 함수를 나타내는 딥러닝 모델로 가치 함수를 복사해서 만든다.
 
### 메서드
* **`__init__`**: 부모 클래스인 `DQNNetwork`의 초기화 함수를 호출해서 네트워크를 초기화하고 가치 함수를 생성하며 탐색을 위해 입실론 그리디 객체를 생성한다