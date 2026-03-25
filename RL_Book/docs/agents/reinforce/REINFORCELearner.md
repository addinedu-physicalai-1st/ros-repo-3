# REINFORCELearner 클래스
`REINFORCELearner`는 PPO 알고리즘의 학습자 클래스로 `Learner`를 상속한다.

![REINFORCE 알고리즘의 에이전트, 학습자, 네트워크 클래스](img/class_diagram.png)

## REINFORCELearner 클래스
### 속성
* **옵티마이저(`optimizer`)**: 딥러닝 모델의 학습을 위한 최적화 알고리즘으로 `Adam`을 사용한다.
* **학습률 스케줄러(`policy_lr_scheduler`)**: 학습률을 스케줄링 하기 위한 코사인 스케줄러이다. 초기 학습률을 0도에서 90도 사이의 코사인 곡선을 따라 최대 환경 스텝까지 서서히 감소시키는 방식으로 구현돼 있다.

### 메서드
* **`__init__`**: Learner 클래스의 초기화 메서드를 호출해 학습자를 초기화 하고 `Adam` 옵티마이저 학습률 스케줄러를 생성한다.
* **`_calc_returns`**: 데이터셋에 저장된 모든 트랜지션에 대해 몬테카를로 리턴을 계산해서 데이터셋에 추가 필드로 저장한다.
* **`update`**: REINFORCE 알고리즘의 목적 함수에 따라 손실을 계산해서 정책을 학습하고 성능 정보를 로깅한다.