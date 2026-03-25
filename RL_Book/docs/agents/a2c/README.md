# A2C 알고리즘

## 1. 디렉토리 구성
| 파일 이름               |설명                        | 설명                                                |
|:--------------------|:--                          |:--------------------------------------------------|
| `a2c.py`       | 에이전트 클래스 | A2C 알고리즘의 에이전트인 `A2C`가 정의돼 있다                                                  | 
| `a2c_learner.py`      | 학습자 클래스 | A2C 알고리즘의 학습자인 `A2CLearner`가 정의돼 있다. | 
| `a2c_network.py`      | 네크워크 클래스 | A2C 알고리즘의 네트워크인 `A2CNetwork`가 정의돼 있다. | 

##  2. 클래스
A2C 알고리즘은 에이전트, 네트워크, 학습자 클래스인 `A2C`, `A2CLearner`, `A2CNetwork`로 구현돼 있으며 
이들은 `Agent`, `Learner`, `Network`를 상속하고 있다

![A2C 알고리즘의 에이전트, 학습자, 네트워크 클래스](img/class_diagram.png)

* [`A2C`](A2C.md)는 A2C 알고리즘의 에이전트 클래스로 `Agent`를 상속한다.
* [`A2CNetwork`](A2CNetwork.md)는 A2C 알고리즘의 네트워크 클래스로 `Network`를 상속한다.
* [`A2CLearner`](A2CLearner.md)는 A2C 알고리즘의 학습자 클래스로 `Learner`를 상속한다.

##  3. 클래스 관계도
A2C 알고리즘을 실행하면 다음과 같은 순서로 관련 객체가 생성된다.
![클래스 관계도](img/class_tree.png)

객체의 생성 순서를 설명하면 다음과 같다.
* `main.py`는 `Runner`를 생성하고 `run()` 메서드를 호출한다.
* `Runner`의 `run()` 메서드에서는 강화학습의 핵심 구성 모듈인 에이전트 `A2C`와 에이전트와 환경이 상호작용을 하도록 환경 루프 `EnvironmentLoop`를 생성한다.
* `A2C`는 에이전트를 구성하는 네트워크, 학습자, 데이터셋인 `A2CNetwork`, `A2CLearner`, `RolloutBuffer를`를 생성한다.
* `A2CNetwork`는 연속 행동인 경우에는 `GaussianPolicyMLP`를 생성하고, 이산 행동인 경우에는 `CategoricalPolicyMLP`를 생성한다.
* 환경 루프 `EnvironmentLoop`는 환경인 `OpenGym`과 액터 `Actor`를 생성한다.