# A2CNetwork 클래스
`A2CNetwork`는 A2C 알고리즘의 네크워크 클래스로 `Network`를 상속한다.

![A2C 알고리즘의 에이전트, 학습자, 네트워크 클래스](img/class_diagram.png)

## A2CNetwork

### 속성
* **정책 모델(`policy`)**: 정책을 나타내는 딥러닝 모델로 이산 행동인 경우 `CategoricalPolicyMLP`로 연속 행동인 경우 `GaussianPolicyMLP`로 생성된다.
* **가치 함수 모델(`critic`)**: 가치 함수를 나타내는 딥러닝 모델로 `ValueFunctionMLP`로 생성된다.

### 메서드
* **`__init__`**: 부모 클래스인 Network의 초기화 함수를 호출해서 네트워크를 초기화하고 정책과 가치 함수를 생성한다.
* **`make_policy`**: 연속 행동인 경우 가우시안 분포를 출력하는 MLP 정책인 `GaussianPolicyMLP`를 생성하고, 이산 행동인 경우 카테고리 분포를 출력하는 MLP 정책인 `CategoricalPolicyMLP`를 생성한다.
* **`make_ciritic`**: 상태 기반의 MLP 가치 함수인 `ValueFunctionMLP`를 생성한다.
* **`select_action`**: 정책 모델을 실행해서 행동을 선택한다. 학습 모드와 추론 모드에서 행동을 선택하는 방식이 달라진다.
* **`cuda`**: 정책과 가치 함수 모델의 상태(파라미터와 버퍼)를 GPU로 이동한다.
* **`forward`**:학습자에서 정책과 가치 함수의 손실을 계산할 때 필요한 정보를 한꺼번에 제공하기 위해 네트워크를 실행해서 해당 상태에서의 행동의 로그 가능도, 엔트로피, 가치를 계산한다.