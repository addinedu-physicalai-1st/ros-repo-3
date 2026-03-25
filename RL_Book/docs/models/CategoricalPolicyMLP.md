# CategoricalPolicyMLP 클래스
`CategoricalPolicyMLP`는 상태 벡터를 입력 받아서 이산 행동에 대해 카테고리 분포를 출력하는 `MLP`로 구현된 확률적 정책 클래스이다.

![정책 클래스의 구성도](img/class_diagram.png)

## CategoricalPolicyMLP
### 속성
* **소프트맥스(`Softmax`)**: 카테고리 분포의 확률 벡터를 출력하기 위한 출력 계층의 활성 함수이다.

### 메서드
* **`__init__`**: 모델 정보를 입력 받아서 `MLP`를 구성한다.
* **`forward`**: 정책을 실행해서 카테고리 분포의 확률 벡터 $𝜇=(𝜇_1,𝜇_2,...,𝜇_K)^T$를 출력한다.