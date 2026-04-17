# CategoricalPolicy 클래스
`CategoricalPolicy`는 이산 행동에 대해 카테고리 분포를 출력하는 확률적 정책 클래스이다.

![정책 클래스의 구성도](img/class_diagram.png)

## CategoricalPolicy 클래스

### 메서드
* **`distribution`**: 카테고리 분포의 확률 벡터를 `Categorical`로 변환한다.
* **`select_action`**: 정책을 실행해서 행동의 카테고리 분포를 구한 후 행동을 선택한다.