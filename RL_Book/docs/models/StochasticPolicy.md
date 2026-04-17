# StochasticPolicy 클래스
`StochasticPolicy`는 행동의 확률 분포를 출력하는 확률적 정책 클래스이다.

![정책 클래스의 구성도](img/class_diagram.png)

## StochasticPolicy
`StochasticPolicy`는 행동의 확률 분포를 출력하는 확률적 정책 클래스이다.

### 메서드
* **`distribution`**: 정책이 출력한 분포의 파라미터를 이용해서 행동의 확률 분포를 생성하는 추상 메서드이다.
* **`select_action`**: 정책을 실행해서 행동의 확률 분포를 구한 후 행동을 선택하는 추상 메서드이다.