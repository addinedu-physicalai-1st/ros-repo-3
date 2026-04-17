# GaussianPolicy 클래스
`GaussianPolicy`는 연속 행동에 대해 가우시안 분포를 출력하는 확률적 정책 클래스이다.

![정책 클래스의 구성도](img/class_diagram.png)


## 4. GaussianPolicy 클래스

### 메서드
* **`distribution`**: 가우시안 분포의 (평균, 로그 표준편차)를 `Normal`로 변환해서 반환한다.
* **`select_action`**: 정책을 실행해서 행동의 가우시안 분포를 구한 후 행동을 선택한다.