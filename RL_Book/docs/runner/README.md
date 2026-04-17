# 러너 | [환경 루프](README_EnvironmentLoop.md)
러너는 강화학습의 구성 요소를 생성하고 추론과 학습을 위한 전체적인 실행을 관장한다.

## 1. 디렉토리 구성
| 파일 이름                  | 설명                        |
|:-----------------------|:--------------------------|
| `runner.py`	          | 강화학습 프레임워크 실행자인 `Runner`가 정의돼 있다. | 
| `multienv_runner.py`  |다중 환경에서 동기식으로 분산 처리를 하는 강화학습 프레임워크 실행자인 `MultiEnvRunner`가 정의돼 있다.  | 
| `multienv_async_runner.py`  | 다중 환경에서 비동기식으로 분산 처리를 하는 강화학습 프레임워크 실행자인 `MultiEnvAsyncRunner`가 정의돼 있다.  | 

##  2. 클래스
러너의 클래스는 다음과 같이 구성돼 있다.

![러너 클래스의 구성도](img/class_diagram.png)

* **[`Runner`](Runner.md)**: 환경이 한 개일 때 강화학습의 구성 요소를 생성하고 추론과 학습을 위한 전체적인 실행을 관장
* **[`MultiEnvRunner`](MultiEnvRunner.md)**: 여러 개의 환경을 동기적으로 분산 처리를 하기 위해 강화학습의 구성 요소를 생성하고 추론과 학습을 위한 전체적인 실행을 관장
* **[`MultiEnvAsyncRunner`](MultiEnvAsyncRunner.md)**: 여러 개의 환경을 비동기적으로 분산 처리를 하기 위해 강화학습의 구성 요소를 생성하고 추론과 학습을 위한 전체적인 실행을 관장