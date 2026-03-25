# This file is part of RL_Book Project.
#
# Copyright (C) 2025 SeongJin Yoon
#
# RL_Book is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# RL_Book is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
import abc
import torch
import torch.nn as nn
from typing import Callable, List
from types import SimpleNamespace
from torch.distributions import Normal, Categorical

def orthogonal_init(module, activation="tanh"):
    """
        가중치 직교 초기화 (Orthogonal Initialization)
    Args:
        module: 모듈
        activation: 활성 함수
    """

    # 1. 활성 함수의 게인 계산
    gain = 0.01
    if activation != "policy":
        gain = torch.nn.init.calculate_gain(activation)

    # 2. 가중치/편향 초기화
    if isinstance(module, nn.Linear):
        torch.nn.init.orthogonal_(module.weight.data, gain)
        torch.nn.init.zeros_(module.bias.data)


class MLP(nn.Module):
    """순방향 신경망 클래스."""

    def __init__(self,
                 config: SimpleNamespace,
                 input_size: int,
                 layer_sizes: List[int],
                 activation: Callable[[torch.Tensor],torch.Tensor] = nn.ReLU,
                 output_activation: Callable[[torch.Tensor],torch.Tensor]
                 = nn.Identity):
        """
            각 계층의 뉴런 수와 활성 함수를 전달받아서 MLP를 구성
        Args:
            config: 설정
            input_size: 입력 계층의 크기
            layer_sizes: 은닉과 출력 계층의 크기 리스트
            activation: 은닉 계층의 활성 함수
            output_activation: 출력 계층의 활성 함수
        """

        super().__init__()

        # 1. 전달받은 인자 저장
        self.config = config

        # 2. 계층 별 활성함수 리스트 생성
        activations = [activation for _ in layer_sizes[:-1]]
        activations.append(output_activation)

        # 3. MLP 계층 생성
        layers = []
        for output_size, activation in zip(layer_sizes, activations):
            layer = nn.Sequential(nn.Linear(input_size, output_size),
                                  activation())
            layers.append(layer)
            input_size = output_size
        self.layers = nn.Sequential(*layers)

        # 4. 모델 파라미터 초기화
        self.apply(orthogonal_init)

    def forward(self, x: torch.Tensor):
        """
            전달 받은 입력 데이터를 전체 계층에 대해 순차적으로 실행
        Args:
            x: 입력 데이터

        Returns:
            모델의 출력 데이터
        """

        # 1. MLP 계층 실행
        for layer in self.layers:
            x = layer(x)

        return x  # 2. 실행 결과 반환


class Policy(abc.ABC):
    """정책 클래스의 최상위 클래스."""
    # 상태외 행동의 크기 변수 선언
    state_size = 0      # 상태 크기
    action_size = 0     # 행동 크기


class StochasticPolicy(Policy):
    """행동의 확률 분포를 출력하는 정책."""

    @abc.abstractmethod
    def distribution(self, state):
        """
            정책이 출력한 분포의 파라미터를 이용해서 행동의 확률 분포를 생성
        Args:
            state: 상태

        Returns:
            행동의 분포
        """

    @abc.abstractmethod
    def select_action(self, state: torch.Tensor, training_mode: bool = True):
        """
            정책을 실행해서 행동의 확률 분포를 구한 후 행동을 선택
        Args:
            state: 상태
            training_mode: 훈련 모드

        Returns:
            선택된 행동
        """


class CategoricalPolicy(StochasticPolicy):
    """카테고리 분포를 출력하는 정책."""

    def distribution(self, state):
        """
            카테고리 분포의 확률 벡터를 Categorical로 변환한다.
        Args:
            state: 상태

        Returns:
            행동의 카테고리 분포
        """
        return Categorical(self(state))

    @torch.no_grad()
    def select_action(self, state: torch.Tensor, training_mode: bool = True):
        """
            정책을 실행해서 행동의 카테고리 분포를 구한 후 행동을 선택
        Args:
            state: 상태
            training_mode: 훈련 모드

        Returns:
            선택된 행동
        """

        # 1. 카테고리 분포 생성
        distribution = self.distribution(state)

        # 2. 카테고리 분포에서 행동 선택
        if training_mode:
            # 학습 모드: 행동 샘플링
            action = distribution.sample()
        else:
            # 추론 모드: 최대 확률로 선택
            action = distribution.probs.argmax(dim=-1, keepdim=True)
        return action

class GaussianPolicy(StochasticPolicy):
    """가우시안 분포를 출력하는 정책."""


    def distribution(self, state):
        """
            가우시안 분포의 (평균, 로그 표준편차)를 Normal로 변환해서 반환
        Args:
            state: 상태

        Returns:
            행동의 가우시안 분포
        """

        # 1. 정책 실행
        mean, log_std = self(state)
        log_std = torch.clamp(log_std, min=-20, max=2)  # log_std 안정화

        # 2. 표준 편차 계산
        std = log_std.exp()
        action_std = torch.ones_like(mean) * std

        # 3. 가우시안 분포 생성
        return Normal(mean, action_std)

    @torch.no_grad()
    def select_action(self, state: torch.Tensor, training_mode: bool = True):
        """
            정책을 실행해서 행동의 가우시안 분포를 구한 후 행동을 선택
        Args:
            state: 상태
            training_mode: 훈련 모드

        Returns:
            선택된 행동
        """

        # 1. 가우시안 분포 생성
        distribution = self.distribution(state)

        # 2. 가우시안 분포에서 행동 선택
        if training_mode:
            # 학습 모드: 행동 샘플링
            action = distribution.sample()
            action = torch.tanh(action)
        else:
            # 추론 모드: 평균으로 선택
            action = distribution.mean
        return action.detach()


class CategoricalPolicyMLP(MLP, CategoricalPolicy):
    """카테고리 분포를 출력하는 MLP 정책."""

    def __init__(self,
                 config: SimpleNamespace,
                 state_size: int,
                 hidden_dims: List[int],
                 action_size: int):
        """
            모델 정보를 입력 받아서 MLP를 구성
        Args:
            config: 설정
            state_size: 상태의 크기
            hidden_dims: 은닉 계층의 뉴런 수 리스트
            action_size: 행동의 크기
        """

        # 1. 계층 별 뉴런 수 리스트 생성
        layer_sizes = hidden_dims + [action_size]

        # 2. MLP 생성
        super().__init__(config,
                         state_size,
                         layer_sizes,
                         activation=nn.Tanh,
                         output_activation=nn.Identity)

        # 3. 전달 받은 인자 저장
        self.state_size = state_size
        self.action_size = action_size

        # 4. 소프트맥스 활성 함수
        self.Softmax = nn.Softmax(dim=-1)

        # 5. 모델 파라미터 초기화
        self.apply(lambda m: orthogonal_init(m, "policy"))

    def forward(self, state):
        """
            정책을 실행해서 카테고리 분포의 확률 벡터를 출력
        Args:
            state: 상태

        Returns:
            이산 행동의 확률 벡터
        """

        # 1. MLP 실행 및 로짓 출력
        logits = super().forward(state)

        # 2. 소프트맥스 실행
        return self.Softmax(logits)


class GaussianPolicyMLP(MLP, GaussianPolicy):
    """가우시안 분포를 출력하는 MLP 정책."""

    def __init__(self,
                 config: SimpleNamespace,
                 state_size: int,
                 hidden_dims: List[int],
                 action_size: int,
                 log_std_init: float = 0.0):
        """
            모델 정보를 입력 받아서 MLP를 구성하고
            가우시안 분포의 평균을 출력하는 해드와
            로그 표준편차를 학습하는 모델 파라미터를 정의한다.
        Args:
            config: 설정
            state_size: 상태의 크기
            hidden_dims: 은닉 계층의 뉴런 수 리스트
            action_size: 행동의 크기
            log_std_init: 로그 표준편차 파라미터의 초기화 값
        """

        # 1. MLP 생성
        super().__init__(config, state_size, hidden_dims)

        # 2. 전달 받은 인자 저장
        self.state_size = state_size
        self.action_size = action_size

        # 3. 평균 해드 구성
        self.mean_head = nn.Linear(hidden_dims[-1], self.action_size)

        # 4. 로그 표준편차 파라미터 정의
        self.log_std = nn.Parameter(torch.ones(self.action_size) * log_std_init,
                                    requires_grad=True)

        # 5. 모델 파라미터 초기화
        self.apply(orthogonal_init)

    def forward(self, state):
        """
            정책을 실행해서 가우시안 분포의 평균을 출력하고
            평균과 로그 표준편차를 반환한다.
        Args:
            state: 상태

        Returns:
            연속 행동의 가우시안 분포의 평균과 로그 표준 편차
        """

        # 1. 은닉 계층까지 실행
        x = super(GaussianPolicyMLP, self).forward(state)

        # 2. 평균 출력
        mean = self.mean_head(x)

        # 3. 평균/로그 표준 편차 반환
        return mean, self.log_std


class ValueFunction(abc.ABC):
    """가치 함수 클래스의 최상위 클래스."""

    # 상태의 크기 변수 선언
    state_size = 0


class StateValueFunction(ValueFunction):
    """상태 기반 가치 함수."""


class ActionValueFunction(ValueFunction):
    """행동 기반 가치 함수."""

    # 행동의 크기 변수 선언
    action_size = 0


class ValueFunctionMLP(MLP, StateValueFunction):
    """상태 기반의 가치 함수 클래스 (A2C, PPO에서 사용)"""

    def __init__(self,
                 config: SimpleNamespace,
                 state_size: int,
                 hidden_dims: List[int]):
        """
            모델 정보를 입력 받아서 MLP를 구성
        Args:
            config: 설정
            state_size: 상태의 크기
            hidden_dims: 은닉 계층의 뉴런 수 리스트
        """

        # 1. 전달 받은 인자 저장
        self.state_size = state_size

        # 2. 계층 별 뉴런 수 목록 생성
        output_size = 1
        layer_sizes = hidden_dims + [output_size]

        # 3. MLP 생성
        super().__init__(config, state_size, layer_sizes)

    def forward(self, state):
        """
            상태를 입력 받아서 가치를 출력
        Args:
            state: 상태

        Returns:
            가치
        """

        # MLP 계층 별 실행
        return super().forward(state)

class QFunctionMLP(MLP, ActionValueFunction):
    """상태와 행동을 입력 받아서 Q-가치를 출력하는 Q 가치 함수 클래스."""
    def __init__(self,
                 config: SimpleNamespace,
                 state_size: int,
                 action_size: int,
                 hidden_dims: List[int]):
        """
            모델 정보를 입력 받아서 MLP를 구성
        Args:
            config: 설정
            state_size: 상태의 크기
            action_size: 행동의 크기
            hidden_dims: 은닉 계층의 뉴런 수 리스트
        """

        # 1. 전달 받은 인자 저장
        self.state_size = state_size
        self.action_size = action_size

        # 2. 계층 별 뉴런 수 목록 생성
        input_size = state_size + action_size
        output_size = 1
        layer_sizes = hidden_dims + [output_size]

        # 3. MLP 생성
        super().__init__(config, input_size, layer_sizes)

        # 4. 모델 파라미터 초기화
        self.apply(orthogonal_init)

    def forward(self, state, action):
        """
            상태와 행동을 입력 받아서 Q-가치를 출력
        Args:
            state: 상태
            action: 행동

        Returns:
            Q-가치
        """
        # 1. 상태와 행동을 연결
        state_action = torch.cat([state, action], dim=1)

        # 2. MLP 계층 별 실행
        return super().forward(state_action)


class QFunctionMLPDQN(MLP, ActionValueFunction):
    """
        상태를 입력하고 모든 이산 행동에 대한 Q 가치를 한꺼번에 출력하는 Q 가치 함수 클래스
        (DQN, Double DQN에서 사용)
    """
    def __init__(self,
                 config: SimpleNamespace,
                 state_size: int,
                 action_size: int,
                 hidden_dims: List[int]):
        """
            모델 정보를 입력 받아서 MLP를 구성한다.
        Args:
            config: 설정
            state_size: 상태의 크기
            action_size: 행동의 크기
            hidden_dims: 은닉 계층의 뉴런 수 리스트
        """
        # 1. 전달 받은 인자 저장
        self.state_size = state_size
        self.action_size = action_size

        # 2. 계층 별 뉴런 수 목록 생성
        input_size = state_size
        output_size = action_size
        layer_sizes = hidden_dims + [output_size]

        # 3. MLP 생성
        super().__init__(config, input_size, layer_sizes)

        # 4. 모델 파라미터 초기화
        self.apply(orthogonal_init)

    def forward(self, state):
        """
            상태를 입력 받아서 모든 이산 행동의 Q-가치를 출력
        Args:
            state: 상태

        Returns:
            모든 행동의 Q-가치
        """
        # MLP 계층 별 실행
        return super().forward(state)