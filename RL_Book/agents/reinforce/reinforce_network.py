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
import torch
from types import SimpleNamespace
from envs.environment import EnvironmentSpec
from models.model import GaussianPolicyMLP, CategoricalPolicyMLP
from agents.base import Network


class REINFORCENetwork(Network):
    """REINFORCE 알고리즘 네트워크 클래스."""
    def __init__(self,
                 config: SimpleNamespace,
                 environment_spec: EnvironmentSpec):
        """
            부모 클래스인 Network의 초기화 함수를 호출해서
            네트워크를 초기화하고 정책을 생성
        Args:
            config: 설정
            environment_spec: 환경 정보
        """

        # 1. 네트워크 초기화
        super().__init__(config, environment_spec)

        # 2. 정책 모델 생성
        self.policy = self.make_policy()

    def make_policy(self):
        """
            연속 행동인 경우
            - 가우시안 분포를 출력하는 MLP 정책인 GaussianPolicyMLP를 생성
            이산 행동인 경우
            - 카테고리 분포를 출력하는 MLP 정책인 CategoricalPolicyMLP를 생성

        Returns:
            정책 모델
        """

        # 1. 정책 인자 정의
        hidden_dim = self.config.actor_hidden_dims
        policy_argument = [self.config,
                           self.state_size,
                           hidden_dim,
                           self.action_size]

        # 2. 연속 행동: 가우시안 정책 생성
        if self.b_continuous_action:
            return GaussianPolicyMLP(*policy_argument)

        # 3. 이산 행동: 카테고리 정책 생성
        return CategoricalPolicyMLP(*policy_argument)

    def select_action(self,
                      state: torch.Tensor,
                      total_n_timesteps: int) -> torch.Tensor:
        """
            정책 모델을 실행해서 행동을 선택
        Args:
            state: 상태
            total_n_timesteps: 현재 타임 스텝

        Returns:
            선택된 행동
        """
        # 정책을 실행해서 행동을 선택
        return self.policy.select_action(state, self.config.training_mode)

    def cuda(self):
        """정책 모델의 상태(파라미터와 버퍼)를 GPU로 이동."""

        self.policy.cuda(self.config.device_num)    # 정책 모델 cuda() 호출

    def forward(self, state, action) -> torch.Tensor:
        """
            학습자에서 정책의 손실을 계산할 때
            필요한 정보를 제공하기 위해, 네트워크를 실행해서
            해당 상태에서의 행동의 로그 가능도를 계산
        Args:
            state: 상태
            action: 행동

        Returns:
            행동의 로그 가능도
        """
        # 1. 행동의 분포를 구함
        distribution = self.policy.distribution(state)

        # 2. 로그 가능도 계산
        log_prob = self._log_prob(distribution, action)

        # 3. 로그 가능도 반환
        return log_prob
