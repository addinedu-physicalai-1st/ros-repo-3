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
from copy import deepcopy
from utils.util import hard_update, soft_update
from envs.environment import EnvironmentSpec
from models.model import QFunctionMLPDQN
from agents.base import Network
from utils.action_selectors import EpsilonGreedyActionSelector

class DQNNetwork(Network):
    """DQN 알고리즘 네트워크 클래스."""
    def __init__(self,
                 config: SimpleNamespace,
                 environment_spec: EnvironmentSpec):
        """
            부모 클래스인 Network의 초기화 함수를 호출해서
            네트워크를 초기화하고 가치 함수를 생성하며 탐색을 위해 입실론 그리디를 생성
        Args:
            config: 설정
            environment_spec: 환경 정보
        """

        # 1. 네트워크 초기화
        super(DQNNetwork, self).__init__(config, environment_spec)

        # 2. 가치 함수 모델 생성
        self.critic = self.make_critic()

        # 3. 타깃 가치 함수 모델 생성
        self.target_critic = deepcopy(self.critic)
        hard_update(self.critic, self.target_critic)

        # 4. 입실론 그리디 생성
        self.action_selector = EpsilonGreedyActionSelector(self.config)

    def make_critic(self):
        """
            모든 이산 행동에 대한 Q-가치를 한꺼번에 출력하는
            행동 기반 가치 함수 클래스인 QFunctionMLPDQN을 생성.
        Returns:
            Q-가치 함수 모델
        """

        # Q 가치 함수 모델 생성
        return QFunctionMLPDQN(config=self.config,
                               state_size=self.state_size,
                               action_size=self.action_size,
                               hidden_dims=self.config.critic_hidden_dims)

    def hard_update_target(self):
        """가치 함수 모델의 파라미터를 타깃 가치 함수 모델에 복제."""
        # 타깃 하드 업데이트
        hard_update(self.critic, self.target_critic)

    def soft_update_target(self):
        """
            가치 함수 모델의 파라미터와 타깃 가치 함수 모델의 파라미터를
            가중 평균하여 타깃 가치 함수 모델에 적용.
        """
        # 타깃 소프트 업데이트
        soft_update(self.critic, self.target_critic, self.config.tau)

    @torch.no_grad()
    def select_action(self,
                      state: torch.Tensor,
                      total_n_timesteps: int) -> torch.Tensor:
        """
            입실론 그리디 알고리즘에 따라 가치 함수에서 최대 가치를 갖는
            행동을 선택하거나 랜덤한 행동을 선택.
        Args:
            state: 상태
            total_n_timesteps: 현재 타임 스텝

        Returns:
            선택된 행동
        """

        # 1. 모든 행동의 Q 가치를 계산
        q_values = self.critic(state)

        # 2. 입실론 그리디를 이용해서 행동 선택
        chosen_actions = self.action_selector.select_action(
            agent_input=q_values,
            total_n_timesteps=total_n_timesteps)

        # 3. 선택한 행동을 반환
        return chosen_actions

    def cuda(self):
        """가치 함수와 타깃 가치 함수 모델의 상태(파라미터와 버퍼)를 GPU로 이동."""
        # 1. 가치 함수 모델 cuda() 호출
        self.critic.cuda(self.config.device_num)

        # 2. 타깃 가치 함수 모델 cuda() 호출
        self.target_critic.cuda(self.config.device_num)