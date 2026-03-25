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
from typing import Tuple, Any
from types import SimpleNamespace
from envs.environment import EnvironmentSpec
from models.model import MLP
from agents.reinforce.reinforce_network import REINFORCENetwork


class REINFORCEBNetwork(REINFORCENetwork):
    """REINFORCE 베이스라인 적용 알고리즘 네트워크 클래스."""
    def __init__(self,
                 config: SimpleNamespace,
                 environment_spec: EnvironmentSpec):
        """
            부모 클래스인 REINFORCENetwork의 초기화 함수를 호출해서
            네트워크를 초기화 및 정책을 생성하고 추가적으로 베이스라인을 생성
        Args:
            config: 설정
            environment_spec: 환경 정보
        """

        # 1. 부모 클래스의 초기화 호출
        super(REINFORCEBNetwork, self).__init__(config,
                                                environment_spec)
        # 2. 베이스라인 생성
        self.baseline = self.make_baseline()

    def make_baseline(self):
        """
            상태를 입력 받고 베이스라인을 출력하는 베이스라인 모델 생성.

        Returns:
            베이스라인 모델
        """

        # 1. 정책과 같은 크기로 모델 구성
        layer_sizes = self.config.actor_hidden_dims + [1]

        # 2. MLP로 베이스라인 모델 생성
        return MLP(config=self.config,
                   input_size=self.state_size,
                   layer_sizes=layer_sizes)

    def cuda(self):
        """
            정책과 베이스라인 모델의 상태(파라미터와 버퍼)를 GPU로 이동.
        """

        # 1. 부모 클래스의 cuda() 호출
        super(REINFORCEBNetwork, self).cuda()

        # 2. 베이스라인의 cuda() 호출
        self.baseline.cuda(self.config.device_num)

    def forward(self, state, action) -> tuple[Any, Any]:
        """
            학습자에서 정책과 베이스라인의 손실을 계산할 때
            필요한 정보를 제공하기 위해, 네트워크를 실행해서
            해당 상태에서의 행동의 로그 가능도와 베이스라인을 계산
        Args:
            state: 상태
            action: 행동

        Returns:
            행동의 로그 가능도, 베이스라인
        """

        # 1. 행동의 분포를 구함
        distribution = self.policy.distribution(state)

        # 2. 로그 가능도 계산
        log_prob = self._log_prob(distribution, action)

        # 3. 베이스라인 계산
        baseline = self.baseline(state)

        # 4. 로그 가능도와 베이스라인 반환
        return log_prob, baseline