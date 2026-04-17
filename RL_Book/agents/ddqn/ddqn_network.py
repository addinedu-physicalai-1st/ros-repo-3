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
from types import SimpleNamespace
from envs.environment import EnvironmentSpec
from agents.dqn.dqn_network import DQNNetwork


class DDQNNetwork(DQNNetwork):
    """더블 DQN 알고리즘 네트워크 클래스."""

    def __init__(self,
                 config: SimpleNamespace,
                 environment_spec: EnvironmentSpec):
        """
            부모 클래스인 DQNNetwork 초기화 함수를 호출해서
            네트워크를 초기화하고 정책과 가치 함수를 생성
        Args:
            config: 설정
            environment_spec: 환경 정보
        """

        # 부모 클래스 초기화 호출
        super().__init__(config, environment_spec)