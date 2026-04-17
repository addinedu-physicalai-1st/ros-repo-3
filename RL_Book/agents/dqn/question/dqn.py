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
from utils.logging import Logger
from envs.environment import Environment
from agents.agent import Agent
from agents.dqn.dqn_network import DQNNetwork
from agents.dqn.dqn_learner import DQNLearner


class DQN(Agent):
    """DQN 알고리즘 에이전트 클래스."""

    def __init__(self,
                 config: SimpleNamespace,
                 logger: Logger,
                 env: Environment):
        """
            DQN 알고리즘을 실행하는 학습자, 네트워크, 데이터셋으로 구성된 에이전트를 생성
        Args:
            config: 설정
            logger: 로거
            env: 환경
        """

        # 1. 연속 행동인 경우 예외 발생
        if env.environment_spec().b_continuous_action:
            raise Exception("DQN doesn't support continuous action space")

        # 2. 에이전트 초기화
        super(DQN, self).__init__(
            config=config,
            logger=logger,
            env=env,
            network_class=DQNNetwork,
            learner_class=DQNLearner,
            policy_type="off_policy")