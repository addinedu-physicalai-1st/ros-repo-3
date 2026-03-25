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
import numpy as np
from utils.array_types import Array, BoundedArray


class Environment(abc.ABC):
    """
        환경을 정의하는 베이스 클래스.
    """

    @abc.abstractmethod
    def render(self):
        """
            강화학습 환경을 화면에 렌더링.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def reset(self):
        """
            환경을 리셋한다.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def step(self, action):
        """
            환경에 대해 행동을 취하고 결과를 반환.
        Args:
            action: 행동

        Returns:
            (다음 상태, 보상, 에피소드 종료 여부, 환경 정보)
        """
        raise NotImplementedError

    @abc.abstractmethod
    def close(self):
        """
            환경 종료
        """
        raise NotImplementedError

    @abc.abstractmethod
    def environment_spec(self):
        """
            환경의 MDP 정보를 표준화된 형태로 제공.
        Returns:
            환경 정보
        """
        raise NotImplementedError

    @abc.abstractmethod
    def max_episode_limit(self):
        """
            환경의 최대 에피소드 길이를 반환.
        Returns:
            환경의 최대 에피소드 길이
        """
        raise NotImplementedError

class EnvironmentSpec:
    """
        환경의 MDP 정보를 표준화된 형태로 제공.
    """

    def __init__(self,
                 action_shape: list,
                 action_dtype,
                 action_high,
                 action_low,
                 action_size,
                 b_continuous_action,
                 state_shape: list,
                 state_dtype=np.float32,):
        """
            1) 연속 행동 여부, 2) 행동 스펙, 3) 행동 크기, 4) 상태 스펙을 초기화
        Args:
            action_shape: 행동의 모양
            action_dtype: 행동의 데이터 타입
            action_high: 행동의 상한
            action_low: 행동의 하한
            action_size: 행동의 크기 (이산 행동의 개수)
            b_continuous_action: 연속 행동 여부
            state_shape: 상태의 모양
            state_dtype: 상태의 데이터 타입
        """

        # 1. 연속 행동 여부 설정
        self.b_continuous_action = b_continuous_action

        # 2. 행동 정보 및 행동 크기 정의
        self.action_spec = BoundedArray(action_shape,
                                        action_dtype,
                                        action_low,
                                        action_high)
        self.action_size = action_size

        # 3. 상태 정보 정의
        self.state_spec = Array(state_shape, state_dtype)
