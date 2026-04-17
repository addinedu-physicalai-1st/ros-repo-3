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
"""action_selector.py: 입실론 그리디 탐색 기법 정의."""
from types import SimpleNamespace

import numpy as np
import torch
from torch.distributions import Categorical


class EpsilonGreedyActionSelector():
    """입실론 그리디 탐색 기법."""

    def __init__(self, config: SimpleNamespace):
        """입실론 스케쥴러 생성하고 입실론 값을 초기화 한다.

        Args:
            config: 설정 객체
        """

        # 1. 설정 저장
        self.config = config

        # 2. 𝜀 스케쥴러 생성
        self.schedule = DecayThenFlatSchedule(
            start=config.epsilon_start,
            finish=config.epsilon_finish,
            time_length=config.epsilon_anneal_time,
            decay="linear")

        # 3. 𝜀 초기화
        self.epsilon = self.schedule.eval(0)

    def select_action(self, agent_input, total_n_timesteps: int):
        """입실론 그리디 방식으로 최적 행동 또는 랜덤한 행동을 선택한다.

        Args:
            agent_input: 정책이 예측한 행동의 확률 벡터
            total_n_timesteps: 현재 타입 스텝

        Returns:
            picked_actions: 선택된 행동
        """

        # 1. 현재 스텝에 맞는 𝜀을 계산
        self.epsilon = self.schedule.eval(total_n_timesteps)

        # 2. 추론 모드에서는 𝜀=0
        if not self.config.training_mode:
            # 랜덤한 행동을 선택하지 않음
            self.epsilon = 0.0

        # 3. 랜덤 행동 선택
        random_actions = \
            Categorical(torch.ones_like(agent_input).float()).sample().long()

        # 4. 최대 Q-가치를 갖는 행동 선택
        selected_action = agent_input.max(dim=-1)[1]

        # 5. 행동 선택
        # 난수 생성
        random_numbers = torch.rand_like(agent_input[:, 0])
        # 행동 선택을 위한 이진 변수 생성
        pick_random = (random_numbers < self.epsilon).long()
        # 난수 < 𝜀: 1 (랜덤 행동 선택)
        # 난수 ≥ 𝜀: 0 (최적 행동 선택)
        picked_actions = pick_random * random_actions \
                         + (1 - pick_random) * selected_action

        return picked_actions


class DecayThenFlatSchedule():
    """[start, finish] 구간에서는 감쇄, 구간 이후에는 값을 유지하는 스케쥴러."""

    def __init__(self,
                 start: int,
                 finish: int,
                 time_length: int,
                 decay: str = "exp"):
        """감쇄 스케쥴러 초기화.

        Args:
            start: 감쇄 시작 시점
            finish: 감쇄 종료 시점
            time_length: 전체 타입 스텝의 길이
            decay: 감쇄 방식 {linear: 선형 감쇄, exp: 지수 감쇄)
        """

        assert decay in ["linear", "exp"]

        self.start = start
        self.finish = finish
        self.time_length = time_length
        self.delta = (self.start - self.finish) / self.time_length
        self.decay = decay

        if self.decay in ["exp"]:
            self.exp_scaling = (-1) * self.time_length / np.log(self.finish) if self.finish > 0 else 1

    def eval(self, timestep: int):
        """특정 시점의 감쇄된 값을 계산해서 반환한다.

        Args:
            timestep: 감쇄를 계산할 타입 스텝
        Returns:
            decayed_value: 감쇄된 값
        """

        if self.decay in ["linear"]:
            return max(self.finish, self.start - self.delta * float(timestep))
        elif self.decay in ["exp"]:
            return min(self.start, max(self.finish, np.exp(- float(timestep) / self.exp_scaling)))
