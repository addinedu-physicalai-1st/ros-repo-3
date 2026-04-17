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
"""lr_scheduler.py: 학습률 스케쥴러."""
import abc
import math

from utils.logging import Logger


class LRScheduler(abc.ABC):
    """학습률 스케쥴러 베이스 클래스."""

    @abc.abstractmethod
    def step(self, current_timestep: int = 0):
        """
            학습률 스케쥴
        Args:
            current_timestep: 현재 타입 스텝

        Returns:
            현재 타입 스텝에 맞계 스케쥴링된 학습률
        """

        raise NotImplementedError


class CosineLR(LRScheduler):
    """코사인 학습률 스케쥴러 클래스 ([0,90]도 사이의 코사인 스케쥴을 따르는 방식)"""

    def __init__(self,
                 logger: Logger,
                 param_groups,
                 start_lr: float,
                 end_timesteps: int,
                 interval: int = 1,
                 start_timesteps: int = 0,
                 name: str = ''):
        """
            코사인 스케쥴링을 위한 파라미터 초기화
        Args:
            logger: 로거
            param_groups: 모델의 파라미터 그룹
            start_lr: 시작 학습률
            end_timesteps: 종료 타입 스텝
            interval: 스케쥴 간격
            start_timesteps: 시작 타입 스텝
            name: 이름
        """

        # 1. 스케쥴 파라미터 지정
        self.logger = logger
        self.param_groups = param_groups
        self.start_lr = start_lr
        self.start_timesteps = start_timesteps
        self.end_timesteps = end_timesteps
        self.interval = interval
        self.name = name

        # 2. 최종 스케쥴링 타입 스텝 업데이트
        self.last_timestep = 0

    def step(self, current_timestep: int = 0):
        """
            학습률 스케쥴링
        Args:
            current_timestep: 현재 타입 스텝
        """
        # 1. 다음의 경우 스케쥴링 없이 반환
        #   1) 시작 타입 스텝에 도달하지 않은 경우
        #   2) 시간 간격에 도달하지 않은 경우
        if current_timestep < self.start_timesteps or \
                (current_timestep - self.last_timestep) < self.interval:
            return

        # 2. [시작 타입 스텝, 종료 타입 스텝] 구간에서 현재 타입 스텝의 위치를 비율로 계산
        rate = (current_timestep - self.start_timesteps) / (self.end_timesteps - self.start_timesteps)
        fraction = math.cos((math.pi / 2.) * rate)

        # 3. 비율에 따라 스케쥴 값을 계산
        self.param_groups['lr'] = self.start_lr * fraction

        # 4. 최종 스케쥴링을 한 타입 스텝 업데이트
        self.last_timestep = current_timestep
