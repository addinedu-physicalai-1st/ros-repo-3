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
"""scheduler.py: 스케쥴러 (PPO 입실론 스케쥴링에 사용)"""
import abc


class Scheduler(abc.ABC):
    """스케쥴러 베이스 클래스."""

    @abc.abstractmethod
    def eval(self, timestep: int = 0):
        """
            타입 스텝에 해당하는 스케쥴 값을 반환
        Args:
            timestep: 타입 스텝

        Returns:
            스케쥴링 된 값
        """
        raise NotImplementedError


class LinearScheduler(Scheduler):
    """선형 스케쥴러."""

    def __init__(self,
                 start_value: float,
                 end_value: float = 0,
                 start_timesteps: int = 1,
                 end_timesteps: int = -1,
                 interval: int = 1):
        """
            선형 스케쥴 파라미터 지정
        Args:
            start_value: 시작 값
            end_value: 종료 값
            start_timesteps: 시작 타입 스텝
            end_timesteps: 종료 타입 스텝
            interval: 스케쥴 간격
        """

        # 1. 스케쥴 파라미터 지정
        self.start_value = start_value
        self.end_value = end_value
        self.start_timesteps = start_timesteps
        self.end_timesteps = end_timesteps
        self.interval = interval

        # 2. 현재 스케쥴 값을 시작 값으로 초기화
        self.current_value = start_value

        # 3. 최종 스케쥴링 타입 스텝 업데이트
        self.last_timestep = 0

    def eval(self, timestep: int) -> float:
        """
            타임 스텝에 맞는 값 계산
        Args:
            timestep: 타임 스텝

        Returns:
            스케쥴링 된 값
        """

        # 1. 다음의 경우 현재 값을 그대로 반환
        #   1) 종료 타입 스텝이 -1인 경우
        #   2) 현재 타입 스텝이 [시작 타입 스텝, 종료 타입 스텝]의 범위에 있지 않은 경우
        #   3) 현재 타입 스텝이 [시작 타입 스텝, 종료 타입 스텝]의 범위에 있지만 시간 간격에 도달하지 않은 경우
        if self.end_timesteps == -1 or \
                timestep < self.start_timesteps or \
                timestep > self.end_timesteps or \
                (timestep - self.last_timestep) < self.interval:
            return self.current_value

        # 2. [시작 타입 스텝, 종료 타입 스텝] 구간에서 현재 타입 스텝의 위치를 비율로 계산
        fraction = (timestep - self.start_timesteps) / (self.end_timesteps - self.start_timesteps)

        # 3. 비율에 따라 스케쥴 값을 계산
        self.current_value = self.start_value + (self.end_value - self.start_value) * fraction

        # 4. 최종 스케쥴링을 한 타입 스텝 업데이트
        self.last_timestep = timestep

        # 5. 스케쥴링 된 값 반환
        return self.current_value
