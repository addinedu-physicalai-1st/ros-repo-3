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
from typing import Tuple
import numpy as np
from types import SimpleNamespace
from utils.logging import Logger
from datasets.buffer import Buffer
from envs.environment import EnvironmentSpec

class VariableSource(abc.ABC):
    """네트워크의 상태(파라미터와 버퍼)를 제공 인터페이스."""

    @abc.abstractmethod
    def get_variables(self) -> dict:
        """네트워크의 상태(파라미터와 버퍼)를 반환."""


class Saveable(abc.ABC):
    """네트워크의 체크포인트 관리 인터페이스."""

    @abc.abstractmethod
    def save(self, checkpoint_path: str):
        """
            네트워크와 옵티마이저의 상태(파라미터와 버퍼)를 체크포인트로 저장.
        Args:
            checkpoint_path: 체크포인트 저장 디렉토리 경로
        """

    @abc.abstractmethod
    def restore(self, checkpoint_path: str):
        """
            체크포인트에서 네트워크와 옵티마이저의 상태(파라미터와 버퍼)를 복구.
        Args:
            checkpoint_path: 체크포인트 저장 디렉토리 경로
        """


class Network(nn.Module, VariableSource, Saveable):
    """
        정책과 가치 함수 모델을 통합적으로 관리하는
        네트워크의 베이스 클래스.
    """

    def __init__(self,
                 config: SimpleNamespace,
                 environment_spec: EnvironmentSpec):
        """
            1) 정책과 가치 함수의 입출력 데이터인 상태와 행동의 크기를 계산
            2) 행동이 연속 행동인지 이산 행동인지 구분.
        Args:
            config: 설정
            environment_spec: 환경 정보
        """

        super(Network, self).__init__()

        # 1. 전달 받은 인자 저장
        self.config = config
        self.environment_spec = environment_spec

        # 2. 연동 행동 여부
        self.b_continuous_action = \
            self.environment_spec.b_continuous_action

        # 3. 행동과 상태의 벡터 크기 계산
        self.action_size = self.environment_spec.action_size
        self.state_size = \
            np.array(self.environment_spec.state_spec.shape).prod()

    @abc.abstractmethod
    def select_action(self,
                      state: torch.Tensor,
                      total_n_timesteps: int) -> torch.Tensor:
        """
            정책 또는 가치 함수를 통해 행동을 선택.
        Args:
            state: 상태
            total_n_timesteps: 현재 타입 스텝

        Returns:
            선택된 행동
        """

    def _log_prob(self, distribution, action):
        """
            책의 손실을 계산할 때 필요한 행동의 로그 가능도 계산.
        Args:
            distribution: 행동의 분포
            action: 행동

        Returns:
            행동의 로그 가능도
        """

        # 1. 이산 행동: 1차원으로 변경
        b_squeeze = self.b_continuous_action is False \
                    and action.shape[-1] == 1
        if b_squeeze: action = action.squeeze()

        # 2. 로그 가능도 계산
        if self.b_continuous_action:
            action = torch.atanh(torch.clamp(action, -1 + 1e-7, 1 - 1e-7))
        log_prob = distribution.log_prob(action)

        # 3. 차원 변경 시 원래 차원으로 복구
        if b_squeeze: log_prob = log_prob.unsqueeze(-1)

        # 4. 로그 가능도 반환
        return log_prob

    def forward(self, state, action) -> Tuple[torch.Tensor]:
        """
            정책과 가치 함수를 실행해서
            정책과 가치 함수의 손실을 계산할 때 필요한 정보를 반환.
        Args:
            state: 상태
            action: 행동

        Returns:
            로그 가능도, 엔트로피, 가치 등 학습에 필요한 정보
        """
        return None

    @abc.abstractmethod
    def cuda(self):
        """네트워크의 상태(파라미터와 버퍼)를 GPU로 이동."""

    def save(self, checkpoint_path: str):
        """
            네트워크의 상태(파라미터와 버퍼)를 체크포인트로 저장.
        Args:
            checkpoint_path: 체크포인트 저장 디렉토리 경로
        """

        # 네트워크의 상태 저장
        torch.save(self.state_dict(), "{}/network.th".format(checkpoint_path))

    def restore(self, checkpoint_path: str):
        """
            채크포인트에서 네트워크의 상태(파라미터와 버퍼)를 로딩.
        Args:
            checkpoint_path: 체크포인트 저장 디렉토리 경로
        """

        # 네트워크의 상태 복구
        state_dict = torch.load("{}/network.th".format(checkpoint_path),
                                map_location=torch.device(self.config.device))
        self.load_state_dict(state_dict)

    def get_variables(self) -> dict:
        """
            네트워크의 상태(파라미터와 버퍼)를 반환.
        Returns:
            네트워크의 상태
        """

        # 네트워크의 상태 반환
        return self.state_dict()


class Learner(Saveable):
    """
        정책을 평가하고 개선하기 위한 학습자의 베이스 클래스.
    """

    def __init__(self,
                 config: SimpleNamespace,
                 logger: Logger,
                 environment_spec: EnvironmentSpec,
                 network: Network,
                 buffer: Buffer):
        """
            1) 전달 받은 인자를 저장하고,
            2) 행동이 연속 행동인지 이산 행동인지 구분하며,
            3) 학습 타입 스텝을 0으로 초기화.
        한다.
        Args:
            config: 설정
            logger: 로거
            environment_spec: 환경 정보
            network: 네트워크
            buffer: 버퍼
        """

        # 1. 전달 받은 인자 저장
        self.config = config
        self.logger = logger
        self.environment_spec = environment_spec
        self.network = network
        self.buffer = buffer

        # 2. 연동 행동 여부
        self.b_continuous_action = \
            self.environment_spec.b_continuous_action

        # 3. 학습 타입 스텝 초기화
        self.learner_step = 0

    @abc.abstractmethod
    def update(self, total_n_timesteps: int, total_n_episodes:int) -> bool:
        """
            정책 평가 및 개선
        Args:
            total_n_timesteps: 현재 타입 스텝
            total_n_episodes: 현재 에피소드

        Returns:
            정책 평가 및 개선 실행 여부
        """

    def save(self, checkpoint_path: str):
        """
            네트워크와 옵티마이저의 상태(파라미터와 버퍼)를
            체크포인트로 저장
        Args:
            checkpoint_path: 체크포인트 저장 디렉토리 경로
        """

        # 1. 네트워크 상태 저장
        self.network.save(checkpoint_path)

        # 2. 옵티마이저 상태 저장
        torch.save(self.optimizer.state_dict(),
                   "{}/opt.th".format(checkpoint_path))

    def restore(self, checkpoint_path: str):
        """
            체크포인트로 저장된 네트워크와 옵티마이저의
            상태(파라미터와 버퍼)를 로딩
        Args:
            checkpoint_path: 체크포인트 저장 디렉토리 경로
        """

        # 1. 네트워크 상태 복구
        self.network.restore(checkpoint_path)

        # 2. 옵티마이저 상태 복구
        self.optimizer.load_state_dict(
            torch.load("{}/opt.th".format(checkpoint_path),
                       map_location=lambda storage, loc: storage))