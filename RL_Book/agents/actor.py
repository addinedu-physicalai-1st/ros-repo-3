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
from typing import Dict, List
from copy import deepcopy
from types import SimpleNamespace
from envs.environment import Environment
from agents.base import Network
from datasets.buffer_schema import BufferSchema
from datasets.buffer import Buffer
from datasets.rollout_buffer import RolloutBuffer
from utils.util import to_tensor, to_device, to_numpy


class Actor():
    """
        환경과의 상호작용을 수행하는 액터의 베이스 클래스.
    """

    def __init__(self,
                 config: SimpleNamespace,
                 env: Environment,
                 buffer_schema: BufferSchema,
                 network: Network,
                 actor_id: int = 0):
        """
            에이전트의 네트워크를 복제해서 자신의 네트워크를 생성
            학습 모드에서는 트랜지션 데이터를 저장할 롤아웃 버퍼를 생성.
        Args:
            config: 설정
            env: 환경
            buffer_schema: 버퍼 스키마
            network: 네트워크
            actor_id: 액터 ID
        """
        # 1. 전달받은 인자 저장
        self.config = config
        self.env = env
        self.actor_id = actor_id

        # 2. 네트워크 복사본 생성
        self.network = deepcopy(network)
        state_dict = network.get_variables()
        self.network.load_state_dict(state_dict)

        self.buffer = None
        if self.config.training_mode:
            # 3. 버퍼 생성 (버퍼 클래스와 버퍼 모양에 따라 버퍼 생성)
            self.buffer = self.buffer_class()(
                config=self.config,
                buffer_schema=buffer_schema,
                buffer_shape=self.buffer_shape(self.config, self.env))

    def buffer_class(self) -> Buffer:
        """
            액터의 롤아웃 버퍼 클래스를 반환.
        Returns:
            버퍼 클래스
        """
        return RolloutBuffer    # 버퍼 클래스 반환

    @staticmethod
    def buffer_shape(config: SimpleNamespace, env: Environment) -> List[int]:
        """
            설정과 환경 정보를 이용해서 액터의 롤아웃 버퍼 모양을 계산.
        Args:
            config: 설정
            env: 환경

        Returns:
            버퍼 모양
        """
        # 1. 타입 스텝 수로 환경 루프 실행 시: 지정된 타입 스텝 수 반환
        if config.n_steps != 0: return [config.n_steps]

        # 2. 에피소드 수로 환경 루프 실행 시: 최대 에피소드 길이x에피소드 수
        return [env.max_episode_limit()*config.n_episodes]

    def rollouts(self) -> RolloutBuffer:
        """
            액터의 롤아웃 버퍼를 반환.
        Returns:
            액터의 롤아웃버퍼
        """
        return self.buffer

    def clear_rollouts(self):
        """
            액터의 롤아웃 버퍼를 비움.
        """

        # 1. 추론 모드이면 반환
        if not self.config.training_mode: return

        # 2. 버퍼 비우기
        self.buffer.clear()

    def select_action(self,
                      state: torch.Tensor,
                      total_n_timesteps: int) -> torch.Tensor:
        """
            정책을 실행해서 전달 받은 상태에 대한 행동을 선택.
        Args:
            state: 상태
            total_n_timesteps: 현재 타임 스텝

        Returns:
            선택된 행동
        """

        # 1. 상태를 텐서로 변환 (numpy to tensor)
        state = to_device(to_tensor(state), self.config).unsqueeze(dim=0)

        # 2. 네트워크에 행동 선택 요청
        action = self.network.select_action(state=state,
                                            total_n_timesteps=total_n_timesteps)

        # 3. 행동을 넘파이로 변환 (tensor to numpy)
        action = to_numpy(action, self.config).squeeze()
        return action

    def observe(self, rollout: Dict):
        """
            트랜지션 데이터를 관측하고,
            훈련 모드에서는 관측한 트랜지션 데이터를 롤아웃 버퍼에 저장.
        Args:
            rollout: 트랜지션 데이터
        """

        # 1. 추론 모드이면 반환
        if not self.config.training_mode: return

        # 2. 롤아웃 버퍼에 관측 데이터 저장
        self.buffer += rollout

    def update(self, state_dict):
        """
            에이전트의 네트워크의 파라미터를 액터의 네트워크에 로딩.
        Args:
            state_dict: 에이전트의 네트워크 상태 딕셔너리(가중치 및 버퍼)
        """
        self.network.load_state_dict(state_dict)

    def cuda(self):
        """
            네트워크의 상태(파라미터와 버퍼)를 GPU로 이동한다.
        """
        self.network.cuda()