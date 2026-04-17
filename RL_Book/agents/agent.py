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
from typing import List, Union
import torch
from types import SimpleNamespace
from utils.logging import Logger
from envs.environment import Environment
from agents.base import Network
from datasets.buffer_schema import BufferSchema
from datasets.buffer import Buffer
from datasets.rollout_buffer import RolloutBuffer
from datasets.replay_buffer import ReplayBuffer
from agents.base import Learner
from agents.actor import Actor


class Agent(Learner):
    """
        에이전트의 베이스 클래스로 
            1) 네트워크, 데이터셋, 학습자를 생성하고, 
            2) 액터와 데이터셋 및 네트워크를 동기화하기 위한 인터페이스와 
            3) 학습자의 인터페이스를 제공.
    """

    def __init__(self,
                 config: SimpleNamespace,
                 logger: Logger,
                 env: Environment,
                 network_class: Network,
                 learner_class: Learner,
                 actor_class=Actor,
                 policy_type: str = "on_policy"):
        """
            인자로 전달받은 클래스 정보와 환경 정보,
            설정 정보를 이용하여 서브 모듈인 네트워크를 생성
            훈련 모드인 경우 데이터셋과 학습자까지 생성
        Args:
            config: 설정
            logger: 로거
            env: 환경
            network_class: 네트워크 클래스
            learner_class: 학습자 클래스
            actor_class: 액터 클래스
            policy_type: 정책 타입 (온라인 정책, 오프라인 정책)
        """

        # 1. 전달받은 인자 저장
        self.config = config
        self.logger = logger
        self.env = env
        self.policy_type = policy_type
        self.actor_class = actor_class

        # 2. 네트워크 생성
        self.network = network_class(
            config=config,
            environment_spec=env.environment_spec(),
        )

        self.learner = None
        self.buffer_schema = None
        self.buffer = None

        # 훈련 모드
        if config.training_mode:
            # 3. 버퍼 스키마 생성
            self.buffer_schema = BufferSchema(self.config, self.env)

            # 4. 4.	데이터셋을 관리하는 버퍼 생성 (버퍼 클래스와 모양에 따라 생성)
            self.buffer = self.buffer_class()(
                config=config,
                buffer_schema=self.buffer_schema,
                buffer_shape=self.buffer_shape())

            # 5. 학습자 생성
            self.learner = learner_class(
                config=config,
                logger=logger,
                environment_spec=env.environment_spec(),
                network=self.network,
                buffer=self.buffer)

    def buffer_class(self) -> Buffer:
        """
            에이전트의 데이터셋을 관리하는 버퍼 클래스 반환.
            1) 온라인 정책: RolloutBuffer
            2) 오프라인 정책: ReplayBuffer
        Returns:
            버퍼 클래스
        """

        if self.policy_type == "on_policy": return RolloutBuffer
        return ReplayBuffer

    def buffer_shape(self) -> List[int]:
        """
            에이전트의 데이터셋을 관리하는 버퍼의 모양을 반환.
            1. 온라인 정책:
                - 환경이 하나이거나 동기적 분산 처리 방식: 액터의 롤아웃버퍼 × 환경 개수
                - 비동기적 분산 처리 방식: 액터의 롤아웃버퍼
            2. 오프라인 정책: 설정된 리플레이 버퍼 크기
        Returns:
            버퍼 모양
        """

        # 1. 온라인 정책
        if self.policy_type == "on_policy":
            actor_buffer_size = \
                self.actor_class.buffer_shape(self.config, self.env)[0]
            if self.config.distributed_processing_type == "async":
                return [actor_buffer_size]                      # 비동기적 분산 처리 방식
            # 환경이 하나이거나 동기적 분산 처리 방식
            return [actor_buffer_size * self.config.n_envs]

        # 2. 오프라인 정책
        return [self.config.replay_buffer_size]

    def add_rollouts(self,
        list_of_buffers: Union[RolloutBuffer, List[RolloutBuffer]]):
        """
            액터의 롤아웃 버퍼에 있는 데이터를 에이전트의 데이터셋에 추가.
        Args:
            list_of_buffers: 롤아웃버퍼 or 롤아웃버퍼 리스트
        """

        # 1. 롤아웃 버퍼 리스트로 변환
        if isinstance(list_of_buffers, RolloutBuffer):
            list_of_buffers = [list_of_buffers]

        # 2. 데이터셋에 롤아웃 버퍼 내용 추가
        for buffer in list_of_buffers:
            self.buffer += buffer

    def update(self,total_n_timesteps: int,total_n_episodes: int):
        """
            학습자를 통해 정책을 평가하고 개선.
        Args:
            total_n_timesteps: 현재 타입 스텝
            total_n_episodes: 현재 에피소드

        Returns:
            정책 개선 및 평가 실행 여부 반환
        """

        # 학습자에게 모델 업데이트 요청 및 업데이트 실행 여부 반환
        return self.learner.update(total_n_timesteps, total_n_episodes)

    def load(self, model_path):
        """
            지정된 경로에서 추론 모델의 파라미터를 읽어서 에이전트의 네트워크에 로딩.
        Args:
            model_path: 모델 파일 경로

        Returns:
            모델 로딩 성공 여부
        """

        # 1. 모델 파라미터 읽기
        try:
            state_dict = torch.load(model_path,
                                    map_location=lambda storage, loc: storage)
        except:
            return False

        # 2. 모델 파라미터를 네트워크에 로딩
        self.network.load_state_dict(state_dict)
        return True

    def save(self, checkpoint_path):
        """
            학습자를 통해 모델과 옵티마이저의 체크포인트를 저장.
        Args:
            checkpoint_path: 체크포인트 저장 디렉토리 경로
        """

        self.learner.save(checkpoint_path)  # 학습자의 save() 호출

    def restore(self, path):
        """
            학습자를 통해 모델과 옵티마이저의 체크포인트 복구.
        Args:
            path: 체크포인트 저장 디렉토리 경로
        """

        self.learner.restore(path)  # 학습자의 restore() 호출

    def cuda(self):
        """네트워크의 상태(파라미터와 버퍼)를 GPU로 이동."""

        self.network.cuda() # 네트워크의 cuda() 호출