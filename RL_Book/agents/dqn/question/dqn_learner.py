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
import torch.nn as nn
from types import SimpleNamespace
from datasets.replay_buffer import ReplayBuffer
from agents.dqn.dqn_network import DQNNetwork
from envs.environment import EnvironmentSpec
from utils.logging import Logger
from agents.base import Learner
from utils.lr_scheduler import CosineLR


class DQNLearner(Learner):
    """DQN 알고리즘 학습자 클래스."""

    def __init__(self,
                 config: SimpleNamespace,
                 logger: Logger,
                 environment_spec: EnvironmentSpec,
                 network: DQNNetwork,
                 buffer: ReplayBuffer):
        """
            Learner 클래스의 초기화 메서드를 호출하여
            학습자를 초기화 하고, 가치 함수를 학습하기 위해
            Adam 옵티마이저와 학습률 스케쥴러, 휴버 손실 함수를 생성
        Args:
            config: 설정
            logger: 로거
            environment_spec: 환경 정보
            network: 네트워크
            buffer: 버퍼
        """

        # 1. 부모 클래스 초기화 호출
        super().__init__(config, logger, environment_spec, network, buffer)

        # 2. 가치 함수/타깃 가치 함수 속성 정의
        self.critic = self.network.critic
        self.target_critic = self.network.target_critic

        # 3. 옵티마이저 생성
        self.optimizer = torch.optim.Adam([
            {'params': self.critic.parameters(),
             'lr': self.config.lr_critic}])

        # 4. 학습률 스케쥴러 생성
        if self.config.lr_annealing:
            self.critic_lr_scheduler = CosineLR(
                logger=self.logger,
                param_groups=self.optimizer.param_groups[0],
                start_lr=self.config.lr_critic,
                end_timesteps=self.config.max_environment_steps,
                name="critic lr"
            )

        # 5. 휴버 손실 정의
        self.loss = nn.HuberLoss()

        # 6. 최종 타깃 업데이트 스텝 초기화
        self.last_target_update_step = 0

    def update(self, total_n_timesteps: int, total_n_episodes: int) -> bool:
        """
            Q-러닝 업데이트 식에 따라 타깃을 계산해서 휴버 손실로 가치 함수를 학습
            타깃 가치 함수 모델을 하드/소프트 업데이트하고 성능 정보를 로깅
        Args:
            total_n_timesteps: 현재 타임 스텝
            total_n_episodes: 현재 에피소드

        Returns:
            정책 평가 및 개선 실행 여부
        """

        # 1. 워밍업 상태면 반환
        if len(self.buffer) < self.config.warmup_step: return False

        # 2. 버퍼 샘플링이 안되면 반환
        if not self.buffer.can_sample(
                batch_size=self.config.batch_size): return False

        # 3. 학습 루프
        for i in range(self.config.gradient_steps):
            # 4. 리플레이 버퍼에서 배치 샘플링
            sample_batched = self.buffer.sample(self.config.batch_size)

            # 5. 특징 별 변수 처리
            state = sample_batched["state"]
            action = sample_batched["action"]
            reward = sample_batched["reward"]
            next_state = sample_batched["next_state"]
            done = sample_batched["done"]

            # 6. 학습 타입 스텝 증가
            self.learner_step += 1

            # 7. 타깃 Q 가치 계산
            with torch.no_grad():
                target_q_value = None # your code

            # 8. Q 가치 계산
            q_value = None # your code

            # 9. 가치 함수의 손실 계산
            value_loss = self.loss(target_q_value, q_value)

            # 10. 백워드 패스 실행 (그레이디언트 계산)
            self.optimizer.zero_grad()
            value_loss.backward()

            # 11. 그레이디언트 클리핑
            torch.nn.utils.clip_grad_norm_(
                self.critic.parameters(), self.config.grad_norm_clip
            )

            # 12. 파라미터 업데이트
            self.optimizer.step()

            # 13. 소프트 타깃 업데이트
            if self.config.target_update_type == "soft":
                self.network.soft_update_target()

            # 14. 손실 로깅
            self.logger.log_stat("value_loss",
                                 value_loss.item(),
                                 total_n_timesteps)

        # 15. 타깃 하드 업데이트
        if (self.config.target_update_type == "hard" and
                (total_n_timesteps - self.last_target_update_step)
                >= self.config.target_update_interval):
            self.network.hard_update_target()
            self.last_target_update_step = total_n_timesteps

        # 16. 학습률 스케쥴 업데이트
        if self.config.lr_annealing:
            self.critic_lr_scheduler.step(total_n_timesteps)
            # 17. 학습률 로깅
            self.logger.log_stat("critic learning rate",
                                 self.optimizer.param_groups[0]['lr'],
                                 total_n_timesteps)

        return True