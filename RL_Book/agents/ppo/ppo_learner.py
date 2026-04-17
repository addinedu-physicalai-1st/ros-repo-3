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
from typing import Any
from types import SimpleNamespace
from datasets.rollout_buffer import RolloutBuffer
from agents.ppo.ppo_network import PPONetwork
from envs.environment import EnvironmentSpec
from utils.logging import Logger
from agents.base import Learner
from utils.lr_scheduler import CosineLR
from utils.value_util import REGISTRY as RETURN_REGISTRY
from utils.schduler import LinearScheduler


class PPOLearner(Learner):
    """PPO 알고리즘 학습자 클래스."""

    def __init__(self,
                 config: SimpleNamespace,
                 logger: Logger,
                 environment_spec: EnvironmentSpec,
                 network: PPONetwork,
                 buffer: RolloutBuffer):
        """
            Learner 클래스의 초기화 메서드를 호출하여 학습자를 초기화 하고,
            정책과 가치 함수를 학습하기 위한 Adam 옵티마이저,
            학습률 스케쥴러, 입실론 스케쥴러를 생성한다.
        Args:
            config: 설정
            logger: 로거
            environment_spec: 환경 정보
            network: 네트워크
            buffer: 버퍼
        """

        # 1. 부모 클래스 초기화 호출
        super().__init__(config, logger, environment_spec, network, buffer)

        # 2. 정책과 가치 함수 속성 정의
        self.policy = self.network.policy
        self.critic = self.network.critic

        # 3. 옵티마이저 생성
        self.optimizer = torch.optim.Adam([
                        {'params': self.network.policy.parameters(),
                         'lr': self.config.lr_policy},
                        {'params': self.network.critic.parameters(),
                         'lr': self.config.lr_critic}
                    ])

        # 4. 학습률 스케쥴러 생성
        self.make_lr_scheduler()

        # 5. 평균 제곱 오차 손실 정의
        self.MSELoss = nn.MSELoss()

        # 6. 입실론 스케쥴러 생성
        end_timesteps = -1
        if self.config.clip_schedule:
            end_timesteps = self.config.max_environment_steps

        self.clip_scheduler = LinearScheduler(
            start_value=self.config.ppo_clipping_epsilon,
            start_timesteps=1,
            end_timesteps=end_timesteps)

    def make_lr_scheduler(self):
        """
            정책과 가치 함수를 학습할 때
            학습률을 조정하기 위한 스케쥴러를 생성.
        """

        self.policy_lr_scheduler = None
        self.critic_lr_scheduler = None
        if not self.config.lr_annealing: return

        # 1. 정책 학습률 스케쥴러 생성
        self.policy_lr_scheduler = CosineLR(
            logger=self.logger,
            param_groups=self.optimizer.param_groups[0],
            start_lr=self.config.lr_policy,
            end_timesteps=self.config.max_environment_steps,
            name="policy lr"
            )

        # 2. 가치 함수 학습률 스케쥴러 생성
        self.critic_lr_scheduler = CosineLR(
            logger=self.logger,
            param_groups=self.optimizer.param_groups[1],
            start_lr=self.config.lr_critic,
            end_timesteps=self.config.max_environment_steps,
            name="critic lr"
            )

    def _calc_target_value(self):
        """
            가치 함수의 타깃과 이득을
                1) 몬테카를로 리턴,
                2) n-스텝 리턴,
                3) GAE 방식
            중 하나로 계산하고 버퍼에 추가 데이터 필드로 저장.
        """

        # 1. 버퍼가 비어 있으면 반환
        if len(self.buffer) == 0: return

        # 2. 타깃 가치와 이득 계산 (MC 리턴, n-스텝 리턴, GAE)
        target_value, advantage =\
            RETURN_REGISTRY[self.config.advantage_type](
            self.config,
            self.buffer['state'],
            self.buffer['next_state'],
            self.buffer['reward'],
            self.buffer['done'],
            self.critic
        )

        # 3. 이전 정책의 로그 가능도 계산
        state = self.buffer['state']
        action = self.buffer['action']
        with torch.no_grad():
            log_probs_old, _, _ = self.network(state, action)

        # 4. 버퍼 스키마 확장
        if self.buffer["advantage"] is None:
            schema = {
                'advantage': {'shape': (1,)},
                'target_value': {'shape': (1,)},
                'log_probs_old': {'shape': (log_probs_old.shape[-1],),},
            }
            self.buffer.extend_schema(schema)

        # 5. 버퍼에 손실 계산 정보 저장
        self.buffer['advantage'] = advantage            # 이득
        self.buffer['target_value'] = target_value      # 타깃 가치
        self.buffer['log_probs_old'] = log_probs_old    # 이전 정책의 로그 가능도

    def _loss(self,
              states: torch.FloatTensor,
              actions: torch.FloatTensor,
              target_values: torch.FloatTensor,
              log_probs_old: torch.FloatTensor,
              advantages: torch.FloatTensor=None,
    ) -> tuple[Any, dict[str, Any]]:
        """
            PPO 알고리즘의 목적 함수에 따라 정책의 손실을 계산하고,
            가치 함수의 손실은 평균 제곱 오차로 계산
        Args:
            states: 상태
            actions: 행동
            target_values: 타깃 가치
            log_probs_old: 이전 정책의 로그 가능도
            advantages: 이득

        Returns:
            전체 손실
            손실 딕셔너리 {전체 손실, 정책의 손실, 가치 함수의 손실, 엔트로피 보너스}
        """

        # 1. 로그 가능도, 엔트로피, 가치 계산
        log_probs, entropy, values = self.network(states, actions)

        # 2. 정책의 로그 가능도 비율 계산
        ratios = torch.exp(log_probs - log_probs_old)
        ratios = ratios.prod(1, keepdim=True)

        # 3. 정책 손실 계산
        surrogate1 = ratios * advantages
        surrogate2 = torch.clamp(
            ratios,
            1 - self.clipping_epsilon,
            1 + self.clipping_epsilon) * advantages
        policy_loss = -torch.min(surrogate1, surrogate2).mean()

        # 4. 가치 함수 손실 계산
        value_loss = self.MSELoss(values, target_values)

        # 5. 엔트로피 보너스 계산
        entropy_loss = -entropy.mean()

        # 6. 총 손실 계산
        total_loss = (
                policy_loss
                + self.config.vloss_coef * value_loss
                + self.config.eloss_coef * entropy_loss
        )

        # 7. 손실 딕셔너리 반환
        return total_loss, {
          'total_loss': total_loss.item(),
          'policy_loss': policy_loss.item(),
          'value_loss': value_loss.item(),
          'entropy_loss': entropy_loss.item(),
        }

    def update(self, total_n_timesteps: int, total_n_episodes: int) -> bool:
        """
            PPO 알고리즘의 목적 함수에 따라 손실을 계산해서
            정책과 가치 함수를 학습하고 성능 정보를 로깅
        Args:
            total_n_timesteps: 현재 타임 스텝
            total_n_episodes: 현재 에피소드

        Returns:
            정책 평가 및 개선 실행 여부
        """

        # 1. 버퍼가 비어 있으면 반환
        if len(self.buffer) == 0: return False

        # 2. 타깃 가치와 이득 계산
        self._calc_target_value()

        # 3. PPO 클리핑 입실론 계산
        self.clipping_epsilon = self.clip_scheduler.eval(total_n_timesteps)

        # 4. 배치 실행 횟수 계산
        num_batch_times = (len(self.buffer)-1)//self.config.batch_size+1

        # 5. 학습 루프
        for epoch in range(0, self.config.n_epochs):
            for i in range(num_batch_times):
                # 6. 롤아웃버퍼에서 배치 샘플링
                sample_batched = self.buffer.sample(self.config.batch_size)

                # 7. 특징 별 변수 처리
                state = sample_batched["state"]
                action = sample_batched["action"]
                advantage = sample_batched["advantage"]
                target_value = sample_batched["target_value"]
                log_probs_old = sample_batched["log_probs_old"]

                # 8. 학습 타입 스텝 증가
                self.learner_step += 1

                # 9. 손실 계산
                total_loss, loss_results = self._loss(state,
                                                      action,
                                                      target_value,
                                                      log_probs_old,
                                                      advantage)

                # 10. 백워드 패스 실행 (그레이디언트 계산)
                self.optimizer.zero_grad(set_to_none=True)
                total_loss.backward()

                # 11. 그레이디언트 클리핑
                torch.nn.utils.clip_grad_norm_(
                    self.network.parameters(), self.config.grad_norm_clip
                )

                # 12. 파라미터 업데이트
                self.optimizer.step()

                # 13. 손실 로깅
                # 총 손실
                self.logger.log_stat("total_loss",
                                     loss_results['total_loss'],
                                     self.learner_step)
                # 정책 손실
                self.logger.log_stat("policy_loss",
                                     loss_results['policy_loss'],
                                     self.learner_step)
                # 가치 함수 손실
                self.logger.log_stat("value_loss",
                                     loss_results['value_loss'],
                                     self.learner_step)
                # 엔트로피 보너스
                self.logger.log_stat("entropy_loss",
                                     loss_results['entropy_loss'],
                                     self.learner_step)

        # 14. 학습률 스케쥴 업데이트
        if self.config.lr_annealing:
            self.policy_lr_scheduler.step(total_n_timesteps)
            self.critic_lr_scheduler.step(total_n_timesteps)
            # 15. 학습률 로깅
            self.logger.log_stat("policy learning rate",
                                 self.optimizer.param_groups[0]['lr'],
                                 total_n_timesteps)
            self.logger.log_stat("critic learning rate",
                                 self.optimizer.param_groups[1]['lr'],
                                 total_n_timesteps)

        # 16. 데이터셋 삭제
        self.buffer.clear()

        return True