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
from datasets.rollout_buffer import RolloutBuffer
from agents.reinforce_b.reinforce_b_network import REINFORCEBNetwork
from envs.environment import EnvironmentSpec
from utils.logging import Logger
from agents.reinforce.reinforce_learner import REINFORCELearner
from utils.lr_scheduler import CosineLR
from utils.value_util import monte_carlo_returns


class REINFORCEBLearner(REINFORCELearner):
    """REINFORCE 베이스라인 적용 알고리즘 학습자 클래스."""
    def __init__(self,
                 config: SimpleNamespace,
                 logger: Logger,
                 environment_spec: EnvironmentSpec,
                 network: REINFORCEBNetwork,
                 buffer: RolloutBuffer):
        """
            REINFORCELearner 클래스의 초기화 메서드를 호출하여 학습자를 초기화 하고,
            베이스라인 모델의 학습을 위한 Adam 옵티마이저와 학습률 스케쥴러를 생성
        Args:
            config: 설정
            logger: 로거
            environment_spec: 환경 정보
            network: 네트워크
            buffer: 버퍼
        """

        # 1. 부모 클래스 초기화 호출
        super(REINFORCEBLearner, self).__init__(config,
                                                logger,
                                                environment_spec,
                                                network, buffer)

        # 2. 베이스라인 모델을 변수에 할당
        self.baseline = self.network.baseline

        # 3. 옵티마이저 생성
        self.optimizer = torch.optim.Adam([
                        {'params': self.network.policy.parameters(),
                         'lr': self.config.lr_policy},
                        {'params': self.network.baseline.parameters(),
                         'lr': self.config.lr_policy},
                    ])

        # 4. 학습률 스케쥴러 생성
        self.make_lr_scheduler()

        # 5. 평균 제곱 오차 손실 함수 생성
        self.MSELoss = nn.MSELoss()

    def make_lr_scheduler(self):
        """학습율 스케쥴러 생성."""

        self.policy_lr_scheduler = None
        self.critic_lr_scheduler = None

        # 1. 학습률 스케쥴링을 하지 않으면 반환
        if not self.config.lr_annealing: return

        # 2. 정책 학습률 스케쥴러 생성
        self.policy_lr_scheduler = CosineLR(
            logger=self.logger,
            param_groups=self.optimizer.param_groups[0],
            start_lr=self.config.lr_policy,
            end_timesteps=self.config.max_environment_steps,
            name="policy lr"
            )

        # 3. 베이스라인 학습률 스케쥴러 생성
        self.baseline_lr_scheduler = CosineLR(
            logger=self.logger,
            param_groups=self.optimizer.param_groups[1],
            start_lr=self.config.lr_policy,
            end_timesteps=self.config.max_environment_steps,
            name="policy lr"
        )

    def _calc_returns(self):
        """
            몬테카를로 리턴을 계산할 때 베이스라인을 이용해서
            리턴에서 베이스라인을 뺀 값을 이득으로 계산한 후
            버퍼에 추가 데이터 필드로 저장.
        """

        # 1. 버퍼가 비어 있으면 반환
        if len(self.buffer) == 0: return

        # 2. 몬테카를로 리턴 계산
        returns, _ = monte_carlo_returns(
            self.config,
            self.buffer['state'],
            self.buffer['next_state'],
            self.buffer['reward'],
            self.buffer['done'],
        )

        # 3. 베이스라인과 이득 계산
        with torch.no_grad():
            baseline = self.baseline(self.buffer['state'])
            advantage = returns - baseline

        # 4. 버퍼 스키마 확장
        if self.buffer["returns"] is None:
            schema = {'returns': {'shape': (1,)},
                      'advantage': {'shape': (1,)},
                      }
            self.buffer.extend_schema(schema)

        # 5. 버퍼에 리턴과 이득 저장
        self.buffer['returns'] = returns
        self.buffer['advantage'] = advantage

    def update(self, total_n_timesteps: int, total_n_episodes: int) -> bool:
        """
            정책의 손실 함수에 리턴 대신 베이스라인을 적용한 이득을 사용하고
            정책과 함께 베이스라인 모델도 학습
        Args:
            total_n_timesteps: 현재 타임 스텝
            total_n_episodes: 현재 에피소드

        Returns:
            정책 평가 및 개선 실행 여부
        """

        # 1. 버퍼가 비어 있으면 반환
        if len(self.buffer) == 0: return False

        # 2. 리턴 계산
        self._calc_returns()

        # 3. 배치 실행 횟수 계산
        num_batch_times = (len(self.buffer)-1)//self.config.batch_size+1

        # 4. 학습 루프
        for epoch in range(0, self.config.n_epochs):
            for i in range(num_batch_times):
                # 5. 롤아웃버퍼에서 배치 샘플링
                sample_batched = self.buffer.sample(self.config.batch_size)

                # 6. 특징 별 변수 처리
                state = sample_batched["state"]
                action = sample_batched["action"]
                returns = sample_batched["returns"]
                advantage = sample_batched["advantage"]

                # 7. 학습 타입 스텝 증가
                self.learner_step += 1

                # 8. 로그 가능도과 베이스라인 계산
                log_probs, baseline = self.network(state, action)

                # 9. 손실 계산
                baseline_loss = self.MSELoss(baseline, returns) # 베이스라인 손실
                policy_loss = -(log_probs * advantage).mean()   # 정책 손실
                total_loss = baseline_loss + policy_loss        # 총 손실

                # 10. 백워드 패스 실행  (그레이디언트 계산)
                self.optimizer.zero_grad(set_to_none=True)
                total_loss.backward()

                # 11. 그레이디언트 클리핑
                torch.nn.utils.clip_grad_norm_(
                    self.network.parameters(), self.config.grad_norm_clip
                )

                # 12. 파라미터 업데이트
                self.optimizer.step()

                # 13. 손실 로깅
                # 정책 손실
                self.logger.log_stat("policy_loss",
                                     policy_loss.item(),
                                     self.learner_step)
                # 베이스라인 손실
                self.logger.log_stat("baseline_loss",
                                     baseline_loss.item(),
                                     self.learner_step)

        if self.config.lr_annealing:
            # 14. 학습률 스케쥴 업데이트
            self.policy_lr_scheduler.step(total_n_timesteps)
            self.baseline_lr_scheduler.step(total_n_timesteps)

            # 15. 학습률 로깅
            self.logger.log_stat("policy learning rate",
                                 self.optimizer.param_groups[0]['lr'],
                                 total_n_timesteps)
            self.logger.log_stat("baseline learning rate",
                                 self.optimizer.param_groups[1]['lr'],
                                 total_n_timesteps)

        # 16. 데이터셋 삭제
        self.buffer.clear()

        return True