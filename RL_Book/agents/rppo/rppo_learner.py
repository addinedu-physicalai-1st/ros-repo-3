import torch
import torch.nn as nn
from typing import Any, Dict, Tuple
from types import SimpleNamespace
from datasets.rollout_buffer import RolloutBuffer
from agents.rppo.rppo_network import RecurrentPPONetwork
from envs.environment import EnvironmentSpec
from utils.logging import Logger
from agents.base import Learner
from utils.lr_scheduler import CosineLR
from utils.value_util import REGISTRY as RETURN_REGISTRY
from utils.schduler import LinearScheduler

class RecurrentPPOLearner(Learner):
    """Learner for Recurrent PPO."""

    def __init__(self,
                 config: SimpleNamespace,
                 logger: Logger,
                 environment_spec: EnvironmentSpec,
                 network: RecurrentPPONetwork,
                 buffer: RolloutBuffer):
        super().__init__(config, logger, environment_spec, network, buffer)

        # 1. 옵티마이저 생성 (공유 레이어+정책 / 가치 함수 분리)
        self.optimizer = torch.optim.Adam([
            {'params': list(self.network.feature_extractor.parameters()) +
                       list(self.network.gru.parameters()) +
                       list(self.network.policy_mean.parameters()) +
                       [self.network.policy_log_std],
             'lr': self.config.lr_policy},
            {'params': self.network.critic.parameters(),
             'lr': self.config.lr_critic}
        ])

        # 2. 학습률 스케쥴러 생성
        self.policy_lr_scheduler = None
        self.critic_lr_scheduler = None
        if self.config.lr_annealing:
            self.policy_lr_scheduler = CosineLR(
                logger=self.logger,
                param_groups=self.optimizer.param_groups[0],
                start_lr=self.config.lr_policy,
                end_timesteps=self.config.max_environment_steps,
                name="policy lr")
            self.critic_lr_scheduler = CosineLR(
                logger=self.logger,
                param_groups=self.optimizer.param_groups[1],
                start_lr=self.config.lr_critic,
                end_timesteps=self.config.max_environment_steps,
                name="critic lr")

        # 3. PPO Clipping Scheduler
        end_timesteps = self.config.max_environment_steps if self.config.clip_schedule else -1
        self.clip_scheduler = LinearScheduler(
            start_value=self.config.ppo_clipping_epsilon,
            start_timesteps=1,
            end_timesteps=end_timesteps)

        self.MSELoss = nn.MSELoss()

    def _calc_target_value(self):
        """Calculate targets and advantages for Recurrent PPO."""
        if len(self.buffer) == 0: return

        # 1. Get data from buffer
        states = self.buffer['state']
        next_states = self.buffer['next_state']
        rewards = self.buffer['reward']
        dones = self.buffer['done']
        hidden_states = self.buffer['hidden_state'] # (batch, 1, hidden_dim)
        
        # 2. Compute values for all states in one go
        # Note: RolloutBuffer might contain multiple episodes, 
        # but hidden_states are stored for each step.
        state_seq = states.unsqueeze(1) # (batch, 1, state_dim)
        hidden_init = hidden_states.permute(1, 0, 2).contiguous() # (1, batch, hidden_dim)
        
        with torch.no_grad():
            policy_mean, log_std, values, updated_hidden = self.network.forward(state_seq, hidden_init)
            values = values.squeeze(1) # (batch, 1)
            
            # For next_values, we need the NEXT hidden states.
            # In Actor.observe, we don't store next_hidden_state, but we can re-calculate.
            _, _, next_values, _ = self.network.forward(next_states.unsqueeze(1), updated_hidden)
            next_values = next_values.squeeze(1) # (batch, 1)

            # 3. Calculate GAE
            gamma = self.config.gamma
            gae_lambda = self.config.gae_lambda
            
            deltas = rewards + (1 - dones) * gamma * next_values - values
            advantages = deltas.clone()
            
            # GAE calculation across the buffer
            # Note: This assumes transitions are contiguous and from same episode if done is 0.
            for t in reversed(range(len(advantages) - 1)):
                advantages[t] += (1 - dones[t]) * gamma * gae_lambda * advantages[t + 1]
            
            target_values = advantages + values
            
            # Standardize advantages
            if self.config.gae_standardization:
                advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
                
            # Compute log_probs_old
            dist = self.network.get_distribution(policy_mean, log_std)
            log_probs_old = dist.log_prob(self.buffer['action'].unsqueeze(1)).sum(dim=-1)

        # 4. Store in buffer
        if self.buffer["advantage"] is None:
            schema = {
                'advantage': {'shape': (1,)},
                'target_value': {'shape': (1,)},
                'log_probs_old': {'shape': (log_probs_old.shape[-1],),},
            }
            self.buffer.extend_schema(schema)

        self.buffer['advantage'] = advantages
        self.buffer['target_value'] = target_values
        self.buffer['log_probs_old'] = log_probs_old

    def update(self, total_n_timesteps: int, total_n_episodes: int) -> bool:
        if len(self.buffer) == 0: return False

        self._calc_target_value()
        clipping_epsilon = self.clip_scheduler.eval(total_n_timesteps)

        # ===== 청크 기반 Recurrent Training =====
        # 버퍼의 연속된 transitions를 chunk_length 길이의 시퀀스로 분할하여
        # GRU가 시간적 의존성을 학습할 수 있도록 함.
        chunk_length = getattr(self.config, "recurrent_chunk_length", 16)

        # 1. 버퍼에서 전체 데이터 가져오기
        states = self.buffer['state']               # (T, state_dim)
        actions = self.buffer['action']             # (T, action_dim)
        advantages = self.buffer['advantage']       # (T, 1)
        target_values = self.buffer['target_value'] # (T, 1)
        log_probs_old = self.buffer['log_probs_old']# (T, 1)
        hidden_states = self.buffer['hidden_state'] # (T, 1, hidden_dim)
        dones = self.buffer['done']                 # (T, 1)

        num_transitions = len(self.buffer)

        # 2. 연속된 시퀀스 청크로 분할
        # 마지막에 chunk_length로 나누어 떨어지지 않는 나머지는 버림
        num_chunks = num_transitions // chunk_length
        usable = num_chunks * chunk_length

        if num_chunks == 0:
            # 버퍼 크기가 chunk_length보다 작으면 기존 방식(길이 1)으로 폴백
            chunk_length = 1
            num_chunks = num_transitions
            usable = num_transitions

        # (num_chunks, chunk_length, dim) 형태로 reshape
        c_states = states[:usable].reshape(num_chunks, chunk_length, -1)
        c_actions = actions[:usable].reshape(num_chunks, chunk_length, -1)
        c_advantages = advantages[:usable].reshape(num_chunks, chunk_length, -1)
        c_target_values = target_values[:usable].reshape(num_chunks, chunk_length, -1)
        c_log_probs_old = log_probs_old[:usable].reshape(num_chunks, chunk_length, -1)
        c_dones = dones[:usable].reshape(num_chunks, chunk_length, -1)

        # 각 청크의 첫 번째 스텝의 hidden state 사용
        chunk_start_indices = torch.arange(0, usable, chunk_length)
        c_hidden = hidden_states[chunk_start_indices]  # (num_chunks, 1, hidden_dim)

        # 3. 에폭 루프
        # config.batch_size를 청크 길이 단위로 치환하여 진짜 미니배치 구현!
        chunk_batch_size = max(1, self.config.batch_size // chunk_length)
        batch_size = min(chunk_batch_size, num_chunks)

        for epoch in range(self.config.n_epochs):
            # 청크 단위로 셔플
            perm = torch.randperm(num_chunks)

            num_batches = (num_chunks - 1) // batch_size + 1
            for i in range(num_batches):
                idx = perm[i * batch_size : (i + 1) * batch_size]
                if len(idx) == 0: continue

                # 미니배치 추출: (B, chunk_length, dim)
                b_states = c_states[idx]
                b_actions = c_actions[idx]
                b_advantages = c_advantages[idx]          # (B, L, 1)
                b_target_values = c_target_values[idx]    # (B, L, 1)
                b_log_probs_old = c_log_probs_old[idx]    # (B, L, 1)
                b_dones = c_dones[idx]                    # (B, L, 1)
                b_hidden = c_hidden[idx].permute(1, 0, 2).contiguous()  # (1, B, H)

                # Forward: GRU가 chunk_length 스텝을 순차 처리
                # done 마스크를 전달하여 에피소드 경계에서 hidden state 리셋
                log_probs, entropy, values = self.network.evaluate(
                    b_states, b_hidden, b_actions, done=b_dones)

                # (B, L, 1) → (B*L, 1) 으로 평탄화하여 loss 계산
                values = values.reshape(-1, 1)
                log_probs = log_probs.reshape(-1, 1)
                entropy = entropy.reshape(-1, 1)
                b_advantages_flat = b_advantages.reshape(-1, 1)
                b_target_values_flat = b_target_values.reshape(-1, 1)
                b_log_probs_old_flat = b_log_probs_old.reshape(-1, 1)

                # PPO Loss
                ratios = torch.exp(log_probs - b_log_probs_old_flat)
                surr1 = ratios * b_advantages_flat
                surr2 = torch.clamp(ratios, 1 - clipping_epsilon, 1 + clipping_epsilon) * b_advantages_flat
                policy_loss = -torch.min(surr1, surr2).mean()

                value_loss = self.MSELoss(values, b_target_values_flat)
                entropy_loss = -entropy.mean()

                total_loss = policy_loss + self.config.vloss_coef * value_loss + self.config.eloss_coef * entropy_loss

                self.optimizer.zero_grad()
                total_loss.backward()
                torch.nn.utils.clip_grad_norm_(self.network.parameters(), self.config.grad_norm_clip)
                self.optimizer.step()

                self.learner_step += 1
                self.logger.log_stat("total_loss", total_loss.item(), self.learner_step)
                self.logger.log_stat("policy_loss", policy_loss.item(), self.learner_step)
                self.logger.log_stat("value_loss", value_loss.item(), self.learner_step)
                self.logger.log_stat("entropy_loss", entropy_loss.item(), self.learner_step)

        # 학습률 스케쥴 업데이트
        if self.config.lr_annealing:
            self.policy_lr_scheduler.step(total_n_timesteps)
            self.critic_lr_scheduler.step(total_n_timesteps)
            self.logger.log_stat("policy learning rate",
                                 self.optimizer.param_groups[0]['lr'],
                                 total_n_timesteps)
            self.logger.log_stat("critic learning rate",
                                 self.optimizer.param_groups[1]['lr'],
                                 total_n_timesteps)

        self.buffer.clear()
        return True
