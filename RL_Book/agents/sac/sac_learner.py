"""
SAC Learner — Soft Actor-Critic 학습 로직

학습 루프:
    1. 리플레이 버퍼에서 미니배치 샘플링
    2. Twin Q-Networks 업데이트 (Bellman + min Q target)
    3. Actor 업데이트 (max Q + α × entropy)
    4. α 업데이트 (엔트로피 목표 추적)
    5. Target Q 소프트 업데이트 (EMA)
"""

import torch
import torch.nn.functional as F
from types import SimpleNamespace
from envs.environment import EnvironmentSpec
from utils.logging import Logger
from agents.base import Learner
from agents.sac.sac_network import SACNetwork
from datasets.replay_buffer import ReplayBuffer


class SACLearner(Learner):
    """SAC 학습자."""

    def __init__(self,
                 config: SimpleNamespace,
                 logger: Logger,
                 environment_spec: EnvironmentSpec,
                 network: SACNetwork,
                 buffer: ReplayBuffer):
        super().__init__(config, logger, environment_spec, network, buffer)

        self.gamma = getattr(config, "gamma", 0.99)
        self.tau = getattr(config, "soft_target_tau", 0.005)
        self.batch_size = getattr(config, "batch_size", 256)
        self.warmup_step = getattr(config, "warmup_step", 5000)

        self.device = torch.device(config.device)

        # 옵티마이저 (Actor, Critic, Alpha 분리)
        lr_actor = getattr(config, "lr_actor", 3e-4)
        lr_critic = getattr(config, "lr_critic", 3e-4)
        lr_alpha = getattr(config, "lr_alpha", 3e-4)

        self.actor_optimizer = torch.optim.Adam(
            self.network.actor.parameters(), lr=lr_actor)
        self.q_optimizer = torch.optim.Adam(
            list(self.network.q1.parameters()) +
            list(self.network.q2.parameters()),
            lr=lr_critic)
        self.alpha_optimizer = torch.optim.Adam(
            [self.network.log_alpha], lr=lr_alpha)

        # 학습 실행 기준
        self.update_every = getattr(config, "update_every", 1)
        self.updates_per_step = getattr(config, "updates_per_step", 1)

    def update(self, total_n_timesteps: int, total_n_episodes: int) -> bool:
        """매 스텝 학습."""
        if len(self.buffer) < self.warmup_step:
            return False
        if total_n_timesteps % self.update_every != 0:
            return False

        for _ in range(self.updates_per_step):
            self._update_step(total_n_timesteps)

        return True

    def _update_step(self, total_n_timesteps: int):
        """SAC 학습 한 스텝."""

        # 1. 미니배치 샘플링
        batch = self._sample_batch()
        states, actions, rewards, next_states, dones = batch

        # 2. Target Q 계산 (no grad)
        with torch.no_grad():
            next_actions, next_log_probs = self.network.actor.sample(
                next_states)
            q1_target = self.network.q1_target(next_states, next_actions)
            q2_target = self.network.q2_target(next_states, next_actions)
            q_target = torch.min(q1_target, q2_target)
            # Soft Bellman: y = r + γ(1-d)(min Q' - α log π')
            target_value = rewards + (1.0 - dones) * self.gamma * (
                q_target - self.network.alpha.detach() * next_log_probs)

        # 3. Q-Networks 업데이트
        q1_pred = self.network.q1(states, actions)
        q2_pred = self.network.q2(states, actions)
        q1_loss = F.mse_loss(q1_pred, target_value)
        q2_loss = F.mse_loss(q2_pred, target_value)
        q_loss = q1_loss + q2_loss

        self.q_optimizer.zero_grad()
        q_loss.backward()
        self.q_optimizer.step()

        # 4. Actor 업데이트
        new_actions, log_probs = self.network.actor.sample(states)
        q1_new = self.network.q1(states, new_actions)
        q2_new = self.network.q2(states, new_actions)
        q_new = torch.min(q1_new, q2_new)
        # max E[Q - α log π] = min E[-Q + α log π]
        actor_loss = (self.network.alpha.detach() * log_probs - q_new).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # 5. α 업데이트 (자동 온도 조절)
        alpha_loss = -(self.network.log_alpha * (
            log_probs.detach() + self.network.target_entropy)).mean()

        self.alpha_optimizer.zero_grad()
        alpha_loss.backward()
        self.alpha_optimizer.step()

        # 6. Target Q 소프트 업데이트
        self._soft_update(self.network.q1, self.network.q1_target)
        self._soft_update(self.network.q2, self.network.q2_target)

        # 7. 로깅
        self.learner_step += 1
        if self.learner_step % 1000 == 0:
            self.logger.log_stat("q_loss", q_loss.item(), total_n_timesteps)
            self.logger.log_stat("actor_loss", actor_loss.item(),
                                 total_n_timesteps)
            self.logger.log_stat("alpha", self.network.alpha.item(),
                                 total_n_timesteps)
            self.logger.log_stat("alpha_loss", alpha_loss.item(),
                                 total_n_timesteps)
            self.logger.log_stat("q1_mean", q1_pred.mean().item(),
                                 total_n_timesteps)

    def _sample_batch(self):
        """리플레이 버퍼에서 미니배치 샘플링."""
        n = len(self.buffer)
        indices = torch.randint(0, n, (self.batch_size,))

        states = self.buffer['state'][indices].to(self.device)
        actions = self.buffer['action'][indices].to(self.device)
        rewards = self.buffer['reward'][indices].to(self.device)
        next_states = self.buffer['next_state'][indices].to(self.device)
        dones = self.buffer['done'][indices].to(self.device)

        # shape 안전성: reward, done → (batch, 1)
        if rewards.dim() == 1:
            rewards = rewards.unsqueeze(-1)
        if dones.dim() == 1:
            dones = dones.unsqueeze(-1)

        return states, actions, rewards, next_states, dones

    def _soft_update(self, source: torch.nn.Module,
                     target: torch.nn.Module):
        """EMA 소프트 업데이트: θ' ← τθ + (1-τ)θ'"""
        for param, target_param in zip(source.parameters(),
                                       target.parameters()):
            target_param.data.copy_(
                self.tau * param.data + (1.0 - self.tau) * target_param.data)

    def save(self, checkpoint_path: str):
        self.network.save(checkpoint_path)
        torch.save({
            'actor_opt': self.actor_optimizer.state_dict(),
            'q_opt': self.q_optimizer.state_dict(),
            'alpha_opt': self.alpha_optimizer.state_dict(),
        }, f"{checkpoint_path}/opt.th")

    def restore(self, checkpoint_path: str):
        self.network.restore(checkpoint_path)
        opt_state = torch.load(
            f"{checkpoint_path}/opt.th",
            map_location=lambda storage, loc: storage)
        self.actor_optimizer.load_state_dict(opt_state['actor_opt'])
        self.q_optimizer.load_state_dict(opt_state['q_opt'])
        self.alpha_optimizer.load_state_dict(opt_state['alpha_opt'])
