"""
DreamerV3 Learner — World Model + Actor-Critic 학습

학습 과정:
    1. World Model 학습:
       - 리플레이 버퍼에서 시퀀스 샘플링
       - RSSM observe_sequence → posterior/prior 계산
       - Loss = reconstruction + reward + continue + KL
    2. Actor-Critic 학습 (상상 속):
       - 학습된 world model로 imagination rollout
       - λ-return 계산 (percentile normalization)
       - Actor: return 최대화 + 엔트로피 보너스
       - Critic: twohot distributional value loss

핵심 기법:
    - symlog: 관측/보상 스케일 정규화
    - KL balancing: β_dyn=1.0, β_rep=0.1, free_nats=1.0
    - Percentile return normalization: 5th-95th
    - EMA slow critic: τ=0.02
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from typing import Dict, Tuple

from agents.dreamer.dreamer_networks import (
    DreamerV3Network, RSSM, WorldModel,
    symlog, symexp, TwohotEncoder, UnimixCategorical
)
from datasets.episode_replay_buffer import EpisodeReplayBuffer


class DreamerV3Learner:
    """DreamerV3 학습자."""

    def __init__(self, config, logger, network: DreamerV3Network,
                 replay_buffer: EpisodeReplayBuffer):
        self.config = config
        self.logger = logger
        self.network = network
        self.replay_buffer = replay_buffer
        self.device = config.device

        # Twohot encoder
        self.twohot = TwohotEncoder(
            num_bins=getattr(config, 'num_bins', 255))

        # ─── Hyperparameters ───
        self.seq_len = getattr(config, 'seq_len', 64)
        self.batch_size = getattr(config, 'batch_size', 16)
        self.imagine_horizon = getattr(config, 'imagine_horizon', 15)
        self.gamma = getattr(config, 'gamma', 0.997)
        self.lambda_gae = getattr(config, 'lambda_gae', 0.95)
        self.entropy_coef = getattr(config, 'entropy_coef', 3e-4)
        self.free_nats = getattr(config, 'free_nats', 1.0)
        self.kl_beta_dyn = getattr(config, 'kl_beta_dyn', 1.0)
        self.kl_beta_rep = getattr(config, 'kl_beta_rep', 0.1)
        self.reward_loss_scale = getattr(config, 'reward_loss_scale', 1.0)
        self.continue_loss_scale = getattr(config, 'continue_loss_scale', 1.0)
        self.decoder_loss_scale = getattr(config, 'decoder_loss_scale', 1.0)
        self.grad_norm_clip = getattr(config, 'grad_norm_clip', 100.0)

        # Percentile normalization EMA
        self.return_ema_low = 0.0   # 5th percentile
        self.return_ema_high = 1.0  # 95th percentile
        self.return_ema_decay = 0.99

        # ─── Optimizers ───
        lr_world = getattr(config, 'lr_world', 1e-4)
        lr_actor = getattr(config, 'lr_actor', 3e-5)
        lr_critic = getattr(config, 'lr_critic', 3e-5)
        eps = getattr(config, 'optim_eps', 1e-8)

        self.world_optimizer = torch.optim.Adam(
            network.world_model.parameters(), lr=lr_world, eps=eps)
        self.actor_optimizer = torch.optim.Adam(
            network.actor.parameters(), lr=lr_actor, eps=eps)
        self.critic_optimizer = torch.optim.Adam(
            network.critic.parameters(), lr=lr_critic, eps=eps)

        self.learner_step = 0

    # ═══════════════════════════════════════════════════════════
    # World Model 학습
    # ═══════════════════════════════════════════════════════════

    def update_world_model(self, data: Dict[str, torch.Tensor]
                           ) -> Tuple[Dict, torch.Tensor, torch.Tensor]:
        """
        World Model 학습 한 스텝.

        Args:
            data: {obs, action, reward, done} — (B, T, ...)

        Returns:
            metrics: 로깅용 딕셔너리
            h_seq: (B, T, gru_units)
            z_seq: (B, T, latent_dim)
        """
        wm = self.network.world_model
        rssm = wm.rssm

        obs = data["obs"]       # (B, T, obs_dim)
        action = data["action"] # (B, T, action_dim)
        reward = data["reward"] # (B, T)
        done = data["done"]     # (B, T)
        B, T, _ = obs.shape

        # symlog 변환
        obs_symlog = symlog(obs)

        # 초기 상태
        h_init, z_init = rssm.initial_state(B, obs.device)

        # action을 시프트: a_{t-1} for step t
        # action_seq[:, 0] = zeros (첫 스텝에 이전 행동 없음)
        action_shifted = torch.cat([
            torch.zeros(B, 1, action.shape[-1], device=obs.device),
            action[:, :-1]
        ], dim=1)

        # RSSM sequence 관측
        h_seq, z_seq, prior_logits, post_logits = rssm.observe_sequence(
            obs_symlog, action_shifted, h_init, z_init)

        # Model state
        model_state = RSSM.get_model_state(h_seq, z_seq)  # (B, T, state_dim)

        # ─── Reconstruction loss (symlog MSE) ───
        obs_pred = wm.decoder(model_state)  # (B, T, obs_dim)
        decoder_loss = F.mse_loss(obs_pred, obs_symlog, reduction='none')
        decoder_loss = decoder_loss.mean(dim=-1).mean()  # scalar

        # ─── Reward loss (twohot cross-entropy) ───
        reward_logits = wm.reward_head(model_state)  # (B, T, num_bins)
        reward_target = self.twohot.encode(reward)    # (B, T, num_bins)
        reward_loss = -torch.sum(
            reward_target * F.log_softmax(reward_logits, dim=-1),
            dim=-1).mean()

        # ─── Continue loss (binary cross-entropy) ───
        continue_logit = wm.continue_head(model_state).squeeze(-1)  # (B, T)
        continue_target = 1.0 - done  # done=1 → continue=0
        continue_loss = F.binary_cross_entropy_with_logits(
            continue_logit, continue_target, reduction='mean')

        # ─── KL loss (KL balancing) ───
        kl_loss = self._kl_loss(prior_logits, post_logits)

        # ─── Total world model loss ───
        total_loss = (
            self.decoder_loss_scale * decoder_loss +
            self.reward_loss_scale * reward_loss +
            self.continue_loss_scale * continue_loss +
            kl_loss
        )

        self.world_optimizer.zero_grad()
        total_loss.backward()
        nn.utils.clip_grad_norm_(
            wm.parameters(), self.grad_norm_clip)
        self.world_optimizer.step()

        metrics = {
            "wm/decoder_loss": decoder_loss.item(),
            "wm/reward_loss": reward_loss.item(),
            "wm/continue_loss": continue_loss.item(),
            "wm/kl_loss": kl_loss.item(),
            "wm/total_loss": total_loss.item(),
        }

        return metrics, h_seq.detach(), z_seq.detach()

    def _kl_loss(self, prior_logits: torch.Tensor,
                 post_logits: torch.Tensor) -> torch.Tensor:
        """
        KL Balancing: free bits + asymmetric weighting.

        KL_loss = β_dyn * max(free, KL(sg(post) || prior))
                + β_rep * max(free, KL(post || sg(prior)))

        Args:
            prior_logits: (B, T, num_latents, num_classes)
            post_logits:  (B, T, num_latents, num_classes)

        Returns:
            scalar KL loss
        """
        B, T, N, C = post_logits.shape
        unimix = self.network.world_model.rssm.unimix

        # posterior/prior 분포 생성
        post_dist = UnimixCategorical(post_logits, C, unimix)
        prior_dist = UnimixCategorical(prior_logits, C, unimix)

        # KL(sg(post) || prior) — dynamics 학습용
        post_sg = UnimixCategorical(post_logits.detach(), C, unimix)
        kl_dyn = post_sg.kl_divergence(prior_dist)  # (B, T, N)
        kl_dyn = kl_dyn.sum(dim=-1)  # (B, T) — latent 차원 합산
        kl_dyn = torch.clamp(kl_dyn, min=self.free_nats).mean()

        # KL(post || sg(prior)) — representation 학습용
        prior_sg = UnimixCategorical(prior_logits.detach(), C, unimix)
        kl_rep = post_dist.kl_divergence(prior_sg)  # (B, T, N)
        kl_rep = kl_rep.sum(dim=-1)
        kl_rep = torch.clamp(kl_rep, min=self.free_nats).mean()

        return self.kl_beta_dyn * kl_dyn + self.kl_beta_rep * kl_rep

    # ═══════════════════════════════════════════════════════════
    # Actor-Critic 학습 (상상 속)
    # ═══════════════════════════════════════════════════════════

    def update_actor_critic(self, h_seq: torch.Tensor,
                            z_seq: torch.Tensor) -> Dict:
        """
        상상 속에서 Actor와 Critic 학습.

        1. h_seq, z_seq에서 시작점 샘플링
        2. Imagination rollout (world model으로 미래 궤적 생성)
        3. λ-return 계산 + percentile normalization
        4. Actor update (return 최대화)
        5. Critic update (twohot value loss)

        Args:
            h_seq: (B, T, gru_units) — world model 학습에서 나온 상태
            z_seq: (B, T, latent_dim)

        Returns:
            metrics: 로깅용 딕셔너리
        """
        B, T, _ = h_seq.shape
        rssm = self.network.world_model.rssm

        # 시작점: 시퀀스의 모든 시점에서 시작 가능
        h_start = h_seq.reshape(B * T, -1).detach()  # (B*T, gru_units)
        z_start = z_seq.reshape(B * T, -1).detach()  # (B*T, latent_dim)

        # ─── Imagination Rollout ───
        imagine_states, imagine_actions, imagine_rewards, imagine_continues = \
            self._imagine(h_start, z_start)
        # imagine_states: (H+1, B*T, state_dim)
        # imagine_rewards: (H, B*T)
        # imagine_continues: (H, B*T)

        # ─── Critic values ───
        with torch.no_grad():
            # slow critic으로 타겟 가치 계산
            slow_values = self.network.slow_critic.predict(
                imagine_states)  # (H+1, B*T)

        # ─── λ-return ───
        returns = self._lambda_return(
            imagine_rewards, slow_values, imagine_continues)
        # returns: (H, B*T)

        # Percentile normalization
        returns_norm = self._normalize_returns(returns)

        # ─── Actor Loss (DreamerV3: dynamics backprop + reinforce + entropy) ───
        actor_states = imagine_states[:-1].detach()  # reinforce/entropy용

        # 1. Dynamics backprop: returns를 통해 actor로 직접 역전파
        dynback_loss = -returns_norm.mean()

        # 2. Reinforce with baseline (보조 그래디언트)
        with torch.no_grad():
            baseline = self.network.critic.predict(actor_states)
            scale = max(self.return_ema_high - self.return_ema_low, 1.0)
            baseline_norm = (baseline - self.return_ema_low) / scale
            advantage = returns_norm.detach() - baseline_norm

        actor_log_probs = self.network.actor.log_prob(
            actor_states, imagine_actions.detach())  # (H, B*T)
        reinforce_loss = -(advantage * actor_log_probs).mean()

        # 3. Entropy bonus
        actor_entropy = self.network.actor.entropy(actor_states)  # (H, B*T)
        entropy_loss = -self.entropy_coef * actor_entropy.mean()

        # Combined actor loss
        actor_loss = dynback_loss + reinforce_loss + entropy_loss

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        nn.utils.clip_grad_norm_(
            self.network.actor.parameters(), self.grad_norm_clip)
        self.actor_optimizer.step()

        # ─── Critic Loss ───
        critic_states = imagine_states[:-1].detach()  # (H, B*T, state_dim)
        critic_logits = self.network.critic(critic_states)  # (H, B*T, bins)
        returns_target = self.twohot.encode(returns.detach())  # (H, B*T, bins)
        critic_loss = -torch.sum(
            returns_target * F.log_softmax(critic_logits, dim=-1),
            dim=-1).mean()

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        nn.utils.clip_grad_norm_(
            self.network.critic.parameters(), self.grad_norm_clip)
        self.critic_optimizer.step()

        # Slow critic EMA update
        self.network.update_slow_critic()

        metrics = {
            "ac/actor_loss": actor_loss.item(),
            "ac/dynback_loss": dynback_loss.item(),
            "ac/reinforce_loss": reinforce_loss.item(),
            "ac/critic_loss": critic_loss.item(),
            "ac/actor_entropy": actor_entropy.mean().item(),
            "ac/returns_mean": returns.mean().item(),
            "ac/returns_std": returns.std().item(),
            "ac/advantage_mean": advantage.mean().item(),
        }

        return metrics

    def _imagine(self, h_start: torch.Tensor, z_start: torch.Tensor):
        """
        Imagination rollout: world model으로 미래 궤적 생성.

        Args:
            h_start: (N, gru_units)
            z_start: (N, latent_dim)

        Returns:
            states:    (H+1, N, state_dim)
            actions:   (H, N, action_dim)
            rewards:   (H, N)
            continues: (H, N)
        """
        rssm = self.network.world_model.rssm
        wm = self.network.world_model
        H = self.imagine_horizon

        h, z = h_start, z_start
        states = [RSSM.get_model_state(h, z)]
        actions, rewards, continues = [], [], []

        for t in range(H):
            state = states[-1]

            # Actor 행동 선택 (미분 가능)
            action, _ = self.network.actor.sample(state, training=True)
            actions.append(action)

            # World model으로 한 스텝 상상
            h, z = rssm.imagine_step(h, z, action)
            state_new = RSSM.get_model_state(h, z)
            states.append(state_new)

            # 보상 & continue 예측
            reward = wm.reward_head.predict(state_new)
            cont = wm.continue_head.predict(state_new).squeeze(-1)
            rewards.append(reward)
            continues.append(cont)

        states = torch.stack(states, dim=0)      # (H+1, N, state_dim)
        actions = torch.stack(actions, dim=0)     # (H, N, action_dim)
        rewards = torch.stack(rewards, dim=0)     # (H, N)
        continues = torch.stack(continues, dim=0) # (H, N)

        return states, actions, rewards, continues

    def _lambda_return(self, rewards: torch.Tensor, values: torch.Tensor,
                       continues: torch.Tensor) -> torch.Tensor:
        """
        λ-return 계산 (GAE와 유사).

        R_t^λ = r_t + γ * c_t * ((1-λ) * V(s_{t+1}) + λ * R_{t+1}^λ)
        R_H^λ = V(s_H)

        Args:
            rewards:   (H, N)
            values:    (H+1, N)
            continues: (H, N)

        Returns:
            returns: (H, N)
        """
        H, N = rewards.shape
        returns_list = []

        # Bootstrap from last value
        last_return = values[H]  # V(s_H)

        for t in reversed(range(H)):
            # λ-return (list+stack으로 그래디언트 보존)
            last_return = rewards[t] + \
                self.gamma * continues[t] * (
                    (1.0 - self.lambda_gae) * values[t + 1] +
                    self.lambda_gae * last_return
                )
            returns_list.insert(0, last_return)

        return torch.stack(returns_list, dim=0)  # (H, N)

    def _normalize_returns(self, returns: torch.Tensor) -> torch.Tensor:
        """
        Percentile return normalization (5th-95th).

        EMA로 percentile 추적 → returns를 [0, 1]로 정규화.
        """
        # EMA 추적 (no_grad — 통계 업데이트만)
        with torch.no_grad():
            flat = returns.detach().flatten()
            if flat.numel() > 0:
                low = float(torch.quantile(flat, 0.05))
                high = float(torch.quantile(flat, 0.95))

                self.return_ema_low = (
                    self.return_ema_decay * self.return_ema_low +
                    (1 - self.return_ema_decay) * low)
                self.return_ema_high = (
                    self.return_ema_decay * self.return_ema_high +
                    (1 - self.return_ema_decay) * high)

        # 정규화 (그래디언트 보존 — dynamics backprop용)
        scale = max(self.return_ema_high - self.return_ema_low, 1.0)
        return (returns - self.return_ema_low) / scale

    # ═══════════════════════════════════════════════════════════
    # 통합 업데이트
    # ═══════════════════════════════════════════════════════════

    def update(self, total_n_timesteps: int) -> Dict:
        """
        DreamerV3 전체 학습 한 스텝.

        1. 리플레이 버퍼에서 시퀀스 샘플링
        2. World model 학습
        3. Actor-Critic 학습

        Returns:
            metrics: 로깅용 딕셔너리
        """
        if not self.replay_buffer.is_ready(self.batch_size, self.seq_len):
            return {}

        # 1. 시퀀스 샘플링
        data = self.replay_buffer.sample(
            self.batch_size, self.seq_len,
            device=torch.device(self.device))

        # 2. World model 학습
        wm_metrics, h_seq, z_seq = self.update_world_model(data)

        # 3. Actor-Critic 학습
        ac_metrics = self.update_actor_critic(h_seq, z_seq)

        self.learner_step += 1
        metrics = {**wm_metrics, **ac_metrics}
        return metrics

    def save(self, checkpoint_path: str):
        """체크포인트 저장."""
        self.network.save(checkpoint_path)
        torch.save({
            "world_optimizer": self.world_optimizer.state_dict(),
            "actor_optimizer": self.actor_optimizer.state_dict(),
            "critic_optimizer": self.critic_optimizer.state_dict(),
            "learner_step": self.learner_step,
            "return_ema_low": self.return_ema_low,
            "return_ema_high": self.return_ema_high,
        }, f"{checkpoint_path}/opt.th")

    def restore(self, checkpoint_path: str):
        """체크포인트 복원."""
        self.network.restore(checkpoint_path, device=self.device)
        opt_state = torch.load(
            f"{checkpoint_path}/opt.th",
            map_location=lambda storage, loc: storage)
        self.world_optimizer.load_state_dict(opt_state["world_optimizer"])
        self.actor_optimizer.load_state_dict(opt_state["actor_optimizer"])
        self.critic_optimizer.load_state_dict(opt_state["critic_optimizer"])
        self.learner_step = opt_state["learner_step"]
        self.return_ema_low = opt_state["return_ema_low"]
        self.return_ema_high = opt_state["return_ema_high"]
