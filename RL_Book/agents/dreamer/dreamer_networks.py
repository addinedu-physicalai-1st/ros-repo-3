"""
DreamerV3 Networks — PyTorch Implementation

핵심 구성요소:
    1. symlog / symexp: 스케일 불변 변환
    2. Twohot encoding: 분포 기반 보상/가치 예측 (255 bins)
    3. RMSNorm + SiLU MLP 블록
    4. RSSM: Sequence Model (GRU) + Encoder + Dynamics Predictor
    5. Reward / Continue / Decoder heads
    6. Actor (TruncatedNormal) + Critic (Distributional)

모델 크기 (Proprio Control, 12M):
    - hidden_dim: 256
    - gru_units: 1024 (→ recurrent state h_t)
    - latent: 32 classes × 16 categories = 512D (→ stochastic state z_t)
    - full model state: h_t(1024) + z_t(512) = 1536D

Reference: Hafner et al., "Mastering Diverse Domains through World Models", 2023
"""

import math
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.distributions as D
import numpy as np
from typing import Tuple, Optional


# ═══════════════════════════════════════════════════════════════
# 1. 유틸리티 함수
# ═══════════════════════════════════════════════════════════════

def symlog(x: torch.Tensor) -> torch.Tensor:
    """symlog(x) = sign(x) * ln(|x| + 1)"""
    return torch.sign(x) * torch.log1p(torch.abs(x))


def symexp(x: torch.Tensor) -> torch.Tensor:
    """symexp(x) = sign(x) * (exp(|x|) - 1)"""
    return torch.sign(x) * (torch.exp(torch.abs(x)) - 1.0)


class TwohotEncoder:
    """
    Twohot Encoding for distributional prediction.

    연속 값을 이산 bins 위의 분포로 표현.
    bins: symlog 공간에서 균등 분할된 255개 지점 [-20, +20].
    """

    def __init__(self, num_bins: int = 255, low: float = -20.0, high: float = 20.0):
        self.num_bins = num_bins
        self.low = low
        self.high = high
        # bins: symlog 공간에서 균등 분할
        self.bins = torch.linspace(low, high, num_bins)

    def encode(self, x: torch.Tensor) -> torch.Tensor:
        """
        실수 값 → twohot 분포 (soft one-hot).
        Args:
            x: (...) 임의 shape의 실수 텐서
        Returns:
            (..., num_bins) twohot 분포
        """
        x_symlog = symlog(x)
        bins = self.bins.to(x.device)

        # 가장 가까운 두 bin 사이에 선형 보간
        x_clamped = x_symlog.clamp(self.low, self.high)
        # 각 bin 간격
        bin_width = (self.high - self.low) / (self.num_bins - 1)
        # 연속 인덱스
        idx = (x_clamped - self.low) / bin_width
        lower = idx.floor().long().clamp(0, self.num_bins - 2)
        upper = lower + 1
        # 보간 가중치
        weight_upper = idx - lower.float()
        weight_lower = 1.0 - weight_upper

        # twohot 벡터 생성
        shape = x.shape
        twohot = torch.zeros(*shape, self.num_bins, device=x.device)
        twohot.scatter_(-1, lower.unsqueeze(-1), weight_lower.unsqueeze(-1))
        twohot.scatter_(-1, upper.unsqueeze(-1), weight_upper.unsqueeze(-1))
        return twohot

    def decode(self, logits: torch.Tensor) -> torch.Tensor:
        """
        twohot logits → 실수 값 (기댓값).
        Args:
            logits: (..., num_bins) logits
        Returns:
            (...) 복원된 실수 값
        """
        bins = self.bins.to(logits.device)
        probs = F.softmax(logits, dim=-1)
        # 기댓값 = sum(prob_i * bin_i), then symexp
        expectation = (probs * bins).sum(dim=-1)
        return symexp(expectation)


# ═══════════════════════════════════════════════════════════════
# 2. 기본 빌딩 블록
# ═══════════════════════════════════════════════════════════════

class RMSNorm(nn.Module):
    """Root Mean Square Layer Normalization (DreamerV3 표준)."""

    def __init__(self, dim: int, eps: float = 1e-8):
        super().__init__()
        self.scale = nn.Parameter(torch.ones(dim))
        self.eps = eps

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        rms = torch.sqrt(x.pow(2).mean(dim=-1, keepdim=True) + self.eps)
        return x / rms * self.scale


class DreamerMLP(nn.Module):
    """
    DreamerV3 표준 MLP 블록: Linear → RMSNorm → SiLU.
    여러 층을 쌓아서 사용.
    """

    def __init__(self, input_dim: int, hidden_dim: int, output_dim: int,
                 num_layers: int = 2):
        super().__init__()
        layers = []
        in_dim = input_dim
        for _ in range(num_layers):
            layers.extend([
                nn.Linear(in_dim, hidden_dim),
                RMSNorm(hidden_dim),
                nn.SiLU(),
            ])
            in_dim = hidden_dim
        layers.append(nn.Linear(hidden_dim, output_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class UnimixCategorical:
    """
    1% Uniform Mix Categorical Distribution.

    확률이 0이 되는 것을 방지하기 위해 1% uniform 혼합.
    probs_mixed = (1 - unimix) * softmax(logits) + unimix / num_classes
    """

    def __init__(self, logits: torch.Tensor, num_classes: int,
                 unimix: float = 0.01):
        self.num_classes = num_classes
        self.unimix = unimix

        probs = F.softmax(logits, dim=-1)
        # 1% uniform mixing
        self.probs = (1.0 - unimix) * probs + unimix / num_classes
        self.logits = torch.log(self.probs + 1e-8)

    def sample(self) -> torch.Tensor:
        """Straight-through Gumbel-Softmax 샘플링."""
        # Gumbel-softmax로 미분 가능한 샘플링
        indices = D.Categorical(probs=self.probs).sample()
        one_hot = F.one_hot(indices, self.num_classes).float()
        # Straight-through: forward는 one_hot, backward는 probs
        return one_hot + self.probs - self.probs.detach()

    def log_prob(self, value: torch.Tensor) -> torch.Tensor:
        """value: (..., num_classes) one-hot or soft."""
        return (value * self.logits).sum(dim=-1)

    def entropy(self) -> torch.Tensor:
        return D.Categorical(probs=self.probs).entropy()

    def kl_divergence(self, other: 'UnimixCategorical') -> torch.Tensor:
        """KL(self || other) per latent variable."""
        return (self.probs * (self.logits - other.logits)).sum(dim=-1)


# ═══════════════════════════════════════════════════════════════
# 3. RSSM (Recurrent State-Space Model)
# ═══════════════════════════════════════════════════════════════

class RSSM(nn.Module):
    """
    RSSM: World Model의 핵심.

    구성:
        - Sequence Model: h_t = GRU(h_{t-1}, [z_{t-1}, a_{t-1}])
        - Encoder:        z_t ~ Cat(softmax(MLP(h_t, x_t)))     [posterior]
        - Dynamics:       z^_t ~ Cat(softmax(MLP(h_t)))          [prior]

    State = (h_t, z_t):
        - h_t: deterministic recurrent state (GRU hidden, 1024D)
        - z_t: stochastic latent state (32 vars × 16 classes = 512D flattened)
    """

    def __init__(self, obs_dim: int, action_dim: int, hidden_dim: int = 256,
                 gru_units: int = 1024, num_latents: int = 32,
                 num_classes: int = 16, unimix: float = 0.01):
        super().__init__()
        self.gru_units = gru_units
        self.num_latents = num_latents
        self.num_classes = num_classes
        self.latent_dim = num_latents * num_classes  # 512
        self.unimix = unimix

        # Sequence Model: GRU input = z_{t-1}(512) + a_{t-1}(action_dim)
        self.gru_input = nn.Sequential(
            nn.Linear(self.latent_dim + action_dim, hidden_dim),
            RMSNorm(hidden_dim),
            nn.SiLU(),
        )
        self.gru = nn.GRUCell(hidden_dim, gru_units)

        # Encoder (posterior): input = h_t(1024) + x_t(obs_dim)
        self.encoder = DreamerMLP(
            gru_units + obs_dim, hidden_dim,
            num_latents * num_classes, num_layers=2)

        # Dynamics (prior): input = h_t(1024)
        self.dynamics = DreamerMLP(
            gru_units, hidden_dim,
            num_latents * num_classes, num_layers=2)

    def initial_state(self, batch_size: int, device: torch.device):
        """초기 RSSM 상태 (h_0, z_0)."""
        h = torch.zeros(batch_size, self.gru_units, device=device)
        z = torch.zeros(batch_size, self.latent_dim, device=device)
        return h, z

    def observe_step(self, h_prev: torch.Tensor, z_prev: torch.Tensor,
                     action: torch.Tensor, obs: torch.Tensor
                     ) -> Tuple[torch.Tensor, torch.Tensor,
                                torch.Tensor, torch.Tensor]:
        """
        한 스텝 관측: prior + posterior 모두 계산.

        Args:
            h_prev: (B, gru_units) 이전 deterministic state
            z_prev: (B, latent_dim) 이전 stochastic state
            action: (B, action_dim) 이전 행동
            obs:    (B, obs_dim) 현재 관측 (symlog 적용 후)

        Returns:
            h: (B, gru_units) 현재 deterministic state
            z_post: (B, latent_dim) posterior sample
            prior_logits: (B, num_latents, num_classes)
            post_logits:  (B, num_latents, num_classes)
        """
        # 1. Sequence model: h_t = GRU(h_{t-1}, [z_{t-1}, a_{t-1}])
        gru_in = self.gru_input(torch.cat([z_prev, action], dim=-1))
        h = self.gru(gru_in, h_prev)

        # 2. Prior: p(z_t | h_t)
        prior_logits = self.dynamics(h).reshape(-1, self.num_latents, self.num_classes)

        # 3. Posterior: q(z_t | h_t, x_t)
        post_logits = self.encoder(torch.cat([h, obs], dim=-1)).reshape(
            -1, self.num_latents, self.num_classes)

        # 4. Sample from posterior (straight-through)
        z_dist = UnimixCategorical(post_logits, self.num_classes, self.unimix)
        z_sample = z_dist.sample()  # (B, num_latents, num_classes)
        z_post = z_sample.reshape(-1, self.latent_dim)

        return h, z_post, prior_logits, post_logits

    def imagine_step(self, h_prev: torch.Tensor, z_prev: torch.Tensor,
                     action: torch.Tensor
                     ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        상상 한 스텝: prior만 사용 (관측 없음).

        Returns:
            h: (B, gru_units)
            z_prior: (B, latent_dim)
        """
        # 1. Sequence model
        gru_in = self.gru_input(torch.cat([z_prev, action], dim=-1))
        h = self.gru(gru_in, h_prev)

        # 2. Prior sample
        prior_logits = self.dynamics(h).reshape(-1, self.num_latents, self.num_classes)
        z_dist = UnimixCategorical(prior_logits, self.num_classes, self.unimix)
        z_sample = z_dist.sample()
        z_prior = z_sample.reshape(-1, self.latent_dim)

        return h, z_prior

    def observe_sequence(self, obs_seq: torch.Tensor, action_seq: torch.Tensor,
                         h_init: torch.Tensor, z_init: torch.Tensor
                         ) -> Tuple[torch.Tensor, torch.Tensor,
                                    torch.Tensor, torch.Tensor]:
        """
        시퀀스 관측: world model 학습용.

        Args:
            obs_seq:    (B, T, obs_dim)
            action_seq: (B, T, action_dim) — action_seq[:, t] = a_{t-1}
            h_init:     (B, gru_units)
            z_init:     (B, latent_dim)

        Returns:
            h_seq:         (B, T, gru_units)
            z_post_seq:    (B, T, latent_dim)
            prior_logits:  (B, T, num_latents, num_classes)
            post_logits:   (B, T, num_latents, num_classes)
        """
        B, T, _ = obs_seq.shape
        h_list, z_list = [], []
        prior_list, post_list = [], []

        h, z = h_init, z_init
        for t in range(T):
            h, z, prior_l, post_l = self.observe_step(
                h, z, action_seq[:, t], obs_seq[:, t])
            h_list.append(h)
            z_list.append(z)
            prior_list.append(prior_l)
            post_list.append(post_l)

        h_seq = torch.stack(h_list, dim=1)         # (B, T, gru_units)
        z_seq = torch.stack(z_list, dim=1)          # (B, T, latent_dim)
        prior_logits = torch.stack(prior_list, dim=1)  # (B, T, N, C)
        post_logits = torch.stack(post_list, dim=1)    # (B, T, N, C)

        return h_seq, z_seq, prior_logits, post_logits

    def get_state_dim(self) -> int:
        """Full model state 차원: h_t + z_t."""
        return self.gru_units + self.latent_dim

    @staticmethod
    def get_model_state(h: torch.Tensor, z: torch.Tensor) -> torch.Tensor:
        """h와 z를 연결하여 full model state 생성."""
        return torch.cat([h, z], dim=-1)


# ═══════════════════════════════════════════════════════════════
# 4. World Model Heads (Decoder, Reward, Continue)
# ═══════════════════════════════════════════════════════════════

class ObsDecoder(nn.Module):
    """관측 디코더: model_state → x^_t (symlog 공간)."""

    def __init__(self, state_dim: int, obs_dim: int, hidden_dim: int = 256):
        super().__init__()
        self.mlp = DreamerMLP(state_dim, hidden_dim, obs_dim, num_layers=2)

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        """
        Args:
            state: (..., state_dim) = cat(h_t, z_t)
        Returns:
            (..., obs_dim) predicted observation in symlog space
        """
        return self.mlp(state)


class RewardHead(nn.Module):
    """보상 예측: model_state → twohot logits (255 bins)."""

    def __init__(self, state_dim: int, hidden_dim: int = 256,
                 num_bins: int = 255):
        super().__init__()
        self.num_bins = num_bins
        self.mlp = DreamerMLP(state_dim, hidden_dim, num_bins, num_layers=2)
        self.twohot = TwohotEncoder(num_bins)

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        """Returns logits (for training)."""
        return self.mlp(state)

    def predict(self, state: torch.Tensor) -> torch.Tensor:
        """Returns predicted reward value (for imagination)."""
        logits = self.forward(state)
        return self.twohot.decode(logits)


class ContinueHead(nn.Module):
    """에피소드 지속 예측: model_state → Bernoulli logit."""

    def __init__(self, state_dim: int, hidden_dim: int = 256):
        super().__init__()
        self.mlp = DreamerMLP(state_dim, hidden_dim, 1, num_layers=2)

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        """Returns logit (for training with BCE loss)."""
        return self.mlp(state)

    def predict(self, state: torch.Tensor) -> torch.Tensor:
        """Returns continue probability."""
        return torch.sigmoid(self.forward(state))


# ═══════════════════════════════════════════════════════════════
# 5. Actor (TruncatedNormal)
# ═══════════════════════════════════════════════════════════════

class DreamerActor(nn.Module):
    """
    Actor: model_state → action distribution (TruncatedNormal).

    연속 행동 공간에서 [-1, 1] 범위의 행동을 출력.
    mean = tanh(raw_mean) → [-1, 1]
    std = softplus(raw_std) + min_std
    분포: Normal(mean, std) → tanh squashing → [-1, 1]
    """

    def __init__(self, state_dim: int, action_dim: int,
                 hidden_dim: int = 256, min_std: float = 0.1,
                 max_std: float = 1.0, num_layers: int = 3):
        super().__init__()
        self.action_dim = action_dim
        self.min_std = min_std
        self.max_std = max_std

        self.mlp = DreamerMLP(
            state_dim, hidden_dim, action_dim * 2, num_layers=num_layers)

    def forward(self, state: torch.Tensor
                ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Returns:
            mean: (..., action_dim)
            std:  (..., action_dim)
        """
        out = self.mlp(state)
        raw_mean, raw_std = out.chunk(2, dim=-1)
        mean = torch.tanh(raw_mean)
        std = self.min_std + (self.max_std - self.min_std) * torch.sigmoid(raw_std)
        return mean, std

    def get_dist(self, state: torch.Tensor) -> D.Independent:
        """행동 분포 반환."""
        mean, std = self.forward(state)
        dist = D.Normal(mean, std)
        return D.Independent(dist, 1)

    def sample(self, state: torch.Tensor,
               training: bool = True) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        행동 샘플링.
        Returns:
            action: (..., action_dim) clipped to [-1, 1]
            log_prob: (...)
        """
        dist = self.get_dist(state)
        if training:
            raw_action = dist.rsample()
        else:
            raw_action = dist.mean
        action = raw_action.clamp(-1.0, 1.0)
        log_prob = dist.log_prob(raw_action)
        return action, log_prob

    def log_prob(self, state: torch.Tensor,
                 action: torch.Tensor) -> torch.Tensor:
        """주어진 행동의 log probability."""
        dist = self.get_dist(state)
        return dist.log_prob(action)

    def entropy(self, state: torch.Tensor) -> torch.Tensor:
        """엔트로피 계산."""
        dist = self.get_dist(state)
        return dist.entropy()


# ═══════════════════════════════════════════════════════════════
# 6. Critic (Distributional with Twohot)
# ═══════════════════════════════════════════════════════════════

class DreamerCritic(nn.Module):
    """
    Critic: model_state → value distribution (Twohot, 255 bins).

    DreamerV3 표준: twohot 분포 기반 가치 예측.
    slow_target: EMA로 업데이트되는 타겟 네트워크.
    """

    def __init__(self, state_dim: int, hidden_dim: int = 256,
                 num_bins: int = 255, num_layers: int = 3):
        super().__init__()
        self.num_bins = num_bins
        self.mlp = DreamerMLP(
            state_dim, hidden_dim, num_bins, num_layers=num_layers)
        self.twohot = TwohotEncoder(num_bins)

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        """Returns logits (..., num_bins)."""
        return self.mlp(state)

    def predict(self, state: torch.Tensor) -> torch.Tensor:
        """Returns predicted value (scalar)."""
        logits = self.forward(state)
        return self.twohot.decode(logits)


# ═══════════════════════════════════════════════════════════════
# 7. World Model (통합 모듈)
# ═══════════════════════════════════════════════════════════════

class WorldModel(nn.Module):
    """
    DreamerV3 World Model.

    하위 모듈:
        - RSSM (sequence model + encoder + dynamics)
        - ObsDecoder (관측 복원)
        - RewardHead (보상 예측)
        - ContinueHead (에피소드 지속 예측)
    """

    def __init__(self, obs_dim: int, action_dim: int,
                 hidden_dim: int = 256, gru_units: int = 1024,
                 num_latents: int = 32, num_classes: int = 16,
                 num_bins: int = 255, unimix: float = 0.01):
        super().__init__()

        self.obs_dim = obs_dim
        self.action_dim = action_dim

        # RSSM
        self.rssm = RSSM(
            obs_dim=obs_dim, action_dim=action_dim,
            hidden_dim=hidden_dim, gru_units=gru_units,
            num_latents=num_latents, num_classes=num_classes,
            unimix=unimix)

        state_dim = self.rssm.get_state_dim()  # 1024 + 512 = 1536

        # Prediction heads
        self.decoder = ObsDecoder(state_dim, obs_dim, hidden_dim)
        self.reward_head = RewardHead(state_dim, hidden_dim, num_bins)
        self.continue_head = ContinueHead(state_dim, hidden_dim)

        # Twohot encoder (reward 학습용)
        self.twohot = TwohotEncoder(num_bins)

    def get_state_dim(self) -> int:
        return self.rssm.get_state_dim()


# ═══════════════════════════════════════════════════════════════
# 8. 전체 DreamerV3 네트워크 (Agent용)
# ═══════════════════════════════════════════════════════════════

class DreamerV3Network(nn.Module):
    """
    DreamerV3의 전체 네트워크.

    하위 모듈:
        - WorldModel (RSSM + heads)
        - Actor (정책)
        - Critic (가치 함수)
        - Slow Critic (EMA 타겟)
    """

    def __init__(self, obs_dim: int, action_dim: int,
                 hidden_dim: int = 256, gru_units: int = 1024,
                 num_latents: int = 32, num_classes: int = 16,
                 num_bins: int = 255, unimix: float = 0.01,
                 actor_layers: int = 3, critic_layers: int = 3,
                 min_std: float = 0.1, max_std: float = 1.0,
                 slow_target_tau: float = 0.02):
        super().__init__()

        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.slow_target_tau = slow_target_tau

        # World Model
        self.world_model = WorldModel(
            obs_dim=obs_dim, action_dim=action_dim,
            hidden_dim=hidden_dim, gru_units=gru_units,
            num_latents=num_latents, num_classes=num_classes,
            num_bins=num_bins, unimix=unimix)

        state_dim = self.world_model.get_state_dim()

        # Actor
        self.actor = DreamerActor(
            state_dim=state_dim, action_dim=action_dim,
            hidden_dim=hidden_dim, min_std=min_std, max_std=max_std,
            num_layers=actor_layers)

        # Critic
        self.critic = DreamerCritic(
            state_dim=state_dim, hidden_dim=hidden_dim,
            num_bins=num_bins, num_layers=critic_layers)

        # Slow Critic (EMA target — 학습하지 않음)
        self.slow_critic = DreamerCritic(
            state_dim=state_dim, hidden_dim=hidden_dim,
            num_bins=num_bins, num_layers=critic_layers)
        # 초기 동기화
        self.slow_critic.load_state_dict(self.critic.state_dict())
        for p in self.slow_critic.parameters():
            p.requires_grad = False

    def update_slow_critic(self):
        """EMA로 slow critic 업데이트."""
        tau = self.slow_target_tau
        for p, p_slow in zip(self.critic.parameters(),
                             self.slow_critic.parameters()):
            p_slow.data.lerp_(p.data, tau)

    def select_action(self, obs: torch.Tensor, h: torch.Tensor,
                      z: torch.Tensor, action_prev: torch.Tensor,
                      training: bool = True
                      ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        관측으로부터 행동 선택 (Actor 추론용).

        Args:
            obs:         (B, obs_dim) 현재 관측
            h:           (B, gru_units) 이전 deterministic state
            z:           (B, latent_dim) 이전 stochastic state
            action_prev: (B, action_dim) 이전 행동
            training:    훈련 모드 여부

        Returns:
            action:  (B, action_dim)
            h_new:   (B, gru_units)
            z_new:   (B, latent_dim)
        """
        with torch.no_grad():
            obs_symlog = symlog(obs)
            h_new, z_new, _, _ = self.world_model.rssm.observe_step(
                h, z, action_prev, obs_symlog)
            model_state = RSSM.get_model_state(h_new, z_new)
            action, _ = self.actor.sample(model_state, training=training)
        return action, h_new, z_new

    def save(self, checkpoint_path: str):
        """체크포인트 저장."""
        torch.save(self.state_dict(),
                   f"{checkpoint_path}/network.th")

    def restore(self, checkpoint_path: str, device: str = "cpu"):
        """체크포인트 복원."""
        state_dict = torch.load(
            f"{checkpoint_path}/network.th",
            map_location=torch.device(device))
        self.load_state_dict(state_dict)

    def get_variables(self) -> dict:
        return self.state_dict()
