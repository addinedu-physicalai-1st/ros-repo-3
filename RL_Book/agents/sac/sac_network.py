"""
SAC Network — Soft Actor-Critic (Haarnoja et al., 2018)

Off-policy, Maximum Entropy RL.

구조:
    - Actor: SquashedGaussian 정책 (tanh squashing)
    - Twin Q-Networks: 2개의 독립 Q-함수 (overestimation 방지)
    - Target Q-Networks: 느린 EMA 업데이트
    - 자동 α 조절: 엔트로피 목표에 맞춰 온도 자동 최적화
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal
from typing import Tuple
from types import SimpleNamespace
import math
from envs.environment import EnvironmentSpec
from agents.base import Network


LOG_STD_MIN = -20
LOG_STD_MAX = 2


def weights_init(m):
    """Xavier uniform initialization."""
    if isinstance(m, nn.Linear):
        nn.init.xavier_uniform_(m.weight, gain=1)
        if m.bias is not None:
            nn.init.constant_(m.bias, 0)


class SACActorNet(nn.Module):
    """Squashed Gaussian 정책 네트워크."""

    def __init__(self, state_dim: int, action_dim: int, hidden_dim: int = 256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
        )
        self.mean_head = nn.Linear(hidden_dim, action_dim)
        self.log_std_head = nn.Linear(hidden_dim, action_dim)
        self.apply(weights_init)

    def forward(self, state: torch.Tensor):
        x = self.net(state)
        mean = self.mean_head(x)
        log_std = self.log_std_head(x)
        log_std = torch.clamp(log_std, LOG_STD_MIN, LOG_STD_MAX)
        return mean, log_std

    def sample(self, state: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Reparameterization trick + tanh squashing.

        Returns:
            action: tanh(u) ∈ (-1, 1)
            log_prob: log π(a|s) (squashing 보정 포함)
        """
        mean, log_std = self.forward(state)
        std = log_std.exp()
        dist = Normal(mean, std)

        # Reparameterization: u = mean + std * N(0,1)
        u = dist.rsample()
        action = torch.tanh(u)

        # Log probability with squashing correction (numerically stable)
        # log π(a|s) = log μ(u|s) - Σ log(1 - tanh²(u_i))
        log_prob = dist.log_prob(u) - (2 * (math.log(2) - u - F.softplus(-2 * u)))
        log_prob = log_prob.sum(dim=-1, keepdim=True)

        return action, log_prob

    def deterministic(self, state: torch.Tensor) -> torch.Tensor:
        """추론 모드: mean의 tanh."""
        mean, _ = self.forward(state)
        return torch.tanh(mean)


class SACQNet(nn.Module):
    """Q-value 네트워크 (state-action → scalar)."""

    def __init__(self, state_dim: int, action_dim: int, hidden_dim: int = 256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1),
        )
        self.apply(weights_init)

    def forward(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        return self.net(torch.cat([state, action], dim=-1))


class SACNetwork(Network):
    """
    SAC 통합 네트워크.

    Actor + Twin Q + Target Q + 자동 α.
    """

    def __init__(self, config: SimpleNamespace, environment_spec: EnvironmentSpec):
        super().__init__(config, environment_spec)

        hidden_dim = getattr(config, "hidden_dim", 256)

        # Actor
        self.actor = SACActorNet(self.state_size, self.action_size, hidden_dim)

        # Twin Q-Networks (overestimation 방지)
        self.q1 = SACQNet(self.state_size, self.action_size, hidden_dim)
        self.q2 = SACQNet(self.state_size, self.action_size, hidden_dim)

        # Target Q-Networks (느린 EMA 업데이트)
        self.q1_target = SACQNet(self.state_size, self.action_size, hidden_dim)
        self.q2_target = SACQNet(self.state_size, self.action_size, hidden_dim)
        self.q1_target.load_state_dict(self.q1.state_dict())
        self.q2_target.load_state_dict(self.q2.state_dict())

        # 자동 α (엔트로피 온도)
        self.target_entropy = getattr(
            config, "target_entropy", -float(self.action_size))
        self.log_alpha = nn.Parameter(torch.zeros(1))

    @property
    def alpha(self) -> torch.Tensor:
        return self.log_alpha.exp()

    def select_action(self, state, total_n_timesteps=0,
                      training_mode=True):
        """Actor로 행동 선택."""
        with torch.no_grad():
            if training_mode:
                action, _ = self.actor.sample(state)
            else:
                action = self.actor.deterministic(state)
        return action

    def cuda(self):
        self.to(torch.device(f"cuda:{self.config.device_num}"))

    def save(self, checkpoint_path: str):
        torch.save(self.state_dict(), f"{checkpoint_path}/network.th")

    def restore(self, checkpoint_path: str):
        state_dict = torch.load(
            f"{checkpoint_path}/network.th",
            map_location=torch.device(self.config.device))
        self.load_state_dict(state_dict)
