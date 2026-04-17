import torch
import torch.nn as nn
from typing import Tuple, Dict
from types import SimpleNamespace
from envs.environment import EnvironmentSpec
from agents.base import Network

def orthogonal_init(module, gain=1.0):
    if isinstance(module, (nn.Linear, nn.Conv2d)):
        torch.nn.init.orthogonal_(module.weight, gain=gain)
        if module.bias is not None:
            torch.nn.init.constant_(module.bias, 0)

class RecurrentPPONetwork(Network):
    """PPO + GRU (Recurrent PPO) Network Class."""

    def __init__(self,
                 config: SimpleNamespace,
                 environment_spec: EnvironmentSpec):
        super().__init__(config, environment_spec)
        
        self.hidden_dim = getattr(config, "gru_hidden_dim", 128)
        self.feature_dim = getattr(config, "feature_dim", 128)
        
        # 1. Feature Extractor
        self.feature_extractor = nn.Sequential(
            nn.Linear(self.state_size, self.feature_dim),
            nn.Tanh(),
            nn.Linear(self.feature_dim, self.feature_dim),
            nn.Tanh()
        )
        
        # 2. GRU Layer
        self.gru = nn.GRU(self.feature_dim, self.hidden_dim, batch_first=True)
        
        # 3. Policy Head (Gaussian for continuous)
        self.policy_mean = nn.Linear(self.hidden_dim, self.action_size)
        self.policy_log_std = nn.Parameter(torch.zeros(self.action_size))
        
        # 4. Critic Head
        self.critic = nn.Linear(self.hidden_dim, 1)
        
        # Initialization
        self.apply(lambda m: orthogonal_init(m, gain=nn.init.calculate_gain('tanh')))
        orthogonal_init(self.policy_mean, gain=0.01)
        orthogonal_init(self.critic, gain=1.0)
        
        # Initialize GRU weights
        for name, param in self.gru.named_parameters():
            if 'weight_ih' in name:
                torch.nn.init.orthogonal_(param.data)
            elif 'weight_hh' in name:
                torch.nn.init.orthogonal_(param.data)
            elif 'bias' in name:
                torch.nn.init.constant_(param.data, 0)

    def forward(self, 
                state: torch.Tensor, 
                hidden_state: torch.Tensor, 
                done: torch.Tensor = None) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Args:
            state: (batch_size, seq_len, state_dim)
            hidden_state: (1, batch_size, hidden_dim)
            done: (batch_size, seq_len) - used to reset hidden state if needed during training
        Returns:
            policy_mean, policy_log_std, value, next_hidden_state
        """
        batch_size, seq_len, _ = state.size()
        
        # Flatten for feature extraction
        flat_state = state.reshape(batch_size * seq_len, -1)
        features = self.feature_extractor(flat_state)
        features = features.reshape(batch_size, seq_len, -1)
        
        # GRU forward with episode boundary handling
        if done is not None and seq_len > 1:
            # 청크 기반 학습: 에피소드 경계(done=1)에서 hidden state를 0으로 리셋
            # done[:, step]은 해당 스텝의 transition이 에피소드 종료인지를 나타냄.
            # step t의 done=1이면, step t+1은 새 에피소드의 시작이므로
            # step t+1 진입 전에 hidden state를 리셋해야 함.
            # 따라서 step 0에서는 청크 시작 hidden을 그대로 사용하고,
            # step k (k>=1)에서는 done[k-1]=1이면 hidden을 리셋.
            outputs = []
            h = hidden_state
            for step in range(seq_len):
                if step > 0:
                    # 이전 스텝이 done이면 hidden state 리셋
                    # done shape: (B, L, 1) → done[:, step-1, :] → (B, 1)
                    reset_mask = done[:, step-1, :].unsqueeze(0)  # (1, B, 1)
                    h = h * (1 - reset_mask)
                out, h = self.gru(features[:, step:step+1, :], h)
                outputs.append(out)
            output = torch.cat(outputs, dim=1)
            next_hidden_state = h
        else:
            # 단일 스텝 추론 또는 done 없는 경우: 기존 방식
            output, next_hidden_state = self.gru(features, hidden_state)
        
        # Heads
        policy_mean = self.policy_mean(output)
        value = self.critic(output)
        
        return policy_mean, self.policy_log_std, value, next_hidden_state

    def get_distribution(self, policy_mean, policy_log_std):
        std = policy_log_std.exp()
        return torch.distributions.Normal(policy_mean, std)

    def select_action(self, state, hidden_state, total_n_timesteps=0, training_mode=True):
        """Used by Actor."""
        # state: (1, 1, state_dim)
        # hidden_state: (1, 1, hidden_dim)
        with torch.no_grad():
            policy_mean, log_std, _, next_hidden_state = self.forward(state, hidden_state)
            dist = self.get_distribution(policy_mean, log_std)
            
            if training_mode:
                action = dist.sample()
            else:
                action = policy_mean
                
        return action, next_hidden_state

    def cuda(self):
        self.to(torch.device(f"cuda:{self.config.device_num}"))

    def evaluate(self, state, hidden_state, action, done=None):
        """Used by Learner for training."""
        # state: (batch_size, seq_len, state_dim)
        # hidden_state: (1, batch_size, hidden_dim)
        # action: (batch_size, seq_len, action_dim)
        
        policy_mean, log_std, value, _ = self.forward(state, hidden_state, done)
        dist = self.get_distribution(policy_mean, log_std)
        
        log_prob = dist.log_prob(action).sum(dim=-1, keepdim=True)
        entropy = dist.entropy().sum(dim=-1, keepdim=True)
        
        return log_prob, entropy, value
