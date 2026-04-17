"""
DreamerV3 Actor — RSSM 상태를 관리하며 환경과 상호작용.

기존 Actor/RecurrentActor와 달리:
    - GRU hidden state뿐 아니라 stochastic latent z도 관리
    - 관측을 symlog 변환 후 RSSM encoder에 전달
    - 에피소드 종료 시 (h, z) 모두 리셋
    - 에피소드 리플레이 버퍼에 직접 데이터 기록
"""

import torch
import numpy as np
from typing import Dict
from agents.dreamer.dreamer_networks import DreamerV3Network, symlog
from datasets.episode_replay_buffer import EpisodeReplayBuffer


class DreamerV3Actor:
    """
    DreamerV3 전용 Actor.

    환경과 상호작용하면서:
        1. 관측 → RSSM encoder → posterior z_t 업데이트
        2. model_state(h_t, z_t) → Actor → action 선택
        3. 트랜지션을 에피소드 리플레이 버퍼에 저장
        4. 에피소드 종료 시 RSSM 상태 리셋
    """

    def __init__(self, config, network: DreamerV3Network,
                 replay_buffer: EpisodeReplayBuffer):
        self.config = config
        self.network = network
        self.replay_buffer = replay_buffer
        self.device = config.device

        # RSSM 상태 초기화
        self.reset_state()

        # 이전 행동 (첫 스텝에서는 zeros)
        self.prev_action = torch.zeros(
            1, network.action_dim,
            device=self.device)

    def reset_state(self):
        """RSSM 상태 (h, z) 초기화."""
        h, z = self.network.world_model.rssm.initial_state(
            batch_size=1, device=self.device)
        self.h = h
        self.z = z
        self.prev_action = torch.zeros(
            1, self.network.action_dim,
            device=self.device)

    def select_action(self, obs: np.ndarray, training: bool = True
                      ) -> np.ndarray:
        """
        관측으로부터 행동 선택.

        Args:
            obs: (obs_dim,) numpy 관측
            training: 훈련 모드 (True=샘플링, False=mean)

        Returns:
            action: (action_dim,) numpy 행동
        """
        obs_tensor = torch.tensor(
            obs, dtype=torch.float32, device=self.device).unsqueeze(0)

        action, h_new, z_new = self.network.select_action(
            obs=obs_tensor,
            h=self.h,
            z=self.z,
            action_prev=self.prev_action,
            training=training)

        # 상태 업데이트
        self.h = h_new
        self.z = z_new
        self.prev_action = action.detach()

        return action.squeeze(0).cpu().numpy()

    def observe(self, obs: np.ndarray, action: np.ndarray,
                reward: float, done: bool, terminated: bool = None):
        """
        트랜지션 관측 → 리플레이 버퍼에 저장.

        Args:
            obs: (obs_dim,) 현재 관측 (step 이전 상태)
            action: (action_dim,) 실행한 행동
            reward: 보상
            done: 에피소드 종료 여부 (terminated OR truncated)
            terminated: 자연적 종료만 (goal/collision/boundary)
                        None이면 done과 동일 (DreamerV3 외 호환)
        """
        # 버퍼에는 terminated를 저장 (ContinueHead 학습용)
        # 에피소드 경계(finalize)는 done 기준으로 트리거
        done_for_buffer = terminated if terminated is not None else done
        self.replay_buffer.add_step(
            obs, action, reward, done_for_buffer, end_episode=done)

        # RSSM 상태 리셋은 실제 에피소드 경계(done) 기준
        if done:
            self.reset_state()

    def update_network(self, state_dict: dict):
        """에이전트 네트워크의 가중치를 동기화."""
        self.network.load_state_dict(state_dict)
