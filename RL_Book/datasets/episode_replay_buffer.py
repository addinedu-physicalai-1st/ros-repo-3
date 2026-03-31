"""
Episode Replay Buffer — DreamerV3용

DreamerV3는 에피소드 단위로 데이터를 저장하고,
고정 길이 시퀀스를 샘플링하여 world model을 학습한다.

기존 RolloutBuffer/ReplayBuffer와 달리:
    - 트랜지션이 아닌 에피소드 단위 저장
    - 고정 길이(seq_len) 시퀀스 랜덤 샘플링
    - 순환 버퍼: 에피소드 수 초과 시 가장 오래된 에피소드 삭제
"""

import numpy as np
import torch
from collections import deque
from typing import Dict, Optional


class EpisodeReplayBuffer:
    """
    에피소드 기반 리플레이 버퍼.

    저장 구조:
        episodes: deque of episode dicts
        각 episode: {
            "obs": (T, obs_dim),
            "action": (T, action_dim),
            "reward": (T,),
            "done": (T,),      # 0 or 1 (terminal에서만 1)
        }

    샘플링:
        1. 에피소드를 길이에 비례하여 랜덤 선택
        2. 에피소드 내에서 랜덤 시작점 선택
        3. seq_len 길이의 시퀀스 반환
    """

    def __init__(self, max_episodes: int = 1000, max_steps: int = 500000):
        """
        Args:
            max_episodes: 최대 저장 에피소드 수
            max_steps: 최대 총 트랜지션 수
        """
        self.max_episodes = max_episodes
        self.max_steps = max_steps
        self.episodes = deque()
        self.total_steps = 0

        # 현재 진행 중인 에피소드 수집용
        self._current_episode = {
            "obs": [],
            "action": [],
            "reward": [],
            "done": [],
        }

    def add_step(self, obs: np.ndarray, action: np.ndarray,
                 reward: float, done: bool, end_episode: bool = None):
        """
        한 스텝의 데이터를 현재 에피소드에 추가.

        Args:
            done: 버퍼에 저장되는 값 (ContinueHead 학습용).
                  DreamerV3: terminated만 True (timeout은 False).
            end_episode: 에피소드 종료 트리거. True이면 현재 에피소드를
                         완성하여 버퍼에 저장. None이면 done과 동일.
        """
        if end_episode is None:
            end_episode = done

        self._current_episode["obs"].append(obs.copy())
        self._current_episode["action"].append(action.copy())
        self._current_episode["reward"].append(float(reward))
        self._current_episode["done"].append(float(done))

        if end_episode:
            self._finalize_episode()

    def _finalize_episode(self):
        """현재 에피소드를 numpy 배열로 변환하여 버퍼에 저장."""
        ep = {
            "obs": np.stack(self._current_episode["obs"]),
            "action": np.stack(self._current_episode["action"]),
            "reward": np.array(self._current_episode["reward"], dtype=np.float32),
            "done": np.array(self._current_episode["done"], dtype=np.float32),
        }
        ep_len = len(ep["reward"])

        self.episodes.append(ep)
        self.total_steps += ep_len

        # 용량 초과 시 오래된 에피소드 삭제
        while len(self.episodes) > self.max_episodes or \
                self.total_steps > self.max_steps:
            if len(self.episodes) == 0:
                break
            old = self.episodes.popleft()
            self.total_steps -= len(old["reward"])

        # 현재 에피소드 버퍼 초기화
        self._current_episode = {
            "obs": [],
            "action": [],
            "reward": [],
            "done": [],
        }

    def sample(self, batch_size: int, seq_len: int,
               device: torch.device = torch.device("cpu")
               ) -> Dict[str, torch.Tensor]:
        """
        고정 길이 시퀀스를 배치로 샘플링.

        Args:
            batch_size: 배치 크기
            seq_len: 시퀀스 길이
            device: 텐서 디바이스

        Returns:
            {
                "obs":    (B, T, obs_dim),
                "action": (B, T, action_dim),
                "reward": (B, T),
                "done":   (B, T),
            }
        """
        # 에피소드 길이 가중치로 샘플링 (길이가 긴 에피소드가 더 많이 선택)
        ep_lengths = np.array([len(ep["reward"]) for ep in self.episodes])
        # seq_len보다 긴 에피소드만 선택 가능
        valid_mask = ep_lengths >= seq_len
        if not valid_mask.any():
            # seq_len보다 긴 에피소드가 없으면 가장 긴 에피소드에서 패딩
            seq_len = min(seq_len, ep_lengths.max())
            valid_mask = ep_lengths >= seq_len

        valid_indices = np.where(valid_mask)[0]
        valid_lengths = ep_lengths[valid_mask]
        # 길이에 비례한 확률로 에피소드 선택
        weights = valid_lengths.astype(np.float64)
        weights /= weights.sum()

        obs_batch, action_batch, reward_batch, done_batch = [], [], [], []

        for _ in range(batch_size):
            # 1. 에피소드 선택
            ep_idx = np.random.choice(valid_indices, p=weights)
            ep = self.episodes[ep_idx]
            ep_len = len(ep["reward"])

            # 2. 시작점 선택
            start = np.random.randint(0, ep_len - seq_len + 1)

            # 3. 시퀀스 추출
            obs_batch.append(ep["obs"][start:start + seq_len])
            action_batch.append(ep["action"][start:start + seq_len])
            reward_batch.append(ep["reward"][start:start + seq_len])
            done_batch.append(ep["done"][start:start + seq_len])

        return {
            "obs": torch.tensor(np.stack(obs_batch), dtype=torch.float32,
                                device=device),
            "action": torch.tensor(np.stack(action_batch), dtype=torch.float32,
                                   device=device),
            "reward": torch.tensor(np.stack(reward_batch), dtype=torch.float32,
                                   device=device),
            "done": torch.tensor(np.stack(done_batch), dtype=torch.float32,
                                 device=device),
        }

    def __len__(self) -> int:
        """저장된 에피소드 수."""
        return len(self.episodes)

    @property
    def num_steps(self) -> int:
        """총 저장된 트랜지션 수."""
        return self.total_steps

    def is_ready(self, batch_size: int, seq_len: int) -> bool:
        """학습에 충분한 데이터가 있는지 확인."""
        if len(self.episodes) < 1:
            return False
        max_len = max(len(ep["reward"]) for ep in self.episodes)
        return max_len >= seq_len and self.total_steps >= batch_size * seq_len
