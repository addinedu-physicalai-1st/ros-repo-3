"""
DreamerV3 Agent — 프레임워크 통합 에이전트 클래스

기존 Agent(Learner) 패턴을 따르되, DreamerV3 특유의 구조를 반영:
    - 기존 Buffer 시스템 대신 EpisodeReplayBuffer 사용
    - 기존 Actor 대신 DreamerV3Actor 사용
    - 기존 Learner 대신 DreamerV3Learner 사용
    - DreamerRunner에서 직접 호출
"""

import torch
from copy import deepcopy
from types import SimpleNamespace
from utils.logging import Logger
from envs.environment import Environment
from agents.dreamer.dreamer_networks import DreamerV3Network
from agents.dreamer.dreamer_learner import DreamerV3Learner
from agents.dreamer.dreamer_actor import DreamerV3Actor
from datasets.episode_replay_buffer import EpisodeReplayBuffer


class DreamerV3:
    """
    DreamerV3 에이전트.

    기존 Agent 패턴과 달리:
        - nn.Module 기반이 아닌 독립 클래스
        - EpisodeReplayBuffer 직접 관리
        - DreamerRunner에서 호출하는 인터페이스 제공
    """

    def __init__(self, config: SimpleNamespace, logger: Logger,
                 env: Environment):
        self.config = config
        self.logger = logger
        self.env = env

        env_spec = env.environment_spec()
        obs_dim = env_spec.state_spec.shape[0]
        action_dim = env_spec.action_spec.shape[0]

        # ─── 네트워크 생성 ───
        self.network = DreamerV3Network(
            obs_dim=obs_dim,
            action_dim=action_dim,
            hidden_dim=getattr(config, 'hidden_dim', 256),
            gru_units=getattr(config, 'gru_units', 1024),
            num_latents=getattr(config, 'num_latents', 32),
            num_classes=getattr(config, 'num_classes', 16),
            num_bins=getattr(config, 'num_bins', 255),
            unimix=getattr(config, 'unimix', 0.01),
            actor_layers=getattr(config, 'actor_layers', 3),
            critic_layers=getattr(config, 'critic_layers', 3),
            min_std=getattr(config, 'min_std', 0.1),
            max_std=getattr(config, 'max_std', 1.0),
            slow_target_tau=getattr(config, 'slow_target_tau', 0.02),
        )

        # ─── 리플레이 버퍼 ───
        self.replay_buffer = EpisodeReplayBuffer(
            max_episodes=getattr(config, 'max_episodes', 1000),
            max_steps=getattr(config, 'replay_buffer_size', 500000),
        )

        # ─── 학습자 (훈련 모드만) ───
        self.learner = None
        if config.training_mode:
            self.learner = DreamerV3Learner(
                config=config,
                logger=logger,
                network=self.network,
                replay_buffer=self.replay_buffer,
            )

        # ─── Actor (환경 상호작용용) ───
        # Actor는 네트워크의 복사본을 사용
        self.actor_network = deepcopy(self.network)
        self.actor = DreamerV3Actor(
            config=config,
            network=self.actor_network,
            replay_buffer=self.replay_buffer,
        )

    def cuda(self):
        """GPU로 이동."""
        device = torch.device(self.config.device)
        self.network.to(device)
        self.actor_network.to(device)
        # Actor의 RSSM 상태도 리셋
        self.actor.reset_state()

    def update(self, total_n_timesteps: int) -> dict:
        """
        학습 한 스텝 수행.

        Returns:
            metrics: 로깅용 딕셔너리 (비어있으면 학습 skip)
        """
        if self.learner is None:
            return {}
        return self.learner.update(total_n_timesteps)

    def sync_actor(self):
        """에이전트 네트워크의 가중치를 Actor에 동기화."""
        self.actor.update_network(self.network.state_dict())

    def save(self, checkpoint_path: str):
        """체크포인트 저장."""
        if self.learner is not None:
            self.learner.save(checkpoint_path)
        else:
            self.network.save(checkpoint_path)

    def restore(self, checkpoint_path: str):
        """체크포인트 복원."""
        if self.learner is not None:
            self.learner.restore(checkpoint_path)
        else:
            self.network.restore(checkpoint_path, self.config.device)
        # Actor에도 동기화
        self.sync_actor()

    def load(self, model_path: str) -> bool:
        """추론 모델 로딩."""
        try:
            state_dict = torch.load(
                model_path,
                map_location=lambda storage, loc: storage)
            self.network.load_state_dict(state_dict)
            self.sync_actor()
            return True
        except Exception:
            return False
