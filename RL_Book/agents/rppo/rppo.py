import torch
import numpy as np
from types import SimpleNamespace
from typing import Dict, List, Tuple
from utils.logging import Logger
from agents.actor import Actor
from agents.agent import Agent
from agents.rppo.rppo_network import RecurrentPPONetwork
from agents.rppo.rppo_learner import RecurrentPPOLearner
from datasets.rollout_buffer import RolloutBuffer
from datasets.buffer_schema import BufferSchema
from envs.environment import Environment
from utils.util import to_tensor, to_device, to_numpy


class RecurrentActor(Actor):
    """Actor for Recurrent PPO that manages hidden states."""

    def __init__(self, config, env, buffer_schema, network, actor_id=0):
        super().__init__(config, env, buffer_schema, network, actor_id)
        self.reset_hidden_state()

    def reset_hidden_state(self):
        hidden_dim = getattr(self.config, "gru_hidden_dim", 128)
        self.hidden_state = torch.zeros(1, 1, hidden_dim)
        if self.config.use_cuda:
            self.hidden_state = self.hidden_state.cuda(self.config.device_num)

    def select_action(self, state, total_n_timesteps):
        # 1. State to tensor (1, 1, state_dim)
        state_tensor = (
            to_device(to_tensor(state), self.config).unsqueeze(dim=0).unsqueeze(dim=0)
        )

        # 2. Select action with hidden state
        action, next_hidden_state = self.network.select_action(
            state=state_tensor,
            hidden_state=self.hidden_state,
            training_mode=self.config.training_mode,
        )

        # 3. Store current hidden state for buffer (if needed)
        # We should store the hidden state that was used for this action
        self.last_hidden_state = self.hidden_state.clone().cpu().numpy()

        # 4. Update hidden state
        self.hidden_state = next_hidden_state

        # 5. Convert to numpy
        action_numpy = to_numpy(action, self.config).squeeze()
        return action_numpy

    def observe(self, rollout: Dict):
        # Store hidden state in rollout
        rollout["hidden_state"] = self.last_hidden_state.squeeze(0)  # (1, hidden_dim)

        if rollout["done"]:
            self.reset_hidden_state()

        super().observe(rollout)


class RecurrentPPO(Agent):
    """Recurrent PPO Agent."""

    def __init__(self,
                 config: SimpleNamespace,
                 logger: Logger,
                 env: Environment):
        """
            Recurrent PPO 알고리즘을 실행하는
            학습자, 네트워크, 데이터셋으로 구성된 에이전트를 생성.

        Args:
            config: 설정
            logger: 로거
            env: 환경
        """

        # 1. 에이전트 초기화
        super(RecurrentPPO, self).__init__(
            config=config,
            logger=logger,
            env=env,
            network_class=RecurrentPPONetwork,
            learner_class=RecurrentPPOLearner,
            actor_class=RecurrentActor,
            policy_type="on_policy")

        # 2. 훈련 모드인 경우 GRU hidden state를 저장하기 위한 버퍼 스키마 확장
        if config.training_mode:
            hidden_dim = getattr(config, "gru_hidden_dim", 128)
            hidden_state_schema = {"hidden_state": {"shape": (1, hidden_dim)}}
            self.buffer.extend_schema(hidden_state_schema)
            self.buffer_schema.schema.update(hidden_state_schema)
