"""
SAC Agent — Soft Actor-Critic 에이전트

프레임워크 통합:
    - Agent 베이스 클래스 상속
    - Off-policy: ReplayBuffer 사용
    - Runner와 호환 (범용 Runner 사용)
"""

from types import SimpleNamespace
from utils.logging import Logger
from agents.agent import Agent
from agents.actor import Actor
from agents.sac.sac_network import SACNetwork
from agents.sac.sac_learner import SACLearner
from envs.environment import Environment


class SAC(Agent):
    """Soft Actor-Critic 에이전트."""

    def __init__(self,
                 config: SimpleNamespace,
                 logger: Logger,
                 env: Environment):
        super(SAC, self).__init__(
            config=config,
            logger=logger,
            env=env,
            network_class=SACNetwork,
            learner_class=SACLearner,
            actor_class=Actor,
            policy_type="off_policy")
