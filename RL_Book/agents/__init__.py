# This file is part of RL_Book Project.
#
# Copyright (C) 2025 SeongJin Yoon
#
# RL_Book is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# RL_Book is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
from agents.reinforce.reinforce import REINFORCE
from agents.reinforce_b.reinforce_b import REINFORCEB
from agents.a2c.a2c import A2C
from agents.ppo.ppo import PPO
from agents.dqn.dqn import DQN
from agents.ddqn.ddqn import DDQN

# 에이전트 레지스트리
REGISTRY = {}
REGISTRY["reinforce"] = REINFORCE           # REINFORCE 알고리즘
REGISTRY["reinforce_b"] = REINFORCEB        # REINFORCE 베이스라인 적용 알고리즘
REGISTRY["a2c"] = A2C                       # A2C 알고리즘
REGISTRY["dqn"] = DQN                       # DQN 알고리즘
REGISTRY["ddqn"] = DDQN                     # 더블 DQN 알고리즘
REGISTRY["ppo"] = PPO                       # PPO 알고리즘
