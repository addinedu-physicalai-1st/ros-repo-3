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
"""main.py: 강화학습 프레임워크 실행 프로그램."""
import torch
import numpy as np
import random
import utils.config as cu
import argparse
from runner.runner import Runner
from runner.multienv_runner import MultiEnvRunner
from runner.multienv_async_runner import MultiEnvAsyncRunner
from runner.dreamer_runner import DreamerRunner

if __name__ == "__main__":

    # 1. 명령어 인자 파서 생성
    desc = "RL Framework"
    parser = argparse.ArgumentParser(description=desc)

    # 2. 에이전트 이름 인자 추가
    parser.add_argument(
        "-a",
        "--agent",
        help="agent name {"
        "reinforce, "
        "reinforce_b, "
        "a2c, "
        "dqn, "
        "ddqn, "
        "ppo, "
        "rppo, "
        "dreamer, "
        "sac, "
        "}",
        type=str,
        default="reinforce",
    )

    # 3. 환경 이름 인자 추가
    parser.add_argument(
        "-e",
        "--env",
        help="run type {"
        "CartPole-v1, "
        "LunarLanderContinuous-v3, "
        "Acrobot-v1, "
        "AntBulletEnv-v0, "
        "PinkyPro-v0}",
        type=str,
        default="CartPole-v1",
    )

    # 4. 명령어 인자 파싱
    args = parser.parse_args()

    # 5. 난수 발생기 씨드 랜덤 생성
    random_seed = random.randrange(0, 16546)

    # 6. 난수 발생기를 초기화
    np.random.seed(random_seed)
    torch.manual_seed(random_seed)
    print("random_seed=", random_seed)

    # 7. 에이전트 이름과 환경 이름 받기
    agent_name = args.agent
    env_name = args.env

    # 8. 설정 파일 읽기
    config: dict = cu.config_copy(cu.get_config(agent_name, env_name))

    # 9. 설정 값 추가
    config["agent"] = agent_name  # 에이전트 이름 추가
    config["env_name"] = env_name  # 환경 이름 추가
    config["random_seed"] = random_seed  # 난수발생기 씨드 추가
    if config.get("env_args", None) is None:  # 환경 인자 기본값 처리
        config["env_args"] = {}

    # 10. 러너 클래스 선택
    config["distributed_processing_type"] = config.get(
        "distributed_processing_type", "sync"
    )
    if agent_name == "dreamer":
        # DreamerV3: 전용 러너 사용
        RunnerClass = DreamerRunner
    elif config["n_envs"] == 1:
        RunnerClass = Runner
    elif config["distributed_processing_type"] == "sync":
        RunnerClass = MultiEnvRunner
    else:
        RunnerClass = MultiEnvAsyncRunner

    # 11. 러너의 run() 메서드 호출
    RunnerClass(config).run()
