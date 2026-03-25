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

import os
import pprint
import datetime
import torch
from types import SimpleNamespace
from utils.logging import get_console_logger
from utils.logging import Logger
from utils.config import save_config
from agents import REGISTRY as agent_REGISTRY
from envs import REGISTRY as env_REGISTRY
from runner.environment_loop import EnvironmentLoop

class Runner:
    """
        환경이 한 개일 때 강화학습의 구성요소를 생성하고
        추론과 학습을 위한 전체적인 실행을 관장.
    """
    def __init__(self,
                 config: dict,
                 console_logger: Logger = None,
                 logger: Logger = None,
                 verbose: bool = False):
        """
            강화학습 프레임워크 실행에 필요한 유틸리티 객체인
            로거, 실행 토큰, 설정을 생성하고 학습에 필요한 카운터 변수를 초기화.
        Args:
            config: 설정
            console_logger: 콘솔 로거
            logger: 로거
            verbose: 설정을 콘솔에 출력할지 여부
        """

        # 1. 콘솔 로거 생성
        if console_logger is None:
            self.console_logger = get_console_logger()

        # 2. GPU 설정 확인 및 설정 객체 변환
        config = self._sanity_check_config(config)
        # SimpleNameSpace 객체로 변환
        self.config = SimpleNamespace(**config)
        # GPU 디바이스 이름 생성
        self.config.device = (
           "cuda:{}".format(self.config.device_num)
           if self.config.use_cuda else "cpu"
        )

        # 3. 실행 토큰 생성
        unique_token = "{}_{}_{}".format(
            self.config.agent,
            self.config.env_name,
            datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        )
        self.config.unique_token = unique_token

        # 4. 로거 생성
        if logger is None:
            # 로거 생성
            logger = Logger(self.console_logger)

            # 텐서보드 로거 생성
            if self.config.use_tensorboard:
                tb_logs_dir = os.path.join(
                    os.getcwd(),
                    self.config.local_results_path,
                    "tb_logs",
                    unique_token,
                )
                logger.setup_tensorboard(tb_logs_dir)

        self.logger = logger

        # 5. 설정 출력
        if verbose:
            self.logger.console_logger.info("Experiment Parameters:")
            experiment_params = pprint.pformat(config, indent=4, width=1)
            self.logger.console_logger.info("\n\n" + experiment_params + "\n")

        # 6. 체크포인트 폴더에 설정 저장
        if self.config.training_mode and self.config.save_model:
            save_config(self.config)

        # 7. 학습 변수 초기화
        self.total_n_timesteps = 0  # 타입 스텝 카운터 초기화
        self.total_n_episodes = 0   # 에피소드 카운터 초기화

        # 8. CUDNN 결정적 실행 설정
        torch.backends.cudnn.deterministic = self.config.torch_deterministic

    def _sanity_check_config(self, config:dict):
        """
            로컬 실행 환경에 GPU가 있는지 확인해서 설정을 맞게 조정
            추론 모드에서 환경이 여러 개이면 한 개로 조정
        Args:
            config: 설정 딕셔너리

        Returns:
                조정된 설정 딕셔너리
        """

        # 1. GPU 설정 확인
        if config["use_cuda"] and not torch.cuda.is_available():
            config["use_cuda"] = False
            warning_msg = \
                "CUDA flag use_cuda was switched OFF automatically " \
                "because no CUDA devices are available!"
            self.console_logger.warning(warning_msg)

        # 2. 추론 모드에서 환경 개수 확인
        if not config["training_mode"] and config["n_envs"] != 1:
            config["n_envs"] = 1
            warning_msg = "In inference mode, " \
                          "convert the environment count to 1."
            self.console_logger.warning(warning_msg)

        return config

    def run(self):
        """
            환경과 환경 루프를 생성하고
            1) 추론 모드이면 test() 메서드를 실행
            2) 훈련 모드이면 train() 메서드를 실행
        Returns:
            실행 성공 여부
        """

        # 1. 환경 이름 출력
        self.logger.console_logger.info("environment name : "
                                        + self.config.env_name)

        # 2. 에이전트 생성
        self.make_agent()

        # 3. 훈련 모드 (Training Mode)
        if self.config.training_mode:
            # 체크포인트 복구
            if self.config.checkpoint_path != "" \
                    and self.restore() is False: return False
            self.make_environment_loops()           # 환경 루프 생성
            self.train()                            # 학습 (train() 호출)
        else:   # 4. 추론 모드 (Inference Mode)
            if self.load() is False: return False   # 추론 모델 로드
            self.make_environment_loops()           # 환경 루프 생성
            self.test()                             # 추론 (test() 호출)

        return True

    def train(self):
        """
            최대 환경 실행 스텝만큼
                1) 환경 루프를 실행해서 데이터를 수집하고
                2) 정책을 평가하고 개선
            하는 과정을 반복
        """

        # 1. 타임 스텝 초기화
        self.last_model_save_timestep = 0   # 체크포인트 타임 스텝
        self.last_logging_step = 0          # 로깅 타임 스텝

        # 2. 학습 시작 시점 측정
        start_time = datetime.datetime.now().replace(microsecond=0)

        # 3. 학습 루프 실행
        while self.total_n_timesteps < self.config.max_environment_steps:

            # 4. 데이터 수집
            result = self.run_environment_loops()

            # 5. 카운터 증가 (타입스텝 수, 에피소드 수 업데이트)
            self.total_n_timesteps = result['n_timesteps_in_envloop']
            self.total_n_episodes = result['n_episodes_in_envloop']

            # 6. 데이터 동기화
            self.update_dataset(result['rollouts'])

            # 7. 정책 평가 및 개선/네트워크 동기화
            if self.update_agent(): self.update_actors()

            # 8. 환경 루프 통계 정보 로깅
            self.logging_stats(result)

            # 9. 체크포인트 저장
            self.save_checkpoint()

        # 10. 총 학습 시간 출력
        end_time = datetime.datetime.now().replace(microsecond=0)
        self.logger.console_logger.info(f"Start time: {start_time}")
        self.logger.console_logger.info(f"End time: {end_time}")
        self.logger.console_logger.info(f"Total: {end_time - start_time}")

    def logging_stats(self, result):
        """
           환경 루프의 실행 통계 정보를 로깅
           로깅 주기에 따라 학습 성능과 환경 루프의 실행 통계 정보를 콘솔에 출력
        Args:
            result: 환경 루프 실행 결과 딕셔너리
        """

        # 1. 환경 루프 통계 정보 로깅
        for key, value in result['stats'].items():
            # 실행 에피소드 수는 평균 계산에서 제외
            if key == 'n_episodes': continue
            # 환경 루프 실행 통계 정보를 에피소드 단위로 평균을 계산해서 로깅
            mean_value = value / result['stats']['n_episodes']
            self.logger.log_stat(f"{key}_mean",
                                 mean_value,
                                 self.total_n_timesteps)

        # 2. 로깅 주기 체크
        if (self.total_n_timesteps - self.last_logging_step) \
                >= self.config.log_interval:
            # 3. 환경 루프 통계 정보 초기화
            self.reset_stats_environment_loops()

            # 4. 로거의 최근 통계 정보 출력
            self.logger.log_stat("episode",
                                 self.total_n_episodes,
                                 self.total_n_timesteps)
            self.logger.print_recent_stats()

            # 5. 마지막 로깅 시점 업데이트
            self.last_logging_step = self.total_n_timesteps

    def save_checkpoint(self):
        """ 학습 중인 에이전트의 네트워크와 옵티마이저의 체크포인트를 저장."""

        # 1. 체크포인트 주기 체크
        if self.config.save_model and \
                ((self.total_n_timesteps - self.last_model_save_timestep)
                 >= self.config.save_model_interval):
            # 2. 체크포인트 저장
            self.save(self.total_n_timesteps)

            # 3. 마지막 체크포인트 시점 업데이트
            self.last_model_save_timestep = self.total_n_timesteps

    def test(self):
        """ 지정된 에피소드 수만큼 환경 루프를 추론 모드로 실행."""

        # 1. 환경 루프 실행 및 결과 출력 (에피소드 수만큼 실행)
        result = self.environment_loop.run(
            max_n_episodes=self.config.inference_max_episodes)

        # 2. 콘솔에 실행 결과 출력
        self.logger.console_logger.info("Result: {} ".format(result))

    def make_agent(self):
        """" 설정에 지정된 에이전트 이름으로 에이전트를 생성."""

        # 1. 에이전트 생성
        self.agent = agent_REGISTRY[self.config.agent](
            config=self.config,
            logger=self.logger,
            env=self.make_environment(0),
        )
        # 2. 모델 GPU 로딩
        if self.config.use_cuda: self.agent.cuda()

    def update_dataset(self, rollout):
        """
            환경 루프에서 반환 받은 액터가 수집한 경로 데이터를
            에이전트에 전달해서 데이터셋을 구성
        Args:
            rollout: 환경 루프에서 반환 받은 액터의 롤아웃버퍼
        """
        # 데이터 동기화
        self.agent.add_rollouts(rollout)

    def update_agent(self):
        """
            정책을 평가하고 개선하기 위해 에이전트를 학습
            단, 리플레이 버퍼를 워밍업 하는 단계라면 학습을 하지 않고 반환
        Returns:
            정책 개선 및 평가 성공 여부
        """

        # 1. 워밍업 단계면 반환
        if self.total_n_timesteps < self.config.warmup_step: return False

        # 2. 정책 개선 및 평가
        return self.agent.update(self.total_n_timesteps,
                                 self.total_n_episodes)

    def make_environment(self, env_id):
        """
            에이전트 생성에 필요한 환경 정보를 얻기 위해 임시로 환경을 생성
        Args:
            env_id: 환경 ID

        Returns:
            환경 객체
        """

        # 환경 생성
        return env_REGISTRY[self.config.env_wrapper](
            self.config,
            env_id,
            **self.config.env_args)

    def make_environment_loops(self):
        """
            액터를 생성할 때 필요한 정보인
            에이전트의 네트워크, 버퍼 스키마, 액터 클래스와 환경 ID를
            인자로 전달하여 환경 루프를 생성
        """

        # 환경 루프 생성
        env_id = 0
        self.environment_loop = EnvironmentLoop(
            config=self.config,
            network=self.agent.network,
            buffer_schema=self.agent.buffer_schema,
            actor_class=self.agent.actor_class,
            env_id=env_id)

    def run_environment_loops(self):
        """
            환경 루프를 통해 지정된 실행 타입 스텝 수 또는 에피소드 수만큼
            에이전트와 환경의 상호작용을 실행하고 결과를 반환.

        Returns:
            환경 루프 실행 결과 딕셔너리
        """

        # 환경 루프 실행
        result = self.environment_loop.run(
            max_n_timesteps=self.config.n_steps,
            max_n_episodes=self.config.n_episodes)

        return result

    def update_actors(self):
        """
            에이전트의 네트워크 파라미터를 액터의 복사본에 동기화.
        """

        # 1. 에이전트 네트워크 파라미터 읽기
        state_dict = self.agent.network.get_variables()

        # 2. 액터의 복사본에 동기화
        self.environment_loop.update_policy(state_dict)

    def reset_stats_environment_loops(self):
        """
            환경 루프의 실행 통계 정보를 초기화.
        """
        self.environment_loop.reset_stats()

    def load(self):
        """
           지정된 경로에 있는 모델을 에이전트의 네크워크로 로딩
           (학습이 완료된 추론 모델을 로딩할 때 호출)

        Returns:
            모델 로딩 성공 여부
        """

        # 추론 모델 경로에서 모델 로딩
        return self.agent.load(self.config.inference_model_path)

    def save(self, time_step):
        """
            에이전트를 통해 모델과 옵티마이저의 체크포인트를 저장.
        Args:
            time_step: 현재 타입 스텝
        """

        # 1. 체크포인트 경로 생성
        checkpoint_path = os.path.join(
            os.getcwd(),
            self.config.local_results_path,
            "models",
            self.config.unique_token,
            str(time_step),
        )

        # 2. 체크포인트 디렉토리 생성
        os.makedirs(checkpoint_path, exist_ok=True)
        self.logger.console_logger.info(f"Saving models to {checkpoint_path}")

        # 3. 체크포인트 저장
        if self.agent is not None:
            self.agent.save(checkpoint_path)

    def restore(self):
        """
            특정 타임 스텝의 체크포인트를 로딩해서
            에이전트의 모델과 옵티마이저를 복구
        Returns:
            체크포인트 복구 성공 여부
        """
        timesteps = []

        # 1. 체크포인트 경로 확인
        if not os.path.isdir(self.config.checkpoint_path):
            self.logger.console_logger.info(
                "Checkpoint directory {} doesn't exist".format(
                    self.config.checkpoint_path
                )
            )
            return False

        # 2. 저장된 체크포인트 타입 스텝 수집
        for name in os.listdir(self.config.checkpoint_path):
            full_name = os.path.join(self.config.checkpoint_path, name)
            # 디렉토리 이름이 타임 스텝으로 되어 있는지 체크
            if os.path.isdir(full_name) and name.isdigit():
                timesteps.append(int(name))

        # 3. 복구할 타입 스텝이 0이면 마지막 체크포인트로 복구
        if self.config.load_step == 0:
            timestep_to_load = max(timesteps)
        else:
            # 4. 복구할 타입 스텝이 0이 아니면 가장 가까운 체크포인트로 복구
            timestep_to_load = min(
                timesteps, key=lambda x: abs(x - self.config.load_step)
            )

        # 5. 체크포인트 모델 경로 생성
        model_path = os.path.join(self.config.checkpoint_path,
                                  str(timestep_to_load))
        self.total_time_step = timestep_to_load

        # 6. 체크포인트 복구
        self.logger.console_logger.info(f"Loading model from {model_path}")
        self.agent.restore(model_path)

        return True