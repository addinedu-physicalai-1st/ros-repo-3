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
import ray
import datetime
from utils.logging import Logger
from runner.multienv_runner import MultiEnvRunner


class MultiEnvAsyncRunner(MultiEnvRunner):
    """
        여러 개의 환경을 비동기적으로 분산 처리를 하기 위해
        강화학습의 구성요소를 생성하고 추론과 학습을 위한 전체적인 실행을 관장.
    """

    def __init__(self,
                 config: dict,
                 console_logger: Logger = None,
                 logger: Logger = None,
                 verbose: bool = False):
        """
            부모 클래스 Runner의 초기화 메서드를 호출하여 러너를 초기화.
        Args:
            config: 설정
            console_logger: 콘솔 로거
            logger: 로거
            verbose: 설정을 콘솔에 출력할지 여부
        """

        super().__init__(config, console_logger, logger, verbose)

    def train(self):
        """
            최대 환경 실행 스텝만큼 다음 과정을 반복
                1) 환경 루프를 실행해서 데이터를 수집하고
                2) 정책을 평가하고 개선하는 과정

            단, 데이터 수집을 빠르게 하기 위해 환경 루프를 병렬 실행하며,
            환경 루프의 실행이 완료되는 즉시 데이터셋을 구성하고 에이전트를 학습
        """
        # 1. 타임 스텝 초기화
        self.last_model_save_timestep = 0   # 체크포인트 타임 스텝
        self.last_logging_step = 0          # 로깅 타임 스텝

        # 2. 학습 시작 시점 측정
        start_time = datetime.datetime.now().replace(microsecond=0)

        # 3. 전체 환경 루프 실행
        pending_result = {}
        for env_loop in self.environment_loops:
            future = self.run_environemnt_loop(env_loop)
            pending_result[future] = env_loop

        # 3. 학습 루프 실행
        while self.total_n_timesteps < self.config.max_environment_steps:

            # 4. 데이터 수집 결과 받기
            ready_refs, not_ready_refs = ray.wait(
                list(pending_result.keys()))
            result = ray.get(ready_refs[0])
            env_loop = pending_result.pop(ready_refs[0])

            # 5. 카운터 증가
            self.total_n_timesteps += result['n_timesteps_in_run']  # 타입스텝 수
            self.total_n_episodes += result['n_episodes_in_run']    # 에피소드 수

            # 6. 데이터 동기화
            self.update_dataset(result['rollouts'])

            # 7. 정책 평가 및 개선/네트워크 동기화
            if(self.update_agent()):
                self.update_actor(env_loop)
                # 9. 결과를 반환한 환경 루프 재실행
                future = self.run_environemnt_loop(env_loop)
                pending_result[future] = env_loop

            # 10. 환경 루프 통계 정보 로깅
            self.logging_stats(result)

            # 11. 체크포인트 저장
            self.save_checkpoint()

        # 12. 총 학습 시간 출력
        end_time = datetime.datetime.now().replace(microsecond=0)
        self.logger.console_logger.info(f"Start time: {start_time}")
        self.logger.console_logger.info(f"End time: {end_time}")
        self.logger.console_logger.info(f"Total: {end_time - start_time}")

    def run_environemnt_loop(self, env_loop):
        """
            지정된 환경 루프에 대해 지정된 실행 타입 스텝 수 또는
            에피소드 수만큼 에이전트와 환경의 상호작용을 원격 실행. (비동기적 실행)

        Args:
            env_loop: 실행할 환경 루프

        Returns:
            환경 루프의 원결 실행 결과 future 값
        """
        # 환경 루프 원격 실행
        future = env_loop.run.remote(
            max_n_timesteps=self.config.n_steps,
            max_n_episodes=self.config.n_episodes)
        return future

    def update_actor(self, env_loop):
        """
            에이전트의 네트워크 파라미터를
            지정된 환경 루프에 있는 액터의 복사본에 동기화 (동기적 실행)
        Args:
            env_loop: 액터가 있는 환경 루프
        """

        # 1. 에이전트 네트워크 파라미터 읽기
        state_dict = self.agent.network.get_variables()

        # 2. 액터의 복사본에 동기화
        future = env_loop.update_policy.remote(state_dict)

        # 3. 환경 루프 작업 완료 대기
        ray.wait([future])

    def reset_stats_environment_loops(self):
        """
            전체 환경 루프의 통계 정보를 초기화한다.
            단, 실행 완료를 기다리지 않고 바로 다음 실행을 진행한다. (비동기적 실행)
        Returns:
            원격으로 환경 루프 통계 정보 초기화한 후 future 값
        """

        # 환경 루프 통계 정보 초기화
        future = [environment_loop.reset_stats.remote()
                  for environment_loop in self.environment_loops]
        return future