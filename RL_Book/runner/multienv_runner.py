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
from copy import deepcopy
from typing import List, Dict
from utils.logging import Logger
from runner.runner import Runner
from runner.environment_loop import EnvironmentLoop


class MultiEnvRunner(Runner):
    """
        여러 개의 환경을 동기적으로 분산 처리를 하기 위해
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

    def make_environment_loops(self):
        """
            환경 루프 클래스를 ray의 리모트 클래스로 재정의하고
            환경의 개수만큼 환경 루프를 생성.
        """

        # 1. 분산 처리를 위해 ray 초기화
        self.ray_init()

        # 2. 환경 루프를 리모트 클래스로 정의
        num_gpus = self.config.num_gpus / self.config.n_envs
        RemoteEnvironemtnLoop = ray.remote(EnvironmentLoop).options(
                                num_gpus=num_gpus)

        # 3. 환경 개수만큼 환경 루프 생성
        self.environment_loops = []
        for env_id in range(self.config.n_envs):
            environment_loop = RemoteEnvironemtnLoop.remote(
                config=self.config,
                network=self.agent.network,
                buffer_schema=self.agent.buffer_schema,
                actor_class=self.agent.actor_class,
                env_id=env_id)
            self.environment_loops.append(environment_loop)

    def ray_init(self):
        """설정 파일에 설정된 CPU와 GPU 개수를 이용해서 Ray를 초기화."""
        # 1. Ray 초기화
        ray.init(num_cpus=self.config.num_cpus,
                 num_gpus=self.config.num_gpus)

        # 2. Ray에 할당된 자원 출력
        ray_resource = ray.available_resources()
        self.logger.console_logger.info(f"Ray resources: {ray_resource}")

    def run_environment_loops(self):
        """
            전체 환경 루프에 대해 에이전트와 환경의 상호작용을 원격 실행하고
            전체 환경 루프의 실행이 종료될 때까지 대기했다가 결과를 받아서 합침.
            (동기적 실행)

        Returns:
            합쳐진 환경 루프 실행 결과
        """

        # 1. 환경 루프 원격 실행
        results_future = [env_loop.run.remote(
                          max_n_timesteps=self.config.n_steps,
                          max_n_episodes=self.config.n_episodes)
                          for env_loop in self.environment_loops]

        # 2. 환경 루프 결과 가져오기
        results = ray.get(results_future)

        # 3. 환경 루프 실행 결과 병합
        merged_results = self._merge_results(results)
        return merged_results

    def _merge_results(self, results: List[Dict]) -> List:
        """
            전체 환경 루프의 실행 결과를 합침
                1) 숫자 데이터는 더함
                2) 외의 데이터는 리스트로 변환
        Args:
            results: 환경 루프 실행 결과 리스트

        Returns:
            합쳐진 환경 루프 실행 결과
        """
        # 1. 환경 루프의 실행 결과 for 루프
        merged_results = None
        for result in results:
            # 2. 첫 번째 실행 결과
            if merged_results is None:  # first result
                # 실행 결과를 병합된 결과로 복사
                merged_results = deepcopy(result)
                for key, value in result.items():
                    # 각 항목 별로 값의 타입 확인
                    if not isinstance(value, (int, float)):
                        # 숫자가 아니면 리스트로 변경
                        merged_results[key] = [value]
                continue
            # 2. 두 번째 이후 실행 결과
            for key, value in result.items():
                # 각 항목 별로 값의 타입 확인
                if isinstance(value, (int, float)):
                    # 1) 숫자면 합산
                    merged_results[key] += value
                else:
                    # 2) 숫자가 아니면 리스트에 추가
                    merged_results[key].append(value)

        # 3. 하위 딕셔너리 병합 처리 (ex, "stats")
        for key, value in merged_results.items():
            if isinstance(value, list) and isinstance(value[0], Dict):
                # _merge_results 재귀 호출
                merged_value = self._merge_results(merged_results[key])
                # 병합 결과 저장
                merged_results[key] = merged_value

        return merged_results

    def update_actors(self):
        """
            에이전트의 네트워크 파라미터를 전체 액터의 복사본에 동기화하고
            전체 실행이 완료될 때까지 대기. (동기적 실행)
        """

        # 1. 에이전트 네트워크 파라미터 읽기
        state_dict = self.agent.network.get_variables()

        # 2. 액터의 복사본에 동기화
        future = [environment_loop.update_policy.remote(state_dict)
                  for environment_loop in self.environment_loops]

        # 3. 환경 루프 작업 완료 대기
        ray.wait(future, num_returns=len(self.environment_loops))

    def reset_stats_environment_loops(self):
        """
            전체 환경 루프의 통계 정보를 초기화하고
            전체 환경 루프의 실행이 완료될 때까지 대기. (동기적 실행)
        """

        # 1. 환경 루프 통계 정보 초기화
        future = [environment_loop.reset_stats.remote()
                  for environment_loop in self.environment_loops]

        # 2. 환경 루프 작업 완료 대기
        ray.wait(future, num_returns=len(self.environment_loops))