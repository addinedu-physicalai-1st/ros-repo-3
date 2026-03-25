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
from types import SimpleNamespace
from typing import Dict, Any
from envs.environment import Environment
from utils.util import to_torch_type


class BufferSchema():
    """버퍼를 구성하는 데이터 필드의 모양과 데이터 타입을 스키마로 정의."""

    def __init__(self,
                 config: SimpleNamespace,
                 env: Environment,
                 schema: Dict[str, Any] = None):
        """
            설정, 환경 정보, 스키마를 전달 받아서 저장하고
            스키마를 전달 받지 못한 경우 환경 정보를 이용해서 기본 스키마를 생성한다.
        Args:
            config: 설정
            env: 환경
            schema: 버퍼 스키마 딕셔너리
        """

        # 1. 전달 받은 인자 저장
        self.config = config
        self.env = env

        # 2. 기본 스키마 생성
        self.schema = self.create_default_schema() \
            if schema is None else schema

    def create_default_schema(self):
        """
            환경 정보를 이용해서 트랜지션을 저장하기 위한 기본 버퍼 스키마를 생성한다.
        Returns:
            버퍼 스키마 딕셔너리
        """

        # 1. 트랜지션 데이터 저장 버퍼 스키마 정의
        env_spec = self.env.environment_spec()
        schema = {
            "state": {"shape": env_spec.state_spec.shape},                  # 상태
            "action": {"shape": env_spec.action_spec.shape,                 # 행동
                       "dtype": to_torch_type(env_spec.action_spec.dtype)},
            "next_state": {"shape": env_spec.state_spec.shape},             # 다음 상태
            "reward": {"shape": (1,)},                                      # 보상
            "done": {"shape": (1,), "dtype": to_torch_type(int)},           # 에피소드 종료 여부
        }

        # 2. 버퍼 스키마 반환
        return schema