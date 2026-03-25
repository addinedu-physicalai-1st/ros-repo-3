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
from typing import Dict, Any, Union, Tuple
from types import SimpleNamespace
from datasets.rollout_buffer import RolloutBuffer
from datasets.buffer_schema import BufferSchema


class ReplayBuffer(RolloutBuffer):
    """
        오프라인 정책을 위한 버퍼로, 새로운 데이터는 계속해서 추가하고
        버퍼가 꽉 차면 오래된 데이터부터 지우는 순환 버퍼로 운영.
    """
    def __init__(self,
                 config: SimpleNamespace,
                 buffer_schema: BufferSchema,
                 buffer_shape: Tuple,
                 data: Dict = None):
        """
            부모 클래스인 RolloutBuffer의 초기화 함수를 호출하여
            버퍼를 초기화하고 버퍼에 저장된 트랜지션 수를 설정한다.
        Args:
            config: 설정
            buffer_schema: 버퍼 스키마
            buffer_shape: 버퍼 모양
            data: 버퍼 데이터
        """

        # 1. 부모 클래스 초기화 호출
        super().__init__(config, buffer_schema, buffer_shape, data)

        # 2. 버퍼에 저장된 트랜지션 수 초기화
        self.n_transitions = self.transition_index

    def append_from_other_buffer(self, other):
        """
            다른 버퍼의 내용을 현재 위치부터 이어서 저장
            (순환 버퍼이기 때문에 버퍼가 꽉 차면 처음으로 연결해서 저장함)
        Args:
            other: 추가할 버퍼

        Returns:
            버퍼를 추가하고 난 자신의 레퍼런스
        """

        # 1. 남은 버퍼의 크기 계산
        buffer_left = self.buffer_size - self.transition_index

        # 2. 남은 공간에 저장할 수 있는 경우
        if other.buffer_size <= buffer_left:
            # 남은 공간에 저장
            self.update(other.data,
                        slices=slice(self.transition_index,
                                     self.transition_index + other.buffer_size))
            # 버퍼의 현재 위치 이동
            self.increment_transition_index(offset=other.buffer_size)
        else:
            # 3. 남은 버퍼를 초과하는 크기라면 Other를 분할해서 저장
            self += other[0:buffer_left]
            self += other[buffer_left:]
        return self

    def increment_transition_index(self, offset):
        """
            버퍼의 현재 위치를 오프셋만큼 뒤로 이동
            (순환 버퍼이기 때문에 마지막에서 처음으로 연결해서 이동)
        Args:
            offset: 인덱스 오프셋
        """

        # 1. 현재 위치를 오프셋만큼 이동
        self.transition_index += offset

        # 2. 저장 데이터 개수 업데이트
        self.n_transitions = min(self.buffer_size,
                                 max(self.n_transitions, self.transition_index))

        # 3. 현재 위치를 모듈러 계산
        self.transition_index %= self.buffer_size
        assert self.transition_index < self.buffer_size

    def clear(self):
        """
            버퍼를 초기화 하고 버퍼의 현재 위치와
            버퍼에 저장된 트랜지션 수를 0으로 설정.
        """

        # 1. 버퍼에 저장된 데이터 삭제
        super().clear()

        # 2. 저장된 데이터 개수 초기화
        self.transition_index = 0
        self.n_transitions = 0

    def __len__(self):
        """
            저장된 데이터의 개수 반환
        Returns:
            저장된 데이터의 개수
        """
        return self.n_transitions