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
import torch
import numpy as np
from typing import Dict, Any, Union, Tuple
from types import SimpleNamespace
from datasets.buffer import Buffer
from datasets.buffer_schema import BufferSchema

class RolloutBuffer(Buffer):
    """
        온라인 정책을 위한 버퍼로 트랜지션 데이터를 저장할 수 있는 1차원 버퍼
        인덱스 기반의 접근과 데이터의 추가, 배치 샘플링 기능을 제공.
    """

    def __init__(self,
                 config: SimpleNamespace,
                 buffer_schema: BufferSchema,
                 buffer_shape: Tuple,
                 data: Dict = None):
        """
            부모 클래스인 Buffer의 초기화 함수를 호출하여
            버퍼를 초기화 하고, 현재 버퍼의 위치를 초기화
        Args:
            config: 설정
            buffer_schema: 버퍼 스키마
            buffer_shape: 버퍼 모양
            data: 버퍼 데이터
        """

        # 1. 부모 클래스 호출로 버퍼 초기화
        super().__init__(
            config=config,
            buffer_schema=buffer_schema,
            buffer_shape=buffer_shape,
            data=data)

        # 2. 버퍼의 현재 위치 초기화
        self.transition_index = 0 if data is None else self.buffer_size

    def __getitem__(self, item: Union[str, tuple, slice]):
        """
            1) 데이터 필드 이름 또는 2) 인덱스 기반으로 버퍼의 값을 읽음
        Args:
            item: 1) 데이터 필드 이름, 2) 인덱스, 3) 인덱스 리스트, 4) 인덱스 슬라이스

        Returns:
            1) 데이터 필드 이름인 경우: 해당 필드의 데이터 텐서
            2) 인덱스, 인덱스 리스트, 인덱스 슬라이스인 경우:
                    해당 인덱스의 데이터로 재구성된 새로운 버퍼 객체
        """

        # 1. 데이터 필드 이름인 경우 데이터 필드 전체 반환
        if isinstance(item, str):
            if item in self.data: return self.data[item]
            return None

        # 2. 인덱스 슬라이스인 경우 슬라이스 객체로 변환
        slices = self._parse_slices(item)

        # 3. 버퍼 슬라이싱
        new_data = {}
        for k, v in self.data.items():
            new_data[k] = v[slices]

        # 4. 새로운 버퍼 모양 계산
        buffer_shape = [self._get_num_items(slices, self.buffer_size)]

        # 5. 버퍼 객체 생성 및 반환
        return type(self)(config=self.config,
                               buffer_schema=self.buffer_schema,
                               buffer_shape=buffer_shape,
                               data=new_data)

    def __setitem__(self,
                    item: Union[str, tuple, slice],
                    value: Union[torch.tensor, Any]):
        """
            1) 데이터 필드 이름 또는 2) 인덱스 기반으로 버퍼에 값을 씀
        Args:
            item: 1) 데이터 필드 이름, 2) 인덱스, 3) 인덱스 리스트, 4) 인덱스 슬라이스
            value: 버퍼에 저장할 데이터
        """

        # 1. item이 데이터 필드 이름인 경우
        if isinstance(item, str):
            # 데이터 필드가 버퍼에 있는지 확인
            if item in self.data:
                # 데이터 필드의 모양이 같은지 확인
                if not self._same_shape(self.data[item].shape, value.shape):
                    raise IndexError(
                        f"Data shape {value.shape} is not proper"
                        f" to buffer shape {self.data[item].shape}")
                # 데이터 필드 전체 값을 변경
                self.data[item] = value
                return
            else:   # 오류
                raise ValueError

        # 2. item이 인덱스 슬라이스인 경우 버퍼 슬라이스 업데이트
        self.update(value.data, slice=item)

    def can_sample(self, batch_size):
        """
            배치 샘플링이 가능한지 여부를 반환
            1) 배치 크기보다 버퍼에 저장된 데이터가 많으면 True를 반환
            2) 그렇지 않으면 False를 반환
        Args:
            batch_size: 배치 크기

        Returns:
            배치 샘플링이 가능한지 여부
        """
        return len(self) >= batch_size

    def sample(self, batch_size, allow_remainder: bool = True):
        """
            배치 랜덤 샘플링
        Args:
            batch_size: 배치 크기
            allow_remainder: 배치 크기보다 버퍼에 저장된 데이터가 적더라도 데이터를 반환할지 여부

        Returns:
            샘플링된 배치를 저장하고 이는 버퍼 객체
        """

        # 1. 버퍼에 데이터가 충분한지 확인
        if not allow_remainder: assert self.can_sample(batch_size)

        # 2. 데이터가 적으면 배치 크기를 조정
        if len(self) < batch_size: batch_size = len(self)

        # 3. 배치 크기만큼 데이터를 샘플링
        time_ids = np.random.choice(len(self), batch_size, replace=False)

        # 4. 배치 반환
        return self[time_ids]

    def __add__(self, other: Union[Buffer, Dict]):
        """
            + 연산자 구현 메서드로 버퍼에
            1) 트랜지션 데이터를 추가하거나 2) 다른 버퍼 내용을 추가.

        Args:
            other: 1) 트랜지션 데이터, 2) 다른 버퍼 중 하나

        Returns:
            업데이트된 버퍼 객체
        """

        # 1. 트랜지션 데이터 추가
        if isinstance(other, Dict):
            return self.append_one_transition(other)

        # 2. 다른 버퍼 내용 추가
        return self.append_from_other_buffer(other)

    def append_from_other_buffer(self, other):
        """
            다른 버퍼의 내용을 버퍼에 추가
        Args:
            other: 다른 버퍼

        Returns:
            버퍼를 추가하고 난 자신의 레퍼런스
        """

        # 1. 버퍼가 꽉 차 있으면 반환
        if self.is_full(): return self

        # 2. Other 버퍼의 데이터를 가져옴
        if not other.is_full(): other = other[:len(other)]

        # 3. 버퍼에 other 데이터 추가
        self.update(other.data,
                    slices=slice(self.transition_index,
                                 self.transition_index + other.buffer_size))

        # 4. 버퍼의 현재 위치 이동
        self.increment_transition_index(offset=other.buffer_size)
        return self

    def append_one_transition(self, data: Dict):
        """
            트랜지션 데이터를 버퍼에 추가
        Args:
            data: 트랜지션 데이터

        Returns:
            버퍼를 추가하고 난 자신의 레퍼런스
        """

        # 1. 버퍼 끝에 트랜지션 데이터를 추가
        self.update(data, slices=len(self))

        # 2. 버퍼 인덱스를 1 증가 시킴
        self.increment_transition_index(offset=1)

        return self

    def increment_transition_index(self, offset):
        """
            버퍼의 현재 위치를 오프셋만큼 뒤로 옮김
        Args:
            offset: 인덱스 오프셋
        """

        # 버퍼의 현재 위치 오프셋만큼 이동
        self.transition_index += offset
        assert self.transition_index <= self.buffer_size

    def clear(self):
        """버퍼를 초기화 하고 버퍼의 현재 위치를 0으로 설정한다."""

        # 1. 버퍼에 저장된 데이터 삭제
        super().clear()

        # 2. 현재 버퍼 위치 초기화
        self.transition_index = 0

    def __len__(self):
        """
            저장된 데이터의 개수 반환
        Returns:
            저장된 데이터의 개수
        """

        return self.transition_index    # 현재 버퍼 위치 반환