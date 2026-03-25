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
import abc
import torch
import numpy as np
import copy
from types import SimpleNamespace
from typing import Dict, Any, Union, Tuple
from utils.util import to_tensor, to_device
from datasets.buffer_schema import BufferSchema

class Buffer():
    """버퍼 스키마에 따라 메모리 공간을 할당하고 관리."""

    def __init__(self,
                 config: SimpleNamespace,
                 buffer_schema: BufferSchema,
                 buffer_shape: Tuple[int],
                 data: Dict = None):
        """
            인자로 전달받은 버퍼 스키마, 버퍼 모양, 버퍼 데이터를 속성으로 저장하고
            데이터가 None이면 버퍼 스키마를 이용하여 버퍼를 할당한다.
        Args:
            config: 설정
            buffer_schema: 버퍼 스키마
            buffer_shape: 버퍼 모양
            data: 버퍼 데이터
        """

        # 1. 전달 받은 인수 저장
        self.config = config
        self.buffer_schema = copy.deepcopy(buffer_schema)
        self.buffer_shape = buffer_shape
        self.buffer_size = self.buffer_shape[0]
        self.data = data

        # 2. 버퍼 할당
        if self.data is None:
            self.data = self._create_buffer_from_schema(
                schema=buffer_schema.schema,
                buffer_shape=buffer_shape)

    @abc.abstractmethod
    def __len__(self):
        """
            버퍼에 저장된 데이터의 개수를 반환
        Returns:
            버퍼에 저장된 데이터의 개수
        """

    def is_full(self):
        """
            버퍼가 찼는지 여부를 반환
        Returns:
            버퍼가 찼는지 여부
        """

        # 버퍼 크기와 저장 데이터 개수 확인
        return len(self) == self.buffer_size

    def clear(self):
        """버퍼를 비우기 위해 모든 데이터를 0으로 초기화."""

        # 버퍼 데이터 필드 별 초기화
        for key in self.data.keys():
            self.data[key][:, :] = 0

    def _create_buffer_from_schema(
            self,
            schema: Dict[str, Any],
            buffer_shape: Tuple[int],) -> Dict[str, torch.Tensor]:
        """
            버퍼 모양과 버퍼 스키마를 이용해서 버퍼를 할당
        Args:
            schema: 버퍼 스키마
            buffer_shape: 버퍼 모양

        Returns:
            할당된 버퍼
        """

        # 1. 데이터 딕셔너리 생성
        data = {}

        # 2. 버퍼 스키마의 필드 별 for 루프
        for key, info in schema.items():
            assert "shape" in info,\
                "schema must define shape for {}".format(key)

            # 3. 데이터 모양과 타입 읽기 및 교정
            data_shape, dtype = info["shape"], info.get("dtype", torch.float32)
            # 스칼라인 경우 데이터 모양으로 (1,)로 변경
            if len(data_shape) == 0: data_shape = (1,)
            # 데이터 모양이 정수면 1차원으로 변경
            if isinstance(data_shape, int): data_shape = (data_shape,)

            # 4. 데이터 필드 별로 버퍼 할당
            data[key] = to_device(torch.zeros((*buffer_shape, *data_shape),
                                              dtype=dtype), self.config)

        return data # 5. 버퍼 반환

    def extend_schema(self, schema: Dict[str, Any]):
        """
            전달받은 추가 데이터 필드의 스키마에 맞게 버퍼와 버퍼 스키마를 확장
        Args:
            schema: 버퍼 스키마
        """
        # 1. 새로운 스키마에 따라 버퍼 할당
        self.data.update(self._create_buffer_from_schema(schema,
                                                         self.buffer_shape))
        # 2. 버퍼 스키마에 새로운 스키마 추가
        self.buffer_schema.schema.update(schema)

    def update(self, data: Dict, slices):
        """
            전달 받은 데이터를 버퍼 슬라이스에 저장
        Args:
            data: 데이터 딕셔너리
            slices: 데이터 인덱스 슬라이스

        Returns:

        """

        # 1. 슬라이스/인덱스 리스트로 변환
        slices = self._parse_slices((slices))

        # 2. 데이터 필드 별 업데이트 for 루프
        for key, value in data.items():

            # 3. 키가 버퍼에 있는지 확인
            if key not in self.data:
                raise KeyError("{} not found in data".format(key))

            # 4. 텐서 타입으로 변환
            dtype = self.buffer_schema.schema[key].get("dtype", torch.float32)
            value = to_device(to_tensor(value, dtype=dtype), self.config)
            if len(value.shape) == 0: value = value.view((1,))

            # 5. 버퍼 슬라이스에 저장
            # 버퍼의 모양과의 호환성 체크
            self._check_safe_view(value, self.data[key][slices])
            # 버퍼의 모양으로 변경 후 저장
            self.data[key][slices] = value.view_as(self.data[key][slices])

    def _check_safe_view(self, src: Any, dst: Any):
        """
            두 개의 다차원 배열이 서로 변환될 수 있는 모양인지 확인.
        Args:
            src: 소스 다차원 배열
            dst: 대상 다차원 배열
        """

        # 1. 소스 인덱스 계산
        idx = len(src.shape) - 1
        for dst_size in dst.shape[::-1]:
            # 2. 대상과 소스의 차원 별 크기 계산
            src_size = src.shape[idx] if idx >= 0 else 0

            if src_size != dst_size:
                # 3. 소스와 대상의 크기가 다를 때 대상의 크기가 1이면 통과
                # 대상의 크기가 1이 아니면 오류
                if dst_size != 1:
                    raise ValueError(f"Unsafe reshape of"
                                     f"{src.shape} to {dst.shape}")
            else:
                # 4. 소스와 대상의 크기가 같으면 소스 인덱스를 앞으로 이동
                idx -= 1

    def _parse_slices(self,
                      slices: Union[int, list, slice, tuple]) -> Tuple[slice, slice]:
        """
            인덱스 정보의 유효성을 검사하고 인덱스 리스트 또는 슬라이스 형태로 통일한다.
                1) 인덱스
                2) 인덱스 리스트
                3) 인덱스 슬라이스
        Args:
            slices: 1)인덱스, 2) 인덱스 리스트, 3) 인덱스 슬라이스 중 하나

        Returns:
            인덱스 리스트 또는 슬라이스
        """

        # 1. 정수 인덱스는 슬라이스로 변환
        if isinstance(slices, int): return slice(slices, slices + 1)

        # 2. 정수 인덱스가 아니면 유효성 검증 후 반환
        index_list_type = \
            (tuple, list, np.ndarray, torch.LongTensor, torch.cuda.LongTensor)
        assert isinstance(slices, (slice, *index_list_type))
        return slices

    def _get_num_items(self,
                       item: Union[list, np.ndarray, slice],
                       max_size: int = 0):
        """
            리스트, 넘파이 배열 또는 슬라이스 객체가 표현하는 요소의 개수를 센다.
        Args:
            item: 1) 리스트, 2) 넘파이 배열, 3) 슬라이스 객체 중 하나
            max_size: (슬라이스 객체인 경우) 최대 크기

        Returns:
            데이터 개수
        """

        # 1. 리스트나 넘파이 배열이면 길이 반환
        if isinstance(item, (list, np.ndarray)):
            return len(item)

        # 2. 슬라이스 객체이면 범위 내 항목 수 계산
        if isinstance(item, slice):
            _range = item.indices(max_size)
            # start, stop, step으로 계산
            return 1 + (_range[1] - _range[0] - 1)//_range[2]

    def _same_shape(self, this_shape, that_shape):
        """
            두 다차원 배열이 같은 모양인지 확인
        Args:
            this_shape: 첫번째 다차원 배열의 모양
            that_shape: 두번째 다차원 배열의 모양

        Returns:
            두 다차원 배열이 같은지 여부
        """

        # 모든 차원의 크기가 같은지 확인
        if all((i == j for i, j in zip(this_shape, that_shape))):
            return True     # 모두 같으면 True 반환
        return False        # 모두 같지 않으면 False 반환