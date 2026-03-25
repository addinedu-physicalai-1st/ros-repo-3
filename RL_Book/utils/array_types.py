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
"""array_types.py: 데이터의 모양과 타입, 값의 범위를 기술하기 위한 배열 정의."""
import numpy as np


class Array:
    """넘파이 배열의 정보 관리 (모양과 데이타 터입)"""
    __slots__ = ('_shape', '_dtype', '_name')
    __hash__ = None

    def __init__(self, shape, dtype):
        """넘파이 배열 정보 초기화.

        Args:
          shape: 넘파이 배열의 모양
          dtype: 넘파이 배열의 numpy 데이터 타입.
        """
        self._shape = tuple(int(dim) for dim in shape)
        self._dtype = np.dtype(dtype)

    @property
    def shape(self):
        """넘파이 배열의 데이터 모양."""
        return self._shape

    @property
    def dtype(self):
        """넘파이 배열의 데이터 타입."""
        return self._dtype


class BoundedArray(Array):
    """상한과 하한을 갖는 넘파이 배열 정보 관리."""
    __slots__ = ('_minimum', '_maximum')
    __hash__ = None

    def __init__(self, shape, dtype, minimum, maximum):
        """상한과 하한을 갖는 넘파이 배열 정보 초기화.

        Args:
          shape: 넘파이 배열의 모양
          dtype: 넘파이 배열의 numpy 데이터 타입.
          minimum: 넘파이 배열에 저잘될 데이터 값의 상한
          maximum: 넘파이 배열에 저잘될 데이터 값의 하한
        """
        super(BoundedArray, self).__init__(shape, dtype)

        self._minimum = np.array(minimum, dtype=self.dtype)
        self._minimum.setflags(write=False)

        self._maximum = np.array(maximum, dtype=self.dtype)
        self._maximum.setflags(write=False)

    @property
    def minimum(self):
        """넘파이 배열에 저잘될 데이터 값의 상한."""
        return self._minimum

    @property
    def maximum(self):
        """넘파이 배열에 저잘될 데이터 값의 하한."""
        return self._maximum
