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
"""
    util.py: 유틸리티 기능
        1) 넘파이 배열과 텐서의 변환
        2) 값의 구간 변환을 위한 스케일과 편향 계산
        3) 모델의 상태 하드 업데이트, 소프트 업데이트.
"""
import numpy as np
import torch


def to_torch_type(dtype):
    """
        데이터 타입을 텐서의 데이터 타입으로 변환
    Args:
        dtype: 데이터 타입

    Returns:
        텐서의 데이터 타입
    """
    if np.issubdtype(dtype, np.integer) or np.issubdtype(dtype, int):
        return torch.int
    return torch.float32


def to_device(a_tensor, config):
    """
        텐서의 디바이스를 설정에 지정된 디바이스로 재설정
    Args:
        a_tensor: 텐서
        config: 설정

    Returns:
        디바이스가 재설정된 텐서
    """
    if a_tensor.device != config.device:
        a_tensor = a_tensor.to(config.device)
    return a_tensor


def to_numpy(a_tensor, config):
    """
        텐서를 넘파이 배열로 변환
    Args:
        a_tensor: 텐서
        config: 설정

    Returns:
        변환된 넘파이 배열
    """
    if config.use_cuda: return a_tensor.cpu().data.numpy()
    return a_tensor.data.numpy()


def to_tensor(data, dtype=torch.float32):
    """
        넘파이 배열을 텐서로 변환
    Args:
        data: 넘파이 배열
        dtype: 텐서 데이터 타입

    Returns:
        변환된 텐서
    """
    if isinstance(data, (list, tuple)):  data = np.array(data)
    if not isinstance(data, (np.ndarray, int, float)): return data
    return torch.tensor(data, dtype=dtype)


def scale_bias(high, low):
    """
        [-1,1] 구간을 [low, high] 구간으로 스케일링 할 때 필요한 스케일과 편향 계산
    Args:
        high: 상한
        low: 하한

    Returns:
        scale: 스케일
        bias: 편향
    """
    scale = (high - low) / 2.
    bias = (high + low) / 2.
    return scale, bias


def hard_update(source, target):
    """
        타깃 하드 업데이트 (소스의 가중치를 타깃의 가중치에 복사)
    Args:
        source: 소스 모델
        target: 타깃 모델
    """

    for source_param, target_param in zip(source.parameters(), target.parameters()):
        target_param.data.copy_(source_param.data)


def soft_update(source, target, tau):
    """
        타깃 소프트 업데이트 (소스의 가중치를 타깃의 가중치와 가중 평균)
    Args:
        source: 소스 모델
        target: 타깃 모델
        tau: 가중 평균을 하기 위한 가중치
    """

    for source_param, target_param in zip(source.parameters(), target.parameters()):
        target_param.data.copy_(target_param.data * (1.0 - tau) + source_param.data * tau)
