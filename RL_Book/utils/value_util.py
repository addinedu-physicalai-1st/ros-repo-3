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
"""value_util.py: 리턴 및 이득 계산 1) 몬테카를로 리턴 계산 2) n-스텝 리턴 계산 3) GAE (Generalized
Advantage Estimate) 계산."""
from types import SimpleNamespace
import torch
from models.model import ValueFunctionMLP


def monte_carlo_returns(
        config: SimpleNamespace,
        state: torch.FloatTensor,
        next_state: torch.FloatTensor,
        reward: torch.FloatTensor,
        done: torch.int,
        critic: ValueFunctionMLP = None) -> torch.FloatTensor:
    """
        몬테카를로 리턴 계산
    Args:
        config: 설정
        state: 상태
        next_state: 다음 상태
        reward: 보상
        done: 에피소드 완료 여부
        critic: 가치 함수

    Returns:
        returns: 리턴
        advantage: 이득
    """

    # 1. 전체 타입 스텝 수 계산
    total_len = reward.shape[-2]

    # 2. 보상을 리턴으로 복사
    returns = reward.clone()

    # 3. 리턴과 종료 여부의 차원 변경 (시간 차원을 마지막으로 이동)
    returns = returns.view(-1, total_len)  # [total_len,1] -> [1,total_len]
    done = done.view(-1, total_len)  # [total_len,1] -> [1,total_len]

    # 4. 리턴 계산
    for t in reversed(range(total_len - 1)):
        returns[:, t] += (1 - done[:, t]) * config.gamma * returns[:, t + 1]

    # 5. 리턴 표준화
    if config.return_standardization:
        returns = (returns - returns.mean(dim=1, keepdim=True)) / \
                  (returns.std(dim=1, keepdim=True) + config.epsilon)

    # 6. 리턴의 모양 복구
    returns = returns.view_as(reward)  # [1,total_len] -> [total_len,1]

    # 7. 이득 계산
    advantage = returns
    if critic is not None:
        with torch.no_grad():
            value = critic(state)
        advantage = returns - value

    # 8. 리턴과 이득 반환
    return returns, advantage


def padding(data, axis, padding_size):
    """
        텐서 타입의 데이터를 특정 차원으로 원하는 크기만큼 0으로 패딩
    Args:
        data: 패딩할 텐서 데이터
        axis: 패딩할 텐서 데이터의 차원
        padding_size: 패딩의 크기

    Returns:
        패딩된 텐서 데이터
    """

    # 1. 패딩된 데이터 모양 계산
    new_shape = list(data.shape)  # 데이터 모양을 리스트로 변환
    new_shape[axis] += padding_size  # 지정된 차원에 패딩 크기를 더해줌

    # 2. 패딩된 크기의 텐서 생성
    new_data = torch.zeros(new_shape,
                           dtype=data.dtype,
                           device=data.device)
    # 3. 데이터를 새로운 텐서에 복사
    # 데이터 영역의 슬라이스를 생성
    slices = [slice(0, size) for size in data.shape]
    new_data[slices] = data  # 원래 데이터 복사

    # 4. 새로운 텐서 반환
    return new_data


def n_step_return(
        config: SimpleNamespace,
        state: torch.FloatTensor,
        next_state: torch.FloatTensor,
        reward: torch.FloatTensor,
        done: torch.int,
        critic: ValueFunctionMLP,
) -> (torch.FloatTensor, torch.FloatTensor):
    """
        n-스텝 리턴 계산
    Args:
        config: 설정
        state: 상태
        next_state: 다음 상태
        reward: 보상
        done: 에피소드 완료 여부
        critic: 가치 함수

    Returns:
        returns: 리턴
        advantage: 이득
    """

    # 1. 타입 스텝 수 계산
    n_steps = config.n_steps_of_return  # n 스텝
    total_len = reward.shape[-2]  # 전체 타입 스텝 수 계산
    ext_len = total_len + n_steps  # 패딩된 전체 타입 스텝 수 계산

    # 2. 가치 계산
    with torch.no_grad():
        value = critic(state)

    # 3. n 스텝 만큼 패딩
    p_value = padding(value, axis=-2, padding_size=n_steps)
    reward = padding(reward, axis=-2, padding_size=n_steps)
    done = padding(done, axis=-2, padding_size=n_steps)

    # 4. n 스텝에서 리턴에 가치 할당
    returns = p_value[n_steps:, :]

    # 5. 시간 차원을 마지막으로 이동
    returns = returns.view(-1, total_len)  # [total_len,1] -> [1,total_len]
    reward = reward.view(-1, ext_len)  # [ext_len,1] -> [1,ext_len]
    done = done.view(-1, ext_len)  # [ext_len,1] -> [1,ext_len]

    # 6. 리턴 계산 : n-1스텝에서 0 스텝의 보상 합산
    for t in reversed(range(n_steps)):
        returns = reward[:, t:total_len + t] + \
                  (1 - done[:, t:total_len + t]) * config.gamma * returns

    # 7. 리턴 표준화
    if config.return_standardization:
        returns = (returns - returns.mean(dim=1, keepdim=True)) / \
                  (returns.std(dim=1, keepdim=True) + config.epsilon)

    # 8. 리턴 모양 복구
    returns = returns.view_as(value)  # [1,total_len] -> [total_len,1]

    # 9. 이득 계산
    advantage = returns - value

    return returns, advantage  # 10. 리턴과 이득 반환


def gae_advantages(
        config: SimpleNamespace,
        state: torch.FloatTensor,
        next_state: torch.FloatTensor,
        reward: torch.FloatTensor,
        done: torch.int,
        critic: ValueFunctionMLP,
) -> (torch.FloatTensor, torch.FloatTensor):
    """
        GAE (Generalized Advantage Estimate) 계산
    Args:
        config: 설정
        state: 상태
        next_state: 다음 상태
        reward: 보상
        done: 에피소드 완료 여부
        critic: 가치 함수

    Returns:
        returns: 리턴
        advantage: 이득
    """

    # 1. 전체 타입 스텝 수 계산
    total_len = reward.shape[-2]

    # 2. 가치 계산
    with torch.no_grad():
        value = critic(state)  # 현재 상태 가치
        next_value = critic(next_state)  # 다음 상태 가치

    # 3. TD-잔차 계산
    delta = reward + (1 - done) * config.gamma * next_value - value

    # 4. TD-잔차을 GAE로 복사
    gae = delta.clone()

    # 5. 시간 차원을 마지막으로 이동
    gae = gae.view(-1, total_len)  # [total_len,1] -> [1,total_len]
    done = done.view(-1, total_len)  # [total_len,1] -> [1,total_len]

    # 6. Truncated GAE 계산
    for t in reversed(range(total_len - 1)):
        gae[:, t] += (1 - done[:, t]) * config.gamma * config.gae_lambda * gae[:, t + 1]

    # 7. GAE 표준화
    if config.gae_standardization:
        gae = (gae - gae.mean(dim=1, keepdim=True)) / \
              (gae.std(dim=1, keepdim=True) + config.epsilon)

    # 8. GAE의 모양 복구
    gae = gae.view_as(delta)  # [1,total_len] -> [total_len,1]

    # 9. 리턴 (Q-가치) 계산
    returns = gae + value

    return returns, gae  # 10. 리턴과 GAE 반환


# 리턴 또는 이득 계산 함수 레지스트리
REGISTRY = {}

REGISTRY["mc"] = monte_carlo_returns
REGISTRY["n_step"] = n_step_return
REGISTRY["gae"] = gae_advantages
