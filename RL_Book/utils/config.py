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
"""config.py: 설정 파일 관리 (읽기, 복사, 저장)"""
import os
from copy import deepcopy
from typing import Any

import yaml


def read_yaml(dirpath, filename):
    """
        Yaml 파일 읽기
    Args:
        dirpath: yaml 파일이 있는 디렉토리 위치
        filename: yaml 파일 이름

    Returns:
        yaml 파일을 읽어서 만든 딕셔너리 객체
    """
    # 1. yaml 파일 경로 생성
    filepath = os.path.join(dirpath, filename)

    # 2. yaml 파일이 없으면 None 반환
    if not os.path.isfile(filepath): return None

    # 3. 설정 파일 읽기
    with open(filepath, 'r') as f:
        try:
            config_dict = yaml.load(f, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            assert False, "{}.yaml error: {}".format(filename, exc)

    # 4. 설정 객체 반환
    return config_dict


def get_config(agent_name: str, env_name: str):
    """
        에이전트와 환경에 따른 설정 파일 읽기
    Args:
        agent_name: 에이전트 이름
        env_name: 환경 이름

    Returns:
        config_dict: 설정 딕셔너리
    """
    # 1. 설정 파일 이름 생성 {agent}/{env}.yaml
    filename = f'{env_name}.yaml'
    dirpath = os.path.join('config', 'agents', f'{agent_name}')

    # 2. 설정 파일 읽기
    config_dict = read_yaml(dirpath, filename)

    # 3. 설정 객체 반환
    return config_dict


def config_copy(config: Any):
    """
        딕셔너리나 리스트로 되어 있는 설정 객체를 복사
    Args:
        config: 설정 객체

    Returns:
        복사된 설정 객체
    """
    if isinstance(config, dict):
        # 1. 설정이 딕셔너리면
        # - value를 config_copy 재귀 호출하여 복사하고 새로운 딕셔너리 생성
        return {k: config_copy(v) for k, v in config.items()}
    elif isinstance(config, list):
        # 2. 설정이 리스트면
        # - 각 요소를 config_copy 재귀 호출하여 복사하고 새로운 리스트 생성
        return [config_copy(v) for v in config]
    else:
        # 3. 그 외의 경우에는 설정 객체를 복사
        return deepcopy(config)


def save_config(config):
    """
        설정 객체의 내용을 파일로 저장
    Args:
        config: 설정 SimpleNamespace 객체.
    """
    # 1. 설정 파일을 저장할 디렉토리 경로
    config_dirpath = os.path.join(
        os.getcwd(),
        config.local_results_path,
        "models",
        config.unique_token
    )
    # 2. 디렉토리 생성
    os.makedirs(config_dirpath, exist_ok=True)

    # 3. 저장할 설정 파일 경로
    config_filepath = os.path.join(
        config_dirpath,
        "{}.yaml".format(config.unique_token))

    if os.path.isfile(config_filepath): return

    # 4. 설정 파일 저장
    with open(config_filepath, 'w', encoding="utf-8") as file:
        for key, value in config.__dict__.items():
            if isinstance(value, str):
                file.write(f"{key}: '{value}' \n")
            else:
                file.write(f"{key}: {value} \n")
        file.close()
