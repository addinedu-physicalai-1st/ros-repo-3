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
"""logging.py: 로거 정의 (콘솔 로깅 및 텐서보드 로깅)"""
from collections import defaultdict

import numpy as np
import torch
from torch.utils.tensorboard import SummaryWriter

import logging


class Logger:
    """콘솔 로깅과 텐서보드 로깅과 성능 통계를 관리하는 로거 클래스."""

    def __init__(self, console_logger):
        """로거 초기화 @param console_logger: 콘솔 로거."""
        # 1. 콘솔 로거
        self.console_logger = console_logger

        # 2. 텐서보드 로거 사용하지 않음 설정
        self.use_tb = False

        # 3. 성능 통계 정보 저장 딕셔너리 생성
        self.stats = defaultdict(lambda: [])

    def setup_tensorboard(self, dirpath):
        """
            텐서보드 로거 생성
        Args:
            dirpath: 텐서보드 로그 파일 생성 디렉토리
        """
        # 1. 텐서보드 로거 생성
        self.tb_logger = SummaryWriter(log_dir=dirpath)

        # 2. 텐서보르 로거 사용 설정
        self.use_tb = True

    def log_stat(self, key, value, t):
        """
            성능에 대한 통계 정보를 관리
        Args:
            key: 통계 정보 이름
            value: 통계 정보 값
            t: 타입 스텝
        """
        # 1. 통계 정보 추가
        self.stats[key].append((t, value))

        # 2. 텐서보드 로깅
        if self.use_tb: self.tb_logger.add_scalar(key, value, t)

    def print_recent_stats(self, item_per_line: int = 4, window: int = 5):
        """
            콘솔에 통계 정보 출력
        Args:
            item_per_line: 한 줄에 표현할 통게 정보 개수
            window: 통계 정보의 평균을 계산하기 위한 윈도우 크기
        """
        # 1. 로그 문자열 생성 (타임 스텝 및 에피소드 정보)
        log_str = "Steps: {:>8} | Episode: {:>6}\n".format(*self.stats["episode"][-1])

        i = 0
        for (key, value) in sorted(self.stats.items()):
            # 2. 통계 정보가 에피소드면 처리하지 않음
            if key == "episode": continue

            # 3. 통계 정보의 평균을 출력하기 위한 로그 문자열 생성

            # 3-1. 과거 window 만큼의 통계 정보를 리스트로 추출 (디바이스를 cpu()로 변환)
            i += 1
            stats_list = [x[1].cpu() if isinstance(x[1], torch.Tensor) else x[1] for x in self.stats[key][-window:]]

            # 3-2. 평균 계산
            item = "{:.6f}".format(np.mean(stats_list))

            # 3-3. 통계 정보의 Key와 평균의 출력 문자열 생성
            log_str += "{:<25}{:>8}".format(key + ":", item)

            # 4. 전체 로그 문자열에 추가 (한 줄 처리, 정보간 tab 처리) 
            log_str += "\n" if i % item_per_line == 0 else "\t"

        # 5. 생성한 로그 문자열을 콘솔로 출력
        self.console_logger.info(log_str)


def get_console_logger():
    """콘솔 로거 생성."""
    # 1. 로거 생성
    logger = logging.getLogger()

    # 2. 핸들러 생성
    logger.handlers = []
    ch = logging.StreamHandler()

    # 3. 포맷터 생성 및 핸들러에 지정
    formatter = logging.Formatter('[%(levelname)s %(asctime)s] %(name)s %(message)s', '%H:%M:%S')
    ch.setFormatter(formatter)

    # 4. 로거에 핸들러 지정
    logger.addHandler(ch)

    # 5. 로깅 레벨 지정
    logger.setLevel('DEBUG')

    return logger
