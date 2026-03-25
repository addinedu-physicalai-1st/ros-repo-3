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
"""run_gpy.py: OpenGym 환경이 정상적으로 실행되는지 점검하기 위한 프로그램."""
import argparse
import gymnasium as gym


def run_gym(env_name, n_steps=100):
    """OpenGym 환경 실행.

    @param env_name: 환경 이름 @param n_steps: 환경과의 상호작용 횟수
    """

    # 1. 환경 생성 (render_mode를 생성 시 지정)
    env = gym.make(env_name, render_mode="human")

    # 2. 행동 공간 및 환경 초기화 (seed는 reset() 시 전달)
    env.action_space.seed(42)
    observation, info = env.reset(seed=42)
    print(f"[START] env={env_name}, max_steps={n_steps}")

    # 3. 환경과의 상호작용
    step = 0
    episode = 0
    episode_reward = 0.0
    quit_requested = False  # 창 X 버튼 종료 플래그

    for step in range(n_steps):
        # pygame 창 닫기 이벤트 감지
        try:
            import pygame
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print(f"\n[INFO] 창 닫기 요청 감지. 종료합니다.")
                    quit_requested = True
        except Exception:
            pass

        if quit_requested:
            break

        # 행동 선택
        action = env.action_space.sample()
        # 행동 실행 및 환경 정보 반환
        next_state, reward, terminated, truncated, env_info = env.step(action)
        done = terminated or truncated
        episode_reward += reward

        # 4. 환경 리셋
        if done:
            episode += 1
            print(f"  Episode {episode:3d} | steps={step+1:5d} | reward={episode_reward:8.2f}")
            episode_reward = 0.0
            observation, info = env.reset()

    # 환경 종료
    print(f"\n[DONE] Total steps={step+1}, episodes={episode}")
    env.close()

if __name__ == '__main__':

    # 1. 명령어 인자 파서 생성
    desc = 'OpenGym Test'
    parser = argparse.ArgumentParser(description=desc)
    
    # 2. 환경 이름 인자 추가
    parser.add_argument('-e',
                        '--env',
                        help='run type {CartPole-v1, LunarLanderContinuous-v3}',
                        type=str,
                        default='LunarLanderContinuous-v3')

    # 3. 환경과의 상호작용 횟수 인자 추가
    parser.add_argument('-s',
                        '--steps',
                        help='Number of environment step executions',
                        type=int,
                        default=1000)

    # 4. 명령어 인자 파싱
    args = parser.parse_args()
    print(args)

    # 5. OpenGym 환경 실행
    run_gym(args.env, n_steps=args.steps)