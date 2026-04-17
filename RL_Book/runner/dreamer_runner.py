"""
DreamerRunner — DreamerV3 전용 학습 러너

기존 Runner와의 핵심 차이:
    1. Step-level 학습: 매 환경 스텝마다 학습 가능 (rollout-then-train이 아님)
    2. EpisodeReplayBuffer: 기존 RolloutBuffer/ReplayBuffer 대신 에피소드 기반
    3. Warmup: 초기 N 스텝 동안 랜덤 행동으로 리플레이 버퍼 채움
    4. 학습 비율: train_every 스텝마다 train_steps 횟수 학습

학습 루프:
    while total_steps < max_steps:
        1. 환경에서 한 스텝 실행 (Actor)
        2. 리플레이 버퍼에 저장 (Actor.observe)
        3. train_every마다 world model + actor-critic 학습 (Learner)
        4. 학습 후 Actor 네트워크 동기화
        5. 로깅 & 체크포인트
"""

import os
import datetime
import torch
from types import SimpleNamespace
from utils.logging import get_console_logger, Logger
from utils.config import save_config
from agents.dreamer.dreamer import DreamerV3
from envs import REGISTRY as env_REGISTRY


class DreamerRunner:
    """DreamerV3 전용 러너."""

    def __init__(self, config: dict, console_logger=None, logger=None,
                 verbose: bool = False):
        # 1. 콘솔 로거
        if console_logger is None:
            self.console_logger = get_console_logger()

        # 2. GPU 설정 확인
        config = self._sanity_check(config)
        self.config = SimpleNamespace(**config)
        self.config.device = (
            f"cuda:{self.config.device_num}"
            if self.config.use_cuda else "cpu"
        )

        # 3. 실행 토큰
        unique_token = "{}_{}_{}".format(
            self.config.agent,
            self.config.env_name,
            datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
        self.config.unique_token = unique_token

        # 4. 로거
        if logger is None:
            logger = Logger(self.console_logger)
            if self.config.use_tensorboard:
                tb_dir = os.path.join(
                    os.getcwd(),
                    self.config.local_results_path,
                    "tb_logs", unique_token)
                logger.setup_tensorboard(tb_dir)
        self.logger = logger

        # 5. 설정 출력 & 저장
        if verbose:
            import pprint
            self.logger.console_logger.info(
                "\n\n" + pprint.pformat(config, indent=4, width=1) + "\n")

        if self.config.training_mode and self.config.save_model:
            save_config(self.config)

        # 6. 카운터 초기화
        self.total_n_timesteps = 0
        self.total_n_episodes = 0

        # CUDNN
        torch.backends.cudnn.deterministic = self.config.torch_deterministic

    def _sanity_check(self, config: dict) -> dict:
        if config["use_cuda"] and not torch.cuda.is_available():
            config["use_cuda"] = False
            self.console_logger.warning(
                "CUDA not available, switched to CPU")
        if not config["training_mode"] and config["n_envs"] != 1:
            config["n_envs"] = 1
        return config

    def run(self):
        """메인 실행 진입점."""
        self.logger.console_logger.info(
            f"environment name: {self.config.env_name}")

        # 1. 환경 & 에이전트 생성
        self.env = env_REGISTRY[self.config.env_wrapper](
            self.config, 0, **self.config.env_args)
        self.agent = DreamerV3(
            config=self.config, logger=self.logger, env=self.env)

        if self.config.use_cuda:
            self.agent.cuda()

        # 2. 훈련/추론
        if self.config.training_mode:
            if self.config.checkpoint_path != "":
                if not self._restore():
                    return False
            self.train()
        else:
            if not self._load_inference():
                return False
            self.test()

        return True

    def train(self):
        """DreamerV3 학습 루프."""
        self.last_model_save_step = 0
        self.last_logging_step = 0
        start_time = datetime.datetime.now().replace(microsecond=0)

        # 하이퍼파라미터
        warmup_steps = getattr(self.config, 'warmup_step', 5000)
        train_every = getattr(self.config, 'train_every', 5)
        train_steps = getattr(self.config, 'train_steps', 1)

        # 에피소드 통계
        ep_reward = 0.0
        ep_length = 0
        n_episodes_in_log = 0
        sum_returns = 0.0
        sum_lengths = 0.0

        # 환경 초기 리셋
        state = self.env.reset()

        self.logger.console_logger.info(
            f"[DreamerV3] 학습 시작 (warmup: {warmup_steps} steps, "
            f"max: {self.config.max_environment_steps} steps)")

        while self.total_n_timesteps < self.config.max_environment_steps:
            # ─── 행동 선택 ───
            if self.total_n_timesteps < warmup_steps:
                # Warmup: 랜덤 행동
                action = self.env.select_action()
            else:
                # Actor 행동
                action = self.agent.actor.select_action(
                    state, training=True)

            # ─── 환경 스텝 ───
            next_state, reward, done, info = self.env.step(action)

            # ─── 리플레이 버퍼 저장 ───
            # DreamerV3: 버퍼에는 terminated만 저장 (ContinueHead용)
            # RSSM 리셋은 done(terminated OR truncated) 기준
            terminated = info.get('terminated', done) \
                if isinstance(info, dict) else done
            self.agent.actor.observe(
                state, action, reward, done, terminated=terminated)

            # ─── 카운터 업데이트 ───
            self.total_n_timesteps += 1
            ep_reward += float(reward)
            ep_length += 1

            # ─── 학습 ───
            if (self.total_n_timesteps >= warmup_steps and
                    self.total_n_timesteps % train_every == 0):
                for _ in range(train_steps):
                    metrics = self.agent.update(self.total_n_timesteps)
                    if metrics:
                        # TensorBoard 로깅
                        for key, value in metrics.items():
                            self.logger.log_stat(
                                key, value, self.total_n_timesteps)
                # Actor 동기화
                self.agent.sync_actor()

            # ─── 에피소드 종료 처리 ───
            if done:
                self.total_n_episodes += 1
                n_episodes_in_log += 1
                sum_returns += ep_reward
                sum_lengths += ep_length

                # 통계 로깅
                self.logger.log_stat(
                    "returns_mean", ep_reward, self.total_n_timesteps)
                self.logger.log_stat(
                    "len_episodes_mean", ep_length, self.total_n_timesteps)
                self.logger.log_stat(
                    "replay_episodes", len(self.agent.replay_buffer),
                    self.total_n_timesteps)
                self.logger.log_stat(
                    "replay_steps", self.agent.replay_buffer.num_steps,
                    self.total_n_timesteps)

                # 리셋
                state = self.env.reset()
                ep_reward = 0.0
                ep_length = 0
            else:
                state = next_state

            # ─── 주기적 콘솔 로깅 ───
            if (self.total_n_timesteps - self.last_logging_step) >= \
                    self.config.log_interval:
                self.logger.log_stat(
                    "episode", self.total_n_episodes,
                    self.total_n_timesteps)
                self.logger.print_recent_stats()
                self.last_logging_step = self.total_n_timesteps
                n_episodes_in_log = 0
                sum_returns = 0.0
                sum_lengths = 0.0

            # ─── 체크포인트 ───
            if (self.config.save_model and
                    (self.total_n_timesteps - self.last_model_save_step) >=
                    self.config.save_model_interval):
                self._save(self.total_n_timesteps)
                self.last_model_save_step = self.total_n_timesteps

        # 학습 종료
        end_time = datetime.datetime.now().replace(microsecond=0)
        self.logger.console_logger.info(f"Start time: {start_time}")
        self.logger.console_logger.info(f"End time: {end_time}")
        self.logger.console_logger.info(f"Total: {end_time - start_time}")

    def test(self):
        """추론 모드."""
        max_episodes = self.config.inference_max_episodes
        self.logger.console_logger.info(
            f"[DreamerV3] 추론 시작 ({max_episodes} 에피소드)")

        for ep in range(max_episodes):
            state = self.env.reset()
            done = False
            ep_reward = 0.0
            ep_length = 0

            while not done:
                action = self.agent.actor.select_action(
                    state, training=False)
                next_state, reward, done, info = self.env.step(action)
                ep_reward += float(reward)
                ep_length += 1
                state = next_state

            self.logger.console_logger.info(
                f"Episode {ep + 1}: reward={ep_reward:.2f}, "
                f"length={ep_length}")

    def _save(self, timestep: int):
        """체크포인트 저장."""
        path = os.path.join(
            os.getcwd(), self.config.local_results_path,
            "models", self.config.unique_token, str(timestep))
        os.makedirs(path, exist_ok=True)
        self.logger.console_logger.info(f"Saving models to {path}")
        self.agent.save(path)

    def _restore(self) -> bool:
        """체크포인트 복원."""
        cp = self.config.checkpoint_path
        if not os.path.isdir(cp):
            self.logger.console_logger.info(
                f"Checkpoint directory {cp} doesn't exist")
            return False

        timesteps = [int(n) for n in os.listdir(cp)
                     if os.path.isdir(os.path.join(cp, n)) and n.isdigit()]
        if not timesteps:
            self.logger.console_logger.info(f"No checkpoints in {cp}")
            return False

        if self.config.load_step == 0:
            ts = max(timesteps)
        else:
            ts = min(timesteps, key=lambda x: abs(x - self.config.load_step))

        path = os.path.join(cp, str(ts))
        self.logger.console_logger.info(f"Loading model from {path}")
        self.agent.restore(path)
        self.total_n_timesteps = ts
        return True

    def _load_inference(self) -> bool:
        """추론 모델 로딩."""
        return self.agent.load(self.config.inference_model_path)
