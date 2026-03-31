"""
PinkyDreamerEnv — DreamerV3용 환경 래퍼

PinkyRppoEnv를 상속하되, DreamerV3에 필요한 인터페이스를 오버라이드:

    1. step() 반환값에 terminated/truncated 구분 추가
       - DreamerV3의 ContinueHead 학습:
         continue_target = 1.0 - terminated (truncated가 아닌 terminated만)
       - timeout(truncated)은 환경의 자연적 종료가 아니므로 continue=1 유지

    2. Stuck 감지
       - 로봇이 벽에 밀착하여 이동하지 못하는 상태 감지
       - 실제 속도가 임계값 이하인 상태가 N스텝 지속 시 강제 종료
       - DreamerV3의 학습 오버헤드로 인해 750스텝 도달이 매우 느릴 수 있음

    3. 환경 격리
       - RPPO 환경 수정이 DreamerV3에 영향 주지 않도록 step() 레벨에서 차단
"""

from types import SimpleNamespace
from envs.pinky_rppo_env import PinkyRppoEnv


class PinkyDreamerEnv(PinkyRppoEnv):
    """
    DreamerV3용 Pinky 환경 래퍼.

    PinkyRppoEnv와의 핵심 차이:
        - step() 반환 info에 'terminated' 플래그를 정확히 제공
        - stuck 감지: 속도 0 상태 지속 시 강제 종료 (collision 처리)
        - done = terminated OR truncated (에피소드 종료 판정)
    """

    # Stuck 감지 파라미터
    STUCK_VEL_THRESHOLD = 0.01   # m/s 이하면 정지 판정
    STUCK_STEPS_LIMIT = 50       # 연속 정지 스텝 한계 (~2.5초)

    def __init__(self, config: SimpleNamespace, env_id: int, **kwargs):
        super().__init__(config, env_id, **kwargs)
        self._stuck_counter = 0

    def reset(self):
        state = super().reset()
        self._stuck_counter = 0
        return state

    def step(self, action):
        """
        DreamerV3 전용 step: terminated/truncated 구분 + stuck 감지.

        Returns:
            state:  (36,) numpy observation
            reward: float
            done:   bool (terminated OR truncated OR stuck)
            info:   {
                'terminated': bool — 자연적 종료 (goal/collision/boundary/stuck)
                'truncated':  bool — 시간 초과 (max_steps 도달)
            }
        """
        state, reward, done, info = super().step(action)

        # 부모 클래스의 info에서 terminated 추출
        terminated = info.get('terminated', False) if isinstance(info, dict) else False
        truncated = done and not terminated

        # ─── Stuck 감지 ───
        # 에피소드가 아직 끝나지 않았고, 로봇이 거의 움직이지 않는 경우
        if not done:
            actual_speed = abs(self.node.current_v) + abs(self.node.current_w) * 0.1
            if actual_speed < self.STUCK_VEL_THRESHOLD:
                self._stuck_counter += 1
            else:
                self._stuck_counter = 0

            if self._stuck_counter >= self.STUCK_STEPS_LIMIT:
                # Stuck → collision과 동일하게 처리
                terminated = True
                done = True
                reward = -self.RC.collision
                self._term_reason = f"stuck({self._stuck_counter}steps)"

        # DreamerV3용 info 재구성
        info = {
            'terminated': terminated,
            'truncated': truncated,
        }

        return state, reward, done, info
