"""
PinkySacEnv — SAC용 환경 래퍼

RPPO/DreamerV3와의 핵심 차이:
    1. 행동 공간 수정:
       - v ∈ [0.05, 0.26] m/s  (최소 전진 속도 강제 → 제자리 회전 불가)
       - w ∈ [-1.0, 1.0] rad/s (최대 각속도 제한 → 급회전 방지)

    2. 보상 단순화 — 4항만 유지:
       - r = -0.5 (시간 패널티) + 20 × Δdist (진행 보상)
       - goal: +20, collision: -10
       - 9개 항목 → 4개: 신호 상쇄 없음, 학습 안정

    3. 관측 축소:
       - 36D → 28D (중복/불필요 제거)
       - LiDAR 24D + dist + cos(angle) + sin(angle) + progress

    4. Stuck 감지 + 경계 판정
"""

import rclpy
from geometry_msgs.msg import Twist
import numpy as np
import math
import time
import subprocess
import threading
from collections import deque
from types import SimpleNamespace

from envs.environment import Environment, EnvironmentSpec
from envs.pinky_env import PinkyNode, SAFE_WAYPOINTS


# Lidar sector indices (after np.roll, index 12 = front)
_FRONT = slice(8, 17)   # Front ±60 deg


class PinkySacEnv(Environment):
    """SAC용 Pinky 환경."""

    WORLD_NAME = "pinky_factory"
    ROBOT_NAME = "pinky"

    # 행동 공간: 회전 고착 물리적 차단 해제 (v3 대응)
    V_MIN, V_MAX = 0.0, 0.26    # m/s: 정지 가능
    W_MAX = 1.0                 # rad/s: 급회전 방지

    # PD 제어
    KP_V, KD_V = 1.5, 0.3
    KP_W, KD_W = 1.5, 0.3

    # 종료 조건
    COLLISION_M = 0.15
    GOAL_M = 0.05
    BOUND_X = 3.5
    BOUND_Y = 3.0

    # Stuck 감지
    STUCK_VEL = 0.01      # m/s
    STUCK_LIMIT = 50      # steps

    # v3 신호 분리 보상 (Blended Instinct)
    RC = SimpleNamespace(
        survive    = -0.1,
        heading    = 0.3,
        speed      = 2.0,
        brake      = 1.0,
        danger_pen = 0.1,
        progress   = 25.0,
        retreat    = 10.0,
        goal       = 100.0,
        collision  = -50.0,
        boundary   = -50.0
    )

    def __init__(self, config: SimpleNamespace, env_id: int, **kwargs):
        self.config = config
        self.env_id = env_id

        if not rclpy.ok():
            rclpy.init()
        self.node = PinkyNode()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(
            target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.state_shape = [28]
        self.action_shape = [2]
        self.max_steps = 750
        self.current_step = 0

        self.spawn_x = self.spawn_y = 0.0
        self.goal_x = self.goal_y = 0.0
        self.odom_start_x = self.odom_start_y = 0.0
        self.spawn_yaw = self.heading_offset = 0.0

        self.prev_error_v = self.prev_error_w = 0.0
        self.prev_distance = None
        self.cumulative_reward = 0.0
        self._stuck_counter = 0

        # Trajectory diagnostics
        self._actual_dist = 0.0
        self._prev_wx = self._prev_wy = None
        self._trajectory = []
        self._started = False
        self._term_reason = "timeout"

    def __deepcopy__(self, memo):
        memo[id(self)] = self
        return self

    # ──────── Infrastructure ────────

    def _teleport(self, x, y, qw, qz):
        self.node.cmd_vel_pub.publish(Twist())
        pose = (f"name: '{self.ROBOT_NAME}', "
                f"position: {{x: {x}, y: {y}, z: 0.1}}, "
                f"orientation: {{w: {qw}, x: 0.0, y: 0.0, z: {qz}}}")
        for _ in range(5):
            try:
                r = subprocess.run(
                    ["gz", "service", "-s",
                     f"/world/{self.WORLD_NAME}/set_pose",
                     "--reqtype", "gz.msgs.Pose",
                     "--reptype", "gz.msgs.Boolean",
                     "--timeout", "1000", "-r", pose],
                    capture_output=True, text=True, timeout=2)
                if r.returncode == 0:
                    time.sleep(0.1)
                    return True
            except Exception:
                pass
            time.sleep(0.2)
        return False

    def _world_pos(self):
        dx = self.node.x - self.odom_start_x
        dy = self.node.y - self.odom_start_y
        c, s = math.cos(self.heading_offset), math.sin(self.heading_offset)
        return self.spawn_x + dx * c - dy * s, self.spawn_y + dx * s + dy * c

    @staticmethod
    def _norm(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    # ──────── Observation (28D) ────────

    def _observe(self):
        wx, wy = self._world_pos()
        dx, dy = self.goal_x - wx, self.goal_y - wy
        dist = math.hypot(dx, dy)
        yaw = self._norm(self.node.yaw + self.heading_offset)
        g_angle = self._norm(math.atan2(dy, dx) - yaw)

        scan = self.node.scan_data
        
        # 전방 섹터 최솟값 (위험도 계산용)
        _FRONT = slice(8, 17)
        front_min_m = float(np.min(scan[_FRONT])) * self.node.MAX_LIDAR_DIST
        all_min_m = float(np.min(scan)) * self.node.MAX_LIDAR_DIST

        # 위험도 계산 (0.6m부터 긴장, 0.2m 이내 최대 위험)
        danger_level = float(np.clip((0.6 - front_min_m) / 0.4, 0.0, 1.0))

        s = np.zeros(28, dtype=np.float32)
        s[0:24] = scan                                       # LiDAR 24D
        s[24] = np.clip(dist / 5.0, 0.0, 1.0)              # 거리
        s[25] = math.cos(g_angle)                            # 방향 cos
        s[26] = math.sin(g_angle)                            # 방향 sin
        s[27] = self.current_step / self.max_steps           # 진행률

        return s, dist, wx, wy, all_min_m, g_angle, danger_level

    # ──────── Episode Lifecycle ────────

    def reset(self):
        if self._started:
            reason = self._term_reason
            opt = math.hypot(
                self.goal_x - self.spawn_x, self.goal_y - self.spawn_y)
            eff = (opt / max(self._actual_dist, 0.01)) * 100
            if self._prev_wx is not None:
                self._trajectory.append((self._prev_wx, self._prev_wy))
            print(
                f"[Pinky] 에피소드 종료 (길이: {self.current_step}, "
                f"보상: {self.cumulative_reward:.2f}, 원인: {reason})")
            print(
                f"[Pinky] 경로분석: actual={self._actual_dist:.2f}m, "
                f"optimal={opt:.2f}m, eff={eff:.0f}%")
            path = ' -> '.join(
                f"({x:.1f},{y:.1f})" for x, y in self._trajectory)
            print(f"[Pinky] 이동경로: {path}")

        self.current_step = 0
        self.prev_error_v = self.prev_error_w = 0.0
        self.prev_distance = None
        self.cumulative_reward = 0.0
        self._stuck_counter = 0
        self._started = True
        self._term_reason = "timeout"
        self._actual_dist = 0.0
        self._prev_wx = self._prev_wy = None
        self._trajectory = []

        stop = Twist()
        for _ in range(3):
            self.node.cmd_vel_pub.publish(stop)
            time.sleep(0.01)

        # 랜덤 spawn & goal (커리큘럼: 0.5~1.5m)
        si = np.random.randint(len(SAFE_WAYPOINTS))
        self.spawn_x, self.spawn_y = SAFE_WAYPOINTS[si]
        cands = [w for i, w in enumerate(SAFE_WAYPOINTS)
                 if i != si and 0.5 <= math.hypot(
                     w[0] - self.spawn_x, w[1] - self.spawn_y) <= 1.5]
        if not cands:
            cands = [w for i, w in enumerate(SAFE_WAYPOINTS) if i != si]
        self.goal_x, self.goal_y = cands[np.random.randint(len(cands))]

        yaw = np.random.uniform(-math.pi, math.pi)
        self.spawn_yaw = yaw
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)

        while not self._teleport(self.spawn_x, self.spawn_y, qw, qz):
            time.sleep(0.5)

        time.sleep(0.3)
        for _ in range(20):
            ox, oy = self.node.x, self.node.y
            time.sleep(0.05)
            if abs(self.node.x - ox) < 0.001 and abs(self.node.y - oy) < 0.001:
                break

        self.odom_start_x, self.odom_start_y = self.node.x, self.node.y
        self.heading_offset = self._norm(self.spawn_yaw - self.node.yaw)

        state, dist, wx, wy, *_ = self._observe()
        self.prev_distance = dist
        self._trajectory = [(wx, wy)]
        print(
            f"[Pinky] 새 에피소드: spawn=({self.spawn_x:.1f}, "
            f"{self.spawn_y:.1f}) -> goal=({self.goal_x:.1f}, "
            f"{self.goal_y:.1f}), dist={dist:.2f}m")
        return state

    def step(self, action):
        self.current_step += 1

        # 1. Action mapping (행동 공간 수정)
        target_v = float(
            self.V_MIN + (action[0] + 1.0) / 2.0 * (self.V_MAX - self.V_MIN))
        target_w = float(action[1] * self.W_MAX)

        # 2. PD control
        ev = target_v - self.node.current_v
        ew = target_w - self.node.current_w
        cv = target_v + self.KP_V * ev + self.KD_V * (ev - self.prev_error_v)
        cw = target_w + self.KP_W * ew + self.KD_W * (ew - self.prev_error_w)
        self.prev_error_v, self.prev_error_w = ev, ew

        msg = Twist()
        msg.linear.x = float(np.clip(cv, self.V_MIN, self.V_MAX))
        msg.angular.z = float(np.clip(cw, -self.W_MAX, self.W_MAX))
        self.node.cmd_vel_pub.publish(msg)
        time.sleep(0.05)

        # 3. Observe
        state, dist, wx, wy, all_min_m, g_angle, danger_level = self._observe()

        # 4. Trajectory recording
        if self._prev_wx is not None:
            self._actual_dist += math.hypot(
                wx - self._prev_wx, wy - self._prev_wy)
        self._prev_wx, self._prev_wy = wx, wy
        if self.current_step % 75 == 0:
            self._trajectory.append((wx, wy))

        # 5. Reward (v3 신호 분리)
        RC = self.RC
        cos_angle = math.cos(g_angle)

        # 1) 시간 압박 + 위험도 패널티
        reward = RC.survive - danger_level * RC.danger_pen

        # 2) 방향 신호 (v무관)
        reward += RC.heading * cos_angle

        # 3) 속도 보너스 (정렬+안전)
        if cos_angle > 0:
            safety = 1.0 - danger_level
            reward += target_v * cos_angle * safety * RC.speed

        # 4) 감속 유도 (위험시)
        reward -= target_v * danger_level * RC.brake

        # 5) 진척 신호 (방향 무관)
        if self.prev_distance is not None:
            approach = self.prev_distance - dist
            if approach > 0:
                reward += approach * RC.progress
            else:
                reward += approach * RC.retreat

        # 6. Terminal conditions
        terminated = False
        if dist < self.GOAL_M:
            reward = RC.goal
            terminated = True
            self._term_reason = "goal"
        elif all_min_m < self.COLLISION_M:
            reward = RC.collision
            terminated = True
            self._term_reason = f"collision({all_min_m:.3f}m)"
        elif abs(wx) > self.BOUND_X or abs(wy) > self.BOUND_Y:
            reward = RC.boundary
            terminated = True
            self._term_reason = f"boundary({wx:.1f},{wy:.1f})"

        # 7. Stuck detection
        if not terminated:
            speed = abs(self.node.current_v) + abs(self.node.current_w) * 0.1
            if speed < self.STUCK_VEL:
                self._stuck_counter += 1
            else:
                self._stuck_counter = 0
            if self._stuck_counter >= self.STUCK_LIMIT:
                reward = RC.collision
                terminated = True
                self._term_reason = f"stuck({self._stuck_counter}steps)"

        self.prev_distance = dist
        truncated = self.current_step >= self.max_steps
        done = terminated or truncated
        self.cumulative_reward += float(reward)

        return state, float(reward), bool(done), {'terminated': terminated}

    # ──────── Interface ────────

    def select_action(self):
        """Warmup 시 랜덤 행동 선택 (-1 ~ 1)."""
        return np.random.uniform(-1.0, 1.0, size=self.action_shape).astype(
            np.float32)

    def render(self):
        pass

    def close(self):
        self.node.cmd_vel_pub.publish(Twist())

    def environment_spec(self):
        return EnvironmentSpec(
            action_shape=self.action_shape,
            action_dtype=np.float32,
            action_high=1.0,
            action_low=-1.0,
            action_size=self.action_shape[0],
            b_continuous_action=True,
            state_shape=self.state_shape,
            state_dtype=np.float32)

    def max_episode_limit(self):
        return self.max_steps
