"""
RPPO v4 — Blended Instinct Reward (HFSM 제거)

v1-v3 실패 원인 (135k step 분석):
    HFSM이 agent의 행동을 과도하게 제약
    - 6 Phase 중 4개: v=0 강제 (전진 불가)
    - CRUISE 평균 1-2 step -> 즉시 DECELERATE
    - FSM 전이가 결정적 -> RL 학습 여지 없음

    pinky_env.py (기본 PPO)는 같은 환경에서 회피 성공
    -> 핵심: 연속 행동 제어 + blended instinct reward

v4 핵심 변경:
    1. HFSM 완전 제거 -> agent가 항상 v, w 모두 제어
    2. Blended Instinct Reward (pinky_env.py 검증 완료):
       - danger_level in [0,1]: 전방 장애물 근접도
       - 안전 시: 목표 정렬 + 전진 보상
       - 위험 시: 전진 억제 -> 자연스러운 회피
    3. GRU가 시계열 맥락 담당 (장애물 기억, 우회 계획)

State:  36D = [24 lidar + dist + angle + 2 vel + danger + front/left/right
               + cos/sin(angle) + loop + progress]
Action: 2D continuous in [-1, 1] -> v in [0, 0.5], w in [-1.5, 1.5]
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
_FRONT = slice(8, 17)   # Front +/-60 deg
_LEFT  = slice(15, 21)  # Left 45~135 deg
_RIGHT = slice(4, 10)   # Right 45~135 deg

# Trajectory recording interval (~10 points per 750-step episode)
_TRAJ_INTERVAL = 75


class PinkyRppoEnv(Environment):

    WORLD_NAME = "pinky_factory"
    ROBOT_NAME = "pinky"

    KP_V, KD_V = 1.5, 0.3
    KP_W, KD_W = 1.5, 0.3

    TH = SimpleNamespace(
        danger_far   = 0.6,    # danger_level 시작 거리 (m)
        danger_near  = 0.2,    # danger_level 최대 거리 (m)
        collision_m  = 0.15,
        goal_m       = 0.15,
        bound_x      = 3.5,
        bound_y      = 3.0,
    )

    RC = SimpleNamespace(
        # ── v3: 신호 분리 설계 ──
        # 원칙: 방향/진척/속도 신호를 독립적으로 유지
        survive     = -0.1,    # 시간 압박 (매 스텝 비용)
        heading     = 0.3,     # 방향 신호: cos(g_angle), v/danger 무관
        speed       = 2.0,     # 속도 보너스: v × cos × (1-danger)
        brake       = 1.0,     # 감속 유도: v × danger (전진 자체는 죽이지 않음)
        danger_pen  = 0.1,     # 장애물 근접 경고
        progress    = 25.0,    # 진척 보상: 거리 감소 시 (danger 무관)
        retreat     = 10.0,    # 후퇴 감점: 거리 증가 시 (danger 무관)
        goal        = 100.0,
        collision   = 100.0,
        boundary    = 100.0,
    )

    def __init__(self, config: SimpleNamespace, env_id: int, **kwargs):
        self.config = config
        self.env_id = env_id

        if not rclpy.ok():
            rclpy.init()
        self.node = PinkyNode()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.state_shape = [36]
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

        # Path tracker (loop detection)
        self.cell_size = 0.5
        self.path_q = deque()
        self.visit_memo = {}
        self.max_q = 80

        # Trajectory & diagnostics
        self._actual_dist = 0.0
        self._prev_wx = self._prev_wy = None
        self._trajectory = []

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
                    ["gz", "service", "-s", f"/world/{self.WORLD_NAME}/set_pose",
                     "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
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
        return self.spawn_x + dx*c - dy*s, self.spawn_y + dx*s + dy*c

    @staticmethod
    def _norm(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    # ──────── Path Tracking ────────

    def _track(self, wx, wy):
        cell = (round(wx / self.cell_size), round(wy / self.cell_size))
        revisit = float(self.visit_memo.get(cell, 0) > 0)

        self.path_q.append(cell)
        self.visit_memo[cell] = self.visit_memo.get(cell, 0) + 1

        if len(self.path_q) > self.max_q:
            old = self.path_q.popleft()
            self.visit_memo[old] -= 1
            if self.visit_memo[old] <= 0:
                del self.visit_memo[old]
        return revisit

    # ──────── Observation (36D) ────────

    def _observe(self):
        wx, wy = self._world_pos()
        dx, dy = self.goal_x - wx, self.goal_y - wy
        dist = math.hypot(dx, dy)
        yaw = self._norm(self.node.yaw + self.heading_offset)
        g_angle = self._norm(math.atan2(dy, dx) - yaw)

        scan = self.node.scan_data  # normalized [0, 1]
        front_min = float(np.min(scan[_FRONT]))
        left_min  = float(np.min(scan[_LEFT]))
        right_min = float(np.min(scan[_RIGHT]))
        all_min   = float(np.min(scan))
        front_min_m = front_min * self.node.MAX_LIDAR_DIST
        all_min_m   = all_min * self.node.MAX_LIDAR_DIST

        TH = self.TH
        danger_range = TH.danger_far - TH.danger_near
        danger_level = float(np.clip(
            (TH.danger_far - front_min_m) / danger_range, 0.0, 1.0))

        loop = self._track(wx, wy)

        s = np.zeros(36, dtype=np.float32)
        s[0:24]  = scan                                    # Lidar (24D)
        s[24]    = np.clip(dist / 5.0, 0.0, 1.0)          # Distance
        s[25]    = g_angle / math.pi                       # Goal angle
        s[26]    = self.node.current_v / 0.5               # Linear vel
        s[27]    = self.node.current_w / 1.0               # Angular vel
        s[28]    = danger_level                             # Front danger (0~1)
        s[29]    = front_min                               # Front sector min
        s[30]    = left_min                                # Left sector min
        s[31]    = right_min                               # Right sector min
        s[32]    = math.cos(g_angle)                       # Alignment quality
        s[33]    = math.sin(g_angle)                       # Lateral direction
        s[34]    = loop                                    # Loop detection
        s[35]    = self.current_step / self.max_steps      # Time progress

        return s, dist, wx, wy, g_angle, danger_level, all_min_m, loop

    # ──────── Reward (Blended Instinct) ────────

    def _reward(self, target_v, target_w, dist, g_angle,
                danger_level, all_min_m, loop):
        """
        Reward v3 — 신호 분리 설계

        v1-v2 공통 실패 원인:
            nav_r = v × cos(g_angle) × (1 - danger)
            → v=0이면 방향 신호 소멸, danger 높으면 진척 신호 소멸
            → factory 환경에서 3가지가 동시에 0 → 로봇이 "어디로 가야 하는지" 모름
            → 결과: 직진 → 충돌 → v=0 학습 → 제자리 회전 수렴

        v3 핵심 원칙:
            방향/진척/속도 신호를 독립적으로 유지
            하나가 0이어도 나머지는 살아있음

            1. heading: cos(g_angle) — v, danger 무관, 항상 방향 안내
            2. progress: 거리 변화 — danger 무관, 항상 진척 피드백
            3. speed: v × cos × (1-danger) — 안전+정렬 시에만 속도 보너스
            4. brake: v × danger — 부드러운 감속 (전진을 죽이지 않음)
        """
        RC = self.RC
        cos_angle = math.cos(g_angle)

        # ── 1. 시간 압박 + 장애물 근접 경고 ──
        r = RC.survive - danger_level * RC.danger_pen

        # ── 2. 방향 신호 (v, danger 무관 — 항상 활성) ──
        # 목표 정면: +0.3, 옆: 0, 뒤: -0.3
        # 어느 각도에서든 gradient 존재 → 회전 방향 학습 가능
        r += RC.heading * cos_angle

        # ── 3. 속도 보너스 (정렬 + 안전 시) ──
        # 목표 방향으로 빠르게 갈수록 보상 증가
        if cos_angle > 0:
            safety = 1.0 - danger_level
            r += target_v * cos_angle * safety * RC.speed

        # ── 4. 감속 유도 (위험 시) ──
        # avoid_v_pen과 달리 선형 감속 — 전진 자체를 죽이지 않음
        r -= target_v * danger_level * RC.brake

        # ── 5. 진척 신호 (danger 무관 — 항상 활성) ──
        # w_nav 곱셈 제거 → 장애물 근처에서도 거리 줄이면 보상
        if self.prev_distance is not None:
            approach = self.prev_distance - dist
            if approach > 0:
                r += approach * RC.progress
            else:
                r += approach * RC.retreat  # approach < 0 이므로 감점

        return r

    # ──────── Episode Lifecycle ────────

    def reset(self):
        if hasattr(self, "_started") and self._started:
            reason = getattr(self, "_term_reason", "timeout")
            opt = math.hypot(self.goal_x - self.spawn_x, self.goal_y - self.spawn_y)
            eff = (opt / max(self._actual_dist, 0.01)) * 100

            if self._prev_wx is not None:
                self._trajectory.append((self._prev_wx, self._prev_wy))

            print(
                f"[Pinky] 에피소드 종료 (길이: {self.current_step}, "
                f"보상: {self.cumulative_reward:.2f}, 원인: {reason})")
            print(
                f"[Pinky] 경로분석: actual={self._actual_dist:.2f}m, "
                f"optimal={opt:.2f}m, eff={eff:.0f}%")
            path = ' -> '.join(f"({x:.1f},{y:.1f})" for x, y in self._trajectory)
            print(f"[Pinky] 이동경로: {path}")

        # Reset
        self.current_step = 0
        self.prev_error_v = self.prev_error_w = 0.0
        self.prev_distance = None
        self.cumulative_reward = 0.0
        self._started = True
        self._term_reason = "timeout"
        self._actual_dist = 0.0
        self._prev_wx = self._prev_wy = None
        self._trajectory = []
        self.path_q.clear()
        self.visit_memo.clear()

        stop = Twist()
        for _ in range(3):
            self.node.cmd_vel_pub.publish(stop)
            time.sleep(0.01)

        # Random spawn & goal (curriculum: 0.5~1.5m)
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
            f"[Pinky] 새 에피소드: spawn=({self.spawn_x:.1f}, {self.spawn_y:.1f}) "
            f"-> goal=({self.goal_x:.1f}, {self.goal_y:.1f}), dist={dist:.2f}m")
        return state

    def step(self, action):
        self.current_step += 1

        # 1. Action mapping (v in [0, 0.5], w in [-1.5, 1.5])
        target_v = float((action[0] + 1.0) / 2.0 * 0.5)
        target_w = float(action[1] * 1.5)

        # 2. PD control
        ev = target_v - self.node.current_v
        ew = target_w - self.node.current_w
        cv = target_v + self.KP_V * ev + self.KD_V * (ev - self.prev_error_v)
        cw = target_w + self.KP_W * ew + self.KD_W * (ew - self.prev_error_w)
        self.prev_error_v, self.prev_error_w = ev, ew

        msg = Twist()
        msg.linear.x = float(np.clip(cv, 0.0, 0.5))
        msg.angular.z = float(np.clip(cw, -1.5, 1.5))
        self.node.cmd_vel_pub.publish(msg)
        time.sleep(0.05)

        # 3. Observe
        state, dist, wx, wy, g_angle, danger_level, all_min_m, loop = \
            self._observe()

        # 4. Trajectory recording
        if self._prev_wx is not None:
            self._actual_dist += math.hypot(wx - self._prev_wx, wy - self._prev_wy)
        self._prev_wx, self._prev_wy = wx, wy
        if self.current_step % _TRAJ_INTERVAL == 0:
            self._trajectory.append((wx, wy))

        # 5. Reward (blended instinct)
        reward = self._reward(target_v, target_w, dist, g_angle,
                              danger_level, all_min_m, loop)

        # 6. Terminal conditions
        TH = self.TH
        terminated = False
        if dist < TH.goal_m:
            reward = self.RC.goal
            terminated = True
            self._term_reason = "goal"
        elif all_min_m < TH.collision_m:
            reward = -self.RC.collision
            terminated = True
            self._term_reason = f"collision({all_min_m:.3f}m)"
        elif abs(wx) > TH.bound_x or abs(wy) > TH.bound_y:
            reward = -self.RC.boundary
            terminated = True
            self._term_reason = f"boundary({wx:.1f},{wy:.1f})"

        self.prev_distance = dist
        truncated = self.current_step >= self.max_steps
        done = terminated or truncated
        self.cumulative_reward += float(reward)

        return state, float(reward), bool(done), {"terminated": bool(terminated)}

    # ──────── Interface ────────

    def render(self):
        pass

    def close(self):
        self.node.cmd_vel_pub.publish(Twist())
        self.node.destroy_node()
        rclpy.shutdown()

    def environment_spec(self):
        return EnvironmentSpec(
            action_shape=self.action_shape, action_dtype=np.float32,
            action_high=1.0, action_low=-1.0,
            action_size=self.action_shape[0], b_continuous_action=True,
            state_shape=self.state_shape, state_dtype=np.float32,
        )

    def select_action(self):
        return np.random.uniform(-1.0, 1.0, size=self.action_shape).astype(np.float32)

    def max_episode_limit(self):
        return self.max_steps
