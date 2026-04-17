import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import threading
import math
import time
import subprocess
from types import SimpleNamespace
from envs.environment import Environment, EnvironmentSpec


class PinkyNode(Node):
    """Phase 3 목표 도달 자율주행 ROS2 노드 (단일 로봇)"""

    def __init__(self, namespace=""):
        super().__init__("pinky_nav_node")

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

        self.scan_data = np.zeros(24)
        self.current_v = 0.0
        self.current_w = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.MAX_LIDAR_DIST = 3.5

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        
        # Gazebo에서 너무 가까운 경우 NaN이나 음의 무한대가 나올 수 있음
        ranges[np.isnan(ranges)] = 0.0
        ranges[np.isposinf(ranges)] = self.MAX_LIDAR_DIST
        ranges[np.isneginf(ranges)] = 0.0

        n = len(ranges)
        ranges = np.roll(ranges, n // 2)
        
        # 단순 추출 대신 24개 구간별 최솟값(min) 추출
        sector_size = max(1, n // 24)
        min_ranges = np.zeros(24)
        for i in range(24):
            start_idx = i * sector_size
            end_idx = start_idx + sector_size if i < 23 else n
            min_ranges[i] = np.min(ranges[start_idx:end_idx])

        # 0 ~ 1.0 사이로 정규화하여 저장
        self.scan_data = min_ranges / self.MAX_LIDAR_DIST

    def odom_callback(self, msg):
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)


SAFE_WAYPOINTS = [
    (0.0, 0.0),
    (0.0, -1.0),
    (-1.5, -1.0),
    (-1.5, 0.0),
    (-1.5, 1.1),
    (-0.5, 1.1),
    (0.5, 1.2),
    (0.5, 0.0),
    (0.5, -1.2),
    (0.0, 0.8),
]


class PinkyNavEnv(Environment):
    """
    Phase 3: 목표 도달 자율주행 환경
    - 특징 1: Blended Instinct Reward (안전=목표지향, 위험=회피 본능 활성화)
    - 특징 2: 선형 보간 기반 연속 액션 매핑 (Dead Gradient 제거)
    - 특징 3: Curriculum Learning (초기 목표 거리 조절)
    """

    WORLD_NAME = "pinky_factory"
    ROBOT_NAME = "pinky"

    # 모터 제어 부드러움을 위한 PID 게인
    KP_V = 1.5
    KD_V = 0.3
    KP_W = 1.5
    KD_W = 0.3

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

        self.state_shape = [28]
        self.action_shape = [2]

        self.max_steps = 750
        self.current_step = 0

        self.spawn_x = 0.0
        self.spawn_y = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0

        self.odom_start_x = 0.0
        self.odom_start_y = 0.0

        # Heading offset 보정 (DiffDrive 텔레포트 후 내부 θ 불일치 수정)
        self.spawn_yaw = 0.0
        self.heading_offset = 0.0

        self.prev_error_v = 0.0
        self.prev_error_w = 0.0

        self.prev_distance = None
        self.cumulative_reward = 0.0

    def __deepcopy__(self, memo):
        memo[id(self)] = self
        return self

    def _teleport(self, x, y, qw, qz):
        stop_msg = Twist()
        self.node.cmd_vel_pub.publish(stop_msg)

        pose_str = (
            f"name: '{self.ROBOT_NAME}', "
            f"position: {{x: {x}, y: {y}, z: 0.1}}, "
            f"orientation: {{w: {qw}, x: 0.0, y: 0.0, z: {qz}}}"
        )
        for attempt in range(5):
            try:
                result = subprocess.run(
                    [
                        "gz",
                        "service",
                        "-s",
                        f"/world/{self.WORLD_NAME}/set_pose",
                        "--reqtype",
                        "gz.msgs.Pose",
                        "--reptype",
                        "gz.msgs.Boolean",
                        "--timeout",
                        "1000",
                        "-r",
                        pose_str,
                    ],
                    capture_output=True,
                    text=True,
                    timeout=2,
                )
                if result.returncode == 0:
                    time.sleep(0.1)
                    return True
            except:
                pass
            time.sleep(0.2)
        return False

    def _get_world_position(self):
        dx_odom = self.node.x - self.odom_start_x
        dy_odom = self.node.y - self.odom_start_y
        # DiffDrive의 내부 heading이 텔레포트 후 실제와 다르므로
        # heading_offset 만큼 2D 회전하여 실제 월드 프레임 이동량으로 보정
        cos_h = math.cos(self.heading_offset)
        sin_h = math.sin(self.heading_offset)
        dx_world = dx_odom * cos_h - dy_odom * sin_h
        dy_world = dx_odom * sin_h + dy_odom * cos_h
        return self.spawn_x + dx_world, self.spawn_y + dy_world

    def get_state(self):
        world_x, world_y = self._get_world_position()

        dx = self.goal_x - world_x
        dy = self.goal_y - world_y
        distance = math.hypot(dx, dy)
        # DiffDrive의 yaw도 텔레포트 후 실제와 다르므로 heading_offset으로 보정
        corrected_yaw = self.node.yaw + self.heading_offset
        angle = math.atan2(dy, dx) - corrected_yaw
        angle = (angle + math.pi) % (2 * math.pi) - math.pi

        state = np.zeros(28, dtype=np.float32)
        state[0:24] = self.node.scan_data
        state[24] = np.clip(distance / 5.0, 0.0, 1.0)
        state[25] = angle / math.pi
        state[26] = self.node.current_v / 0.5
        state[27] = self.node.current_w / 1.0
        return state, distance

    def reset(self):
        if hasattr(self, "_episode_started") and self._episode_started:
            reason = getattr(self, "_term_reason", "timeout")
            print(
                f"[Pinky] 에피소드 종료 (길이: {self.current_step}, "
                f"보상: {self.cumulative_reward:.2f}, 원인: {reason})"
            )

        self.current_step = 0
        self.prev_error_v = 0.0
        self.prev_error_w = 0.0
        self.prev_distance = None
        self.cumulative_reward = 0.0
        self._episode_started = True
        self._term_reason = "timeout"

        # 1. 속도 정지 명령
        stop_msg = Twist()
        for _ in range(3):
            self.node.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.01)

        # 2. 스폰 웨이포인트 랜덤 선택
        spawn_idx = np.random.randint(len(SAFE_WAYPOINTS))
        self.spawn_x, self.spawn_y = SAFE_WAYPOINTS[spawn_idx]

        # 3. 목표 지점 선택 (커리큘럼 러닝: 0.5m ~ 1.5m 사이의 가까운 곳 우선 선택)
        candidates = [
            wp
            for i, wp in enumerate(SAFE_WAYPOINTS)
            if i != spawn_idx
            and 0.5 <= math.hypot(wp[0] - self.spawn_x, wp[1] - self.spawn_y) <= 1.5
        ]

        if not candidates:
            candidates = [wp for i, wp in enumerate(SAFE_WAYPOINTS) if i != spawn_idx]

        self.goal_x, self.goal_y = candidates[np.random.randint(len(candidates))]

        # 4. 랜덤 방향 설정
        yaw = np.random.uniform(-math.pi, math.pi)
        self.spawn_yaw = yaw  # 실제 텔레포트 heading 저장
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        # 5. 텔레포트
        while not self._teleport(self.spawn_x, self.spawn_y, qw, qz):
            print("[Pinky] 텔레포트 실패, 재시도 중...")
            time.sleep(0.5)

        # 6. 센서 데이터 수신 대기 + odom 안정화
        time.sleep(0.3)  # 물리 엔진 안정화 대기
        for _wait in range(20):  # 최대 20 * 0.05s = 1.0s 추가 대기
            ox, oy = self.node.x, self.node.y
            time.sleep(0.05)
            ox2, oy2 = self.node.x, self.node.y
            if abs(ox2 - ox) < 0.001 and abs(oy2 - oy) < 0.001:
                break
        self.odom_start_x = self.node.x
        self.odom_start_y = self.node.y

        # 7. Heading offset 계산
        #    DiffDrive의 내부 heading은 텔레포트로 변경되지 않으므로
        #    실제 heading(spawn_yaw)과 DiffDrive heading(odom yaw)의 차이를 보정값으로 저장
        odom_yaw_at_spawn = self.node.yaw
        self.heading_offset = self.spawn_yaw - odom_yaw_at_spawn
        # [-π, π] 범위로 정규화
        self.heading_offset = (self.heading_offset + math.pi) % (2 * math.pi) - math.pi

        state, distance = self.get_state()
        self.prev_distance = distance

        print(
            f"[Pinky] 새 에피소드: spawn=({self.spawn_x:.1f}, {self.spawn_y:.1f}) "
            f"-> goal=({self.goal_x:.1f}, {self.goal_y:.1f}), dist={distance:.2f}m"
        )

        return state

    def step(self, action):
        self.current_step += 1

        # ── 1. 행동 매핑 및 PID 제어 ──
        # 선형 보간: [-1.0, 1.0] -> [0.0, 1.0] 변환 후 최대 속도 곱하기 (후진 금지, 기울기 손실 방지)
        target_v = float((action[0] + 1.0) / 2.0 * 0.5)
        target_w = float(action[1] * 1.5)

        error_v = target_v - self.node.current_v
        error_w = target_w - self.node.current_w

        cmd_v = (
            target_v + self.KP_V * error_v + self.KD_V * (error_v - self.prev_error_v)
        )
        cmd_w = (
            target_w + self.KP_W * error_w + self.KD_W * (error_w - self.prev_error_w)
        )

        self.prev_error_v = error_v
        self.prev_error_w = error_w

        msg = Twist()
        msg.linear.x = float(np.clip(cmd_v, 0.0, 0.5))
        msg.angular.z = float(np.clip(cmd_w, -1.5, 1.5))
        self.node.cmd_vel_pub.publish(msg)

        time.sleep(0.05)

        state, distance = self.get_state()

        # 스캔 데이터 미터(m) 단위 복원
        scan = self.node.scan_data
        min_scan_norm = float(np.min(scan))
        min_scan_m = min_scan_norm * self.node.MAX_LIDAR_DIST

        goal_angle = state[25] * math.pi
        abs_angle = abs(goal_angle)

        reward = 0.0
        terminated = False

        # ── 2. 과학적 Reward Shaping (우회 경로 학습 허용) ──
        # [A] 장애물 위험도 계산 (0.6m부터 긴장 시작, 0.2m 이내 최대 위험)
        danger_level = np.clip((0.6 - min_scan_m) / 0.4, 0.0, 1.0)
        weight_avoidance = danger_level
        weight_navigation = 1.0 - danger_level

        # 기본 생존 패널티 + "벽 앞 체류" 패널티 (뜨거운 바닥 효과)
        # 위험 구역에 가만히 서있으면 계속 감점을 받아 어떻게든 몸을 틀어 탈출하도록 강제함
        reward = -0.05 - (danger_level * 0.1)

        # [B] 본능 1: 목표 지향 (안전할 때 활성화)
        if weight_navigation > 0:
            alignment = math.cos(goal_angle)

            # [핵심 수정 1] 장애물을 우회하기 위해 목표를 등질 때(alignment < 0) 감점을 주지 않음!
            if alignment > 0:
                nav_reward = target_v * alignment * 2.5
            else:
                nav_reward = 0.0

            # 주변 1.0m 이내에 아무것도 없는 완전한 공터에서만 불필요한 회전(와이퍼 주행) 감점
            if min_scan_m > 1.0:
                nav_reward -= abs(target_w) * 0.1

            reward += weight_navigation * nav_reward

        # [C] 본능 2: 장애물 회피 (위험할 때 활성화)
        if weight_avoidance > 0:
            # 위험 구역에서 전진(v)하면 무조건 강력한 패널티!
            avoid_reward = -target_v * 3.0
            reward += weight_avoidance * avoid_reward

        # [D] 목표 접근 보상
        if self.prev_distance is not None:
            approach = self.prev_distance - distance
            # [핵심 수정 2] 우회를 위해 목표와 잠시 멀어지는 것(approach < 0)을 감점하지 않음!
            # 오직 목표와 가까워질 때만 칭찬하여 자연스러운 우회(Detour)를 장려함
            if approach > 0:
                reward += approach * 25.0 * weight_navigation
        self.prev_distance = distance

        # ── 3. 종료 조건 및 패널티 ──
        if distance < 0.15:  # 목표 도달 (성공)
            reward = 100.0
            terminated = True
            self._term_reason = "goal"
            # 디버그: goal 판정 시 위치 정보 출력 (보정 검증용)
            dbg_wx, dbg_wy = self._get_world_position()
            print(
                f"[Pinky] 🎯 GOAL | "
                f"pos=({dbg_wx:.3f}, {dbg_wy:.3f}), "
                f"goal=({self.goal_x:.1f}, {self.goal_y:.1f}), "
                f"dist={distance:.3f}m, "
                f"h_offset={math.degrees(self.heading_offset):.1f}°, "
                f"step={self.current_step}"
            )
        elif min_scan_m < 0.15:  # 충돌 (실패)
            reward = -100.0
            terminated = True
            self._term_reason = f"collision(min={min_scan_m:.3f}m)"
        else:
            world_x, world_y = self._get_world_position()
            if abs(world_x) > 3.5 or abs(world_y) > 3.0:  # 작업 공간 이탈 (실패)
                reward = -100.0
                terminated = True
                self._term_reason = f"boundary({world_x:.1f},{world_y:.1f})"

        truncated = self.current_step >= self.max_steps
        done = terminated or truncated
        self.cumulative_reward += float(reward)

        reward = float(reward)
        done = bool(done)
        info = {"terminated": bool(terminated)}

        return state, reward, done, info

    def render(self):
        pass

    def close(self):
        msg = Twist()
        self.node.cmd_vel_pub.publish(msg)
        self.node.destroy_node()
        rclpy.shutdown()

    def environment_spec(self):
        return EnvironmentSpec(
            action_shape=self.action_shape,
            action_dtype=np.float32,
            action_high=1.0,
            action_low=-1.0,
            action_size=self.action_shape[0],
            b_continuous_action=True,
            state_shape=self.state_shape,
            state_dtype=np.float32,
        )

    def select_action(self):
        return np.random.uniform(-1.0, 1.0, size=self.action_shape).astype(np.float32)

    def max_episode_limit(self):
        return self.max_steps
