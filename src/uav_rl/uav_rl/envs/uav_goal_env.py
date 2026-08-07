from __future__ import annotations

from dataclasses import dataclass

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # pragma: no cover - keeps smoke tests usable before installing RL deps.
    class _Box:
        def __init__(self, low, high, shape=None, dtype=np.float32):
            self.low = np.full(shape, low, dtype=dtype) if shape is not None else np.asarray(low, dtype=dtype)
            self.high = np.full(shape, high, dtype=dtype) if shape is not None else np.asarray(high, dtype=dtype)
            self.shape = self.low.shape
            self.dtype = dtype

        def sample(self):
            return np.random.uniform(self.low, self.high).astype(self.dtype)

    class _Env:
        metadata = {}

    class _Spaces:
        Box = _Box

    class _Gym:
        Env = _Env

    gym = _Gym()
    spaces = _Spaces()

from uav_rl.sim.math_utils import quat_from_euler, quat_to_euler
from uav_rl.sim.multirotor import MultirotorModel
from uav_rl.sim.sixdof import SixDofModel, UavParams, UavState, Wrench


@dataclass
class UavGoalEnvConfig:
    # RL 환경의 시간 해상도. policy는 0.05초마다 action을 내고, 내부 동역학은 0.01초 RK4로 적분한다.
    sim_dt: float = 0.01
    policy_dt: float = 0.05

    # 한 episode의 성공/실패/시간 제한 조건.
    max_episode_s: float = 12.0
    goal_radius: float = 0.35
    max_abs_xy: float = 12.0
    min_z: float = 0.0
    max_z: float = 6.0

    # 목표점 sampling 범위. train/eval의 --task 옵션이 이 값을 바꿔 curriculum을 구성한다.
    goal_low: tuple[float, float, float] = (-3.0, 2.0, 1.5)
    goal_high: tuple[float, float, float] = (3.0, 5.0, 2.3)
    init_xy_std: float = 0.15
    init_z: float = 0.0

    # PPO action을 속도/yaw-rate 명령으로 해석할 때 사용하는 최대 명령 크기.
    max_vxy_cmd: float = 2.5
    max_vz_cmd: float = 2.5

    # Residual RL 설정. 기본 guidance 명령 위에 PPO가 제한된 보정값만 더한다.
    use_guidance_prior: bool = True
    guidance_max_vxy: float = 1.5
    guidance_max_vz: float = 1.0
    guidance_k_xy: float = 0.35
    guidance_k_z: float = 0.8
    residual_action_scale: float = 0.4
    max_yaw_rate_cmd: float = 1.0

    # 저수준 velocity/attitude controller 및 actuator 보호용 제한값.
    max_tilt_deg: float = 25.0
    max_acc_xy: float = 5.0
    max_acc_z: float = 6.0
    kp_vel_xy: float = 2.2
    kp_vel_z: float = 3.5
    kp_att_rp: float = 4.0
    kd_att_rp: float = 0.45
    kp_yaw_rate: float = 0.08
    moment_max_rp: float = 0.80
    moment_max_yaw: float = 0.20
    thrust_min: float = 0.0
    thrust_max: float = 55.0
    wind_randomization_n: float = 0.0
    hover_before_xy_m: float = 1.3

    # 학습 안정화를 위한 action smoothing과 reward shaping 계수.
    action_smoothing: float = 0.65
    success_bonus: float = 60.0
    crash_penalty: float = 40.0
    timeout_distance_penalty: float = 2.0


class UavGoalEnv(gym.Env):
    """Single-UAV goal reaching environment.

    Action:
        normalized residual [vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd].

    Observation:
        normalized [goal_error, velocity, quaternion, angular_rate, distance, last_action].

    By default, a simple goal-directed guidance prior creates a nominal velocity
    command and the policy learns a bounded correction around it. A built-in
    low-level controller converts the result to a desired body wrench, then a
    Quad-X allocator and first-order motor lag feed the 6-DOF model.
    """

    metadata = {"render_modes": []}

    def __init__(self, config: UavGoalEnvConfig | None = None, seed: int | None = None):
        super().__init__()
        self.cfg = config if config is not None else UavGoalEnvConfig()
        self.rng = np.random.default_rng(seed)

        # PPO가 내는 action은 [-1, 1]로 정규화된 4차원 벡터다.
        # observation은 목표 오차, 속도, 자세 quaternion, 각속도, 거리, 이전 action을 묶은 18차원 벡터다.
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-10.0, high=10.0, shape=(18,), dtype=np.float32)

        # C++ simulator와 같은 개념의 6-DOF dynamics와 F450 actuator model을 Python 학습 환경에 붙인다.
        self.params = UavParams()
        self.dynamics = SixDofModel(self.params)
        self.actuator = MultirotorModel()
        self.state = UavState.zero()
        self.goal = np.array([0.0, 0.0, 2.0], dtype=np.float64)
        self.last_action = np.zeros(4, dtype=np.float64)
        self.t = 0.0
        self.prev_dist = 0.0
        self.steps_per_policy = max(1, int(round(self.cfg.policy_dt / self.cfg.sim_dt)))

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        # Gymnasium 규약: reset()은 새 episode의 초기 observation과 info를 반환한다.
        if seed is not None:
            self.rng = np.random.default_rng(seed)

        # 목표점은 task 난이도별 범위에서 무작위 sampling한다. options["goal"]로 고정 목표 테스트도 가능하다.
        goal_low = np.asarray(self.cfg.goal_low, dtype=np.float64)
        goal_high = np.asarray(self.cfg.goal_high, dtype=np.float64)
        if options and "goal" in options:
            self.goal = np.asarray(options["goal"], dtype=np.float64)
        else:
            self.goal = self.rng.uniform(goal_low, goal_high)

        # 매 episode마다 드론은 지면 근처에서 시작하고, yaw는 무작위로 둬 특정 방향에만 과적합되지 않게 한다.
        self.state = UavState.zero()
        self.state.p[:2] = self.rng.normal(0.0, self.cfg.init_xy_std, size=2)
        self.state.p[2] = self.cfg.init_z
        self.state.q = quat_from_euler(0.0, 0.0, self.rng.uniform(-np.pi, np.pi))
        self.state.v[:] = 0.0
        self.state.w[:] = 0.0
        self.last_action[:] = 0.0
        self.t = 0.0
        self.actuator.reset()

        # 추후 domain randomization을 위한 외란 항목. 기본은 0이라 바람 없이 학습한다.
        wind = self.cfg.wind_randomization_n
        if wind > 0.0:
            self.params.wind_force = self.rng.uniform(-wind, wind, size=3)
            self.params.wind_force[2] = 0.0
        else:
            self.params.wind_force = np.zeros(3, dtype=np.float64)

        self.prev_dist = self._distance_to_goal()
        return self._obs(), self._info()

    def step(self, action):
        # Gymnasium 규약: step(action)은 action 적용 후 obs, reward, terminated, truncated, info를 반환한다.
        raw_action = np.asarray(action, dtype=np.float64)
        raw_action = np.clip(raw_action, -1.0, 1.0)

        # policy 출력이 급격히 바뀌면 자세/로터 명령이 흔들리므로 1차 smoothing을 적용한다.
        action = self.cfg.action_smoothing * self.last_action + (1.0 - self.cfg.action_smoothing) * raw_action
        action_delta = np.linalg.norm(action - self.last_action)

        prev_dist = self._distance_to_goal()

        # policy_dt 동안 같은 action을 유지하고, 내부 6-DOF 동역학은 더 작은 sim_dt로 여러 번 적분한다.
        for _ in range(self.steps_per_policy):
            desired_wrench = self._action_to_wrench(action)
            applied_wrench = self.actuator.update(desired_wrench, self.cfg.sim_dt)
            self.state = self.dynamics.rk4_step(self.state, applied_wrench, self.cfg.sim_dt)
            self._apply_ground_constraint()
            self.t += self.cfg.sim_dt

        dist = self._distance_to_goal()

        # terminated는 성공/충돌처럼 episode가 의미상 끝난 경우, truncated는 시간 제한으로 끝난 경우다.
        terminated = dist < self.cfg.goal_radius
        crashed = (
            abs(self.state.p[0]) > self.cfg.max_abs_xy
            or abs(self.state.p[1]) > self.cfg.max_abs_xy
            or self.state.p[2] < self.cfg.min_z - 1e-6
            or self.state.p[2] > self.cfg.max_z
            or np.linalg.norm(self.state.w) > 8.0
        )
        truncated = self.t >= self.cfg.max_episode_s

        reward = self._reward(
            prev_dist=prev_dist,
            dist=dist,
            action=action,
            action_delta=action_delta,
            crashed=crashed,
            reached=terminated,
            truncated=truncated,
        )
        self.last_action = action.copy()
        if crashed:
            terminated = True

        return self._obs(), float(reward), bool(terminated), bool(truncated), self._info(crashed=crashed)

    def _action_to_wrench(self, action: np.ndarray) -> Wrench:
        # PPO action을 바로 로터 추력으로 쓰지 않고, 먼저 고수준 속도 명령으로 해석한다.
        cfg = self.cfg

        if cfg.use_guidance_prior:
            v_cmd = self._guidance_velocity()
            # Residual RL: PPO adjusts a stable guidance command instead of
            # discovering takeoff and goal-seeking entirely from random actions.
            residual = np.array(
                [
                    action[0] * cfg.max_vxy_cmd,
                    action[1] * cfg.max_vxy_cmd,
                    action[2] * cfg.max_vz_cmd,
                ],
                dtype=np.float64,
            )
            v_cmd += cfg.residual_action_scale * residual
        else:
            v_cmd = np.array(
                [
                    action[0] * cfg.max_vxy_cmd,
                    action[1] * cfg.max_vxy_cmd,
                    action[2] * cfg.max_vz_cmd,
                ],
                dtype=np.float64,
            )

        # Takeoff curriculum built into the environment: before reaching a safe
        # altitude, suppress aggressive XY velocity commands.
        if self.state.p[2] < cfg.hover_before_xy_m:
            scale = np.clip((self.state.p[2] - 0.4) / max(1e-6, cfg.hover_before_xy_m - 0.4), 0.0, 1.0)
            v_cmd[0:2] *= scale

        acc_cmd = np.array(
            [
                cfg.kp_vel_xy * (v_cmd[0] - self.state.v[0]),
                cfg.kp_vel_xy * (v_cmd[1] - self.state.v[1]),
                cfg.kp_vel_z * (v_cmd[2] - self.state.v[2]),
            ],
            dtype=np.float64,
        )
        acc_cmd[0:2] = self._limit_norm(acc_cmd[0:2], cfg.max_acc_xy)
        acc_cmd[2] = np.clip(acc_cmd[2], -cfg.max_acc_z, cfg.max_acc_z)

        # desired acceleration을 만들기 위해 필요한 world-frame thrust 방향과 크기를 계산한다.
        thrust_world = self.params.mass * (acc_cmd - np.array([0.0, 0.0, -self.params.g], dtype=np.float64))
        thrust_mag = np.clip(np.linalg.norm(thrust_world), cfg.thrust_min, cfg.thrust_max)
        zb_des = thrust_world / max(np.linalg.norm(thrust_world), 1e-6)

        # world-frame thrust 방향을 현재 yaw 기준 roll/pitch 목표로 바꾼다.
        # 이 yaw 보정을 빼면 초기 yaw에 따라 같은 world velocity 명령이 엉뚱한 body tilt로 변환될 수 있다.
        max_tilt = np.deg2rad(cfg.max_tilt_deg)
        roll, pitch, yaw = quat_to_euler(self.state.q)
        cy = np.cos(yaw)
        sy = np.sin(yaw)
        z_safe = max(zb_des[2], 1e-6)
        desired_pitch = np.arctan2(cy * zb_des[0] + sy * zb_des[1], z_safe)
        desired_roll = np.arctan2(sy * zb_des[0] - cy * zb_des[1], z_safe)
        desired_roll = np.clip(desired_roll, -max_tilt, max_tilt)
        desired_pitch = np.clip(desired_pitch, -max_tilt, max_tilt)

        # 간단한 PD attitude loop로 body moment를 만든다.
        moment_x = cfg.kp_att_rp * (desired_roll - roll) - cfg.kd_att_rp * self.state.w[0]
        moment_y = cfg.kp_att_rp * (desired_pitch - pitch) - cfg.kd_att_rp * self.state.w[1]

        yaw_rate_cmd = action[3] * cfg.max_yaw_rate_cmd
        moment_z = cfg.kp_yaw_rate * (yaw_rate_cmd - self.state.w[2])

        moment = np.array(
            [
                np.clip(moment_x, -cfg.moment_max_rp, cfg.moment_max_rp),
                np.clip(moment_y, -cfg.moment_max_rp, cfg.moment_max_rp),
                np.clip(moment_z, -cfg.moment_max_yaw, cfg.moment_max_yaw),
            ],
            dtype=np.float64,
        )
        return Wrench(
            thrust_body=np.array([0.0, 0.0, thrust_mag], dtype=np.float64),
            moment_body=moment,
        )

    def _guidance_velocity(self) -> np.ndarray:
        # 목표점 방향으로 향하는 nominal velocity command다.
        # PPO는 이 기본 명령을 대체하지 않고 residual correction만 학습한다.
        cfg = self.cfg
        e = self.goal - self.state.p
        horizontal_error = e[:2]
        horizontal_dist = np.linalg.norm(horizontal_error)

        v_cmd = np.zeros(3, dtype=np.float64)
        if horizontal_dist > 1e-6:
            horizontal_dir = horizontal_error / horizontal_dist
            v_cmd[:2] = horizontal_dir * min(cfg.guidance_max_vxy, cfg.guidance_k_xy * horizontal_dist)
        v_cmd[2] = np.clip(cfg.guidance_k_z * e[2], -cfg.guidance_max_vz, cfg.guidance_max_vz)
        return v_cmd

    def _obs(self) -> np.ndarray:
        # neural network 입력이 되도록 물리량을 대략 비슷한 크기로 정규화한다.
        e = self.goal - self.state.p
        dist = np.linalg.norm(e)
        obs = np.concatenate(
            [
                e / 10.0,
                self.state.v / 5.0,
                self.state.q,
                self.state.w / 5.0,
                np.array([dist / 10.0], dtype=np.float64),
                self.last_action,
            ]
        )
        return obs.astype(np.float32)

    def _reward(
        self,
        prev_dist: float,
        dist: float,
        action: np.ndarray,
        action_delta: float,
        crashed: bool,
        reached: bool,
        truncated: bool,
    ) -> float:
        # reward는 목표에 가까워지는 progress를 가장 크게 보상하고,
        # 거리/진동/큰 action/충돌/timeout을 벌점으로 둔다.
        progress = prev_dist - dist
        tilt = np.linalg.norm(quat_to_euler(self.state.q)[0:2])
        effort = np.linalg.norm(self.actuator.rotor_thrusts) / 30.0

        reward = 14.0 * progress
        reward += -0.35 * dist
        reward += 0.8 * np.exp(-2.0 * dist)
        reward += -0.03 * np.linalg.norm(self.state.v)
        reward += -0.02 * np.linalg.norm(self.state.w)
        reward += -0.04 * tilt
        reward += -0.01 * effort
        reward += -0.10 * action_delta
        reward += self.cfg.success_bonus if reached else 0.0
        reward += -self.cfg.crash_penalty if crashed else 0.0
        reward += -self.cfg.timeout_distance_penalty * dist if truncated and not reached else 0.0
        return reward

    def _apply_ground_constraint(self) -> None:
        # 지면 아래로 내려가는 수치 적분 결과를 막는 간단한 ground constraint.
        if self.state.p[2] < 0.0:
            self.state.p[2] = 0.0
            if self.state.v[2] < 0.0:
                self.state.v[2] = 0.0

    def _distance_to_goal(self) -> float:
        return float(np.linalg.norm(self.goal - self.state.p))

    @staticmethod
    def _limit_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
        n = np.linalg.norm(v)
        if n > max_norm > 0.0:
            return v * (max_norm / n)
        return v

    def _info(self, crashed: bool = False) -> dict:
        # rollout/evaluation 로그에서 사람이 확인할 수 있는 물리량을 담는다.
        roll, pitch, yaw = quat_to_euler(self.state.q)
        return {
            "t": self.t,
            "goal": self.goal.copy(),
            "position": self.state.p.copy(),
            "velocity": self.state.v.copy(),
            "rpy": np.array([roll, pitch, yaw], dtype=np.float64),
            "distance": self._distance_to_goal(),
            "crashed": crashed,
            "rotor_thrusts": self.actuator.rotor_thrusts.copy(),
            "applied_action": self.last_action.copy(),
        }
