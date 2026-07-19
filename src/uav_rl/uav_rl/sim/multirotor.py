from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from uav_rl.sim.sixdof import Wrench


@dataclass
class MultirotorConfig:
    motor_tau: float = 0.04
    yaw_torque_to_thrust: float = 0.016
    rotor_positions_body: np.ndarray = None
    rotor_yaw_signs: np.ndarray = None
    rotor_thrust_min: np.ndarray = None
    rotor_thrust_max: np.ndarray = None

    def __post_init__(self) -> None:
        if self.rotor_positions_body is None:
            arm = 0.225
            xy = arm / np.sqrt(2.0)
            self.rotor_positions_body = np.array(
                [
                    [xy, xy, 0.0],
                    [xy, -xy, 0.0],
                    [-xy, -xy, 0.0],
                    [-xy, xy, 0.0],
                ],
                dtype=np.float64,
            )
        if self.rotor_yaw_signs is None:
            self.rotor_yaw_signs = np.array([1.0, -1.0, 1.0, -1.0], dtype=np.float64)
        n = self.rotor_positions_body.shape[0]
        if self.rotor_thrust_min is None:
            self.rotor_thrust_min = np.zeros(n, dtype=np.float64)
        if self.rotor_thrust_max is None:
            self.rotor_thrust_max = np.full(n, 15.0, dtype=np.float64)


class MultirotorModel:
    def __init__(self, config: MultirotorConfig | None = None):
        self.config = config if config is not None else MultirotorConfig()
        self.rotor_thrusts = np.zeros(self.config.rotor_positions_body.shape[0], dtype=np.float64)
        self.last_cmd = self.rotor_thrusts.copy()

    def reset(self) -> None:
        self.rotor_thrusts[:] = 0.0
        self.last_cmd = self.rotor_thrusts.copy()

    def update(self, desired_wrench: Wrench, dt: float) -> Wrench:
        thrust_cmd = self._allocate(desired_wrench)
        self.last_cmd = thrust_cmd.copy()
        tau = self.config.motor_tau
        alpha = np.clip(dt / tau, 0.0, 1.0) if tau > 1e-6 else 1.0
        self.rotor_thrusts += alpha * (thrust_cmd - self.rotor_thrusts)
        return self._to_wrench(self.rotor_thrusts)

    def _allocate(self, desired_wrench: Wrench) -> np.ndarray:
        cfg = self.config
        cols = []
        for pos, yaw_sign in zip(cfg.rotor_positions_body, cfg.rotor_yaw_signs):
            cols.append([1.0, pos[1], -pos[0], yaw_sign * cfg.yaw_torque_to_thrust])
        b = np.asarray(cols, dtype=np.float64).T
        cmd = np.array(
            [
                max(0.0, desired_wrench.thrust_body[2]),
                desired_wrench.moment_body[0],
                desired_wrench.moment_body[1],
                desired_wrench.moment_body[2],
            ],
            dtype=np.float64,
        )
        bbt = b @ b.T + np.eye(4) * 1e-8
        thrusts = b.T @ np.linalg.solve(bbt, cmd)
        return np.clip(thrusts, cfg.rotor_thrust_min, cfg.rotor_thrust_max)

    def _to_wrench(self, rotor_thrusts: np.ndarray) -> Wrench:
        thrust_body = np.zeros(3, dtype=np.float64)
        moment_body = np.zeros(3, dtype=np.float64)
        cfg = self.config
        for thrust, pos, yaw_sign in zip(rotor_thrusts, cfg.rotor_positions_body, cfg.rotor_yaw_signs):
            force = np.array([0.0, 0.0, thrust], dtype=np.float64)
            thrust_body += force
            moment_body += np.cross(pos, force)
            moment_body[2] += yaw_sign * cfg.yaw_torque_to_thrust * thrust
        return Wrench(thrust_body=thrust_body, moment_body=moment_body)

