from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from uav_rl.sim.math_utils import quat_multiply, quat_normalize, rotate_body_to_world


@dataclass
class UavParams:
    mass: float = 2.0
    inertia: np.ndarray = None
    g: float = 9.80665
    use_drag: bool = True
    k1: float = 0.15
    k2: float = 0.02
    wind_force: np.ndarray = None

    def __post_init__(self) -> None:
        if self.inertia is None:
            self.inertia = np.array([0.02, 0.02, 0.04], dtype=np.float64)
        if self.wind_force is None:
            self.wind_force = np.zeros(3, dtype=np.float64)


@dataclass
class UavState:
    p: np.ndarray
    v: np.ndarray
    q: np.ndarray
    w: np.ndarray

    @staticmethod
    def zero() -> "UavState":
        return UavState(
            p=np.zeros(3, dtype=np.float64),
            v=np.zeros(3, dtype=np.float64),
            q=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            w=np.zeros(3, dtype=np.float64),
        )

    def copy(self) -> "UavState":
        return UavState(self.p.copy(), self.v.copy(), self.q.copy(), self.w.copy())


@dataclass
class Wrench:
    thrust_body: np.ndarray
    moment_body: np.ndarray

    @staticmethod
    def zero() -> "Wrench":
        return Wrench(np.zeros(3, dtype=np.float64), np.zeros(3, dtype=np.float64))


@dataclass
class Deriv:
    dp: np.ndarray
    dv: np.ndarray
    dq: np.ndarray
    dw: np.ndarray


class SixDofModel:
    def __init__(self, params: UavParams | None = None):
        self.params = params if params is not None else UavParams()

    def derivatives(self, state: UavState, wrench: Wrench) -> Deriv:
        p = self.params
        thrust_world = rotate_body_to_world(state.q, wrench.thrust_body)
        gravity = np.array([0.0, 0.0, -p.g], dtype=np.float64)

        a_drag = np.zeros(3, dtype=np.float64)
        if p.use_drag:
            vnorm = np.linalg.norm(state.v)
            f_drag = -p.k1 * state.v - p.k2 * vnorm * state.v
            a_drag = f_drag / p.mass

        dv = thrust_world / p.mass + gravity + a_drag + p.wind_force / p.mass
        omega = np.array([0.0, state.w[0], state.w[1], state.w[2]], dtype=np.float64)
        dq = 0.5 * quat_multiply(state.q, omega)

        iw = p.inertia * state.w
        wx_iw = np.cross(state.w, iw)
        dw = (wrench.moment_body - wx_iw) / p.inertia

        return Deriv(dp=state.v.copy(), dv=dv, dq=dq, dw=dw)

    @staticmethod
    def _add_scaled(state: UavState, deriv: Deriv, h: float) -> UavState:
        out = state.copy()
        out.p = out.p + deriv.dp * h
        out.v = out.v + deriv.dv * h
        out.q = quat_normalize(out.q + deriv.dq * h)
        out.w = out.w + deriv.dw * h
        return out

    def rk4_step(self, state: UavState, wrench: Wrench, dt: float) -> UavState:
        k1 = self.derivatives(state, wrench)
        k2 = self.derivatives(self._add_scaled(state, k1, dt * 0.5), wrench)
        k3 = self.derivatives(self._add_scaled(state, k2, dt * 0.5), wrench)
        k4 = self.derivatives(self._add_scaled(state, k3, dt), wrench)

        out = state.copy()
        out.p = out.p + (k1.dp + 2.0 * k2.dp + 2.0 * k3.dp + k4.dp) * (dt / 6.0)
        out.v = out.v + (k1.dv + 2.0 * k2.dv + 2.0 * k3.dv + k4.dv) * (dt / 6.0)
        out.q = quat_normalize(out.q + (k1.dq + 2.0 * k2.dq + 2.0 * k3.dq + k4.dq) * (dt / 6.0))
        out.w = out.w + (k1.dw + 2.0 * k2.dw + 2.0 * k3.dw + k4.dw) * (dt / 6.0)
        return out

