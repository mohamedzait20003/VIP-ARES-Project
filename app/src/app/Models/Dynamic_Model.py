from __future__ import annotations

import numpy as np
import casadi as ca
from dataclasses import dataclass


@dataclass(frozen=True)
class DroneParameters:
    # Match SDF inertial values (scaled 1.5kg iris → 1.95kg, factor 1.3)
    mass: float = 1.95
    gravity: float = 9.81
    Ixx: float = 0.0451832
    Iyy: float = 0.0596608
    Izz: float = 0.0910
    thrust_max: float = 30.0
    torque_max: float = 2.0


class Dynamic_Model:
    STATE_DIM = 12
    CONTROL_DIM = 4

    def __init__(self, params: DroneParameters):
        self._params = params

        x  = ca.SX.sym('x', self.STATE_DIM)
        u  = ca.SX.sym('u', self.CONTROL_DIM)
        dt = ca.SX.sym('dt')

        xdot = self._build_xdot(x, u)
        self._f = ca.Function('f', [x, u], [xdot], ['x', 'u'], ['xdot'])

        k1 = self._f(x, u)
        k2 = self._f(x + (dt / 2.0) * k1, u)
        k3 = self._f(x + (dt / 2.0) * k2, u)
        k4 = self._f(x + dt * k3, u)
        x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        self._rk4 = ca.Function('rk4', [x, u, dt], [x_next], ['x', 'u', 'dt'], ['x_next'])

    @property
    def state_dim(self) -> int:
        return self.STATE_DIM

    @property
    def control_dim(self) -> int:
        return self.CONTROL_DIM

    @property
    def params(self) -> DroneParameters:
        return self._params

    def continuous_dynamics(self, x, u):
        return self._f(x, u)

    def rk4_step(self, x, u, dt):
        return self._rk4(x, u, dt)

    def _build_xdot(self, x: ca.SX, u: ca.SX) -> ca.SX:
        p = self._params

        vx, vy, vz = x[3], x[4], x[5]
        phi, theta, psi = x[6], x[7], x[8]
        p_rate, q_rate, r_rate = x[9], x[10], x[11]

        T, tau_phi, tau_theta, tau_psi = u[0], u[1], u[2], u[3]

        cphi = ca.cos(phi); sphi = ca.sin(phi)
        cth  = ca.cos(theta);  sth  = ca.sin(theta)
        cpsi = ca.cos(psi); spsi = ca.sin(psi)

        ax = (T / p.mass) * (cpsi * sth * cphi + spsi * sphi)
        ay = (T / p.mass) * (spsi * sth * cphi - cpsi * sphi)
        az = (T / p.mass) * (cth * cphi) - p.gravity

        phi_dot   = p_rate + (q_rate * sphi + r_rate * cphi) * sth / cth
        theta_dot = q_rate * cphi - r_rate * sphi
        psi_dot   = (q_rate * sphi + r_rate * cphi) / cth

        p_dot = ((p.Iyy - p.Izz) * q_rate * r_rate + tau_phi)   / p.Ixx
        q_dot = ((p.Izz - p.Ixx) * p_rate * r_rate + tau_theta) / p.Iyy
        r_dot = ((p.Ixx - p.Iyy) * p_rate * q_rate + tau_psi)   / p.Izz

        return ca.vertcat(
            vx, vy, vz,
            ax, ay, az,
            phi_dot, theta_dot, psi_dot,
            p_dot, q_dot, r_dot,
        )
