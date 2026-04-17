from __future__ import annotations

import numpy as np
import casadi as ca
from typing import Optional, Tuple

from app.Config import NMPCConfig
from app.Models import DroneParameters, Dynamic_Model

class NMPC_Controller:
    def __init__(
        self,
        config: NMPCConfig,
        drone_params: Optional[DroneParameters] = None,
    ) -> None:
        self._cfg      = config
        self._dynamics = Dynamic_Model(drone_params or DroneParameters())
        self._nx       = self._dynamics.state_dim
        self._nu       = self._dynamics.control_dim

        self._solver, self._lbg, self._ubg = self._build_solver()

        self._x_ws: Optional[np.ndarray] = None
        self._u_ws: Optional[np.ndarray] = None

    def _validate_inputs(self, state: np.ndarray, traj: np.ndarray) -> None:
        if state.shape != (self._nx,):
            raise ValueError(f"Expected state shape ({self._nx},), got {state.shape}")

        expected = (self._cfg.horizon + 1, self._nx)
        if traj.shape != expected:
            raise ValueError(f"Expected trajectory shape {expected}, got {traj.shape}")

    def _make_Q(self) -> ca.DM:
        c = self._cfg
        diag = np.array([
            c.Q_pos, c.Q_pos, c.Q_pos,
            c.Q_vel, c.Q_vel, c.Q_vel,
            c.Q_angle, c.Q_angle, c.Q_angle,
            c.Q_rate, c.Q_rate, c.Q_rate,
        ])
        return ca.DM(np.diag(diag))

    def _make_R(self) -> ca.DM:
        c = self._cfg
        diag = np.array([
            c.R_thrust, c.R_torque, c.R_torque, c.R_torque
        ])
        return ca.DM(np.diag(diag))

    def _build_lbx(self) -> np.ndarray:
        N, nx, nu = self._cfg.horizon, self._nx, self._nu
        x_lb = np.full((nx, N + 1), -np.inf)
        x_lb[6:9, :] = -self._cfg.angle_max

        u_lb       = np.full((nu, N), -self._cfg.torque_max)
        u_lb[0, :] = self._cfg.thrust_min
        return np.concatenate([x_lb.flatten('F'), u_lb.flatten('F')])

    def _build_ubx(self) -> np.ndarray:
        N, nx, nu = self._cfg.horizon, self._nx, self._nu
        x_ub = np.full((nx, N + 1), np.inf)
        x_ub[6:9, :] = self._cfg.angle_max

        u_ub       = np.full((nu, N), self._cfg.torque_max)
        u_ub[0, :] = self._cfg.thrust_max
        return np.concatenate([x_ub.flatten('F'), u_ub.flatten('F')])

    def _build_warm_start(self) -> np.ndarray:
        N, nx, nu = self._cfg.horizon, self._nx, self._nu

        if self._x_ws is None:
            x_ws       = np.zeros((nx, N + 1))
            u_ws       = np.zeros((nu, N))
            u_ws[0, :] = self._dynamics.params.mass * self._dynamics.params.gravity
        else:
            x_ws = np.hstack([self._x_ws[:, 1:], self._x_ws[:, -1:]])
            u_ws = np.hstack([self._u_ws[:, 1:], self._u_ws[:, -1:]])

        return np.concatenate([x_ws.flatten('F'), u_ws.flatten('F')])

    def _shift_warm_start(self, x_opt: np.ndarray, u_opt: np.ndarray) -> None:
        self._x_ws = x_opt
        self._u_ws = u_opt

    def _pack_parameters(self, state: np.ndarray, traj: np.ndarray) -> np.ndarray:
        return np.concatenate([state, traj.flatten()])

    def _unpack_solution(self, sol_vec: ca.DM) -> Tuple[np.ndarray, np.ndarray]:
        N, nx, nu = self._cfg.horizon, self._nx, self._nu
        arr    = np.array(sol_vec).flatten()
        n_x    = nx * (N + 1)
        x_opt  = arr[:n_x].reshape(nx, N + 1, order='F')
        u_opt  = arr[n_x:].reshape(nu, N, order='F')
        return x_opt, u_opt

    def _build_solver(self) -> Tuple[ca.Function, np.ndarray, np.ndarray]:
        N, nx, nu = self._cfg.horizon, self._nx, self._nu

        Q = self._make_Q()
        R = self._make_R()
        P = self._cfg.terminal_alpha * Q

        X = ca.SX.sym('X', nx, N + 1)
        U = ca.SX.sym('U', nu, N)

        n_params = nx + (N + 1) * nx
        p = ca.SX.sym('p', n_params)
        x0_p = p[:nx]
        x_ref = p[nx:]

        cost = ca.SX(0)
        constraints = [X[:, 0] - x0_p]

        for k in range(N):
            ref_k = x_ref[k * nx:(k + 1) * nx]
            e_x = X[:, k] - ref_k
            e_u = U[:, k]
            cost += e_x.T @ Q @ e_x + e_u.T @ R @ e_u

            x_next = self._dynamics.rk4_step(X[:, k], U[:, k], self._cfg.dt)
            constraints.append(X[:, k + 1] - x_next)

        ref_N = x_ref[N * nx:(N + 1) * nx]
        e_term = X[:, N] - ref_N
        cost += e_term.T @ P @ e_term

        opt_vars = ca.vertcat(
            ca.reshape(X, -1, 1),
            ca.reshape(U, -1, 1)
        )

        g = ca.vertcat(*constraints)
        n_g = g.shape[0]
        lbg = np.zeros(n_g)
        ubg = np.zeros(n_g)

        nlp = {
            'x': opt_vars,
            'f': cost,
            'g': g,
            'p': p,
        }

        opts = {
            'ipopt.max_iter':    self._cfg.solver_max_iter,
            'ipopt.tol':         self._cfg.solver_tol,
            'ipopt.print_level': 0,
            'print_time':        False,
        }

        solver = ca.nlpsol('nmpc', 'ipopt', nlp, opts)
        return solver, lbg, ubg

    def solve(
        self,
        current_state: np.ndarray,
        reference_traj: np.ndarray,
    ) -> np.ndarray:
        self._validate_inputs(current_state, reference_traj)

        solution = self._solver(
            x0  = self._build_warm_start(),
            p   = self._pack_parameters(current_state, reference_traj),
            lbg = self._lbg,
            ubg = self._ubg,
            lbx = self._build_lbx(),
            ubx = self._build_ubx(),
        )

        status = self._solver.stats()['return_status']
        if status not in ('Solve_Succeeded', 'Solved_To_Acceptable_Level'):
            raise RuntimeError(f'NMPC solver failed with status: {status}')

        x_opt, u_opt = self._unpack_solution(solution['x'])
        self._shift_warm_start(x_opt, u_opt)
        return u_opt[:, 0]
