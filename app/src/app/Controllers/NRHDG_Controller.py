from __future__ import annotations

import numpy as np
import casadi as ca
from typing import Optional, Tuple

from app.Config import NRHDGConfig
from app.Models import DroneParameters, Dynamic_Model

class NRHDG_Controller:
    def __init__(
        self,
        config: NRHDGConfig,
        drone_params: Optional[DroneParameters] = None,
    ) -> None:
        self._cfg      = config
        self._dynamics = Dynamic_Model(drone_params or DroneParameters())
        self._nx       = self._dynamics.state_dim
        self._nu       = self._dynamics.control_dim
        self._nw       = 3

        B = np.zeros((self._nx, self._nw))
        B[3:6, :] = np.eye(3)
        self._Bw = ca.DM(B)

        self._u_solver, self._u_lbg, self._u_ubg = self._build_u_solver()
        self._w_solver, self._w_lbg, self._w_ubg = self._build_w_solver()

        self._x_ws: Optional[np.ndarray] = None
        self._u_ws: Optional[np.ndarray] = None
        self._w_ws: Optional[np.ndarray] = None

    def solve(
        self,
        current_state: np.ndarray,
        reference_traj: np.ndarray,
    ) -> np.ndarray:
        self._validate_inputs(current_state, reference_traj)
        params = self._pack_parameters(current_state, reference_traj)

        w_curr = self._init_w()
        u_prev: Optional[np.ndarray] = None
        x_opt = u_opt = None

        for _ in range(self._cfg.game_iters):
            x_opt, u_opt = self._min_u(params, w_curr)
            if u_prev is not None and np.linalg.norm(u_opt - u_prev) < self._cfg.game_tol:
                break
            u_prev = u_opt
            w_curr = self._max_w(params, u_opt)

        self._shift_warm_start(x_opt, u_opt, w_curr)
        return u_opt[:, 0]

    def _min_u(self, params: np.ndarray, w_fixed: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        N, nx, nu = self._cfg.horizon, self._nx, self._nu
        sol = self._u_solver(
            x0  = self._build_u_warm_start(),
            p   = np.concatenate([params, w_fixed.flatten('F')]),
            lbg = self._u_lbg,  ubg = self._u_ubg,
            lbx = self._build_u_lbx(), ubx = self._build_u_ubx(),
        )
        status = self._u_solver.stats()['return_status']
        if status not in ('Solve_Succeeded', 'Solved_To_Acceptable_Level'):
            raise RuntimeError(f'NRHDG u-subproblem failed: {status}')
        arr = np.array(sol['x']).flatten()
        n_x = nx * (N + 1)
        return arr[:n_x].reshape(nx, N + 1, order='F'), arr[n_x:].reshape(nu, N, order='F')

    def _max_w(self, params: np.ndarray, u_fixed: np.ndarray) -> np.ndarray:
        N, nx, nw = self._cfg.horizon, self._nx, self._nw
        sol = self._w_solver(
            x0  = self._build_w_warm_start(),
            p   = np.concatenate([params, u_fixed.flatten('F')]),
            lbg = self._w_lbg,  ubg = self._w_ubg,
            lbx = self._build_w_lbx(), ubx = self._build_w_ubx(),
        )
        status = self._w_solver.stats()['return_status']
        if status not in ('Solve_Succeeded', 'Solved_To_Acceptable_Level'):
            return self._w_ws if self._w_ws is not None else np.zeros((nw, N))
        arr = np.array(sol['x']).flatten()
        n_x = nx * (N + 1)
        return arr[n_x:].reshape(nw, N, order='F')

    def _disturbed_next(self, x, u, w):
        return self._dynamics.rk4_step(x, u, self._cfg.dt) + self._cfg.dt * (self._Bw @ w)

    def _stage_cost(self, X_k, U_k, W_k, ref_k, Q, R, gamma2):
        e_x = X_k - ref_k
        return e_x.T @ Q @ e_x + U_k.T @ R @ U_k - gamma2 * (W_k.T @ W_k)

    def _build_u_solver(self) -> Tuple[ca.Function, np.ndarray, np.ndarray]:
        N, nx, nu, nw = self._cfg.horizon, self._nx, self._nu, self._nw
        Q, R = self._make_Q(), self._make_R()
        P = self._cfg.terminal_alpha * Q
        gamma2 = self._cfg.gamma ** 2

        X = ca.SX.sym('X', nx, N + 1)
        U = ca.SX.sym('U', nu, N)

        p = ca.SX.sym('p', nx + (N + 1) * nx + N * nw)
        x0_p  = p[:nx]
        x_ref = p[nx:nx + (N + 1) * nx]
        w_fix = ca.reshape(p[nx + (N + 1) * nx:], nw, N)

        cost = ca.SX(0)
        constraints = [X[:, 0] - x0_p]
        for k in range(N):
            ref_k = x_ref[k * nx:(k + 1) * nx]
            cost += self._stage_cost(X[:, k], U[:, k], w_fix[:, k], ref_k, Q, R, gamma2)
            constraints.append(X[:, k + 1] - self._disturbed_next(X[:, k], U[:, k], w_fix[:, k]))

        ref_N = x_ref[N * nx:(N + 1) * nx]
        e_term = X[:, N] - ref_N
        cost += e_term.T @ P @ e_term

        opt_vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        g = ca.vertcat(*constraints)
        lbg = ubg = np.zeros(g.shape[0])

        solver = ca.nlpsol('nrhdg_u', 'ipopt',
            {'x': opt_vars, 'f': cost, 'g': g, 'p': p},
            {'ipopt.max_iter': self._cfg.solver_max_iter,
             'ipopt.tol': self._cfg.solver_tol,
             'ipopt.print_level': 0, 'print_time': False})
        return solver, lbg, ubg

    def _build_w_solver(self) -> Tuple[ca.Function, np.ndarray, np.ndarray]:
        N, nx, nu, nw = self._cfg.horizon, self._nx, self._nu, self._nw
        Q, R = self._make_Q(), self._make_R()
        P = self._cfg.terminal_alpha * Q
        gamma2 = self._cfg.gamma ** 2

        X = ca.SX.sym('X', nx, N + 1)
        W = ca.SX.sym('W', nw, N)

        p = ca.SX.sym('p', nx + (N + 1) * nx + N * nu)
        x0_p  = p[:nx]
        x_ref = p[nx:nx + (N + 1) * nx]
        u_fix = ca.reshape(p[nx + (N + 1) * nx:], nu, N)

        cost = ca.SX(0)
        constraints = [X[:, 0] - x0_p]
        for k in range(N):
            ref_k = x_ref[k * nx:(k + 1) * nx]
            cost += self._stage_cost(X[:, k], u_fix[:, k], W[:, k], ref_k, Q, R, gamma2)
            constraints.append(X[:, k + 1] - self._disturbed_next(X[:, k], u_fix[:, k], W[:, k]))

        ref_N = x_ref[N * nx:(N + 1) * nx]
        e_term = X[:, N] - ref_N
        cost += e_term.T @ P @ e_term

        opt_vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(W, -1, 1))
        g = ca.vertcat(*constraints)
        lbg = ubg = np.zeros(g.shape[0])

        solver = ca.nlpsol('nrhdg_w', 'ipopt',
            {'x': opt_vars, 'f': -cost, 'g': g, 'p': p},
            {'ipopt.max_iter': self._cfg.solver_max_iter,
             'ipopt.tol': self._cfg.solver_tol,
             'ipopt.print_level': 0, 'print_time': False})
        return solver, lbg, ubg

    def _build_u_lbx(self) -> np.ndarray:
        N, nx, nu = self._cfg.horizon, self._nx, self._nu
        x_lb = np.full((nx, N + 1), -np.inf); x_lb[6:9, :] = -self._cfg.angle_max
        u_lb = np.full((nu, N), -self._cfg.torque_max); u_lb[0, :] = self._cfg.thrust_min
        return np.concatenate([x_lb.flatten('F'), u_lb.flatten('F')])

    def _build_u_ubx(self) -> np.ndarray:
        N, nx, nu = self._cfg.horizon, self._nx, self._nu
        x_ub = np.full((nx, N + 1), np.inf); x_ub[6:9, :] = self._cfg.angle_max
        u_ub = np.full((nu, N), self._cfg.torque_max); u_ub[0, :] = self._cfg.thrust_max
        return np.concatenate([x_ub.flatten('F'), u_ub.flatten('F')])

    def _build_w_lbx(self) -> np.ndarray:
        N, nx, nw = self._cfg.horizon, self._nx, self._nw
        x_lb = np.full((nx, N + 1), -np.inf); x_lb[6:9, :] = -self._cfg.angle_max
        w_lb = np.full((nw, N), -self._cfg.w_max)
        return np.concatenate([x_lb.flatten('F'), w_lb.flatten('F')])

    def _build_w_ubx(self) -> np.ndarray:
        N, nx, nw = self._cfg.horizon, self._nx, self._nw
        x_ub = np.full((nx, N + 1), np.inf); x_ub[6:9, :] = self._cfg.angle_max
        w_ub = np.full((nw, N), self._cfg.w_max)
        return np.concatenate([x_ub.flatten('F'), w_ub.flatten('F')])

    def _build_u_warm_start(self) -> np.ndarray:
        N, nx, nu = self._cfg.horizon, self._nx, self._nu
        if self._x_ws is None:
            x_ws = np.zeros((nx, N + 1))
            u_ws = np.zeros((nu, N))
            u_ws[0, :] = self._dynamics.params.mass * self._dynamics.params.gravity
        else:
            x_ws = np.hstack([self._x_ws[:, 1:], self._x_ws[:, -1:]])
            u_ws = np.hstack([self._u_ws[:, 1:], self._u_ws[:, -1:]])
        return np.concatenate([x_ws.flatten('F'), u_ws.flatten('F')])

    def _build_w_warm_start(self) -> np.ndarray:
        N, nx, nw = self._cfg.horizon, self._nx, self._nw
        if self._x_ws is None:
            x_ws = np.zeros((nx, N + 1))
            w_ws = np.zeros((nw, N))
        else:
            x_ws = np.hstack([self._x_ws[:, 1:], self._x_ws[:, -1:]])
            w_prev = self._w_ws if self._w_ws is not None else np.zeros((nw, N))
            w_ws = np.hstack([w_prev[:, 1:], w_prev[:, -1:]])
        return np.concatenate([x_ws.flatten('F'), w_ws.flatten('F')])

    def _init_w(self) -> np.ndarray:
        N, nw = self._cfg.horizon, self._nw
        if self._w_ws is None:
            return np.zeros((nw, N))
        return np.hstack([self._w_ws[:, 1:], self._w_ws[:, -1:]])

    def _shift_warm_start(self, x_opt, u_opt, w_opt) -> None:
        self._x_ws, self._u_ws, self._w_ws = x_opt, u_opt, w_opt

    def _pack_parameters(self, state, traj) -> np.ndarray:
        return np.concatenate([state, traj.flatten()])

    def _validate_inputs(self, state, traj) -> None:
        if state.shape != (self._nx,):
            raise ValueError(f"Expected state shape ({self._nx},), got {state.shape}")
        expected = (self._cfg.horizon + 1, self._nx)
        if traj.shape != expected:
            raise ValueError(f"Expected trajectory shape {expected}, got {traj.shape}")

    def _make_Q(self) -> ca.DM:
        c = self._cfg
        return ca.DM(np.diag([
            c.Q_pos, c.Q_pos, c.Q_pos,
            c.Q_vel, c.Q_vel, c.Q_vel,
            c.Q_angle, c.Q_angle, c.Q_angle,
            c.Q_rate, c.Q_rate, c.Q_rate,
        ]))

    def _make_R(self) -> ca.DM:
        c = self._cfg
        return ca.DM(np.diag([c.R_thrust, c.R_torque, c.R_torque, c.R_torque]))