from dataclasses import dataclass

@dataclass
class NRHDGConfig:
    horizon: int = 20
    dt: float = 0.05

    Q_pos: float = 10.0
    Q_vel: float = 1.0
    Q_angle: float = 5.0
    Q_rate: float = 0.5

    terminal_alpha: float = 5.0

    R_thrust: float = 0.01
    R_torque: float = 0.1

    angle_max: float = 0.52

    thrust_min: float = 0.0
    thrust_max: float = 30.0
    torque_max: float = 2.0

    gamma: float = 2.0
    w_max: float = 1.0

    game_iters: int = 3
    game_tol: float = 1e-3

    solver_tol: float = 1e-4
    solver_max_iter: int = 100