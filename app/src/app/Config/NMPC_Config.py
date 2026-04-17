from dataclasses import dataclass

@dataclass
class NMPCConfig:
    horizon: int = 20
    dt: float = 0.05


    Q_pos: float = 10.0
    Q_vel: float = 1.0
    Q_angle: float = 5.0
    Q_rate: float = 0.5


    terminal_alpha: float = 5.


    R_thrust: float = 0.01
    R_torque: float = 0.1


    angle_max: float = 0.52


    thrust_min: float = 0.0
    thrust_max: float = 30.0
    torque_max: float = 2.0


    solver_tol: float = 1e-4
    solver_max_iter: int = 150