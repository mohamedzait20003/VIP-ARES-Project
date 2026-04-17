import pytest
import numpy as np
from src.app.Config import NMPCConfig
from src.app.Controllers import NMPC_Controller
from src.app.Models.Dynamic_Model import DroneParameters, Dynamic_Model

NX, NU, N = 12, 4, 20

@pytest.fixture(scope="module")
def controller():
    return NMPC_Controller(NMPCConfig(horizon=10, dt=0.1))

@pytest.fixture
def hover_state():
    """Drone hovering at (0,0,1.5) with no velocity or angular motion."""
    x = np.zeros(NX)
    x[2] = 1.5
    return x

@pytest.fixture
def hover_ref(hover_state):
    ref = np.tile(hover_state, (N + 1, 1))
    return ref[:11]

class TestDynamicModel:
    def test_rk4_shape(self):
        dyn = Dynamic_Model(DroneParameters())
        import casadi as ca
        x = ca.MX.sym('x', 12)
        u = ca.MX.sym('u', 4)
        xnext = dyn.rk4_step(x, u, 0.05)
        assert xnext.shape == (12, 1)

    def test_hover_equilibrium(self):
        """At hover, thrust = m*g and zero torques → near-zero ẋ."""
        dyn = Dynamic_Model(DroneParameters())
        import casadi as ca
        x = ca.DM.zeros(12);  x[2] = 1.5
        u = ca.DM([DroneParameters().mass * DroneParameters().gravity, 0, 0, 0])
        xdot = dyn.continuous_dynamics(x, u)
        assert abs(float(xdot[5])) < 1e-6

class TestNMPCController:
    def test_solve_returns_correct_shape(self, controller, hover_state, hover_ref):
        u = controller.solve(hover_state, hover_ref)
        assert u.shape == (NU,)

    def test_thrust_within_bounds(self, controller, hover_state, hover_ref):
        u = controller.solve(hover_state, hover_ref)
        cfg = controller._cfg
        assert cfg.thrust_min <= u[0] <= cfg.thrust_max

    def test_torques_within_bounds(self, controller, hover_state, hover_ref):
        u = controller.solve(hover_state, hover_ref)
        assert np.all(np.abs(u[1:]) <= controller._cfg.torque_max)

    def test_hover_thrust_near_mg(self, controller, hover_state, hover_ref):
        """At hover setpoint the solver should request ≈ m*g thrust."""
        u = controller.solve(hover_state, hover_ref)
        mg = DroneParameters().mass * DroneParameters().gravity
        assert abs(u[0] - mg) < 2.0

    def test_bad_state_shape_raises(self, controller, hover_ref):
        with pytest.raises(ValueError):
            controller.solve(np.zeros(5), hover_ref)

    def test_bad_ref_shape_raises(self, controller, hover_state):
        with pytest.raises(ValueError):
            controller.solve(hover_state, np.zeros((5, 12)))

    def test_warm_start_populated_after_solve(self, controller, hover_state, hover_ref):
        controller.solve(hover_state, hover_ref)
        assert controller._x_ws is not None
        assert controller._u_ws is not None