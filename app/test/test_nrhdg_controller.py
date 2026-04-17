import pytest
import numpy as np

from src.app.Config import NRHDGConfig
from src.app.Models import DroneParameters
from src.app.Controllers import NRHDG_Controller

NX, NU, NW = 12, 4, 3
HORIZON = 10


@pytest.fixture(scope="module")
def controller():
    return NRHDG_Controller(NRHDGConfig(
        horizon=HORIZON,
        dt=0.1,
        game_iters=2,
        solver_max_iter=300,
    ))


@pytest.fixture
def hover_state():
    x = np.zeros(NX)
    x[2] = 1.5
    return x


@pytest.fixture
def hover_ref(hover_state):
    return np.tile(hover_state, (HORIZON + 1, 1))


class TestNRHDGController:
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
        """High gamma kills the disturbance player; expect NMPC-like hover thrust."""
        nrhdg = NRHDG_Controller(NRHDGConfig(
            horizon=HORIZON, dt=0.1, gamma=100.0,
            game_iters=2, solver_max_iter=300,
        ))
        u = nrhdg.solve(hover_state, hover_ref)
        mg = DroneParameters().mass * DroneParameters().gravity
        assert abs(u[0] - mg) < 2.0

    def test_bad_state_shape_raises(self, controller, hover_ref):
        with pytest.raises(ValueError):
            controller.solve(np.zeros(5), hover_ref)

    def test_bad_ref_shape_raises(self, controller, hover_state):
        with pytest.raises(ValueError):
            controller.solve(hover_state, np.zeros((5, NX)))

    def test_warm_starts_populated_after_solve(self, controller, hover_state, hover_ref):
        controller.solve(hover_state, hover_ref)
        assert controller._x_ws is not None
        assert controller._u_ws is not None
        assert controller._w_ws is not None
        assert controller._x_ws.shape == (NX, HORIZON + 1)
        assert controller._u_ws.shape == (NU, HORIZON)
        assert controller._w_ws.shape == (NW, HORIZON)

    def test_disturbance_within_bounds(self, controller, hover_state, hover_ref):
        controller.solve(hover_state, hover_ref)
        w_max = controller._cfg.w_max
        assert np.all(np.abs(controller._w_ws) <= w_max + 1e-6)

    def test_high_gamma_suppresses_disturbance(self, hover_state, hover_ref):
        """Large gamma penalizes w heavily → adversary chooses w ≈ 0."""
        nrhdg = NRHDG_Controller(NRHDGConfig(
            horizon=HORIZON, dt=0.1, gamma=100.0,
            game_iters=2, solver_max_iter=300,
        ))
        nrhdg.solve(hover_state, hover_ref)
        assert np.linalg.norm(nrhdg._w_ws) < 1e-2

    def test_low_gamma_drives_disturbance_to_bound(self, hover_state, hover_ref):
        """Small gamma makes the adversary aggressive → w saturates at the bound."""
        nrhdg = NRHDG_Controller(NRHDGConfig(
            horizon=HORIZON, dt=0.1, gamma=0.1, w_max=0.5,
            game_iters=2, solver_max_iter=300,
        ))
        nrhdg.solve(hover_state, hover_ref)
        assert np.max(np.abs(nrhdg._w_ws)) > 0.5 - 1e-3
