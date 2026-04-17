import rclpy
import numpy as np
from rclpy.node import Node
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from app.Config import NRHDGConfig
from app.Controllers import NRHDG_Controller
from app.Models import DroneParameters


class DroneState:
    def __init__(self) -> None:
        self._position:    np.ndarray = np.zeros(3)
        self._velocity:    np.ndarray = np.zeros(3)
        self._euler:       np.ndarray = np.zeros(3)
        self._angular_vel: np.ndarray = np.zeros(3)
        self._pose_ready   = False
        self._twist_ready  = False

    @property
    def is_ready(self) -> bool:
        return self._pose_ready and self._twist_ready

    def update_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        self._position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        self._euler = self._quat_to_euler(q.w, q.x, q.y, q.z)
        self._pose_ready = True

    def update_twist(self, msg: TwistStamped) -> None:
        self._velocity = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
        ])
        self._angular_vel = np.array([
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z,
        ])
        self._twist_ready = True

    def get_state(self) -> np.ndarray:
        if not self.is_ready:
            raise ValueError("State is not ready yet")
        return np.concatenate([
            self._position,
            self._velocity,
            self._euler,
            self._angular_vel,
        ])

    @staticmethod
    def _quat_to_euler(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
        roll  = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
        pitch = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
        yaw   = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return np.array([roll, pitch, yaw])


class ReferenceTrajectory:
    def __init__(self, horizon: int, state_dim: int) -> None:
        self._horizon   = horizon
        self._state_dim = state_dim

    def hover_at(self, position: np.ndarray, yaw: float = 0.0) -> np.ndarray:
        ref       = np.zeros((self._horizon + 1, self._state_dim))
        ref[:, 0] = position[0]
        ref[:, 1] = position[1]
        ref[:, 2] = position[2]
        ref[:, 8] = yaw
        return ref


class NRHDGNode(Node):
    _NODE_NAME = "nrhdg_controller"

    def __init__(self) -> None:
        super().__init__(self._NODE_NAME)

        self._declare_ros_parameters()

        drone_params = DroneParameters()
        cfg = self._load_nrhdg_config()

        self._state_buffer = DroneState()
        self._ref_provider = ReferenceTrajectory(cfg.horizon, 12)
        self._controller   = NRHDG_Controller(cfg, drone_params)

        self._target_position = np.array([0.0, 0.0, 1.5])
        self._target_yaw = 0.0

        sensor_qos = self._make_sensor_qos()
        self._setup_subscribers(sensor_qos)
        self._setup_publisher()
        self._timer = self.create_timer(cfg.dt, self._control_loop)

        self.get_logger().info(
            f'{self._NODE_NAME} ready — horizon={cfg.horizon}, dt={cfg.dt}s, '
            f'gamma={cfg.gamma}, w_max={cfg.w_max}, game_iters={cfg.game_iters}'
        )

    def _declare_ros_parameters(self) -> None:
        self.declare_parameter('horizon',         20)
        self.declare_parameter('dt',              0.05)
        self.declare_parameter('Q_pos',           10.0)
        self.declare_parameter('Q_vel',           1.0)
        self.declare_parameter('Q_angle',         5.0)
        self.declare_parameter('Q_rate',          0.5)
        self.declare_parameter('R_thrust',        0.01)
        self.declare_parameter('R_torque',        0.1)
        self.declare_parameter('thrust_max',      30.0)
        self.declare_parameter('torque_max',      2.0)
        self.declare_parameter('angle_max',       0.52)
        self.declare_parameter('gamma',           2.0)
        self.declare_parameter('w_max',           1.0)
        self.declare_parameter('game_iters',      3)
        self.declare_parameter('game_tol',        1e-3)
        self.declare_parameter('solver_max_iter', 100)

    def _load_nrhdg_config(self) -> NRHDGConfig:
        g = self.get_parameter
        return NRHDGConfig(
            horizon         = g('horizon').value,
            dt              = g('dt').value,
            Q_pos           = g('Q_pos').value,
            Q_vel           = g('Q_vel').value,
            Q_angle         = g('Q_angle').value,
            Q_rate          = g('Q_rate').value,
            R_thrust        = g('R_thrust').value,
            R_torque        = g('R_torque').value,
            thrust_max      = g('thrust_max').value,
            torque_max      = g('torque_max').value,
            angle_max       = g('angle_max').value,
            gamma           = g('gamma').value,
            w_max           = g('w_max').value,
            game_iters      = g('game_iters').value,
            game_tol        = g('game_tol').value,
            solver_max_iter = g('solver_max_iter').value,
        )

    @staticmethod
    def _make_sensor_qos() -> QoSProfile:
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

    def _setup_subscribers(self, qos: QoSProfile) -> None:
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._on_pose,
            qos,
        )
        self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self._on_twist,
            qos,
        )
        self.create_subscription(
            PoseStamped,
            '/ares/nrhdg/target_position',
            self._on_target,
            10,
        )

    def _setup_publisher(self) -> None:
        self._cmd_pub = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude',
            10,
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        self._state_buffer.update_pose(msg)

    def _on_twist(self, msg: TwistStamped) -> None:
        self._state_buffer.update_twist(msg)

    def _on_target(self, msg: PoseStamped) -> None:
        self._target_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        q = msg.pose.orientation
        _, _, self._target_yaw = DroneState._quat_to_euler(
            q.w, q.x, q.y, q.z
        )
        self.get_logger().info(
            f'New target → pos={self._target_position}, yaw={self._target_yaw:.3f} rad'
        )

    def _control_loop(self) -> None:
        if not self._state_buffer.is_ready:
            return

        current_state = self._state_buffer.get_state()
        reference     = self._ref_provider.hover_at(
            self._target_position, self._target_yaw
        )

        try:
            u_opt = self._controller.solve(current_state, reference)
        except (RuntimeError, ValueError) as exc:
            self.get_logger().warn(f'NRHDG solver issue: {exc}')
            return

        self._publish_attitude_target(u_opt)

    def _publish_attitude_target(self, u_opt: np.ndarray) -> None:
        msg                 = AttitudeTarget()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.type_mask       = AttitudeTarget.IGNORE_ATTITUDE

        msg.body_rate.x = float(u_opt[1])
        msg.body_rate.y = float(u_opt[2])
        msg.body_rate.z = float(u_opt[3])

        msg.thrust = float(
            np.clip(u_opt[0] / self._controller._cfg.thrust_max, 0.0, 1.0)
        )
        self._cmd_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NRHDGNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt — shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
