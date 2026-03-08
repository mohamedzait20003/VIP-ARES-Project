import time
import pytest
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget


def _make_pose(x=0.0, y=0.0, z=1.5) -> PoseStamped:
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    return msg

def _make_twist() -> TwistStamped:
    return TwistStamped()


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class TestNMPCNodeIntegration:
    def test_publishes_attitude_target_after_pose_and_twist(self, ros_context):
        from app.Nodes.NMPC_Node import NMPCNode

        node = NMPCNode()
        received: list[AttitudeTarget] = []

        spy = rclpy.create_node('test_spy')
        spy.create_subscription(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude',
            lambda m: received.append(m),
            10,
        )

        pose_pub  = spy.create_publisher(PoseStamped, '/mavros/local_position/pose', 10)
        twist_pub = spy.create_publisher(TwistStamped, '/mavros/local_position/velocity_local', 10)

        pose_pub.publish(_make_pose())
        twist_pub.publish(_make_twist())

        deadline = time.time() + 5.0
        while not received and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            rclpy.spin_once(spy,  timeout_sec=0.1)

        node.destroy_node()
        spy.destroy_node()

        assert len(received) > 0, "NMPCNode never published an AttitudeTarget"

    def test_thrust_normalised_between_0_and_1(self, ros_context):
        from app.Nodes.NMPC_Node import NMPCNode

        node = NMPCNode()
        received: list[AttitudeTarget] = []

        spy = rclpy.create_node('test_spy2')
        spy.create_subscription(AttitudeTarget, '/mavros/setpoint_raw/attitude',
                                lambda m: received.append(m), 10)

        pose_pub  = spy.create_publisher(PoseStamped, '/mavros/local_position/pose', 10)
        twist_pub = spy.create_publisher(TwistStamped, '/mavros/local_position/velocity_local', 10)
        pose_pub.publish(_make_pose())
        twist_pub.publish(_make_twist())

        deadline = time.time() + 5.0
        while not received and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            rclpy.spin_once(spy,  timeout_sec=0.1)

        node.destroy_node()
        spy.destroy_node()

        assert received, "No message received"
        assert 0.0 <= received[0].thrust <= 1.0