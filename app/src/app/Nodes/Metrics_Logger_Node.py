import os
import csv
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

_COLUMNS = [
    't', 'ex', 'ey', 'ez',
    'pos_err', 'vel_err', 'ang_err',
    'solve_ms', 'thrust', 'torque', 'ok',
]


class MetricsLoggerNode(Node):
    _NODE_NAME = 'metrics_logger'

    def __init__(self) -> None:
        super().__init__(self._NODE_NAME)

        self.declare_parameter('controller', 'nmpc')

        ctrl = self.get_parameter('controller').value
        topic = f'/ares/metrics/{ctrl}'
        output_file = os.path.expanduser(f'~/ares_metrics/{ctrl}_metrics.csv')

        os.makedirs(os.path.dirname(output_file), exist_ok=True)

        self._file   = open(output_file, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(_COLUMNS)

        self.create_subscription(Float64MultiArray, topic, self._on_metrics, 10)

        self.get_logger().info(
            f'MetricsLogger: {topic} → {output_file}'
        )

    def _on_metrics(self, msg: Float64MultiArray) -> None:
        self._writer.writerow([f'{v:.6f}' for v in msg.data])
        self._file.flush()

    def destroy_node(self) -> None:
        self._file.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MetricsLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt — shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
