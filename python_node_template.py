import rclpy
from rclpy.node import Node

# Create a class inheriting ROS2 node
class CustomNode(Node):

    def __init__(self):
        super().__init__("custom_node") # Initialize the node

        # Set class properties
        self.variable = 0

        # Write code to execute upon node initialization
        self.get_logger().info("Hello World!") # print
        self.create_timer(1.0, self.timer_function) # create loop at 1-second intervals

    def timer_function(self):
        self.variable += 1
        self.get_logger().info(f"Current value: {self.variable}")


def main(args=None):
    rclpy.init(args=args) # Initialize ROS2
    node = CustomNode() # Create a node
    rclpy.spin(node) # Allow node to continue running until keyboard interrupt
    rclpy.shutdown() # Shutdown ROS2

if __name__ == "__main__":
    main()