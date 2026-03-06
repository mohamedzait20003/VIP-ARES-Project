import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Create a class inheriting ROS2 node
class CustomPublisher(Node):

    def __init__(self):
        super().__init__("custom_publisher") # Initialize the node

        # Set class properties
        self.variable = 0

        # Create the publisher object
        self.publisher_ = self.create_publisher(String, '/testing_topic', 10)

        # Write code to execute upon node initialization
        self.get_logger().info("Now running: demo_publisher") # print
        self.create_timer(1.0, self.timer_function) # create loop at 1-second intervals

    def timer_function(self):
        self.variable += 1

        # Prepare message
        message = String()
        message.data = f"Current value: {self.variable}"

        self.get_logger().info(message.data) # Print message
        self.publisher_.publish(message) # Publish message


def main(args=None):
    rclpy.init(args=args) # Initialize ROS2
    node = CustomPublisher() # Create a node
    rclpy.spin(node) # Allow node to continue running until keyboard interrupt
    rclpy.shutdown() # Shutdown ROS2

if __name__ == "__main__":
    main()