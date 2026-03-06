import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Create a class inheriting ROS2 node
class CustomSubscriber(Node):

    def __init__(self):
        super().__init__("custom_subscriber") # Initialize the node

        # Write code to execute upon node initialization
        self.get_logger().info("Now running: demmo_subscriber") # print

        # Create the subscriber object
        self.subscriber_ = self.create_subscription(String, '/testing_topic', self.subscription_function, 10)

    def subscription_function(self, msg: String):
        
        self.get_logger().info(msg.data) # Print message



def main(args=None):
    rclpy.init(args=args) # Initialize ROS2
    node = CustomSubscriber() # Create a node
    rclpy.spin(node) # Allow node to continue running until keyboard interrupt
    rclpy.shutdown() # Shutdown ROS2

if __name__ == "__main__":
    main()