import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    """Minimal publisher that sends FATAL error messages to the console, resulting in a shutdown of boat"""

    def __init__(self):
        super().__init__('minimal_publisher')
        # .fatal() will result in error msg 50
        self.get_logger().fatal('Shutting down')


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
