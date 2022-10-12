import rclpy
from rclpy.node import Node

class BoatShutdown(Node):
    """Minimal publisher that sends FATAL error messages to the console, resulting in a shutdown of boat"""

    def __init__(self):
        super().__init__('shutdown_node')
        # .fatal() will result in error msg 50
        self.get_logger().fatal('Shutting down')


def main(args=None):
    rclpy.init(args=args)
    boat_shutdown = BoatShutdown()
    rclpy.spin(boat_shutdown)
    boat_shutdown.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
