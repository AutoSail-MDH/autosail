import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import Log

import subprocess

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        self.pubIMU_ = self.create_subscription(Log, '/rosout', self.error_callback, 10)
        self.pubIMU_ # prevent unused variable warning
    
    def error_callback(self, msg):
        if (msg.level == 50): #50 is FATAL severity level
            result = subprocess.run("echo hello_world", shell=True, capture_output=True)
            print(result.stdout.decode())



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()