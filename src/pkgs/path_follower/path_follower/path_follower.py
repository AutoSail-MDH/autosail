import rclpy
from rclpy.node import Node

from autosail_message.msg import GNSSMessage
from autosail_message.msg import IMUMessage
from autosail_message.msg import PoseMessage
from autosail_message.msg import NextPositionMessage

import math
import numpy as np

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        #create subscriptions
        self.subGNSS_ = self.create_subscription(GNSSMessage, '/sensor/gnss', self.GNSS_callback, 10)
        self.subGNSS_  # prevent unused variable warning

        #create publishers


def main():
    print('Hi from path_follower.')


if __name__ == '__main__':
    main()
