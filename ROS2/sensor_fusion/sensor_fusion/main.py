import rclpy
from rclpy.node import Node
from sensor_fusion.extended_kalman_filter import ekf_estimation

from std_msgs.msg import Float32MultiArray

import math
import numpy as np


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.subGPS_ = self.create_subscription(
            Float32MultiArray, '/position/GPS', self.GPS_callback, 10)
        self.subGPS_  # prevent unused variable warning

        self.pubIMU_ = self.create_subscription(
            Float32MultiArray, '/position/IMU', self.IMU_callback, 10)
        self.pubIMU_  # prevent unused variable warning

        self.publisher_ = self.create_publisher(
            Float32MultiArray, '/position/fusion', 10)

        self.start = self.get_clock().now()

        self.curr_GPS_ = []

        self.prev_time_IMU_ = self.start
        # self.prev_velocity_ = [0, 0, 0] #[z y x]

        self.prev_yaw_ = 0

        # EKF
        self.xEst = np.zeros((4, 1))  # State vector [x y yaw v]
        self.PEst = np.eye(4)  # Covariance matrix of the state

    def IMU_callback(self, msg):
        message = Float32MultiArray()

        ypr = msg.data[0:3]     # [yaw pitch roll]
        accel = msg.data[3:6]   # [z y x]
        # gyro = msg.data[6:9]  #[z y x]

        GPS = self.curr_GPS_    # [x y]

        yaw = ypr[0]    # z-axis rotation

        curr_time_IMU = self.get_clock().now()

        if (self.curr_GPS_ != []) & (self.prev_time_IMU_ != self.start):  # //Atleast two readings
            x = GPS[0]  # Latitude
            y = GPS[1]  # Longitude

            # Difference in time between IMU readings [s]
            DT_IMU = (curr_time_IMU -
                      self.prev_time_IMU_).nanoseconds / pow(10, 9)

            # yawrate = gyro[0]       #z-axis angular velocity
            yawrate = self.yaw_to_yawrate(yaw, DT_IMU)

            # Convert IMU acceleration to speed
            v = self.accel_to_velocity(accel, DT_IMU)

            self.get_logger().info('x: %f, y: %f, yaw: %f, v: %f, yawrate: %f, DT_IMU: %f' %
                                   (x, y, yaw, v, yawrate, DT_IMU))

            """
            self.xEst = np.array([[x],     #State vector
                                    [y],
                                    [yaw],
                                    [v]])
            """

            u = np.array([[v],  # Input vector
                          [yawrate]])

            z = np.array([[x],  # Observation vector
                          [y]])

            self.xEst, self.PEst = ekf_estimation(
                self.xEst, self.PEst, z, u, DT_IMU)

            message.data = [float(self.xEst[0]), float(self.xEst[1]), float(
                self.xEst[2]), float(self.xEst[3])]  # [x y yaw v]
            self.get_logger().info('x = %f, y = %f, yaw = %f, v = %f' %
                                   (message.data[0], message.data[1], message.data[2], message.data[3]))
            self.publisher_.publish(message)

        self.prev_time_IMU_ = curr_time_IMU
        self.prev_yaw_ = yaw

    def GPS_callback(self, msg):
        self.curr_GPS_ = msg.data

    def yaw_to_yawrate(self, yaw, DT):

        yawrate = (yaw - self.prev_yaw_) / DT

        return yawrate

    def accel_to_velocity(self, accel, DT):

        #vz = accel[0] * DT
        vy = accel[1] * DT
        vx = accel[2] * DT

        #vz = vz + self.prev_velocity_[0]
        #vy = vy + self.prev_velocity_[1]
        #vx = vx + self.prev_velocity_[2]

        # Take acceleration only from x and y heading
        velocity = math.sqrt(pow(vy, 2) + pow(vx, 2))

        #self.prev_velocity_[0] = vz
        #self.prev_velocity_[1] = vy
        #self.prev_velocity_[2] = vx

        return velocity


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
