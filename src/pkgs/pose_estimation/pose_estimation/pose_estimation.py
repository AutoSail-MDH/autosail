import rclpy
from rclpy.node import Node
from pose_estimation.extended_kalman_filter import ekf_estimation

from autosail_message.msg import GNSSMessage
from autosail_message.msg import IMUMessage
from autosail_message.msg import PoseMessage

#include <autosail_message/msg/rudder_control_message.hpp>
#include <autosail_message/msg/gnss_message.hpp>
#include <autosail_message/msg/next_position_message.hpp>
#include <autosail_message/msg/imu_message.hpp>

import math
import numpy as np


class PoseEstimation(Node):

    def __init__(self):
        super().__init__('pose_estimation_node')

        self.subGPS_ = self.create_subscription(
            GNSSMessage, '/sensor/gnss', self.GPS_callback, 10)
        self.subGPS_  # prevent unused variable warning

        self.pubIMU_ = self.create_subscription(
            IMUMessage, '/sensor/imu', self.IMU_callback, 10)
        self.pubIMU_  # prevent unused variable warning

        self.publisher_ = self.create_publisher(
            PoseMessage, '/position/pose', 10)

        self.start = self.get_clock().now()

        self.current_GPS_ = []

        self.previous_time_IMU_ = self.start
        # self.prev_velocity_ = [0, 0, 0] #[z y x]

        self.previous_yaw_ = 0

        # EKF
        self.xEstimate = np.zeros((4, 1))  # State vector [x y yaw v]
        self.PEstimate = np.eye(4)  # Covariance matrix of the state

    def IMU_callback(self, msg):
        message = PoseMessage()

        ypr = [msg.yaw, msg.pitch, msg.roll]       # [yaw pitch roll]
        linear_acceleration = [msg.linear_acceleration_z, msg.linear_acceleration_y, 
                                msg.linear_acceleration_x]  # [z y x]
        # gyro = msg.data[6:9]  #[z y x]

        GPS = self.current_GPS_    # [x y]

        yaw = ypr[0]    # z-axis rotation

        current_time_IMU = self.get_clock().now()

        if (self.current_GPS_ != []) & (self.previous_time_IMU_ != self.start):  # //Atleast two readings
            x = GPS[0]  # Latitude
            y = GPS[1]  # Longitude

            # Difference in time between IMU readings [s]
            DT_IMU = (current_time_IMU -
                      self.previous_time_IMU_).nanoseconds / pow(10, 9)

            yawrate = self.yaw_to_yawrate(yaw, DT_IMU)

            # Convert IMU acceleration to speed
            v = self.acceleration_to_velocity(linear_acceleration, DT_IMU)

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

            self.xEstimate, self.PEstimate = ekf_estimation(
                self.xEstimate, self.PEstimate, z, u, DT_IMU)

            message.position.latitude = float(self.xEstimate[0])
            message.position.longitude = float(self.xEstimate[1])
            message.yaw = float(self.xEstimate[2])
            message.velocity = float(self.xEstimate[3])

            self.get_logger().info('x = %f, y = %f, yaw = %f, v = %f' %
                        (message.position.latitude, message.position.longitude, message.yaw, message.velocity))
            self.publisher_.publish(message)

        self.previous_time_IMU_ = current_time_IMU
        self.previous_yaw_ = yaw

    def GPS_callback(self, msg):
        self.current_GPS_ = [msg.position.latitude, msg.position.longitude]


    def yaw_to_yawrate(self, yaw, DT):

        yawrate = (yaw - self.previous_yaw_) / DT

        return yawrate

    def acceleration_to_velocity(self, acceleration, DT):

        #vz = acceleration[0] * DT
        vy = acceleration[1] * DT
        vx = acceleration[2] * DT

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

    pose_estimation_node = PoseEstimation()

    rclpy.spin(pose_estimation_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_estimation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
