import rclpy
from rclpy.node import Node

from autosail_message.msg import GNSSMessage
from autosail_message.msg import IMUMessage
from autosail_message.msg import PoseMessage
from autosail_message.msg import NextPositionMessage
from autosail_message.msg import RudderControlMessage
#include <autosail_message/msg/rudder_control_message.hpp>


import math
import numpy as np

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        #create subscriptions
        self.subPOSE_ = self.create_subscription(PoseMessage, '/position/pose', self.pose_callback, 10)
        self.subPOSE_  # prevent unused variable warning

        #create publishers
        self.publisherRudderAngle_ = self.create_publisher(RudderControlMessage, '/actuator/rudder', 10)

        #create timers (callbacks triggered by timers)
        period = 0.1
        self.rudderControl = self.create_timer(period, self.rudder_control_callback)
        self.navigation = self.create_timer(period, self.navigation_callback_2)


        #create varibles
        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.yaw = 0.0
        self.velocity = 0.0

    #def PATH_FOLLOWER_callback(self, msg):
    


    def navigation_callback(self):
        o = np.array([8,8]) #self.current_lat   #boats actual position in lat/long
        a = np.array([4,3]) #self.path.a        #previous waypoint
        b = np.array([20,8]) #self.path.a       #next waypoint

        lateral_distance_vector = get_lateral_distance_vector(o, a, b)
        os = lateral_distance_vector



        self.get_logger().info('%f , %f' % (os[0], os[1])) #push message to console
        
    
    def navigation_callback_2(self):
        o = np.array([8,8]) #self.current_lat   #boats actual position in lat/long
        a = np.array([4,3]) #self.path.a        #previous waypoint
        b = np.array([20,8]) #self.path.a       #next waypoint

        lookahead_distance = 4

        #get the lateral_distance_point s. 
        lateral_distance_point= get_lateral_distance_point(o, a, b)
        s = lateral_distance_point

        #use the lateral_distance_point s to create vector sb. This gives only a vector and its length. not the actual position p
        sb = b-s

        #shorten this sb vector to length 1 by (simply multiply by 1/|sb|) and make it the same length as the lookahead distance(*lookahead_distance)
        sp = sb*(1/np.linalg.norm(sb))*lookahead_distance

        #the point p will then be located at
        p = s+sp

        # Set the boats position as Origo[0,0] and make a vector north. Set a new point p with the boats current position o as origo
        #the desired angle will then be the angle between the north axis and op 
        oN = np.array([0,99])
        op = p-o



        desired_angle = np.arccos((np.dot(oN,op))/(np.linalg.norm(oN)*np.linalg.norm(op)))
        desired_angle = desired_angle/np.pi * 360
        desired_angle = 360-desired_angle

        


        self.get_logger().info('Desired angle is %f ' % (desired_angle)) #push message to console




    ## Rudder control callback. Triggered by timer and publishes rudder angle to be set
    def rudder_control_callback(self):
        message = RudderControlMessage()


        message.rudder_angle = 1337.0 #add rudder angle to message

        self.publisherRudderAngle_.publish(message) #add message to publisher

        #self.get_logger().info('%f' % message.rudder_angle) #push message to console





        
    #Pose subscriber used for getting pose and velocity of boat
    def pose_callback(self, msg):
        self.current_latitude = msg.position.latitude
        self.current_longitude = msg.position.longitude 
        self.yaw = msg.yaw
        self.velocity = msg.velocity




def get_path_vector(previous_point, next_point):
    return next_point - previous_point

def get_lateral_distance_vector(current_position, previous_point, next_point):
    o = current_position    #boats actual position in lat/long
    a = previous_point      #previous waypoint
    b = next_point          #next waypoint

    #path vectors used in LOS-angle algorithm
    ab = get_path_vector(a,b) #path vector from point a to b
    ao = get_path_vector(a,o) #path vector from point a to o

    #get path vector from the boat to the path ab
    s = a + np.dot(ao,ab)/np.dot(ab,ab) * ab
    os = get_path_vector(o,s)

    return os

def get_lateral_distance_point(current_position, previous_point, next_point):
    o = current_position    #boats actual position in lat/long
    a = previous_point      #previous waypoint
    b = next_point          #next waypoint

    #path vectors used in LOS-angle algorithm
    ab = get_path_vector(a,b) #path vector from point a to b
    ao = get_path_vector(a,o) #path vector from point a to o

    #get path vector from the boat to the path ab
    s = a + np.dot(ao,ab)/np.dot(ab,ab) * ab

    return s

def los_algorithm(boat_heading_angle, lateral_distance_vector, lookahead_distance):
    lateral_distance = np.linalg.norm(lateral_distance_vector)

    los_angle = math.atan(lateral_distance/lookahead_distance)

    #get angle between ob and sb. the angle between them should be the same sign. so simply set this new angle to size 1 but with the same signa and multiply with the los angle.


    return 1




def main(args=None):
    print('Hi from path_follower.')

    rclpy.init(args=args) #init rclpy library

    path_follower_node = PathFollower() #create path_follower node and initialize the pathfollower class 

    rclpy.spin(path_follower_node) #sets the node in an endless loop causing all its callbacks to be continously called

    #destroy node and stop rclpy
    path_follower_node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
