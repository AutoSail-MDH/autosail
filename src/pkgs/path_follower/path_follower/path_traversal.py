import rclpy
from rclpy.node import Node

from autosail_message.msg import GNSSMessage
from autosail_message.msg import IMUMessage
from autosail_message.msg import PoseMessage
from autosail_message.msg import RudderControlMessage
from autosail_message.msg import WindMessage
from autosail_message.msg import PositionMessage

import math
import numpy as np

class PathTraverser(Node):
    def __init__(self):
        super().__init__('path_traversal_node')

        # Create subscriptions
        #self.subscriberPOSE_ = self.create_subscription(PoseMessage, '/position/pose', self.pose_callback, 10)
        #self.subscriberPOSE_  # prevent unused variable warning
        #self.subscriberTWA_ = self.create_subscription(WindMessage, '/sensor/true_wind', self.twa_callback, 10) #subscription for true wind angle
        #self.subscriberTWA_ = self.create_subscription(WindMessage, '/sensor/wind', self.wind_callback, 10) #ONLY FOR DEVELOPMENT! subscription for wind angle
        #self.subscriberTWA_ # prevent unused variable warning
        #subscription for path. previous_waypoint and next_waypoint

        # Create publishers
        self.publisherNextWaypoint_ = self.create_publisher(PositionMessage, '/path/next_waypoint', 10)
        self.publisherPreviousWaypoint_ = self.create_publisher(PositionMessage, '/path/prev_waypoint', 10)
        

        # Create timers (callbacks triggered by timers)
        period = 0.1 #for a 10hz system
        self.pathTraversal = self.create_timer(period, self.set_next_waypoint_callback)

        self.debugcallback = self.create_timer(period, self.debug_callback)

        # Create varibles
        self.current_position = np.array([59.637171 , 16.584125])#longitude/latitude
        #self.current_position = np.array([59.637020 , 16.584150]) #second boat position test point
        self.previous_waypoint = np.array([0 , 0])#longitude/latitude
        self.next_waypoint = np.array([0 , 0])#longitude/latitude

        self.path = [np.array([59.637053 , 16.583962]), np.array([59.637212 , 16.584512])]
        self.new_path = True
        self.count = 0
        self.count_point_0 = 0
        self.count_point_1 = 0


        self.prev_time = self.get_clock().now().nanoseconds*0.000000001#get current time

    def set_next_waypoint_callback(self):

        #check if the path is new
        if(self.new_path is True):
            self.count_point_0 = 0
            if len(self.path) > 1:
                self.count_point_1 = 1
            else:
                self.count_point_1 = 0

            self.previous_waypoint = self.path[self.count_point_0]
            self.next_waypoint = self.path[self.count_point_1]
            self.new_path = False
            
        #check if one has gotten close enough to next waypoint
        #count_point_0 += 1
        time_now = self.get_clock().now().nanoseconds*0.000000001#get current time
        if time_now - self.prev_time > 2:
            self.prev_time = time_now#for testing
            #self.count_point_0 = (self.count_point_0 + 1) % (len(self.path))
            #self.count_point_1 = (self.count_point_1 + 1) % (len(self.path))
            self.increment_point_counter()
            #self.get_logger().info('count_point_0: %f ' % (self.count_point_0)) #push message to console
            #self.get_logger().info('count_point_1: %f ' % (self.count_point_1)) #push message to console
            self.previous_waypoint = self.path[self.count_point_0]
            self.next_waypoint = self.path[self.count_point_1]

        
        #Publish waypoints
        message_next_point = PositionMessage()
        message_prev_point = PositionMessage()
        message_next_point.latitude = self.next_waypoint[0]
        message_next_point.longitude = self.next_waypoint[1]
        self.publisherNextWaypoint_.publish(message_next_point)
        message_prev_point.latitude = self.previous_waypoint[0]
        message_prev_point.longitude = self.previous_waypoint[1]
        self.publisherPreviousWaypoint_.publish(message_prev_point)



        #DEBUG outputs
        #self.get_logger().info('----------------------------------------') #push message to console
        #self.get_logger().info('Desired lateral-point is: (%f , %f)' % (s[0], s[1])) #push message to console
        #self.get_logger().info('Desired los-point is: (%f , %f)' % (los_point[0], los_point[1])) #push message to console
        #self.get_logger().info('Desired SB is: (%f , %f)' % (sb[0], sb[1])) #push message to console
        #self.get_logger().info('Angle SB in deg: \t %f' % (np.rad2deg(sb_angle))) #push message to console
        #self.get_logger().info('Desired angle is: \t %f' % (desired_angle)) #push message to console
        #self.get_logger().info('Adjusted angle is:\t %f' % (adjusted_angle)) #push message to console
        #self.get_logger().info('True wind angle is:    \t %f)' % (self.twa)) #push message to console


    def debug_callback(self):
        self.get_logger().info('----------------------------------------') #push message to console
        self.get_logger().info('Prev waypoint: %f , %f' % (self.previous_waypoint[0], self.previous_waypoint[1]) ) #push message to console
        self.get_logger().info('Next waypoint: %f , %f' % (self.next_waypoint[0], self.next_waypoint[1]) ) #push message to console        #self.get_logger().info('Rudder angle:%f' % self.rudderAngle) #push message to console
        #self.get_logger().info('True wind angle:%f' % self.twa) #push message to console

    def increment_point_counter(self):
        self.count_point_0 = (self.count_point_0 + 1) % (len(self.path))
        self.count_point_1 = (self.count_point_1 + 1) % (len(self.path))



#Returns the desired boat heading angle in radians determined using a LOS-algorithm 
def los_algorithm(current_position:float, previous_waypoint:float, next_waypoint:float, lookahead_distance:float):
    o = current_position #boats actual position in lat/long
    


        

def main(args=None):
    print('Hi from path_traversal.')

    rclpy.init(args=args) #init rclpy library

    path_traversal_node = PathTraverser() #create path_follower node and initialize the PathTraverser class 

    rclpy.spin(path_traversal_node) #sets the node in an endless loop causing all its callbacks to be continously called

    #destroy node and stop rclpy
    path_traversal_node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
