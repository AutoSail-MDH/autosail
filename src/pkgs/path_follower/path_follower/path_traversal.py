import rclpy
from rclpy.node import Node

from autosail_message.msg import GNSSMessage
from autosail_message.msg import IMUMessage
from autosail_message.msg import PoseMessage
from autosail_message.msg import RudderControlMessage
from autosail_message.msg import WindMessage
from autosail_message.msg import PositionMessage

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
        #self.current_position = np.array([59.637053 , 16.583962])#TEST POINT SAME AS POINT A

        self.path = [np.array([59.637053 , 16.583962]), np.array([59.637212 , 16.584512])]
        self.new_path = True
        self.count = 0
        self.count_point_0 = 0
        self.count_point_1 = 0

        self.waypoint_radii = 2


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
            
        
        #every 2 seconds move forward to the next point(as if the boats moving)
        time_now = self.get_clock().now().nanoseconds*0.000000001#get current time
        if time_now - self.prev_time > 2:
            self.prev_time = time_now#for testing
            self.current_position = self.next_waypoint

        #if current_position is within radii of next wapoint 
        point_north = new_point_from_distance(self.next_waypoint, 0, self.waypoint_radii)
        point_east =  new_point_from_distance(self.next_waypoint, 90, self.waypoint_radii)
        point_south =  new_point_from_distance(self.next_waypoint, 180, self.waypoint_radii)
        point_west =  new_point_from_distance(self.next_waypoint, 270, self.waypoint_radii)
        
        # Check if one has gotten close enough to next waypoint. First check latitude - then longitude(the coordinates are in (lat,long)).
        if(self.current_position[0] < point_north[0] and self.current_position[0] > point_south[0] 
        and self.current_position[1] < point_east[1] and self.current_position[1] > point_west[1]):
            self.increment_waypoint_counter()
            self.previous_waypoint = self.path[self.count_point_0]
            self.next_waypoint = self.path[self.count_point_1]

            self.get_logger().info('INSIDE square of point') #push message to console

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

    def increment_waypoint_counter(self):
        self.count_point_0 = (self.count_point_0 + 1) % (len(self.path))
        self.count_point_1 = (self.count_point_1 + 1) % (len(self.path))


    
# Get new latitude/longitude point using another point, distance and bearing.
def new_point_from_distance(point,bearing,distance):# bearing must be in radians
    r = 6378.1 # radii of Earth
    bear = bearing
    d = distance/1000 # remake distance to km 
    lat1 = np.deg2rad(point[0])#latidude in radians
    lon1 = np.deg2rad(point[1])#longitude in radians

    # Haversine formula
    lat2 = np.arcsin( np.sin(lat1)*np.cos(d/r) + np.cos(lat1)*np.sin(d/r)*np.cos(bearing) )
    lon2 = lon1 + np.arctan2( np.sin(bearing)*np.sin(d/r)*np.cos(lat1), np.cos(d/r)-np.sin(lat1)*np.sin(lat2) )

    #convert lat/long back to degrees
    lat2 = np.rad2deg(lat2)
    lon2 = np.rad2deg(lon2)

    return lat2,lon2

        

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
