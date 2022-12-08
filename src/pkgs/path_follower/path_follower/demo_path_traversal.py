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
        self.publisherPose_ = self.create_publisher(PoseMessage, '/position/pose', 10)
        self.publisherTWA_ = self.create_publisher(WindMessage, '/sensor/true_wind', 10)
        self.publisherAWA_ = self.create_publisher(WindMessage, '/sensor/wind', 10)
        

        # Create timers (callbacks triggered by timers)
        period = 0.1 #for a 10hz system
        self.pathTraversal = self.create_timer(period, self.set_next_waypoint_callback)
        self.demoing = self.create_timer(period, self.demo_data_callback)
        self.posePub = self.create_timer(period,self.pose_publisher_callback)
        self.windPub = self.create_timer(period,self.wind_publisher_callback)

        self.debugcallback = self.create_timer(period, self.debug_callback)

        # Create varibles
        self.current_position = np.array([59.59827, 16.55583])#longitude/latitude

        self.path = [np.array([59.59827, 16.55583]),
        np.array([59.59779, 16.55751]),np.array([59.59718, 16.55952]),np.array([59.5977, 16.56115]),np.array([59.59836, 16.5621]),np.array([59.59925, 16.56111]),np.array([59.59953, 16.55961]),np.array([59.59896, 16.55763]),
        np.array([59.59792, 16.55399]),np.array([59.59725, 16.55193]),np.array([59.59775, 16.55004]),np.array([59.59853, 16.54918]),np.array([59.59946, 16.55025]),np.array([59.59975, 16.55201]),np.array([59.59931, 16.55394])
        ]
        self.new_path = True
        self.count = 0
        self.count_point_0 = 0
        self.count_point_1 = 0

        self.waypoint_radii = 2

        self.current_bearing = 0

        self.twa = 0
        self.awa = 0

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

            #for demo bearing output
            self.set_bearing_to_next_pos()
            self.get_logger().info('WOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP') #push message to console

            

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



    def debug_callback(self):
        self.get_logger().info('----------------------------------------') #push message to console
        self.get_logger().info('Prev waypoint: %f , %f' % (self.previous_waypoint[0], self.previous_waypoint[1]) ) #push message to console
        self.get_logger().info('Next waypoint: %f , %f' % (self.next_waypoint[0], self.next_waypoint[1]) ) #push message to console       
        self.get_logger().info('Current bearing: %f' % (self.current_bearing) ) #push message to console 
        #self.get_logger().info('True wind angle:%f' % self.twa) #push message to console

    def increment_waypoint_counter(self):
        self.count_point_0 = (self.count_point_0 + 1) % (len(self.path))
        self.count_point_1 = (self.count_point_1 + 1) % (len(self.path))


    def demo_data_callback(self):
        time_between_points = 4
        time_now = self.get_clock().now().nanoseconds*0.000000001#get current time

        #set new bearing between next points
        if time_now - self.prev_time > time_between_points/2:
            self.set_bearing_to_next_pos()

        #every 2 seconds move forward to the next point(as if the boats moving)
        if time_now - self.prev_time > time_between_points:
            self.prev_time = time_now#for testing
            self.current_position = self.next_waypoint


    def set_bearing_to_next_pos(self):
        self.current_bearing = np.rad2deg(get_c_angle(np.array([99,0]), self.next_waypoint-self.previous_waypoint))

    def wind_publisher_callback(self):
        message_twa = WindMessage()
        message_awa = WindMessage()
        message_twa.wind_angle = self.twa
        wind_awa_cc = (int)(360-self.current_bearing)
        self.awa = (wind_awa_cc+180)%360-180
        message_awa.wind_angle = self.awa

        message_twa.wind_speed = 1.0
        message_awa.wind_speed = 1.0

        self.publisherTWA_.publish(message_twa)
        self.publisherAWA_.publish(message_awa)
        
    def pose_publisher_callback(self):
        #pub position and heading
        message_pose = PoseMessage()
        message_pose.position.latitude = self.current_position[0]
        message_pose.position.longitude = self.current_position[1]
        
        #publish yaw which is CC as opposed of bearing which is Clockwise
        message_pose.yaw = 360-self.current_bearing#normal yaw
        #message_pose.yaw = self.current_bearing#actual bearing
        self.publisherPose_.publish(message_pose)

    
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

#Get bearing(clockwise) angle between (lat , long) vectors
def get_c_angle(vector1:np.array,vector2:np.array):
    dot = vector1[1]*vector2[1]+vector1[0]*vector2[0]
    det = vector1[1]*vector2[0]-vector1[0]*vector2[1]
    angle = np.arctan2(det,dot)
    angle = (angle)%(2*np.pi)
    return 2*np.pi-angle


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
