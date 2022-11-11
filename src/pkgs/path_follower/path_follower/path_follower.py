import rclpy
from rclpy.node import Node

from autosail_message.msg import GNSSMessage
from autosail_message.msg import IMUMessage
from autosail_message.msg import PoseMessage
from autosail_message.msg import NextPositionMessage
from autosail_message.msg import RudderControlMessage
from autosail_message.msg import WindMessage

import math
import numpy as np

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        #create subscriptions
        self.subscriberPOSE_ = self.create_subscription(PoseMessage, '/position/pose', self.pose_callback, 10)
        self.subscriberPOSE_  # prevent unused variable warning
        #self.subscriberTWA_ = self.create_subscription(WindMessage, '/', self.twa_callback, 10) #subscription for true wind angle
        #self.subscriberTWA_ # prevent unused variable warning
        #subscription for path. previous_waypoint and next_waypoint

        #create publishers
        self.publisherRudderAngle_ = self.create_publisher(RudderControlMessage, '/actuator/rudder', 10)

        #create timers (callbacks triggered by timers)
        period = 0.1 #for a 10hz system
        self.rudderControl = self.create_timer(period, self.rudder_control_callback)
        self.navigation = self.create_timer(period, self.navigation_callback)


        #create varibles
        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.yaw = 0.0
        self.velocity = 0.0
        self.twa = 0.0

    #def PATH_FOLLOWER_callback(self, msg):

    def navigation_callback(self):
        o = np.array([11,1]) #self.current_position   #boats actual position in lat/long
        b = np.array([4,3]) #self.path.a        #previous waypoint
        a = np.array([20,8]) #self.path.a       #next waypoint

        lookahead_distance = 4 #self.lookahead_d

        #CONVERT lookahead_distance IN METERS TO LAT/LONG distance. 1 degree latitude is approximately 111 km(differs on where on the earth someone is). (*1/111000)

        # ALL ANGLES MUST BE IN RADIANS!!!!!!!!!!

        desired_angle, los_point = los_algorithm(o,a,b,lookahead_distance)

        desired_angle = np.rad2deg(desired_angle)#convert angle from radians to degrees

        self.get_logger().info('Desired angle is %f and desired los-point is (%f , %f)' % (desired_angle, los_point[0], los_point[1])) #push message to console

        #adjust the desired heading angle so that it is not in the "no go zone"
        #adjusted_angle = adjust_angle_to_wind(desired_angle)



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

    #TWA subsriber callback used for getting the True Wind Angle
    def twa_callback(self,msg):
        self.twa = msg.twa



#Returns the desired boat heading angle in radians determined using a LOS-algorithm 
def los_algorithm(current_position, previous_waypoint, next_waypoint, lookahead_distance):
    o = current_position #boats actual position in lat/long
    a = previous_waypoint #previous waypoint
    b = next_waypoint#next waypoint

    # ALL ANGLES MUST BE IN RADIANS!!!!!!!!!!

    #get the lateral_distance_point s. 
    lateral_distance_point = get_lateral_distance_point(o, a, b)
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
    
    #calculate counter clockwise angle between vectors op and north_vector
    one = oN
    two = op
    dot = one[0]*two[0]+one[1]*two[1]
    det = one[0]*two[1]-one[1]*two[0]
    desired_angle = np.arctan2(det,dot)
    desired_angle = (desired_angle+2*np.pi)%(2*np.pi)

    return desired_angle, p
    

#gives the lateral_distance_point which is the point where if one made a perpendicular line from the line ab touching the point current position
def get_lateral_distance_point(current_position, previous_point, next_point):
    o = current_position    #boats actual position in lat/long
    a = previous_point      #previous waypoint
    b = next_point          #next waypoint

    #path vectors
    ab = b-a #path vector from point a to b
    ao = o-a #path vector from point a to o

    #get the lateral distance point
    s = a + np.dot(ao,ab)/np.dot(ab,ab) * ab

    return s




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
