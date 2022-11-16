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

        # Create subscriptions
        self.subscriberPOSE_ = self.create_subscription(PoseMessage, '/position/pose', self.pose_callback, 10)
        self.subscriberPOSE_  # prevent unused variable warning
        #self.subscriberTWA_ = self.create_subscription(WindMessage, '/sensor/true_wind', self.twa_callback, 10) #subscription for true wind angle
        self.subscriberTWA_ = self.create_subscription(WindMessage, '/sensor/wind', self.wind_callback, 10) #ONLY FOR DEVELOPMENT! subscription for wind angle
        self.subscriberTWA_ # prevent unused variable warning
        #subscription for path. previous_waypoint and next_waypoint

        # Create publishers
        self.publisherRudderAngle_ = self.create_publisher(RudderControlMessage, '/actuator/rudder', 10)

        # Create timers (callbacks triggered by timers)
        period = 0.1 #for a 10hz system
        self.rudderControl = self.create_timer(period, self.rudder_control_callback)
        self.navigation = self.create_timer(period, self.navigation_callback)

        # Create varibles
        self.current_position = np.array([59.637171 , 16.584125])#longitude/latitude
        #self.current_position = np.array([59.637020 , 16.584150]) #second boat position test point
        self.yaw = 0.0
        self.velocity = 0.0
        self.twa = 0.0
        self.desired_heading_angle = 0.0
        self.previous_waypoint = np.array([59.637053 , 16.583962])#longitude/latitude
        self.next_waypoint = np.array([59.637212 , 16.584512])#longitude/latitude
        self.lookahead_distance = 4#in meters
        
        # Init PID
        self.pid_controller = PID(0.00000000001, 0.00000000001, 0.00000000001)
        self.pid_controller.send(None)

    def navigation_callback(self):
        # Parameters and waypoints
        o = self.current_position#boats actual position in lat/long
        a = self.previous_waypoint
        b = self.next_waypoint
        lookahead_distance = self.lookahead_distance
        no_go_zone = 45.0
        
        # Get desired boat heading angle from los-algorithm. 
        desired_angle, los_point, s, sb_angle, sb = los_algorithm(o,a,b,lookahead_distance)
        desired_angle = np.rad2deg(desired_angle)   #Convert angle from radians to degrees

        # Adjust the desired heading angle so that it is not in the "no go zone"
        adjusted_angle = adjust_angle_to_wind(self.twa,desired_angle,no_go_zone)

        # Set desired angle to be used by rudder_control_callback
        self.desired_heading_angle = adjusted_angle

        #DEBUG outputs
        self.get_logger().info('----------------------------------------') #push message to console
        self.get_logger().info('Desired lateral-point is: (%f , %f)' % (s[0], s[1])) #push message to console
        self.get_logger().info('Desired los-point is: (%f , %f)' % (los_point[0], los_point[1])) #push message to console
        self.get_logger().info('Desired SB is: (%f , %f)' % (sb[0], sb[1])) #push message to console
        self.get_logger().info('Angle SB in deg: \t %f' % (np.rad2deg(sb_angle))) #push message to console
        self.get_logger().info('Desired angle is: \t %f' % (desired_angle)) #push message to console
        self.get_logger().info('Adjusted angle is:\t %f' % (adjusted_angle)) #push message to console
        self.get_logger().info('True wind angle is:    \t %f)' % (self.twa)) #push message to console


    ## Rudder control callback. Triggered by timer and publishes rudder angle to be set
    def rudder_control_callback(self):
        message = RudderControlMessage()

        desired_angle = self.desired_heading_angle
        current_angle = self.yaw

        message.rudder_angle = 1337.0 
        
        timeNow = self.get_clock().now()#get current time

        #use PID which outputs wanted rudder angle
        rudder_angle = self.pid_controller.send([desired_angle,current_angle,timeNow.nanoseconds])

        message.rudder_angle = rudder_angle #add rudder angle to message

        self.publisherRudderAngle_.publish(message) #add message to publisher

        #DEBUG outputs
        #self.get_logger().info('%f' % message.rudder_angle) #push message to console
        #self.get_logger().info('Rudder Control says:%f' % message.rudder_angle) #push message to console
        #self.get_logger().info('Rudder Control says:%f' % message.rudder_angle) #push message to console

    #Pose subscriber used for getting pose and velocity of boat
    def pose_callback(self, msg):
        self.current_latitude = msg.position.latitude
        self.current_longitude = msg.position.longitude 
        self.yaw = msg.yaw
        self.velocity = msg.velocity

    #TWA subsriber callback used for getting the True Wind Angle
    def twa_callback(self,msg):
        self.twa = msg.twa

    #ONLY TO BE USED FOR DEVELOPMENT
    def wind_callback(self,msg):
        self.twa = msg.wind_angle


#Returns the desired boat heading angle in radians determined using a LOS-algorithm 
def los_algorithm(current_position:float, previous_waypoint:float, next_waypoint:float, lookahead_distance:float):
    o = current_position #boats actual position in lat/long
    a = previous_waypoint #previous waypoint
    b = next_waypoint#next waypoint

    oN = np.array([99,0])# lat,long vector pointing north(length does not matter)

    #get the lateral_distance_point s. 
    lateral_distance_point = get_lateral_distance_point(o, a, b)
    s = lateral_distance_point

    #use the lateral_distance_point s to create vector sb. This gives only a vector and its length. not the actual position p
    sb = b-s

    beari = get_cc_angle(oN,sb)
    beari = 2*np.pi-beari#convert to clockwise as bearing angles are the clockwise angle from north

    p = new_point_from_distance(s,beari,lookahead_distance)

    # Set the boats position as Origo[0,0] and make a vector north. Set a new point p with the boats current position o as origo
    #the desired angle will then be the angle between the north axis and op 
    op = p-o
    
    #calculate angle between vectors op and north_vector
    desired_angle = get_cc_angle(oN,op)

    return desired_angle, p, s, beari, sb

#Gives the lateral_distance_point which is the point where if one made a perpendicular line from the line ab touching the point current position
def get_lateral_distance_point(current_position, previous_point, next_point):#Using euclidean calculations
    o = current_position    #boats actual position in lat/long
    a = previous_point      #previous waypoint
    b = next_point          #next waypoint

    #path vectors
    ab = b-a #path vector from point a to b
    ao = o-a #path vector from point a to o

    #get the lateral distance point
    s = a + np.dot(ao,ab)/np.dot(ab,ab) * ab

    return s


#Returns adjusted desired_angle to be outside the sailing "No go zone"
def adjust_angle_to_wind(twa:float,desired_angle:float,no_go_zone:float):
    adjusted_angle = desired_angle

    left_bound = twa - no_go_zone/2
    right_bound = twa + no_go_zone/2

    # The adjusted angle is very unstable as the wind sensor gives very unstable values. Could use filter
    # Maybe save previous output with yield statement such as in PID

    # Adjust angle to be outside the no go zone
    if(desired_angle > left_bound and desired_angle <= twa):
        adjusted_angle = left_bound
    elif(desired_angle >= twa and desired_angle < right_bound):
        adjusted_angle = right_bound
    else:
        adjusted_angle = desired_angle

    return adjusted_angle

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

#Get cc angle between (lat , long) vectors
def get_cc_angle(vector1:np.array,vector2:np.array):
    dot = vector1[1]*vector2[1]+vector1[0]*vector2[0]
    det = vector1[1]*vector2[0]-vector1[0]*vector2[1]
    angle = np.arctan2(det,dot)
    angle = (angle)%(2*np.pi)
    return angle

#PID controller returns controlled variable. Initial control state can be set
def PID(Kp, Ki, Kd, initial_control_state = 0):
    time_previous = 0
    error_previous = 0

    i = 0 #integral sum

    controlled_v = initial_control_state

    while True:
        # yield statement outputs controlled variable and pauses loop. When function called again the loop continues from that point with new inputs
        set_point, process_variable, t = yield controlled_v
        error = set_point-process_variable

        #PID formula
        p = Kp*error
        i = i+Ki*error*(t-time_previous)
        d = Kd*(error-error_previous)/(t-time_previous)
        controlled_v = initial_control_state + p+i+d

        #save values
        time_previous = t
        error_previous = error
        

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
