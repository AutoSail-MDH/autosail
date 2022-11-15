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
        self.current_position = np.array([0.00 , 0.00])#longitude/latitude
        self.yaw = 0.0
        self.velocity = 0.0
        self.twa = 0.0
        self.desired_heading_angle = 0.0
        self.previous_waypoint = np.array([0.00 , 0.00])#longitude/latitude
        self.next_waypoint = np.array([0.00 , 0.00])#longitude/latitude
        
        # Init PID
        self.pid_controller = PID(0.00000000001, 0.00000000001, 0.00000000001)
        self.pid_controller.send(None)

    #TROR INTE DETTA BEHÖVS
    #def PATH_FOLLOWER_callback(self, msg):

    def navigation_callback(self):
        # Parameters and waypoints
        o = self.current_position
        a = self.previous_waypoint
        b = self.next_waypoint
        lookahead_distance = 4 #self.lookahead_d
        no_go_zone = 45.0
        
        #TESTING
        o = np.array([11,1]) #self.current_position   #boats actual position in lat/long
        b = np.array([4,3]) #self.path.a        #previous waypoint
        a = np.array([20,8]) #self.path.a       #next waypoint
        

        # CONVERT lookahead_distance IN METERS TO LAT/LONG distance. 1 degree latitude is approximately 111 km(differs on where on the earth someone is). (*1/111000)

        # Get desired boat heading angle from los-algorithm. 
        desired_angle, los_point = los_algorithm(o,a,b,lookahead_distance)
        desired_angle = np.rad2deg(desired_angle)   #Convert angle from radians to degrees

        # Adjust the desired heading angle so that it is not in the "no go zone"
        adjusted_angle = adjust_angle_to_wind(self.twa,desired_angle,no_go_zone)

        # Set desired angle to be used by rudder_control_callback
        self.desired_heading_angle = adjusted_angle

        #DEBUG outputs
        #self.get_logger().info('Desired los-point is: (%f , %f)' % (los_point[0], los_point[1])) #push message to console
        #self.get_logger().info('Desired angle is: \t %f' % (desired_angle)) #push message to console
        #self.get_logger().info('Adjusted angle is:\t %f' % (adjusted_angle)) #push message to console
        #self.get_logger().info('True wind angle is:    \t %f)' % (self.twa)) #push message to console


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
        self.get_logger().info('Rudder Control says:%f' % message.rudder_angle) #push message to console

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
    

#Gives the lateral_distance_point which is the point where if one made a perpendicular line from the line ab touching the point current position
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
