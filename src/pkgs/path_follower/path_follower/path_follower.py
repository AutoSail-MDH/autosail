import rclpy
from rclpy.node import Node

from autosail_message.msg import PoseMessage
from autosail_message.msg import RudderControlMessage
from autosail_message.msg import WindMessage
from autosail_message.msg import PositionMessage


import math
import numpy as np

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        # Create subscriptions
        self.subscriberPOSE_ = self.create_subscription(PoseMessage, '/position/pose', self.pose_callback, 10)
        self.subscriberPOSE_  # prevent unused variable warning
        self.subscriberTWA_ = self.create_subscription(WindMessage, '/sensor/true_wind', self.twa_callback, 10) #subscription for true wind angle
        self.subscriberTWA_ = self.create_subscription(WindMessage, '/sensor/wind', self.wind_callback, 10) #subscription for true wind angle
        self.subscriberTWA_ # prevent unused variable warning
        self.subscriberNextPos_ = self.create_subscription(PositionMessage, '/path/next_waypoint', self.next_waypoint_callback, 10)
        self.subscriberNextPos_
        self.subscriberPreviousPos_ = self.create_subscription(PositionMessage, '/path/prev_waypoint', self.prev_waypoint_callback, 10)
        self.subscriberPreviousPos_

        # Create publishers
        self.publisherRudderAngle_ = self.create_publisher(RudderControlMessage, '/actuator/rudder', 10)

        # Create timers (callbacks triggered by timers)
        period = 0.1 #for a 10hz system
        self.rudderControl = self.create_timer(period, self.rudder_control_callback)
        self.navigation = self.create_timer(period, self.navigation_callback)

        self.debugcallback = self.create_timer(period, self.debug_callback)

        # Create varibles
        self.current_position = np.array([59.59827, 16.55583])#longitude/latitude
        self.yaw = 0.0
        self.velocity = 0.0
        self.twa = 0.0
        self.awa = 0.0
        self.desired_heading_angle = 0.0
        self.previous_waypoint = np.array([59.637053 , 16.583962])#longitude/latitude
        self.next_waypoint = np.array([59.637212 , 16.584512])#longitude/latitude
        self.lookahead_distance = 4#in meters

        self.rudderAngle = 0
        
        # Init PID
        self.pid_controller = PID(0.3, 0.0, 0.1)
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

        ####ONLY FOR DEMO
        adjusted_angle = np.rad2deg(get_cc_angle(np.array([99,0]), b-o))
        ####ONLY FOR DEMO

        # Set desired angle to be used by rudder_control_callback
        self.desired_heading_angle = adjusted_angle


    ## Rudder control callback. Triggered by timer and publishes rudder angle to be set
    def rudder_control_callback(self):
        message = RudderControlMessage()

        desired_angle = self.desired_heading_angle
        current_angle = self.yaw

        #convert to +-180
        desired_angle = (desired_angle+180)%360-180
        current_angle = (current_angle+180)%360-180

        message.rudder_angle = 0.0 
        
        time_now = self.get_clock().now()#get current time
        time_sec = time_now.nanoseconds*0.000000001#get time in seconds

        #use PID which outputs wanted rudder angle
        rudder_angle = self.pid_controller.send([desired_angle,current_angle,time_sec])

        #FLIP value to whats wanted by the rudder motor
        rudder_angle = rudder_angle*(-1)

        maxangle = 45.0#rudder max angle +-

        if rudder_angle < -maxangle:
            rudder_angle = -maxangle
        elif rudder_angle > maxangle:
            rudder_angle = maxangle

        message.rudder_angle = rudder_angle #add rudder angle to message

        self.publisherRudderAngle_.publish(message) #add message to publisher

        self.rudderAngle = rudder_angle

    #Pose subscriber used for getting pose and velocity of boat
    def pose_callback(self, msg):
        self.current_position[0] = msg.position.latitude
        self.current_position[1] = msg.position.longitude 
        self.yaw = msg.yaw
        self.velocity = msg.velocity

    #TWA subsriber callback used for getting the True Wind Angle
    def twa_callback(self,msg):
        self.twa = msg.wind_angle

    #Subrscribers that take new waypoints
    def next_waypoint_callback(self,msg):
        self.next_waypoint = np.array([msg.latitude,msg.longitude])
    def prev_waypoint_callback(self,msg):
        self.previous_waypoint = np.array([msg.latitude,msg.longitude])

    def wind_callback(self,msg):
        self.awa = msg.wind_angle

    #ONLY TO BE USED FOR DEVELOPMENT
    
    def heading_callback(self,msg):#heading
        self.yaw = msg.yaw
    def debug_callback(self):
        self.get_logger().info('----------------------------------------') #push message to console
        self.get_logger().info('Heading:%f' % self.yaw) #push message to console
        self.get_logger().info('Desired heading:%f' % self.desired_heading_angle) #push message to console
        self.get_logger().info('Rudder angle:%f' % self.rudderAngle) #push message to console
        self.get_logger().info('True wind angle:%f' % self.twa) #push message to console



#Returns the desired boat heading angle in radians determined using a LOS-algorithm 
def los_algorithm(current_position:float, previous_waypoint:float, next_waypoint:float, lookahead_distance:float):
    o = current_position #boats actual position in lat/long
    a = previous_waypoint #previous waypoint
    b = next_waypoint#next waypoint

    oN = np.array([99,0])# lat,long vector pointing north(length does not matter)

    #ADD CODE IN CASE POINT A IS THE EXACT SAME AS POINT B. SIMPLY RETURN CCangle TO THAT POINT
    if (a == b).all():
     return get_cc_angle(oN,a-o)#get angle between desired point and currens position

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
