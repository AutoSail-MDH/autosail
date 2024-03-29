import rclpy
import numpy as np
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import cartopy.crs as ccrs
#from mpl_toolkits.basemap import Basemap

from std_msgs.msg import Float32MultiArray

class DataVisualisation(Node):

    def __init__(self):
        super().__init__('data_visualisation_node') 
        
        self.pubVel_ = self.create_subscription(
            Float32MultiArray, '/position/velocity', self.velocity_callback, 10)
        self.pubVel_ # prevent unused variable warning

        self.pubYaw_ = self.create_subscription(
            Float32MultiArray, '/sensor/imu', self.yaw_callback, 10)
        self.pubYaw_ # prevent unused variable warning

        self.pubGPS_ = self.create_subscription(
            Float32MultiArray, '/sensor/wind', self.wind_callback, 10)
        self.pubGPS_ # prevent unused variable warning

        self.pubGPS_ = self.create_subscription(
            Float32MultiArray, '/actuator/rudder', self.rudder_callback, 10)
        self.pubGPS_ # prevent unused variable warning

        self.pubGPS_ = self.create_subscription(
            Float32MultiArray, '/actuator/sail_angle', self.sail_callback, 10)
        self.pubGPS_ # prevent unused variable warning

        self.pubGPS_ = self.create_subscription(
            Float32MultiArray, '/sensor/gnss', self.gps_callback, 10)
        self.pubGPS_ # prevent unused variable warning
        

        style.use('ggplot') #Style; can be changed but I like this one :)

        self.fig1 = plt.figure() #Fig1 is velocity
        self.ax1 = self.fig1.add_subplot(2,1,1) #Only one subplot for velocity
        #Fig2 holds four subplots: Yaw, Wind, Sail, Rudder
        self.fig2, ((self.ax2, self.ax3), (self.ax4, self.ax5)) = plt.subplots(2, 2, subplot_kw=dict(projection= 'polar'))

        self.ax6 = self.fig1.add_subplot(2,1,2) #Add GPS to first figure, second subplot

        #Create a map for converting coordinates to plottable points
        #Change lat_0 and lon_0 to approximate current location for more accurate plotting (if run in Panama or Colombia)
        #self.m = Basemap(projection='lcc', resolution=None, width=8E6, height=8E6, lat_0=59, lon_0=16) 

        #Initialize variables
        self.Velxs = []
        self.Velys = []
        self.YawX = []
        self.WindX = []
        self.RudderX = []
        self.SailX = []
        self.DrawYaw = 0
        self.DrawWind = 0
        self.DrawRudder = 0
        self.DrawSail = 0

        #Easiest way to draw a line?
        self.PolarY = np.linspace(0, 1, 10) #For all polar plots
        self.SailY = np.linspace(0, 0.5, 10) #For extra sail angle in wind plot

        #Set velocity x to [1, 20]
        for i in range(20):
            self.Velxs.append(i+1)
            self.Velys.append(0)

        #Create plots and put into foreground
        plt.draw()
        plt.pause(0.001)
    
    def velocity_callback(self, msg):
        """Runs whenever velocity data is recieved 

        Args:
            msg (float32multiarray): Holds some other stuff but we only use velocity in msg.data[0]
        """        
        self.get_logger().info('Vel %f' % (msg.data[0])) #VXYDT; Receive data. 0=Velocity, 1=X, 2=Y, 3=dt?
    
        #Shift old Y values back
        for i in range(19):
            self.Velys[i] = self.Velys[i+1]
    
        #Set latest datapoint
        self.Velys[19] = msg.data[0]
    
        #Animation code
        ani = animation.FuncAnimation(self.fig1, self.animateVel, interval=1000)
        self.fig1.canvas.draw_idle() #Use draw_idle to specifically draw fig1 and not all plots to lessen strain
        self.fig1.canvas.flush_events() #Use this instead of pause since pause forces plot to foreground, which is super annoying.

    
    def yaw_callback(self, msg):
        """Runs whenever yaw data is recieved 

        Args:
            msg (float32multiarray): Holds some other stuff but we only use yaw in msg.data[0]
        """  

        self.get_logger().info('Yaw %f' % (msg.data[0])) #YPR; Receive data. 0=Yaw, 1=Pitch, 2=Roll

        #Read and convert to radians
        self.DrawYaw = msg.data[0]
        self.DrawYaw = np.radians(self.DrawYaw)
    
        #Animation updates the plots once run
        ani = animation.FuncAnimation(self.fig2, self.animateYaw, interval=1000)
        self.fig2.canvas.draw_idle() 
        self.fig2.canvas.flush_events()

    def wind_callback(self, msg):
        """Runs whenever wind data is recieved 

        Args:
            msg (float32multiarray): Holds some other stuff but we only use wind direction in msg.data[0]
        """  

        self.get_logger().info('Wind %f' % (msg.data[0])) #Wind direction, raw

        #Read and convert to radians
        self.DrawWind = msg.data[0]
        self.DrawWind= np.radians(self.DrawWind)

        #Animation updates the plots once run
        ani = animation.FuncAnimation(self.fig2, self.animateWind, interval=1000)
        self.fig2.canvas.draw_idle()
        self.fig2.canvas.flush_events()

    def rudder_callback(self, msg):
        """Runs whenever rudder angle data is recieved 

        Args:
            msg (float32multiarray): Holds only rudder angle in msg.data[0]
        """  
        
        self.get_logger().info('Rudder %f' % (msg.data[0])) #Recieve the set rudder angle

        #Read and convert to radians (Should be radiands already)
        self.DrawRudder = msg.data[0]
        #self.DrawRudder = np.radians(self.DrawRudder)

        #Animation updates the plots once run
        ani = animation.FuncAnimation(self.fig2, self.animateRudder, interval=1000)
        self.fig2.canvas.draw_idle()
        self.fig2.canvas.flush_events()

    def sail_callback(self, msg):
        """Runs whenever sail angle data is recieved 

        Args:
            msg (float32multiarray): Holds only sail angle in msg.data[0]
        """  

        self.get_logger().info('Sail %f' % (msg.data[0])) #Recieve the set sail angle

        #Read and convert to radians (Should be radiands already)
        self.DrawSail = msg.data[0]
        #self.DrawSail = np.radians(self.DrawSail)

        #Animation updates the plots once run
        ani = animation.FuncAnimation(self.fig2, self.animateSail, interval=1000)
        self.fig2.canvas.draw_idle()
        self.fig2.canvas.flush_events()


    def gps_callback(self, msg):
        """Runs whenever position data is recieved 

        Args:
            msg (float32multiarray): Holds Latitude in msg.data[0] and Longitude in msg.data[1]
        """  

        self.get_logger().info('Lat: %f' % (msg.data[0])) #LAT, LON

        #Read values
        self.GPSlat = msg.data[0]
        self.GPSlon = msg.data[1]

        #Convert to meters for plotting using the basemap m set up during initialization
        self.xLat, self.yLon = self.m(self.GPSlat, self.GPSlon)

        #Animation updates the plots once run
        ani = animation.FuncAnimation(self.fig1, self.animateGPS, interval=1000)
        self.fig1.canvas.draw_idle()
        self.fig1.canvas.flush_events()




    def animateVel(self, i):
        self.ax1.clear() #Clear old data
        self.ax1.plot(self.Velxs, self.Velys) #Plot new data

    def animateYaw(self, i):
    
        self.YawX = [] #Empty YawX array
        
        #Set every entry in YawX to DrawYaw just to have the same dimensions as PolarY
        for i in range(10):
            self.YawX.append(self.DrawYaw)

        self.ax2.clear() #Clear old data
        self.ax2.plot(self.YawX, self.PolarY) #Plot new data


        self.ax2.set_title("Yaw") #Set title
        self.ax2.set_yticklabels([]) #Remove tick labels since we only care about direction
        self.ax2.set_theta_zero_location('N') #Set 0 to north

    def animateWind(self, i):

        self.WindX = []
        self.windSail = [] #To plot sail in same subplot as wind direction
        
        for i in range(10):
            self.WindX.append(self.DrawWind)
            self.windSail.append(self.DrawSail - np.pi) #Since wind and sail have different zeros we add pi 

        self.ax3.clear()
        self.ax3.plot(self.WindX, self.PolarY)
        self.ax3.plot(self.windSail, self.SailY, 'b') #Plot Sail in blue in the same plot


        self.ax3.set_title("Wind") #Set title
        self.ax3.set_yticklabels([]) #Remove tick labels since we only care about direction
        self.ax3.set_theta_zero_location('N') #Set 0 to north

    def animateRudder(self, i):

        self.RudderX = []

        for i in range(10):
            self.RudderX.append(self.DrawRudder)

        self.ax4.clear()
        self.ax4.plot(self.RudderX, self.PolarY)

        self.ax4.set_title("Rudder") #Set title
        self.ax4.set_yticklabels([]) #Remove tick labels since we only care about direction
        self.ax4.set_theta_zero_location('S') #Set 0 to north

    def animateSail(self, i):

        self.SailX = []

        #Set every entry in YawX to DrawYaw just to have the same dimensions
        for i in range(10):
            self.SailX.append(self.DrawSail)

        self.ax5.clear()
        self.ax5.plot(self.SailX, self.PolarY)

        self.ax5.set_title("Sail") #Set title
        self.ax5.set_yticklabels([]) #Remove tick labels since we only care about direction
        self.ax5.set_theta_zero_location('S') #Set 0 to north

    def animateGPS(self, i):

        self.ax6.plot(self.xLat, self.yLon, 'ok', markersize=5) #Don't clear old points for the GPS plots, simply add the new one

def main(args=None): 
    rclpy.init(args=args)

    data_visualisation_node = DataVisualisation()

    try:
        rclpy.spin(data_visualisation_node)
    except KeyboardInterrupt:
        print("CTRL-C: KeyboardInterrupt registered")
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        data_visualisation_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
