# GPS
This application is creates a ROS2 node that publishes Lat/Long values to a topic on a ROS network. These values come from the GPS NEO-M9N. The version of ROS2 is foxy desktop on ubuntu 20.04. The node publishes to the topic `/sensor/gps`

## Hardware connection
The connections from the GPS to ESP32 using one of the Qwiic connectors:

SDA(Blue cable) is connected to GPIO 21 

SCL(Yellow cable) is connected to GPIO 22 

3V3(Red cable) is connected to 3V3

GND(Black cable) is connected to GND
