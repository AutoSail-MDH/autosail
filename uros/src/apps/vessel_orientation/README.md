# Vessel Orientation
This application creates a micro-ROS node that publishes roll, pitch, yaw values as well as acceleration values to a ROS2 topic using an agent.

These values come from the Adafruit BNO055 9-axis Abs. orientation IMU.

The version of ROS2 is Foxy desktop on Ubuntu 20.04. The node publishes to the topic `/position/imu`

## Hardware connection
The connections from the IMU to ESP32 is:

SDA - 21.

SCL - 22.

VIN - 3v3.

GND - GND.
