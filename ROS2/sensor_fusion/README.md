# Sensor fusion
This application creates a sensor fusion between the IMU and the GPS using EKF. It tries to give a better estimation of the velocity and the position.

The version of ROS2 is Foxy desktop on Ubuntu 20.04. The application will read from /position/IMU and /boat/velocity, and publish a Float32MultiArray [v x y] to /position/fusion.

## Prerequsities
This guide needs the user to install ROS2 Foxy and create a ROS2 workspace:

https://docs.ros.org/en/foxy/Installation.html

https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html

## Usage

Place the sensor_fusion folder inside src in the ROS2 workspace.

### Setup and run
Open up a terminal at the source of your ROS2 workspace and source your ROS2 version.

Run the following commands to start the application:
```
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --packages-select sensor_malfunction
. install/local_setup.bash
ros2 launch sensor_malfunction malfunction_launch.py
```


For this to function it needs to receive IMU and GPS data.

Download a IMU bag here: https://github.com/AutoSail-MDH/AutoSail-HT21/tree/vessel_orientation/ROS2/Testing/bag_files/rosbag2_MPU6050_serial

Download a velocity bag here: https://github.com/AutoSail-MDH/AutoSail-HT21/tree/vessel_orientation/ROS2/Testing/bag_files/rosbag2_vessel_velocity

Put the bag folders in the source of your workspace.

Run the following commands at the source of your workspace with your ROS2 version sourced:
```
. install/local_setup.bash
ros2 bag play [BAG_FOLDER_NAME]
```
