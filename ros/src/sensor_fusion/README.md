# Sensor fusion
This application creates a sensor fusion between the IMU and the GPS using EKF. It tries to give a better estimation of orientation and position.

The version of ROS2 is Foxy desktop on Ubuntu 20.04. The application will read from /position/IMU and /position/GPS, and publish a Float32MultiArray [x y yaw v] to /position/fusion.

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

Download and play this bag for testing: https://github.com/AutoSail-MDH/AutoSail-HT21/tree/main/ROS2/Testing/rosbag2_combined

Put the bag folder in the source of your workspace.

Run the following commands at the source of your workspace with your ROS2 version sourced:
```
. install/local_setup.bash
ros2 bag play [BAG_FOLDER_NAME]
```
