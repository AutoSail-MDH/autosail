# Vessel velocity
This application creates a ROS2 node that subscribes to the lat, lon (Float32MultiArray) values on a ROS2 topic, then calculates and publishes the velocity.

Distance formula: http://en.wikipedia.org/wiki/Haversine_formula

The version of ROS2 is Foxy desktop on Ubuntu 20.04. The node subscribes to the topic `/position/GPS` and publishes to the topic `/boat/velocity`.


## Prerequsities
This guide needs the user to install ROS2 Foxy and create a ROS2 workspace:

https://docs.ros.org/en/foxy/Installation.html

https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html

## Usage

Place the vessel_velocity folder inside src in the ROS2 workspace.

### Setup and run
Open up a terminal at the source of your ROS2 workspace and source your ROS2 version.

Run the following commands to start the application:
```
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --packages-select vessel_orientation
. install/local_setup.bash
ros2 run vessel_orientation sub_pos_gps
```

For it to actually function it needs to receive GPS data.

Download one of the ROS2 bags with GPS data here: https://github.com/AutoSail-MDH/AutoSail-HT21/tree/vessel_position/ROS2/Testing/gps_test_validation/bag_files

Put the bag folder in the source of your workspace.

Run the following commands at the source of your workspace with your ROS2 version sourced:
```
. install/local_setup.bash
ros2 bag play [BAG_FOLDER_NAME]
```
