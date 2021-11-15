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
Source ROS2, then open up a terminal at the source of your ROS2 workspace.

Run the following commands:
```
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --packages-select vessel_orientation
. install/local_setup.bash
ros2 run vessel_orientation sub_pos_gps
```
