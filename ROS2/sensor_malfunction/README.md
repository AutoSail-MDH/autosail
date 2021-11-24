# Sensor malfunction
This application creates multiple nodes to monitor topics for publishers, sending log messages for shutdown request if the publisher is disconnected for too long. The application also starts a shutdown node to monitor the log messages, which executes a user-specified shell command if a log message with severity level 50 (FATAL) is read.

The version of ROS2 is Foxy desktop on Ubuntu 20.04. The monitor nodes will read publisher count from user specified topics and send log messages to the default topic /rosout, while the shutdown node will read from /rosout.


## Prerequsities
This guide needs the user to install ROS2 Foxy and create a ROS2 workspace:

https://docs.ros.org/en/foxy/Installation.html

https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html

## Usage

Place the sensor_malfunction folder inside src in the ROS2 workspace.

### Setup and run
Open up a terminal at the source of your ROS2 workspace and source your ROS2 version.

Run the following commands to start the application:
```
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --packages-select sensor_malfunction
. install/local_setup.bash
ros2 launch sensor_malfunction malfunction_launch.py
```

If all publishers are connected, nothing will happen.
