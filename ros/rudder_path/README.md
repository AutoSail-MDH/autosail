# Information

This application sends the angle for which to set the rudder, so it cna steer towards a goal position. There are three inputs to this application:

GOAL position in Lat/Long coordinates, which is the coordinates for which the boat should steer towards. This data is aquired via the topic `/positiion/GOAL`.

Current position in Lat/Long coordinates, which is the current coordinates for the boat. This data is aquired via the topic `/position/GPS`.

Current heading in degrees, which is the current heading of the boat. This data is aquired via the topic `/positiion/IMU`.

## Prerequisites

This application assumes the topics mentioned for each of the inputs contains data.

## Setup

Place the `rudder_path` folder inside your ROS2 workspace, in the src folder

```bash
cd ~/$ROS2_WORKSPACE/

colcon build

. install/setup.bash
```

## Usage

You have to source the setup file for each new terminal you use.

```bash
cd ~/$ROS2_WORKSPACE/

. install/setup.bash

ros2 run rudder_path rpp
```
In a new terminal:

```bash
cd ~/$ROS2_WORKSPACE/

. install/setup.bash

ros2 run rudder_path pub
```
The pub executable publishes the goal position, which is the Lat/Long values of where the goal position is. You can change this in real time. In a new terminal:

```bash
.cd ~/$ROS2_WORKSPACE/

. install/setup.bash

ros2 param set goal_pub lat $Latitude_Value

ros2 param set goal_pub long $Longitude_Value

```
Make sure that the value is of the float format, which for example means written as `12.0` and not `12`. If it is not written as a decimal number, the program will crash. The rudder angle is set to zero(Steering straight ahead) when the heading of the boat and the angle of the goal are close to eachother. To tweak how close these two values must be for this to trigger, change the parameter `p` like this:
```bash
ros2 param set rudder_angle p $threshhold_value
```
Just like above, this value should also be a float value, so written on the format `XX.X`, such as `12.0`.
