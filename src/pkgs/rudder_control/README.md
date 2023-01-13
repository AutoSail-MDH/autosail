# Information
This package is no longer used, see path_follower instead.


This application sends the angle for which to set the rudder, so it can steer towards a goal position. There are three inputs to this application:


Next position in Lat/Long coordinates, which is the coordinates for which the boat should steer towards. This data is aquired from set_next_position.cpp via the topic `/path/next_position`.
Current position in Lat/Long coordinates, which is the current coordinates for the boat. This data is aquired from via the topic `/sensor/gnss`.

Current heading in degrees, which is the current heading of the boat. This data is aquired via the topic `/sensor/imu`.

## Prerequisites

This application assumes the topics mentioned for each of the inputs contains data.

## Usage

You have to source the setup file for each new terminal you use.

```bash
cd ~/ros2_ws/autosail

. install/setup.bash

ros2 run rudder_control rudder_control
```
In a new terminal:

```bash
cd ~/ros2_ws/autosail

. install/setup.bash

ros2 run rudder_control set_next_position
```
The set_next_position executable publishes the next position, which is the Latitude/Longitude values of where the next position is. You can change this in real time. I`/path/next_position`n a new terminal:

```bash
cd ~/ros2_ws/autosail

. install/setup.bash

ros2 param set set_next_position_node lat [Value]

ros2 param set set_next_position_node long [Value]

```
Make sure that the value is of the float format, which for example means written as `12.0` and not `12`. If it is not written as a decimal number, the program will crash. The rudder angle is set to zero(Steering straight ahead) when the heading of the boat and the angle of the goal are close to eachother. To tweak how close these two values must be for this to trigger, change the parameter `p` like this:
```bash
ros2 param set rudder_control_node p [Value]
```
Just like above, this value should also be a float value, so written on the format `XX.X`, such as `12.0`.
