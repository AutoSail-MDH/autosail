# Information

This application sends the angle for which to set the rudder, so it cna steer towards a goal position. There are three inputs to this application:

GOAL position in Lat/Long coordinates, which is the coordinates for which the boat should steer towards. This data is aquired via the topic `/positiion/GOAL`.

Current position in Lat/Long coordinates, which is the current coordinates for the boat. This data is aquired via the topic `/position/GPS`.

Current heading in degrees, which is the current heading of the boat. This data is aquired via the topic `/positiion/IMU`.

## Prerequisites

This application assumes the topics mentioned for each of the inputs is receiving data.

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
. install/setup.bash

ros2 run rudder_path rpp
```
