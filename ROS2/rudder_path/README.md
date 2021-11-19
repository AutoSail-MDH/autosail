# Rudder Path Pid

This module reads the current heading of the boat(Yaw value from hte IMU), computes the angle2Goal and sets the rudder at an angle as to steer towards the goal angle.
The node reads from the topic `/position/IMU` and publishes to `/position/RUDDER_ANGLE`. This also assumes the `vessel_orientation` module is running and sending data to the `/position/IMU` topic.

## Prerequisites

This application requires ROS2. The version Foxy was used during development and testing. It also requires that a ros2 workspace is created properly.

## Usage

Place the folder `rudder_path` inside the src folder of your ros2 workspace. 

### Setup

Do these the first time you use this module
```
cd ~/$ros2_Workspace_Path/src/

cd ~/$ros2_Workspace_Path

colcon build rudder_path

. install/setup.bash
```
### Post setup

After doing the setup once, you must do this for every new terminal

```
cd ~/$ros2_Workspace_Path/

. install/setup.bash

```
Run the module

```
ros2 run rudder_path rpp
```

### Verification

Below is two ways of checking what the node is publishing, one shows just the data, and one shows more information, usch as timestamps.

**Simple topic monitoring**

```
ros2 topic echo /position/RUDDER_ANGLE
```

**Extensive topic monitoring**

```
cd ~/$ros2_Workspace_Path/

. install/setup.bash

ros2 run rudder_path sub
```
