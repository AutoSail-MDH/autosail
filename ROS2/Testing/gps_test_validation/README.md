# GPS testing and validation
This ros2 application monitors the `/position/GPS` topic and keeps track of how many messages that contain a fault.

## Prerequisites

## Setup
Place the `gps_test_validation` folder in `$ROS2_WORKSPACE/src`

```bash
cd ~/$ROS2_WORKSPACE

colcon build

. install/setup.bash
```
Run the listener app
```bash
ros2 run gps_validation listener
```
Start the publishing of data on the topic, either by publishing live using the `gps` application, or by playing a bag file in a new terminal

```bash
cd ~/$PATH_TO_BAG_FILE

ros2 bag play -r 100 $BAG_FILE
```
This test prints out every time a fault happens, so if the bag file finishes its playback, or when the monitoring stops, and nothing is printed, then this means the number of faults is zero, othervise all faults are printed on the format:
```bash
[Message number] [Lat] [Long] [Total Faults]
```
