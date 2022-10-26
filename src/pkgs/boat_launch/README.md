# Boat Launch
This package is used to start the entire system. This runs the executables in the packages specified in the code.

## Launching the boat
In order to launch the boat, run the following commands:

```
cd ~/ros2_ws/autosail
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash
ros2 launch boat_launch boat_launch.py 
```