# Vessel velocity
This application creates a ROS2 node that subscribes to the /sensor/gnss topic, then calculates and publishes the velocity to /position/velocity.

Distance formula: http://en.wikipedia.org/wiki/Haversine_formula

## Usage
### Setup and run

Run the following commands to start the application:
```
cd ~/ros2_ws/autosail
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select vessel_velocity
. install/local_setup.bash
ros2 run vessel_velocity velocity_calculation
```

For it to actually function it needs to receive GNSS data. This can either be done by running the GNSS reading app at the same time, or by replaying data using ROSBag.

WIP

Download one of the ROS2 bags with GNSS data here:

Put the bag folder in the source of your workspace.

Run the following commands:
```
cd ~/ros2_ws/autosail
. install/local_setup.bash
ros2 bag play [BAG_FOLDER_NAME]
```
