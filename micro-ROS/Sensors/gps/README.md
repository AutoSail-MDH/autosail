# GPS
This application is creates a ros2 node that publishes Lat/Long values to a topic on a ros network. These values come from the GPS NEO-M9N. The version of ros2 is foxy desktop on ubuntu 20.04. The node publishes to the topic `/position/GPS`

## Prerequsities
This application assumes the user has installed ros2 and micro-ros using the following tutorials:

[ros2 installtion script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu)

[micro-ROS](https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/)

## Usage

Place the gps folder in your apps folder under the firmware app in your micro-ROS workspace. 

### Setup
Sources the proper micro-ROS environment and prepares the building and flashing to the correct app.
```
cd ~/microros_ws

colcon build

source install/local_setup.bash
```
```
ros2 run micro_ros_setup configure_firmware.sh gps --transport serial
```

### Build, flash
Builds the gps app and flashes it to the esp32. Make sure the device is plugged in, and that you have your port unlocked, which can be done by [adding your user to the dialout group](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/establish-serial-connection.html#linux-dialout-group)
```
ros2 run micro_ros_setup build_firmware.sh

ros2 run micro_ros_setup flash_firmware.sh
```
### Agent

You only have to create and build the agent the first time, afterwards you only need to run the micro-ROS agent

```
ros2 run micro_ros_setup create_agent_ws.sh

ros2 run micro_ros_setup build_agent.sh`

source install/local_setup.bash`
```
Find the device ID using:
```
ls /dev/serial/by-id/*
```
Use the ID from earlier when running the agent
```
ros2 run micro_ros_agent micro_ros_agent serial --dev [device ID]
```
You might have to press the restart button on the esp32 if the agent does not work properly

In another terminal, monitor the topic you published to
```
ros2 topic echo /position/GPS
```
