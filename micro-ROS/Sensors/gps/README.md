# GPS
This application is creates a ros2 node that publishes Lat/Long values to a topic on a ros network. These values come from the GPS NEO-M9N. The version of ros2 is foxy desktop on ubuntu 20.04. The node publishes to the topic `/sensor/gps`

The GPS is connected using I2C to the MCU. SDA(Blue cable) is connected to GPIO 21 and SCL(Yellow Cable) is connected to GPIO 22. The GPS should be connected(Red cable) to a 3.3V power source. The black cable is ground.

## Prerequsities
This application assumes the user has installed ros2 and micro-ros using the following tutorials:

[ros2 installtion script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu)

[micro-ROS](https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/)

## Usage

Place the gps folder in your apps folder under the firmware app in your micro-ROS workspace. 

### Setup
Sources the proper micro-ROS environment and prepares the building and flashing to the correct app.
```bash
cd ~/uros_ws

colcon build

source install/local_setup.bash
```
```
ros2 run micro_ros_setup configure_firmware.sh gps --transport serial

ln -s src/apps firmware/freertos_apps/apps

```

### Build, flash
Builds the gps app and flashes it to the esp32. Make sure the device is plugged in, and that you have your port unlocked, which can be done by [adding your user to the dialout group](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/establish-serial-connection.html#linux-dialout-group)
```bash
ros2 run micro_ros_setup build_firmware.sh

ros2 run micro_ros_setup flash_firmware.sh
```
### Agent

You only have to create and build the agent the first time, afterwards you only need to run the micro-ROS agent

```bash
ros2 run micro_ros_setup create_agent_ws.sh

ros2 run micro_ros_setup build_agent.sh

source install/local_setup.bash
```
Find the device ID using:
```bash
ls /dev/serial/by-id/*
```
Use the ID from earlier when running the agent
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev [device ID]
```
You might have to press the restart button on the esp32 if the agent does not work properly. See the image below on what is a normal look for the agent. For example, if only the first two rows are shown in the terminal, you might have to press the restart button on the esp32.

![normal_agent](https://user-images.githubusercontent.com/31732187/141467001-6a39c2ac-4bb9-48d2-903c-675f5fb736d9.png)

In another terminal, monitor the topic you published to
```bash
ros2 topic echo /position/GPS
```

