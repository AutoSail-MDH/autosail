# Data acquisition
This application is used to run the GNSS module

## Hardware connection
The GNSS module is connected directly to the ESP32.

SDA(Blue cable) - GPIO 21 

SCL(Yellow cable) - GPIO 22 

3V3(Red cable) - 3V3

GND(Black cable) - GND

## Usage
### Setup
In order to build and flash the app, the micro-ROS environment has to be sourced and configured.

```bash
cd ~/ros2_ws/autosail
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup configure_firmware.sh gnss_reading --transport serial
```

### Build and flash

Builds the gps app and flashes it to the esp32. Make sure the device is plugged in, and that you have your port unlocked, which can be done by [adding your user to the dialout group](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/establish-serial-connection.html#linux-dialout-group)

```bash
ros2 run micro_ros_setup build_firmware.sh

ros2 run micro_ros_setup flash_firmware.sh
```

### Agent

You only have to create and build the agent once, afterwards you only need to run the micro-ROS agent
```bash
ros2 run micro_ros_setup create_agent_ws.sh

ros2 run micro_ros_setup build_agent.sh

source install/local_setup.bash
```

Run the agent using one of the following (depends on number of flashable usb connected):
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1
```

You might have to press the restart button on the esp32 if the agent does not work properly. See the image below on what is a normal look for the agent. For example, if only the first two rows are shown in the terminal, you might have to press the restart button on the esp32.

![normal_agent](https://user-images.githubusercontent.com/31732187/141467001-6a39c2ac-4bb9-48d2-903c-675f5fb736d9.png)

In another terminal, monitor the topic you published to
```bash
ros2 topic echo /sensor/gnss
```
You can check all listed topics using 
```bash
ros2 topic list
```