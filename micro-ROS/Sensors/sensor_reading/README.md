# Data acquisition

This application bundles together three submodules, [GPS](https://github.com/AutoSail-MDH/AutoSail-HT21/tree/main/micro-ROS/Sensors/gps), [Vessel orientation](https://github.com/AutoSail-MDH/AutoSail-HT21/tree/main/micro-ROS/Sensors/vessel_orientation) and [Wind direction](https://github.com/AutoSail-MDH/AutoSail-HT21/tree/main/micro-ROS/Sensors/wind_direction). This is so all three can be ran on one MCU, instead of on seperate ones. See each seperate submodule for information on how to run them seperate.

## Prerequsities
This application assumes the user has Docker and either ESP-IDF version v4.1, v4.2 or v4.3.

If the user has installed the ESP-IDF prerequisites, the micro-ROS ESP-IDF component probably won't work.

### Setup
Source ESP-IDF and open a terminal at the source of the project.

The following command installs micro-ROS ESP-IDF component dependencies and only needs to be ran once per ESP-IDF installation:
```
pip3 install catkin_pkg lark-parser empy colcon-common-extensions importlib-resources
```

The following two commands only needs to be ran once per project build folder:
```
idf.py set-target esp32
idf.py menuconfig
```

### Build, flash
Builds the app and flashes it to the ESP32. Make sure the device is plugged in, and that you have your port unlocked, which can be done by [adding your user to the dialout group](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/establish-serial-connection.html#linux-dialout-group)
```
idf.py build
idf.py flash
```

### Agent

Find the device ID using:
```
ls /dev/serial/by-id/*
```
Open up a serial micro-ROS agent:
```
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:galactic serial --dev [device ID] -v6
```
You might have to press the restart button on the ESP32 if the agent does not work properly. See the image below on what is a normal look for the agent. For example, if only the first two rows are shown in the terminal, you might have to press the restart button on the ESP32.

![normal_agent](https://user-images.githubusercontent.com/31732187/141467001-6a39c2ac-4bb9-48d2-903c-675f5fb736d9.png)

