# Autosail repo for continous development of releases
## Prerequsities
These applications assume the user has installed ros2 and micro-ros using the following tutorial:

[ros2 installtion script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu)


## Setup
Create ros2_ws folder and clone repo to /home/$USER/ros2_ws

Follow this tutorial without creating the directory "microros_ws", instead run all commands from: /home/$USER/ros2_ws/autosail.

[micro-ROS](https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/)

Remember to run all commands from: /home/$USER/ros2_ws/autosail

```bash
cd ~/ros2_ws/autosail
rm -r firmware/freertos_apps/apps
ln -s ../../src/apps firmware/freertos_apps/apps
```

## Usage
In order to test and run an app/package, follow the instructions in the README.md files in the appropriate folder.