#! /bin/bash
sudo apt install python3-rosdep2 python3-vcstool
cd ~/ros2_ws/autosail

source /opt/ros/${ROS_DISTRO}/setup.bash

git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

sudo apt update && rosdep update 

rosdep install --from-paths src --ignore-src -y

sudo apt-get install python3-pip

colcon build
source install/local_setup.bash

ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32

rm -r firmware/freertos_apps/apps
ln -s ../../src/apps firmware/freertos_apps/apps

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
