#! /bin/bash
cd uros/	


source /opt/ros/foxy/setup.bash

git clone -b foxy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

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

cd ..
cp -R uros/src/uros/ ros/src/

cd ros/
source /opt/ros/foxy/setup.bash

colcon build
source install/local_setup.bash
