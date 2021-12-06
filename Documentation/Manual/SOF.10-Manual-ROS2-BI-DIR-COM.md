# ROS 2 - Bidirectional communication

This manual will describe how to initiate a bidirectional communication between two or more machines.

This guide assumes that ROS2-full (not base) is installed on both clients and server.

## Prerequisites

### rosdep

```console
sudo apt-get install python3-rosdep
sudo rosdep init # Only once
rosdep update
```

[More info of rosdep](https://wiki.ros.org/rosdep#Installing_rosdep)

### eProsima Fast DDS

```concole
sudo apt install ros-foxy-rmw-fastrtps-cpp
```

## Switch to rmw_fastrtps

```concole
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp # In every new terminal
# OR 
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> $HOME/.bashrc
source $HOME/.bashrc
```

[More info: Working with multiple RMW implementations](https://ftp-osl.osuosl.org/pub/ros/ros_docs_mirror/en/galactic/How-To-Guides/Working-with-multiple-RMW-implementations.html)

## If communication is not working, enable ip forwarding on mobile network:

> net.ipv4.ip_forward=1 # /etc/systctl.conf

## All machines can now communicate topics on zerotier network

### Boat publish to land

#### Land PC

```console
T1: ros2 run demo_nodes_cpp listener
> [INFO] [1636924614.288293801] [listener]: I heard: [Hello World: 1]

T2: ros2 topic list
> /chatter
> /parameter_events
> /rosout
```

#### Boat PC

```console
T1: ros2 run demo_nodes_cpp talker
> [INFO] [1636924614.287177107] [talker]: Publishing: 'Hello World: 1'

T2: ros2 topic list
> /chatter
> /parameter_events
> /rosout
```

### Land publish to boat

#### Land PC

```console
T1: ros2 run demo_nodes_cpp talker
> [INFO] [1636924614.287177107] [talker]: Publishing: 'Hello World: 1'

T2: ros2 topic list
> /chatter
> /parameter_events
> /rosout
```

#### Boat PC

```console
T1: ros2 run demo_nodes_cpp listener
> [INFO] [1636924614.288293801] [listener]: I heard: [Hello World: 1]

T2: ros2 topic list
> /chatter
> /parameter_events
> /rosout
