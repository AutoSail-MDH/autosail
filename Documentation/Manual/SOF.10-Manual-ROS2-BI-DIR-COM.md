# ROS 2 - Bidirectional communication

This manual will describe how to initiate a bidirectional communication between two or more machines.

This guide assumes that ROS2-full (not base) is installed on both clients and server.

## Setup host (Land PC)

### Start server

    > fastdds discovery -i 0

### Start listener in a new terminal to receive msg from the boat(Do not forget to source ROS 2 in every new terminal)

#### Insert node to be received from boat, example of demo

    > export ROS_DISCOVERY_SERVER="127.0.0.1:11811"

#### The following command will listen to the specified node coming from the boat on land pc loopback address, in this case a demo

    > ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server

### Start talker in a new terminal to transmit msg to the boat(Do not forget to source ROS 2 in every new terminal)

#### Insert node to be transmitted to loopback and listened by boat, example of demo

    > export ROS_DISCOVERY_SERVER="127.0.0.1:11811"

    > ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server

## Setup target (Boat pc)

Fetch land_PC_ip_address from [https://my.zerotier.com/network](https://my.zerotier.com/network)

### Start listener in a new terminal to receive msg from the land pc (Do not forget to source ROS 2 in every new terminal)

    > export ROS_DISCOVERY_SERVER="<land_PC_ip_address>:11811"

    > ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server

### Start talker in a new terminal to transmit msg to the land pc (Do not forget to source ROS 2 in every new terminal)

    > export ROS_DISCOVERY_SERVER="<land_PC_ip_address>:11811"

    > ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server

**Note**: If more targets (clients) are wanted, redo "Setup target (Boat PC)"

Further reading see [Use ROS 2 with Fast-DDS Discovery Server](https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html#discovery-server-v2)
