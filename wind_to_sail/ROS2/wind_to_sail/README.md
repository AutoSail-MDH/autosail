# Wind to Sail

Wind to sail reads the wind direction and sets the angle of the sail according to the direction of the wind. This is a simple stand in for the SNN that will later replace this node. The node subscribes from the topic /direction/wind and publishes to /sail/angle. It is run with ROS2 foxy desktop on Ubuntu 20.04.

## Usage

Place the wind to sail folder in the src folder in your ROS2 workspace.

## Setup

```bash
cd ~/dev_ws

colcon build

source install/local_setup.bash
```

## Run

Run the publisher of the wind sensor by following the steps in the README in the wind_direction folder. (Put in link or path???) [https://github.com/AutoSail-MDH/AutoSail-HT21/tree/wind_direction/micro-ROS/Sensors/wind_direction](hej)

Run the wind to sail that reads from the wind sensor and publishes sail angle.
```bash
ros2 run wind_to_sail sub_pub
```