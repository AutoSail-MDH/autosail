# Wind to Sail

Wind to sail reads the wind direction and sets the angle of the sail according to the direction of the wind. This is a simple stand in for the SNN that will later replace this node. The node subscribes from the topic /direction/wind and publishes to /position/SAIL_ANGLE. It is run with ROS2 foxy desktop on Ubuntu 20.04.

## Prerequsities

Connect the wind sensor and run the publisher of the wind sensor by following the steps in the README in the wind_direction folder, [wind direction README](https://github.com/AutoSail-MDH/AutoSail-HT21/tree/wind_direction/micro-ROS/Sensors/wind_direction).

## Usage

Place the wind_to_sail folder in the src folder in your ROS2 workspace.

## Setup

```bash
cd ~/dev_ws

colcon build

source install/local_setup.bash
```

## Run

Run the wind to sail that reads from the wind sensor and publishes sail angle.
```bash
ros2 run wind_to_sail sub_pub
```
