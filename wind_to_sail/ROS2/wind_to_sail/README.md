# Wind to Sail

Wind to sail reads the wind direction and sets the angle of the sail according to the direction of the wind. This is a simple stand in for the SNN that will later replace this node.

## Usage

Place the wind to sail folder in the src folder in your ROS2 workspace.

## Setup

```bash
cd ~/dev_ws

colcon build

source install/local_setup.bash
```

## Run
```bash
ros2 run wind_to_sail sub_pub
```