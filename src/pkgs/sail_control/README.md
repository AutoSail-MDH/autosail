# Wind to Sail

Wind to sail reads the wind direction and sets the angle of the sail according to the direction of the wind. This is a simple stand in for the SNN that will later replace this node. The node subscribes from the topic /sensor/wind and publishes to /actuator/sail_angle. The angle the node publishes is in the range [-90,90], where [-90,0] means the sail is to be pointed out on the port side of the boat and [0,90] on the starboard side.

## Prerequsities

Connect the wind sensor and run the publisher of the wind sensor by following the steps in the README in the wind direction folder, [wind reading README](https://github.com/AutoSail-MDH/autosail/src/apps/wind_reading).

## Setup

```bash
cd ~/ros2_ws/autosail

colcon build

source install/local_setup.bash
```

## Run

Run the wind to sail node that reads from the wind sensor and publishes the sail angle.
```bash
ros2 run sail_control sail_control
```
