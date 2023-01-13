# True Wind Calculation
This package takes the apparent wind values from the wind sensor and calculates the true wind values.

## Usage
### Setup and run
Run the following commands to start the application:
```
cd ~/ros2_ws/autosail
colcon build --packages-select true_wind_calculation
. install/local_setup.bash
ros2 run true_wind_calculation true_wind
```