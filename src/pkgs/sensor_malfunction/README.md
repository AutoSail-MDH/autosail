# Sensor malfunction
This application creates multiple nodes to monitor topics of interest (IMU, GNSS, Wind), sending log messages for shutdown request if the publisher is disconnected for too long. The application also starts a shutdown node to monitor the log messages, which executes a user-specified shell command if a log message with severity level 50 (FATAL) is read.

The monitor nodes will read publisher count from user specified topics and send log messages to the default topic /rosout, while the shutdown node will read from /rosout.

## Usage
### Setup and run
Run the following commands to start the application:
```
cd ~/ros2_ws/autosail
colcon build --packages-select sensor_malfunction
. install/local_setup.bash
ros2 launch sensor_malfunction malfunction_launch.py
```

If all nodes are connected and publishing, nothing will happen.

### Change behaviour
Edit /launch/malfunction_launch.py file to edit the following parameters.

"topic" is the topic to monitor.

"timeout" is the time in seconds before the monitoring node starts sending FATAL messages.

"iteration" is the amount of times the topic can be unresponsive before FATAL messages are sent.
