# Shutting down boat using ROS messages

The mentioned files can be found at [Autosail-HT21/ROS2/sig_shutdown](https://github.com/AutoSail-MDH/AutoSail-HT21/ROS2/sig_shutdown)

The result of running "sig_control" will publish a FATAL message onto /rosout, which the boat is continuously listening at.
If boat receives message FATAL (50), boat will initiate "shutdown" to kill the PC running the boat.

## Shutdown boat standalone

```console
mkdir -p src
mv sig_control src
colcon build --packages-select sig_control
. install/setup.bash
ros2 run sig_control sig_control
```

## Shutdown boat dependent

Assuming topic "sig_control" is launched on land PC together with other topics then:

```console
ros2 run sig_control sig_control
```
