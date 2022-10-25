# Sensor fusion
This application creates a sensor fusion between the IMU and the GPS using EKF. It tries to give a better estimation of orientation and position.

The application will read from /sensor/imu and /sensor/gnss, and publish [x y yaw v] to /position/pose.

## Usage

### Setup and run
Open up a terminal at the source of your ROS2 workspace and source your ROS2 version.

Run the following commands to start the application:
```
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select pose_estimation
. install/local_setup.bash
ros2 run pose_estimation pose_estimation
```


For this to function it needs to receive IMU and GPS data.

#### ROSBag for testing
WIP

Download and play this bag for testing:

Put the bag folder in the source of your workspace.

Run the following commands at the source of your workspace with your ROS2 version sourced:
```
. install/local_setup.bash
ros2 bag play [BAG_FOLDER_NAME]
```
