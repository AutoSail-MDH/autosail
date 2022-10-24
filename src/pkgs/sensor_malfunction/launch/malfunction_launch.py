from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    	#Shutdown node
    	Node(
            package="sensor_malfunction",
            executable="shutdown.py",
            name="shutdown",
            output="screen",
            emulate_tty=True
        ),
        #Monitor IMU
        Node(
            package="sensor_malfunction",
            namespace="sensor",
            executable="imu_error",
            name="direction",
            parameters=[
                {'my_topic': 'imu'},
                {'my_DL': 3},
                {'my_iteration': 4}
                ],
            output="screen",
            emulate_tty=True
        ),
        #Monitor GNSS
        Node(
            package="sensor_malfunction",
            namespace="sensor",
            executable="gnss_error",
            name="position",
            parameters=[
                {'my_topic': 'gnss'},
                {'my_DL': 3},
                {'my_iteration': 4}
                ],
            output="screen",
            emulate_tty=True
        ),
        #Monitor Wind
        Node(
            package="sensor_malfunction",
            namespace="sensor",
            executable="wind_error",
            name="wind",
            parameters=[
                {'my_topic': 'wind'},
                {'my_DL': 3},
                {'my_iteration': 4}
                ],
            output="screen",
            emulate_tty=True
        )
    ])
