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
        #Monitor node(s)
        Node(
            package="sensor_malfunction",
            namespace="position",
            executable="if_error",
            name="vessel_orientation",
            parameters=[
                {'my_topic': 'IMU'},
                {'my_DL': 3},
                {'my_iteration': 4}
                ],
            output="screen",
            emulate_tty=True
        )
    ])
