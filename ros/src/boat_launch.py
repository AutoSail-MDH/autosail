"""
https://issueexplorer.com/issue/micro-ROS/micro-ROS-Agent/42
https://automationsr.com/exploring-ros2-with-wheeled-robot-1-launch-ros2-simulation/
https://github.com/ros2/launch_ros/blob/master/ros2launch/examples/includes_example.launch.py
on_exit=launch.actions.Shutdown()
"""

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sensor_malfunction = get_package_share_directory('sensor_malfunction')
    return LaunchDescription([
        # Multiserial Agent
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            emulate_tty=True,
            arguments=['multiserial', '--devs',
                       "/dev/ttyUSB0 /dev/ttyUSB1", 'v6']
        ),
        # Velocity
        Node(
            package="vessel_velocity",
            executable="sub_pos_gps",
            name="velocity",
            output="screen",
            emulate_tty=True
        ),
        # Sensor fusion
        Node(
            package="sensor_fusion",
            executable="EKF",
            name="fusion",
            output="screen",
            emulate_tty=True
        ),
        # Rudder path
        Node(
            package="rudder_path",
            executable="rpp",
            name="rudder",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"p": 5.0}
            ]
        ),
        # Rudder path publisher
        Node(
            package="rudder_path",
            executable="pub",
            name="rudder_pub",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"lat": 60.0},
                {"lon": 10.0}
            ]
        ),
        # Wind to Sail
        Node(
            package="wind_to_sail",
            executable="sub_pub",
            name="sail",
            output="screen",
            emulate_tty=True
        ),
        # Data logging
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/record.launch.py'])
        ),
        # Sensor malfunction
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [sensor_malfunction, '/launch', '/malfunction_launch.py'])
        ),
    ])
