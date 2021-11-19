from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sublisher",
            namespace="test1",
            executable="param_sub",
            name="custom_param_sub",
            parameters=[
                {"my_topic": "ESPtoROS"}
            ]
        ),
        Node(
            package="sublisher",
            namespace="test1",
            executable="param_pub",
            name="custom_param_pub",
            parameters=[
                {"my_topic": "ROStoESP"}
            ]
        )
    ])
