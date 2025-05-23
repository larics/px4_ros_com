from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace='px4_1',
            package='px4_ros_com',
            executable='offboard_control',
            name='offboard_control',
            output='screen'
        )
    ])
