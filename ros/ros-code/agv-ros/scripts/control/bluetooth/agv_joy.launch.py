from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=['bluetooth_config.yaml']
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=['bluetooth_config.yaml'],
            remappings=[('/cmd_vel', '/cmd_vel')]  # adjust topic if needed
        ),
    ])