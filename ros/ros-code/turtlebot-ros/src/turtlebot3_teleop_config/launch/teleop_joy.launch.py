from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the config file path
    config_file = os.path.join(
        get_package_share_directory('turtlebot3_teleop_config'),
        'config',
        'joystick.yaml'
    )
    
    return LaunchDescription([
        # Joy node - reads controller input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[config_file]
        ),
        
        # Teleop twist joy - converts joy to cmd_vel
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_file],
            remappings=[('/cmd_vel', '/cmd_vel')]
        ),
    ])