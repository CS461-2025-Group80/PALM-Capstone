import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('um982_gps_manual')
    default_params = os.path.join(pkg_share, 'config', 'um982_gps_manual_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),

        Node(
            package='um982_gps_manual',
            executable='um982_node.py',
            name='um982_gps_manual',
            output='screen',
            parameters=[
                default_params,
                {
                    'port': LaunchConfiguration('port'),
                    'baud': LaunchConfiguration('baud'),
                }
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_tf_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.1', '0', '0', '0', 'base_link', 'gps']
        ),
    ])
