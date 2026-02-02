from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for the LIDAR scan logger node.
    
    Usage:
        ros2 launch lidar_logger scan_logger.launch.py
        
    Or with custom parameters:
        ros2 launch lidar_logger scan_logger.launch.py save_directory:=/path/to/data save_interval:=5.0
    """
    
    # Declare launch arguments
    save_directory_arg = DeclareLaunchArgument(
        'save_directory',
        default_value='~/lidar_data',
        description='Directory to save LIDAR scan data'
    )
    
    save_interval_arg = DeclareLaunchArgument(
        'save_interval',
        default_value='2.0',
        description='Time interval (seconds) between saving scans'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic name for laser scan messages'
    )
    
    file_format_arg = DeclareLaunchArgument(
        'file_format',
        default_value='json',
        description='File format for saving data (json or csv)'
    )
    
    max_range_filter_arg = DeclareLaunchArgument(
        'max_range_filter',
        default_value='0.0',
        description='Maximum range to save (0.0 = no filter)'
    )
    
    # Create the node
    scan_logger_node = Node(
        package='lidar_logger',
        executable='scan_logger',
        name='scan_logger',
        output='screen',
        parameters=[{
            'save_directory': LaunchConfiguration('save_directory'),
            'save_interval': LaunchConfiguration('save_interval'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'file_format': LaunchConfiguration('file_format'),
            'max_range_filter': LaunchConfiguration('max_range_filter'),
        }]
    )
    
    return LaunchDescription([
        save_directory_arg,
        save_interval_arg,
        scan_topic_arg,
        file_format_arg,
        max_range_filter_arg,
        scan_logger_node
    ])
