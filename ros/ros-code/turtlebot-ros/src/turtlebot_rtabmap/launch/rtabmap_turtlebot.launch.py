from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('turtlebot_rtabmap')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')
    rviz = LaunchConfiguration('rviz')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_localization = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Launch in localization mode (requires existing map)'
    )
    
    declare_database_path = DeclareLaunchArgument(
        'database_path',
        default_value='~/.ros/rtabmap.db',
        description='Path to RTAB-Map database file'
    )
    
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    declare_rtabmap_viz = DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='false',
        description='Launch RTAB-Map visualization tool'
    )
    
    # Load parameters from YAML file
    params_file = os.path.join(pkg_share, 'config', 'rtabmap_params.yaml')
    
    # RTAB-Map parameters (can override YAML values)
    rtabmap_parameters = [{
        'frame_id': 'base_footprint',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': True,
        'use_sim_time': use_sim_time,
        'database_path': database_path,
        'localization': localization,
    }, params_file]
    
    # Remappings for TurtleBot topics
    # Adjust these based on your TurtleBot model
    rtabmap_remappings = [
        ('rgb/image',        '/camera/camera/color/image_rect_raw'),
        ('rgb/camera_info',  '/camera/camera/color/camera_info'),
        ('depth/image',      '/camera/camera/depth/image_rect_raw'),
        ('scan',             '/scan'),
        ('odom',             '/odom'),
    ]
    
    # RTAB-Map SLAM node
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=rtabmap_parameters,
        remappings=rtabmap_remappings,
        arguments=['-d'],  # Delete database on startup (remove for persistence)
        namespace=''
    )
    
    # RTAB-Map visualization node (optional)
    rtabmap_viz_node = Node(
        condition=IfCondition(rtabmap_viz),
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'subscribe_scan': True,
            'subscribe_odom_info': True,
            'frame_id': 'base_footprint',
        }],
        remappings=rtabmap_remappings,
        namespace=''
    )
    
    # RGB-D odometry node (optional, improves accuracy)
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'publish_tf': False,
            'wait_for_transform': 0.2,
            'Odom/Strategy': '0',  # Frame-to-Map
            'Vis/CorGuessWinSize': '20',
            'Vis/MinInliers': '10',
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('odom', '/odom_rgbd'),
        ],
        namespace=''
    )
    
    # RViz node
    rviz_config = os.path.join(pkg_share, 'rviz', 'rtabmap.rviz')
    rviz_node = Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_localization,
        declare_database_path,
        declare_rviz,
        declare_rtabmap_viz,
        rtabmap_slam_node,
        rtabmap_viz_node,
        # rgbd_odometry,  # Uncomment if using visual odometry
        rviz_node,
    ])
