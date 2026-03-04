
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nav_25d_pkg = get_package_share_directory('nav_25d')
    rtabmap_launch_pkg = get_package_share_directory('rtabmap_launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Simulation
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_25d_pkg, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # RTAB-Map SLAM
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_pkg, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start', # Start fresh map
            'rgb_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'frame_id': 'base_link',
            'approx_sync': 'true',
            'wait_for_transform': '0.2',
            'use_sim_time': use_sim_time,
            'subscribe_scan_cloud': 'true',
            'scan_cloud_topic': '/velodyne_points',
            'scan_cloud_max_points': '15000',
            'visual_odometry': 'false', # Use diff drive odom
            'odom_topic': '/diff_drive_odom',
            'qos': '2',
            'queue_size': '20',
            'rviz': 'true', # Open RTAB-Map's RViz for mapping
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        simulation_launch,
        rtabmap_launch
    ])
