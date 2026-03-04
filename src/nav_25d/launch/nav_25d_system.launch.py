
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nav_25d_pkg = get_package_share_directory('nav_25d')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = os.path.join(nav_25d_pkg, 'params', 'nav2_params.yaml')

    # Simulation
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_25d_pkg, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # RTAB-Map
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_25d_pkg, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'visual_odometry': 'false', # Use diff drive odom
            'subscribe_scan_cloud': 'true',
            'grid_sensor': '1', # 0=laser, 1=depth, 2=both. We use cloud? 
            # RTAB-Map uses scan_cloud_topic if subscribe_scan_cloud is true.
        }.items()
    )

    # Nav2
    # We use navigation_launch.py which starts controller, planner, behaviors, bt_navigator
    # We do NOT use bringup_launch.py because we don't want AMCL or Map Server (RTAB-Map provides map)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )
    
    # We might need a map_server to publish an empty map if RTAB-Map is slow to start?
    # No, global costmap will wait for map.

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        simulation_launch,
        rtabmap_launch,
        nav2_launch
    ])
