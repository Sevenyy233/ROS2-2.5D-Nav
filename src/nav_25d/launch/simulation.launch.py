import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_description_pkg = get_package_share_directory('robot_description')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Include the robot_description gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world_file_name': 'uneven_terrain',
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        gazebo_launch
    ])
