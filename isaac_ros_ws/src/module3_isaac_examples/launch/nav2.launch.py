import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Launch the Nav2 stack."""
    
    # Get the share directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_share = get_package_share_directory('module3_isaac_examples')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_share, 'maps', 'my_map.yaml'))
    nav2_params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_share, 'config', 'nav2.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Omniverse Isaac Sim) clock'),
        DeclareLaunchArgument(
            'map',
            default_value=map_yaml_file,
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params_file,
            description='Full path to the ROS2 parameters file to use'),
            
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file}.items(),
        ),
    ])
