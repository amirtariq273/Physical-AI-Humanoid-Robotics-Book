from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the launch argument for the robot model
    robot_model_arg = DeclareLaunchArgument(
        'model',
        default_value=TextSubstitution(text=os.path.join(
            get_package_share_directory('module1_ros2_examples'),
            'urdf',
            'humanoid.urdf'
        )),
        description='Path to the robot URDF file'
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': LaunchConfiguration('model'), 'use_sim_time': True}]
    )

    # Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Rviz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('module1_ros2_examples'), 'rviz_config', 'humanoid_display.rviz')], # Placeholder
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
