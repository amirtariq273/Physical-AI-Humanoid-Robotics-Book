import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Launch the Isaac ROS VSLAM node."""

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Omniverse Isaac Sim) clock'
    )
    
    # VSLAM Node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam',
        name='visual_slam',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'denoise_input_images': False,
            'rectified_images': True,
            # Input topics for stereo cameras
            'left_image_topic': '/left/image_raw',
            'left_camera_info_topic': '/left/camera_info',
            'right_image_topic': '/right/image_raw',
            'right_camera_info_topic': '/right/camera_info',
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        visual_slam_node
    ])
