import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch the full VLA pipeline."""

    pkg_dir = get_package_share_directory('module4_vla_examples')
    
    return LaunchDescription([
        # 1. OpenAI Whisper Node (Voice to Text)
        Node(
            package='module4_vla_examples',
            executable='whisper_node',
            name='whisper_node',
            output='screen',
            parameters=[
                {'whisper_model': 'base'}, # or 'small', 'medium', etc.
                {'audio_topic': '/audio'}, # Assuming audio_common_msgs AudioData
                {'transcription_topic': '/voice_commands'},
            ]
        ),
        
        # 2. LLM Planner Node (Text to Action Sequence)
        Node(
            package='module4_vla_examples',
            executable='llm_planner_node',
            name='llm_planner_node',
            output='screen',
            parameters=[
                {'command_topic': '/voice_commands'},
                {'response_topic': '/robot_status'},
                # Add LLM API key here or manage as env var
                # {'llm_api_key': 'YOUR_LLM_API_KEY_HERE'}, 
            ]
        ),

        # 3. Humanoid Action Servers (Execute Actions)
        Node(
            package='module4_vla_examples',
            executable='humanoid_action_servers',
            name='humanoid_action_servers',
            output='screen'
        ),

        # Optional: Audio capture node (e.g., from ros2_audio_common)
        # Node(
        #     package='audio_capture',
        #     executable='audio_capture_node',
        #     name='audio_capture_node',
        #     output='screen',
        #     parameters=[
        #         {'default_microphone': '/dev/audio_device'},
        #         {'audio_topic': '/audio'}
        #     ]
        # ),
    ])

