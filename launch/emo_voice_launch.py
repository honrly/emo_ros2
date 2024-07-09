from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='emo_voice',
            namespace='emo_voice',
            executable='brain_wave',
        ),
        Node(
            package='emo_voice',
            namespace='emo_voice',
            executable='emo_status',
        ),
    ])
