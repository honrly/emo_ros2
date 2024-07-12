from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='emotion_ros',
            namespace='emotion_ros',
            executable='emo_status_rest_base',
        ),
        Node(
            package='emotion_ros',
            namespace='emotion_ros',
            executable='motion_ctrlb',
        ),
    ])
