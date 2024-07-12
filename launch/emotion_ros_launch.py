from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='emotion_ros',
            namespace='emotion_ros',
            executable='face_ctrl',
        ),
        Node(
            package='emotion_ros',
            namespace='emotion_ros',
            executable='motion_ctrl_ja',
        ),
        Node(
            package='emotion_ros',
            namespace='emotion_ros',
            executable='test_motion2',
        ),
    ])
