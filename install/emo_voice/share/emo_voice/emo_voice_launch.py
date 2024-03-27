from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='emo_voice',
            namespace='emo_voice',
            executable='client',
        ),
        Node(
            package='hello',
            namespace='hello',
            executable='server',
        ),
        Node(
            package='emo_voice',
            namespace='emo_voice',
            executable='move',
        ),
        Node(
            package='emo_voice',
            namespace='emo_voice',
            executable='sensor_pnnx',
        ),
        Node(
            package='hello',
            namespace='hello',
            executable='plot',
        ),
    ])
