from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='emotion_ros',
            executable='emo_status_fix_num',
        ),
        Node(
            package='emotion_ros',
            executable='motion_ctrlb_no_distance_ja',
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['udp4', '--port', '50000', '-v6']
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['udp4', '--port', '55000', '-v6']
        ),
        Node(
            package='emotion_ros',
            executable='vad_ctrl',
        ),
    ])
