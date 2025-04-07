from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'turtlebot3_bringup'
    launch_file_name = 'robot.launch.py'
    
    package_share_directory = get_package_share_directory(package_name)

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
            package='emotion_ros',
            executable='face_ctrl_jpg',
        ),
        Node(
            package='emotion_ros',
            executable='sound_ctrl_ja',
        ),
        Node(
            package='emotion_ros',
            executable='test_motion2',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                package_share_directory, '/launch/', launch_file_name
            ]),
            
        ),
    ])
