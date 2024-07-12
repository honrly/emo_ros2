from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'emotion_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/', package_name, 'launch'),glob('launch/*')),
        (os.path.join('share/', package_name, 'rviz'),glob('rviz/*')),
        #(os.path.join('share/', package_name, 'Voice_EN'),glob('Voice_EN/*')),
        #(os.path.join('share/', package_name, 'face_DB'),glob('face_DB/*')),
        #(os.path.join('share/', package_name, 'asking_face_jpg'),glob('asking_face_jpg/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'biodata_bridge_node3b = emotion_ros.biodata_bridge_node3b:main',
            'EasyClassify2b = emotion_ros.EasyClassify2b:main',
            'motion_ctrlb = emotion_ros.motion_ctrlb:main',
            'motion_ctrlb_fsm = emotion_ros.motion_ctrlb_fsm:main',
            'face_ctrl = emotion_ros.face_ctrl:main',
            'face_ctrl_cv = emotion_ros.face_ctrl_cv:main',
            'face_ctrl_cv2 = emotion_ros.face_ctrl_cv2:main',
            'sound_ctrl_eng = emotion_ros.sound_ctrl_eng:main',
            'test_motion2 = emotion_ros.test_motion2:main',
            'emo_status = emotion_ros.emo_status:main',
            'emo_status_rest_base= emotion_ros.emo_status_rest_base:main',
            'motion_ctrlb_no_distance= emotion_ros.motion_ctrlb_no_distance:main',
        ],
    },
)
