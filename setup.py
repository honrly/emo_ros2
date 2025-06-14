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
        #(os.path.join('share/', package_name, 'Voice_EN'),glob('Voice_JA/*')),
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
            'motion_ctrlb_no_distance_eng = emotion_ros.motion_ctrlb_no_distance_eng:main',
            'motion_ctrlb_no_distance_ja = emotion_ros.motion_ctrlb_no_distance_ja:main',
            'motion_ctrlb_exp = emotion_ros.motion_ctrlb_exp:main',
            'motion_ctrlb_no_distance_new = emotion_ros.motion_ctrlb_no_distance_new:main',
            'face_ctrl = emotion_ros.face_ctrl:main',
            'face_ctrl_cv = emotion_ros.face_ctrl_cv:main',
            'face_ctrl_cv2 = emotion_ros.face_ctrl_cv2:main',
            'face_ctrl_jpg = emotion_ros.face_ctrl_jpg:main',
            'sound_ctrl_eng = emotion_ros.sound_ctrl_eng:main',
            'sound_ctrl_ja = emotion_ros.sound_ctrl_ja:main',
            'test_motion2 = emotion_ros.test_motion2:main',
            'emo_status = emotion_ros.emo_status:main',
            'emo_status_fix_num = emotion_ros.emo_status_fix_num:main',
            'emo_status_rest_base = emotion_ros.emo_status_rest_base:main',
            'emo_status_exp_rest_fix = emotion_ros.emo_status_exp_rest_fix:main',
            'emo_status_exp_rest = emotion_ros.emo_status_exp_rest:main',
            'brain_wave = emotion_ros.brain_wave:main',
            'stimulation = emotion_ros.stimulation:main',
            'user_enter = emotion_ros.user_enter:main',
            'visualizer = emotion_ros.visualizer:main',
            'talk_ctrl_ja = emotion_ros.talk_ctrl_ja:main',
            'vad_ctrl = emotion_ros.vad_ctrl:main',
            'vad_ctrl_emo = emotion_ros.vad_ctrl_emo:main',
            'vad_ctrl_demo = emotion_ros.vad_ctrl_demo:main',
            'vad_recog_only_ctrl = emotion_ros.vad_recog_only_ctrl:main',
            'museS = emotion_ros.museS:main',
            'polar = emotion_ros.polar:main',
            'pulseSensor = emotion_ros.pulseSensor:main',
            'time_manager = emotion_ros.time_manager:main',
            'talk_ctrl_api_eng = emotion_ros.talk_ctrl_api_eng:main',
            'talk_ctrl_api_eng2 = emotion_ros.talk_ctrl_api_eng2:main',
            'talk_ctrl_api_ja = emotion_ros.talk_ctrl_api_ja:main',
        ],
    },
)
