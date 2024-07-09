from setuptools import find_packages, setup
import os
from glob import glob

from setuptools import setup

package_name = 'emo_voice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
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
        'brain_wave = emo_voice.brain_wave:main', 
        'emo_status_rest_base = emo_voice.emo_status_rest_base:main', 
        'visualizer = emo_voice.visualizer:main', 
        ],
    },
)
