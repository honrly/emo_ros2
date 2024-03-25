from setuptools import find_packages, setup

package_name = 'emo_voice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
		'sensor_pnnx = emo_voice.sensor_pnnx:main', 
        'move = emo_voice.move:main', 
        'plot = emo_voice.plot:main', 
        ],
    },
)
