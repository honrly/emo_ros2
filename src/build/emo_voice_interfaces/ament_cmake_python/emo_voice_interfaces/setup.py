from setuptools import find_packages
from setuptools import setup

setup(
    name='emo_voice_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('emo_voice_interfaces', 'emo_voice_interfaces.*')),
)
