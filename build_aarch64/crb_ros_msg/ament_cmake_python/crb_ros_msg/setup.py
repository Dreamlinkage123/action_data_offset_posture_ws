from setuptools import find_packages
from setuptools import setup

setup(
    name='crb_ros_msg',
    version='0.1.0',
    packages=find_packages(
        include=('crb_ros_msg', 'crb_ros_msg.*')),
)
