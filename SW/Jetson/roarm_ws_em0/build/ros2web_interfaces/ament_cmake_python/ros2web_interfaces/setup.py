from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2web_interfaces',
    version='0.1.0',
    packages=find_packages(
        include=('ros2web_interfaces', 'ros2web_interfaces.*')),
)
