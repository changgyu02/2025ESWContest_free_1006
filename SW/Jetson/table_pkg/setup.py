from setuptools import setup
import os
from glob import glob

package_name = 'table_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/model', ['table_pkg/model/dirt_classifier.pt']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='changgyu',
    maintainer_email='atyy021127@konkuk.ac.kr',
    description='ROS2 table management and dirt detection package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_stream_node = table_pkg.camera_stream_node:main',
            'table_manager_node = table_pkg.table_manager_node:main',
            'dirt_check_node = table_pkg.dirt_check_node:main',
            'table_align_node = table_pkg.table_align_node:main',
            'table_cleaning_bridge = table_pkg.table_cleaning_bridge:main',
        ],
    },
)

