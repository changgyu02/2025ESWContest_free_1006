from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roarm_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 패키지 인덱스 등록
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml 설치
        ('share/' + package_name, ['package.xml']),
        # launch 파일 설치 (launch/*.py 전부)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dudu',
    maintainer_email='dudu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roarm_driver = roarm_driver.roarm_driver:main'
        ],
    },
)

