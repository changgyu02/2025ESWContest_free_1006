from setuptools import setup

package_name = 'navigate_server'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='FastAPI + ROS2 기반 테이블 자율주행 서버',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetson_server_node = navigate_server.jetson_server_node:main',
        ],
    },
)

