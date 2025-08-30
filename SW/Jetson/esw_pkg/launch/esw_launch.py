from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    drive_pkg_share = get_package_share_directory('drive_pkg')
    sensor_pkg_share = get_package_share_directory('sensor_pkg')
    table_pkg_share = get_package_share_directory('table_pkg')
    roarm_driver_share = get_package_share_directory('roarm_driver')

    drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(drive_pkg_share, 'launch', 'drive_launch.py')
        )
    )

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensor_pkg_share, 'launch', 'sensor_launch.py')
        )
    )

    table_cleaning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(table_pkg_share, 'launch', 'table_cleaning_sys.py')
        )
    )

    roarm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(roarm_driver_share, 'launch', 'roarm_launch.py')
        )
    )

    jetson_server_node = Node(
        package='navigate_server',
        executable='jetson_server_node',
        name='jetson_server_node',
        output='screen'
    )

    return LaunchDescription([
        drive_launch,
        sensor_launch,
        table_cleaning_launch,
        roarm_launch,
        jetson_server_node
    ])

