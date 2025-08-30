from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='table_pkg',
            executable='table_manager_node',
            name='table_manager_node',
            output='screen'
        ),
        Node(
            package='table_pkg',
            executable='dirt_check_node',
            name='dirt_check_node',
            output='screen'
        ),
        Node(
            package='table_pkg',
            executable='table_align_node',
            name='table_align_node',
            output='screen'
        ),
        Node(
            package='table_pkg',
            executable='camera_stream_node',
            name='camera_stream_node',
            output='screen'
        ),
    ])

