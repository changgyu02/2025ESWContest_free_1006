from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drive_pkg',
            executable='goal_server_node',
            name='goal_server',
            output='screen'
        ),
        Node(
            package='drive_pkg',
            executable='planner_node',
            name='planner',
            output='screen'
        ),
        Node(
            package='drive_pkg',
            executable='controller_node',
            name='controller',
            output='screen'
        ),
        Node(
            package='drive_pkg',
            executable='go_home_node',
            name='go_home',
            output='screen'
        ),
        Node(
            package='drive_pkg',
            executable='map_to_odom_tf_broadcaster',
            name='map_to_odom_tf_broadcaster',
            output='screen'
        ),
    ])
