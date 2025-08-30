from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    driver = Node(
        package='roarm_driver',
        executable='roarm_driver',
        name='roarm_driver',
        output='screen'
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('roarm_moveit_cmd'),
                'launch',
                'command_control.launch.py'
            ])
        )
    )

    getpose = Node(
        package='roarm_moveit_cmd',
        executable='getposecmd_tf2',
        name='getposecmd_tf2',
        output='screen'
    )

    tableclean = Node(
        package='roarm_moveit_cmd',
        executable='movepointcmd',
        name='movepointcmd',
        output='screen'
    )

    setgripper = Node(
        package='roarm_moveit_cmd',
        executable='setgrippercmd',
        name='setgrippercmd',
        output='screen'
    )

    actions = [
        driver,                                        # t=0s
        TimerAction(period=3.0, actions=[moveit_launch]),  # t=1s
        TimerAction(period=15.0, actions=[getpose]),        # t=3s
        TimerAction(period=17.0, actions=[tableclean]),     # t=6s
        TimerAction(period=19.0, actions=[setgripper])      # t=8s
    ]
    return LaunchDescription(actions)
