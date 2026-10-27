import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    turtlebot3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'empty_world.launch.py')
        )
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    forward_time = LaunchConfiguration('forward_time')
    turn_time = LaunchConfiguration('turn_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
    )

    declare_forward_time = DeclareLaunchArgument(
        'forward_time',
        default_value='4',
    )

    declare_turn_time = DeclareLaunchArgument(
        'turn_time',
        default_value='1',
    )

    week_4_state_machine = Node(
        package='week_4',
        executable='week_4_state_machine',
        name='week_4_state_machine',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'forward_time': forward_time,
            'turn_time': turn_time
        }]
    )

    ld = LaunchDescription()
    ld.add_action(turtlebot3_world)

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_forward_time)
    ld.add_action(declare_turn_time)

    ld.add_action(week_4_state_machine)

    return ld