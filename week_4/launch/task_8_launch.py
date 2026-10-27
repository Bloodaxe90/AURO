import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    turtlebot3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'turtlebot3_house.launch.py')
        )
    )

    fsm_controller = Node(
        package='week_4',
        executable='task8_TurtleBot3FSMRC',
        name='sim',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    detector = Node(
        package='week_4',
        executable='task8_DetectorFSM',
        name='sim',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()
    ld.add_action(turtlebot3_world)
    ld.add_action(fsm_controller)
    ld.add_action(detector)

    return ld