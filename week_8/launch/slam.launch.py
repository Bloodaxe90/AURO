"""
Build a map with slam_toolbox, in its online asynchronous mode, on simulation time, without Nav2.

To build one while navigating with Nav2, use launch.py nav2:=slam instead.

  ros2 launch week_8 slam.launch.py

Start a simulation with a single TurtleBot3 first, publishing /scan, /tf and /clock, then drive
the robot around. When the map looks complete, save it with:

  ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args -p save_map_timeout:=10.0

which writes ~/my_map.yaml and ~/my_map.pgm, for launch.py nav2:=~/my_map.yaml. map_saver_cli
only waits save_map_timeout for the map, 2 s by default, and as a new process it first has to
discover slam_toolbox, which can take longer than that: it then fails with 'Failed to spin map
subscription'.

Launch arguments:
  slam_params_file  slam_toolbox's parameters (default: its mapper_params_online_async.yaml)
  rviz              whether to start RViz (default: true)
  rviz_config       RViz configuration (default: rviz/tb3_cartographer.rviz, a copy of
                    turtlebot3_cartographer's, showing only the map, odom and base_link
                    frames, and rotated as in launch.py's RViz: the map's x axis, which the
                    robot starts facing, points up rather than right)
  rviz_log_level    RViz's log level (default: warn)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    package_share = get_package_share_directory('week_8')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(slam_toolbox_share, 'config',
                                       'mapper_params_online_async.yaml'),
            description='Full path to the slam_toolbox parameters file to use'),
        DeclareLaunchArgument(
            'rviz', default_value='true', choices=['true', 'false', 'True', 'False'],
            description='Whether to start RViz'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(package_share, 'rviz', 'tb3_cartographer.rviz'),
            description='Full path to the RViz configuration file to use'),
        DeclareLaunchArgument(
            'rviz_log_level', default_value='warn', description='RViz log level'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': LaunchConfiguration('slam_params_file'),
            }.items()),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config'),
                       '--ros-args', '--log-level', LaunchConfiguration('rviz_log_level')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'),
    ])
