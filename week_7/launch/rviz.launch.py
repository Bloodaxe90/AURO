"""
Start RViz for one robot, in its namespace.

Meant to be included by week_7.launch.py inside a robot's GroupAction (see spawn_robot.launch.py),
whose remapping of /tf and /tf_static makes this RViz see only that robot's TF tree. The RViz
config's <robot_namespace> is replaced with the namespace, and <window_x>, <window_y>,
<window_width> and <window_height> with the window's position and size. Closing any RViz window
shuts down the whole launch.

Based on:
https://raw.githubusercontent.com/ros-planning/navigation2/humble/nav2_bringup/launch/rviz_launch.py
https://github.com/ROBOTIS-GIT/turtlebot3/blob/humble-devel/turtlebot3_bringup/launch/rviz2.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from week_7.launch_utils import temporary_file


def rviz_action(context: LaunchContext):
    config = context.launch_configurations
    namespace = config.get('ros_namespace', '')

    with open(config['rviz_config']) as f:
        rviz_config = f.read()
    for placeholder, value in (('<robot_namespace>', namespace),
                               ('<window_x>', config['window_x']),
                               ('<window_y>', config['window_y']),
                               ('<window_width>', config['window_width']),
                               ('<window_height>', config['window_height'])):
        rviz_config = rviz_config.replace(placeholder, value)
    rviz_config_file = temporary_file(namespace.strip('/') or 'rviz', '.rviz', rviz_config)

    start_namespaced_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    exit_event_handler_namespaced = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_namespaced_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    return [start_namespaced_rviz_cmd, exit_event_handler_namespaced]


def generate_launch_description():
    ld = LaunchDescription()

    for name, default, description in (
        ('rviz_config',
         os.path.join(get_package_share_directory('week_7'), 'rviz', 'namespaced.rviz'),
         'Full path to the RViz config file to use'),
        ('window_x', '0', 'RViz window X offset from top-left corner of screen'),
        ('window_y', '0', 'RViz window Y offset from top-left corner of screen'),
        ('window_width', '1280', 'RViz window width'),
        ('window_height', '720', 'RViz window height'),
    ):
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))

    ld.add_action(OpaqueFunction(function=rviz_action))

    return ld
