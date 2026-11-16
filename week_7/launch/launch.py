"""
Simulate several TurtleBot3s in one arena, each in its own ROS namespace, and barrels to pick up.

  ros2 launch week_7 week_7.launch.py num_robots:=2

The robots are named robot1, robot2, ...: each name is the robot's namespace in ROS and its
model's name in Gazebo, with start poses from config/initial_poses.yaml. Every robot gets the same
nodes and topics, under its own namespace, so the same code can drive any of them:

  /robot1/cmd_vel, /robot1/odom, /robot1/scan, /robot1/camera/image_raw, ...
  /robot1/tf, /robot1/tf_static   the robot's own TF tree: odom -> base_footprint -> ...

Nodes in a robot's namespace have /tf and /tf_static remapped to these. Without that, every robot
would publish its odom -> base_footprint on the one /tf, and the trees would clash, as the frame
names are the same. Each robot also gets an RViz window, showing what that robot sees.

Drive a robot, and pick up and offload barrels with it, using:

  ros2 run week_7 teleop_keyboard --ros-args -r __ns:=/robot1

Based on:
https://github.com/ros-planning/navigation2/blob/humble/nav2_bringup/launch/cloned_multi_tb3_simulation_launch.py
"""

import atexit
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from week_7.launch_utils import stop_gazebo_server, temporary_file
import yaml

package_name = 'week_7'
WORLD_NAME = 'week_7'  # as in worlds/week_7.world


def load_per_count(path, num_robots, what):
    """Return the entry for num_robots robots in the YAML file at path."""
    with open(path) as f:
        configuration = yaml.safe_load(f)
    if num_robots not in configuration:
        raise RuntimeError(f'num_robots must be one of {sorted(configuration)}: {path} has no '
                           f'{what} for {num_robots} robots')
    return configuration[num_robots]


def gazebo_server(context: LaunchContext):
    """
    Start the Gazebo server on a temporary copy of the world, and make sure it stops with us.

    The copy is unique to this launch, so that when the launch exits, stop_gazebo_server can find
    its own Gazebo server, and only that, if ros2 launch has left it running.
    """
    with open(os.path.join(get_package_share_directory(package_name), 'worlds',
                           'week_7.world')) as f:
        world_file = temporary_file('world', '.world', f.read())
    # atexit runs the last registered first: this, before temporary_file removes the copy
    atexit.register(stop_gazebo_server, world_file)

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s -v2 {world_file}',
                          'on_exit_shutdown': 'true'}.items())]


def group_action(context: LaunchContext):
    config = context.launch_configurations
    launch_dir = os.path.join(get_package_share_directory(package_name), 'launch')

    num_robots = int(config['num_robots'])
    initial_poses = load_per_count(config['initial_poses'], num_robots, 'initial poses')
    rviz_windows = load_per_count(config['rviz_windows'], num_robots, 'RViz windows')

    bringup_cmd_group = []

    for robot_name, init_pose in initial_poses.items():
        window = rviz_windows[robot_name]
        group = GroupAction([

            # Everything below is in the robot's namespace, and sees its own TF tree
            PushRosNamespace(robot_name),
            SetRemap('/tf', 'tf'),
            SetRemap('/tf_static', 'tf_static'),

            LogInfo(msg=['Launching namespace=', robot_name, ' init_pose=', str(init_pose)]),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
                condition=IfCondition(config['use_rviz']),
                launch_arguments={'rviz_config': config['rviz_config'],
                                  'window_x': str(window['window_x']),
                                  'window_y': str(window['window_y']),
                                  'window_width': str(window['window_width']),
                                  'window_height': str(window['window_height'])}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_robot.launch.py')),
                launch_arguments={'x_pose': str(init_pose['x']),
                                  'y_pose': str(init_pose['y']),
                                  'yaw': str(init_pose['yaw'])}.items()),
        ])

        bringup_cmd_group.append(group)

    return bringup_cmd_group


def generate_launch_description():
    package_share = get_package_share_directory(package_name)
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    ld = LaunchDescription()

    ld.add_action(SetParameter(name='use_sim_time', value=True))

    for name, default, description in (
        ('num_robots', '1', 'Number of robots to spawn'),
        ('use_rviz', 'True', 'Whether to start RViz, one window per robot'),
        ('headless', 'False', 'Whether to run without the Gazebo GUI'),
        ('rviz_config', os.path.join(package_share, 'rviz', 'namespaced.rviz'),
         'Full path to the RViz config file to use'),
        ('rviz_windows', os.path.join(package_share, 'config', 'rviz_windows.yaml'),
         'Full path to the RViz windows YAML file to use'),
        ('initial_poses', os.path.join(package_share, 'config', 'initial_poses.yaml'),
         "Full path to the YAML file of the robots' names and initial poses"),
        ('barrel_manager', 'True', 'Whether to start the barrel manager'),
        ('random_seed', '0', 'Seed for where the barrel manager places the barrels'),
    ):
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))

    # First, so that a bad num_robots stops the launch before anything is started. The robots'
    # spawners wait for the Gazebo server.
    ld.add_action(OpaqueFunction(function=group_action))

    # Gazebo: the server, and the GUI unless headless
    gz_sim_launch = PythonLaunchDescriptionSource(
        os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py'))
    ld.add_action(OpaqueFunction(function=gazebo_server))
    ld.add_action(IncludeLaunchDescription(
        gz_sim_launch,
        condition=UnlessCondition(LaunchConfiguration('headless')),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()))

    # The simulation's clock, for every node (use_sim_time): one bridge, outside any namespace
    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'))

    ld.add_action(Node(
        package=package_name,
        executable='barrel_manager',
        output='screen',
        parameters=[{'world': WORLD_NAME,
                     'random_seed': ParameterValue(LaunchConfiguration('random_seed'),
                                                   value_type=int)}],
        condition=IfCondition(LaunchConfiguration('barrel_manager'))))

    return ld
