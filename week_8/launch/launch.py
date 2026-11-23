"""
Navigate a single TurtleBot3 with Nav2, on simulation time, building the map or on a map.

  ros2 launch week_8 launch.py nav2:=slam
  ros2 launch week_8 launch.py nav2:=~/my_map.yaml

Like assessment/launch/nav2.launch.py, this starts the Nav2 nodes it needs itself, under one
lifecycle manager, with config/nav2_params.yaml, and its nav2 argument works the same way:

  nav2:=slam        slam_toolbox builds the map as the robot moves, and Nav2 plans on what has
                    been mapped so far. config/nav2_slam.yaml is added on top of params_file. Save
                    the map as in slam.launch.py, with:
                      ros2 run nav2_map_server map_saver_cli -f ~/my_map
                          --ros-args -p save_map_timeout:=10.0
  nav2:=<map.yaml>  map_server serves the map, as made by map_saver_cli, and AMCL localises the
                    robot on it. The file is a path, or package://<package>/<path> for a file in a
                    package's share directory, which still works if the workspace is moved. AMCL
                    needs to know where the robot starts: give it with RViz's '2D Pose Estimate',
                    or launch with set_initial_pose:=true and x_pose, y_pose, yaw.

With keepout_mask:=<mask.yaml>, also as a path or package://<package>/<path>, both costmaps keep
out of the mask's black areas: a map_server serves the mask, a costmap_filter_info_server tells
the costmaps' keepout filters about it, and config/nav2_keepout.yaml is added on top of
params_file, to add the filters. Make a mask from shapes with keepout_mask's make_mask:

  ros2 run keepout_mask make_mask shapes.yaml --output ~/my_mask
  ros2 launch week_8 launch.py nav2:=~/my_map.yaml keepout_mask:=~/my_mask.yaml

Unlike the assessment, the lifecycle manager starts the nodes at once (autostart).

Bonds are off (bond_timeout 0, bond_heartbeat_period 0), as in the assessment: slam_toolbox fails
to activate with one, and bond timers run on wall time, so a process that stalls for a few seconds
on a busy machine would make the manager reset all of Nav2, even if the simulation had slowed down
with it.

Launch arguments:
  nav2             slam, or a map's YAML file (required)
  keepout_mask     a keepout mask's YAML file (default: none)
  params_file      Nav2's parameters (default: config/nav2_params.yaml)
  set_initial_pose with a map, whether AMCL starts at x_pose, y_pose, yaw (default: false)
  x_pose, y_pose, yaw
  rviz             whether to start RViz (default: true)
  rviz_config      RViz configuration (default: rviz/tb3_navigation2.rviz, a copy of
                   turtlebot3_navigation2's, showing only the map, odom and base_link frames)
  rviz_log_level   RViz's log level (default: warn)
"""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
import launch.logging
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

package_name = 'week_8'

# The lifecycle nodes, in the order the lifecycle manager starts them: the keepout nodes, with a
# keepout mask, before the costmaps that use them
MAPPING_NODES = {'slam': ['slam_toolbox'], 'map': ['map_server', 'amcl']}
KEEPOUT_NODES = ['keepout_filter_mask_server', 'keepout_costmap_filter_info_server']
NAVIGATION_NODES = ['planner_server', 'controller_server', 'behavior_server',
                    'velocity_smoother', 'bt_navigator', 'waypoint_follower']


def resolve_path(value):
    """Return the absolute path of a file given as a path or as package://<package>/<path>."""
    if not value.startswith('package://'):
        return os.path.abspath(os.path.expanduser(value))
    package, _, relative = value[len('package://'):].partition('/')
    try:
        return os.path.join(get_package_share_directory(package), relative)
    except PackageNotFoundError:
        raise RuntimeError(f'{value}: there is no package {package!r}. Is it built and sourced?')


def existing_map(argument, value):
    """
    Return the absolute path of a map's YAML file, which has to exist, as does its image.

    argument is the launch argument it was given as, for the error messages.
    """
    path = resolve_path(value)
    if not os.path.isfile(path):
        raise RuntimeError(f'{argument}:={value}: there is no file {path}')
    try:
        with open(path) as f:
            image = yaml.safe_load(f)['image']
    except (yaml.YAMLError, KeyError, TypeError):
        raise RuntimeError(f"{argument}:={value}: {path} is not a map's YAML file: it has no "
                           'image')
    if not os.path.isfile(os.path.join(os.path.dirname(path), image)):
        raise RuntimeError(f'{argument}:={value}: {path} refers to an image that does not exist: '
                           f'{image}')
    return path


def parse_nav2(value):
    """Return (mode, map file) for the nav2 argument: ('slam', None) or ('map', <path>)."""
    if not value:
        raise RuntimeError('Launch with nav2:=slam, to build the map, or nav2:=<map.yaml>, as a '
                           'path or package://<package>/<path>')
    if value == 'slam':
        return 'slam', None
    return 'map', existing_map('nav2', value)


def nav2_nodes(context):
    config = context.launch_configurations
    mode, map_file = parse_nav2(config['nav2'])
    keepout_mask = (existing_map('keepout_mask', config['keepout_mask'])
                    if config['keepout_mask'] else None)
    set_initial_pose = config['set_initial_pose'].lower() == 'true'

    package_config = os.path.join(get_package_share_directory(package_name), 'config')
    common = {'use_sim_time': True, 'bond_heartbeat_period': 0.0}
    params = [resolve_path(config['params_file'])]
    if mode == 'slam':
        params.append(os.path.join(package_config, 'nav2_slam.yaml'))
    if keepout_mask:
        params.append(os.path.join(package_config, 'nav2_keepout.yaml'))
    params.append(common)

    # Velocity commands go through the smoother on their way to the robot
    to_smoother = [('cmd_vel', 'cmd_vel_nav')]
    specs = [
        ('nav2_planner', 'planner_server', 'planner_server', params, []),
        ('nav2_controller', 'controller_server', 'controller_server', params, to_smoother),
        ('nav2_behaviors', 'behavior_server', 'behavior_server', params, to_smoother),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', params,
         to_smoother + [('cmd_vel_smoothed', 'cmd_vel')]),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', params, []),
        ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', params, []),
    ]
    if mode == 'slam':
        if set_initial_pose:
            launch.logging.get_logger(package_name).warning(
                'set_initial_pose is ignored with nav2:=slam: the map starts where the robot is')
        specs.insert(0, ('slam_toolbox', 'async_slam_toolbox_node', 'slam_toolbox', params, []))
    else:
        amcl_params = list(params)
        if set_initial_pose:
            amcl_params.append({
                'set_initial_pose': True,
                'initial_pose.x': float(config['x_pose']),
                'initial_pose.y': float(config['y_pose']),
                'initial_pose.yaw': float(config['yaw']),
            })
        specs[:0] = [
            ('nav2_map_server', 'map_server', 'map_server',
             params + [{'yaml_filename': map_file}], []),
            ('nav2_amcl', 'amcl', 'amcl', amcl_params, []),
        ]

    if keepout_mask:
        specs += [
            ('nav2_map_server', 'map_server', 'keepout_filter_mask_server',
             params + [{'yaml_filename': keepout_mask}], []),
            ('nav2_map_server', 'costmap_filter_info_server',
             'keepout_costmap_filter_info_server', params, []),
        ]

    nodes = [Node(package=package, executable=executable, name=name, parameters=parameters,
                  remappings=remappings, output='screen')
             for package, executable, name, parameters, remappings in specs]

    manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': (MAPPING_NODES[mode] + (KEEPOUT_NODES if keepout_mask else [])
                           + NAVIGATION_NODES),
            'bond_timeout': 0.0,
            'attempt_respawn_reconnection': False,
            # Each node gets as long as it needs: no limit on reaching it or on its transitions
            'service_timeout': -1.0,
        }],
        output='screen')

    return nodes + [manager]


def generate_launch_description():
    package_share = get_package_share_directory(package_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'nav2', default_value='',
            description="slam: build the map as the robot moves; otherwise a map's YAML file, "
                        'as a path or package://<package>/<path>'),
        DeclareLaunchArgument(
            'keepout_mask', default_value='',
            description="Keepout mask's YAML file, as a path or package://<package>/<path>; "
                        'empty: no keepout areas'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_share, 'config', 'nav2_params.yaml'),
            description='Nav2 parameters file, as a path or package://<package>/<path>'),
        DeclareLaunchArgument(
            'set_initial_pose', default_value='false', choices=['true', 'false', 'True', 'False'],
            description='With a map, whether AMCL starts at x_pose, y_pose, yaw, rather than '
                        "waiting for RViz's '2D Pose Estimate'"),
        DeclareLaunchArgument('x_pose', default_value='0.0',
                              description='Initial x in the map, with set_initial_pose'),
        DeclareLaunchArgument('y_pose', default_value='0.0',
                              description='Initial y in the map, with set_initial_pose'),
        DeclareLaunchArgument('yaw', default_value='0.0',
                              description='Initial yaw in the map, with set_initial_pose'),
        DeclareLaunchArgument(
            'rviz', default_value='true', choices=['true', 'false', 'True', 'False'],
            description='Whether to start RViz'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(package_share, 'rviz', 'tb3_navigation2.rviz'),
            description='Full path to the RViz configuration file to use'),
        DeclareLaunchArgument(
            'rviz_log_level', default_value='warn', description='RViz log level'),

        OpaqueFunction(function=nav2_nodes),

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
