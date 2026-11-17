"""
Spawn one TurtleBot3 in a running simulation, with everything it needs in its own namespace.

Meant to be included by week_7.launch.py inside a GroupAction that has pushed the robot's
namespace (PushRosNamespace) and remapped /tf and /tf_static to tf and tf_static (SetRemap). The
namespace is also the name of the robot's model in Gazebo, e.g. robot1:

  Gazebo model robot1, whose topics are moved to /robot1/... (see week_7/robot_model.py)
  /robot1/robot_state_publisher  publishing /robot1/robot_description and /robot1/tf_static
  /robot1/bridge                 /robot1/{cmd_vel,odom,tf,joint_states,imu,scan,camera/...}

Based on:
https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/humble-devel/turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py
https://github.com/ros-planning/navigation2/blob/humble/nav2_bringup/launch/tb3_simulation_launch.py
"""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from week_7.launch_utils import temporary_file
from week_7.robot_model import namespace_robot_model

TURTLEBOT3_MODEL = 'waffle_pi'


def bringup_actions(context: LaunchContext):
    config = context.launch_configurations
    robot_name = config.get('ros_namespace', '').strip('/')
    if not robot_name:
        raise RuntimeError("spawn_robot.launch.py must be included in a robot's namespace "
                           '(PushRosNamespace): it is also the name of the robot in Gazebo')

    turtlebot3_share = get_package_share_directory('turtlebot3_gazebo')
    with open(os.path.join(turtlebot3_share, 'urdf', f'turtlebot3_{TURTLEBOT3_MODEL}.urdf')) as f:
        robot_description = f.read()

    tree = ET.parse(config['robot_sdf'])
    namespace_robot_model(tree, robot_name)
    robot_sdf = temporary_file(robot_name, '.sdf', ET.tostring(tree.getroot(), encoding='unicode'))

    # Relative names in the node's namespace: robot_description, joint_states, and tf_static
    # (from /tf_static, remapped by week_7.launch.py)
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}])

    start_gazebo_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', robot_name,
            '-file', robot_sdf,
            '-x', config['x_pose'],
            '-y', config['y_pose'],
            '-z', config['z_pose'],
            '-R', config['roll'],
            '-P', config['pitch'],
            '-Y', config['yaw']])

    # expand_gz_topic_names puts the relative Gazebo topics of the config file in the node's
    # namespace too, where namespace_robot_model has moved the model's topics.
    start_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(get_package_share_directory('week_7'), 'config',
                                        'robot_bridge.yaml'),
            'expand_gz_topic_names': True}])

    return [start_robot_state_publisher_cmd, start_gazebo_spawner_cmd, start_bridge_cmd]


def generate_launch_description():
    ld = LaunchDescription()

    default_sdf = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models',
                               f'turtlebot3_{TURTLEBOT3_MODEL}', 'model.sdf')
    for name, default, description in (
        ('x_pose', '0.0', 'Initial pose: x'),
        ('y_pose', '0.0', 'Initial pose: y'),
        ('z_pose', '0.01', 'Initial pose: z'),
        ('roll', '0.0', 'Initial pose: roll'),
        ('pitch', '0.0', 'Initial pose: pitch'),
        ('yaw', '0.0', 'Initial pose: yaw'),
        ('robot_sdf', default_sdf,
         'Full path to the robot SDF file to spawn in Gazebo, before its topics are namespaced'),
    ):
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))

    ld.add_action(OpaqueFunction(function=bringup_actions))

    return ld
