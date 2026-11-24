"""
Simulate a single TurtleBot3 in the TurtleBot3 world, with asymmetric obstacles.

  ros2 launch week_8 turtlebot3_world_auro.launch.py

Then map it, or navigate in it, with:

  ros2 launch week_8 slam.launch.py
  ros2 launch week_8 launch.py nav2:=slam
  ros2 launch week_8 launch.py nav2:=<map.yaml>

The world, worlds/turtlebot3_world_auro.world, is turtlebot3_gazebo's turtlebot3_world.world with
its nine evenly spaced pillars replaced by four different obstacles, so that the arena is not
symmetric. The robot is in no namespace: /cmd_vel, /odom, /scan, /tf, /clock, and, unless the
model is the burger, /camera/image_raw.

Launch arguments:
  model           burger, waffle or waffle_pi (default: waffle_pi)
  x_pose, y_pose  the robot's start position in the world (default: -2.0, -0.5)
  yaw_pose        the robot's start heading (default: 0.0)
  headless        whether to run without the Gazebo GUI (default: false)
"""

import atexit
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from week_8.launch_utils import stop_gazebo_server, temporary_file

package_name = 'week_8'
WORLD = 'turtlebot3_world_auro.world'
SPAWN_Z = '0.01'


def gazebo_server(context):
    """
    Start the Gazebo server on a temporary copy of the world, and make sure it stops with us.

    The copy is unique to this launch, so that when the launch exits, stop_gazebo_server can find
    its own Gazebo server, and only that, if ros2 launch has left it running: it does when it shuts
    itself down, e.g. because the Gazebo window was closed, as it then only stops the gz tool's
    wrapper process.
    """
    with open(os.path.join(get_package_share_directory(package_name), 'worlds', WORLD)) as f:
        world_file = temporary_file('world', '.world', f.read())
    # atexit runs the last registered first: this, before temporary_file removes the copy
    atexit.register(stop_gazebo_server, world_file)

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s -v2 {world_file}',
                          'on_exit_shutdown': 'true'}.items())]


def robot(context):
    """Spawn the robot, and start its state publisher and bridges, for the model chosen."""
    model = LaunchConfiguration('model').perform(context)
    model_folder = f'turtlebot3_{model}'
    turtlebot3_share = get_package_share_directory('turtlebot3_gazebo')
    # turtlebot3_gazebo's robot_state_publisher.launch.py reads this variable
    os.environ['TURTLEBOT3_MODEL'] = model

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_share, 'launch', 'robot_state_publisher.launch.py')),
            # A node parameter there, which would otherwise override SetParameter
            launch_arguments={'use_sim_time': 'true'}.items()),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', model,
                       '-file', os.path.join(turtlebot3_share, 'models', model_folder,
                                             'model.sdf'),
                       '-x', LaunchConfiguration('x_pose'),
                       '-y', LaunchConfiguration('y_pose'),
                       '-z', SPAWN_Z,
                       '-Y', LaunchConfiguration('yaw_pose')],
            output='screen'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='turtlebot3_bridge',
            parameters=[{'config_file': os.path.join(turtlebot3_share, 'params',
                                                     f'{model_folder}_bridge.yaml')}],
            output='screen'),
    ]
    if model != 'burger':
        actions.append(Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['/camera/image_raw'],
            output='screen'))
    return actions


def generate_launch_description():
    package_share = get_package_share_directory(package_name)
    turtlebot3_share = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    ld = LaunchDescription()

    ld.add_action(SetParameter(name='use_sim_time', value=True))

    ld.add_action(DeclareLaunchArgument(
        'model', default_value='waffle_pi', choices=['burger', 'waffle', 'waffle_pi'],
        description='TurtleBot3 model'))
    for name, default, description in (
        ('x_pose', '-2.0', 'Robot start position in x (m)'),
        ('y_pose', '-0.5', 'Robot start position in y (m)'),
        ('yaw_pose', '0.0', 'Robot start heading (rad)'),
        ('headless', 'false', 'Whether to run without the Gazebo GUI'),
    ):
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))

    # The arena's model is in this package; its meshes, and the robots, are in turtlebot3_gazebo
    ld.add_action(SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [os.path.join(package_share, 'models'), ':',
         os.path.join(turtlebot3_share, 'models'), ':',
         EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')]))

    # Gazebo: the server, and the GUI unless headless
    ld.add_action(OpaqueFunction(function=gazebo_server))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        condition=UnlessCondition(LaunchConfiguration('headless')),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()))

    ld.add_action(OpaqueFunction(function=robot))

    return ld
