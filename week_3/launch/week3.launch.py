import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL

    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    pkg_share = get_package_share_directory('week_3')

    sdf_file = os.path.join(pkg_share, 'models', 'arm', 'model.sdf')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    world = os.path.join(
        pkg_share,
        'worlds',
        'minimal_world_2.sdf'
    )

    # 1. Launch Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world], 'on_exit_shutdown': 'true'}.items()
    )

    bridge_params = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'params',
        model_folder+'_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # 3. Spawn Controller Nodes
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', os.path.join(pkg_share, 'config', 'controllers.yaml')],
        parameters=[{'use_sim_time': True},{'robot_description': robot_desc}]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    spawn_joint_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_controller', '--param-file', os.path.join(pkg_share, 'config', 'controllers.yaml')],
        parameters=[{'use_sim_time': True},{'robot_description': robot_desc}]
    )

    return LaunchDescription([
        gazebo,
        spawn_joint_state_broadcaster,
        spawn_joint_controller,
        robot_state_publisher,
        start_gazebo_ros_bridge_cmd
    ])