"""
Sketch of a solution to task 1: the mobile robot.

wait_for_simulation() waits until the simulation monitor says, on /simulation_ready, that the
simulation is running.
"""

import math

from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose

from solution.task1.environment_helper import map_to_world, world_to_map
from solution.task1.fetch_end_effector_fsm import FetchEndEffector, RobotPlatform

from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from solution.task1.tag_identifier import detect_tags

import numpy as np

TAG_IDENTIFY_FRAMES = 5
CAMERA_TIMEOUT = 50 # Max time allowed for the camera to return frame (5s)

class Task1(Node):

    def __init__(self):
        super().__init__('task1')
        # slam_toolbox or amcl, set by solution.launch.py from its nav2 argument
        self.localizer = self.declare_parameter('localizer', 'slam_toolbox').value

        # only used to wait for nav2 to start up, the goals are sent with nav_client
        self.navigator = BasicNavigator('task1_navigator')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.image = None

        # subscribers
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # platform of state machine
        self.platform = RobotPlatform(
            navigate_to=self.navigate_to, identify_tags=self.identify_tags,
            robot_pose=self.robot_pose, stop=self.stop)

        # state machine
        self.timer_period = 0.1
        self.time_unit = 0.1 # so timeouts are in tenths of a second
        self.fetch_fsm = FetchEndEffector(self.get_logger(), self.timer_period, self.time_unit,
                                          parent=self.platform, on_change=self.on_change)
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # pre-existing code
        self.simulation_ready = False
        # Latched: the last value is received whenever the subscription is made
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/simulation_ready', self.on_ready, latched)

    # PUBLISHERS

    def publish_velocity(self, linear):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = linear
        self.cmd_vel_pub.publish(msg)

    # CALL BACKS

    def image_callback(self, msg):
        self.image = msg

    # ACTION CALLS

    def navigate_to(self, x, y, yaw, timeout):
        # send a nav2 goal at (x, y, yaw) in the world frame
        goal = NavigateToPose.Goal()
        goal.pose = self.map_pose(x, y, yaw)

        while not self.nav_client.server_is_ready():
            yield

        send_future = self.nav_client.send_goal_async(goal)
        while not send_future.done():
            yield
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().warning(f'Nav2 rejected goal ({x:.2f}, {y:.2f})')
            return False

        result_future = goal_handle.get_result_async()
        waited = 0
        while not result_future.done():
            if waited >= timeout:
                self.get_logger().warning(f'Failed to reach ({x:.2f}, {y:.2f}) after {timeout / 10:.0f} s, '
                                          'giving up')
                goal_handle.cancel_goal_async()
                return False
            waited = waited + 1
            yield
        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    # other stuff

    def identify_tags(self):
        # camera noise is random in each frame so use median of several frames
        frames = []
        while len(frames) < TAG_IDENTIFY_FRAMES:
            # forget older frames and wait for the next one
            self.image = None
            waited = 0
            while self.image is None:
                if waited >= CAMERA_TIMEOUT:
                    self.get_logger().warning(f'Failed to recieve any frames after {CAMERA_TIMEOUT / 10:.0f}s, giving up')
                    return None
                waited = waited + 1
                yield
            frames.append(self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8'))
        image = np.median(np.stack(frames), axis=0).astype(np.uint8)
        return detect_tags(image)

    def stop(self):
        self.publish_velocity(0.0)

    def map_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = world_to_map(x, y)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = quaternion_from_euler(0, 0, yaw)
        return pose

    def robot_pose(self):
        # (x, y, yaw) of the robot in the world frame None if the transform isnt available
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException as e:
            self.get_logger().warning(f'{e}')
            return None
        q = t.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        x, y = map_to_world(t.transform.translation.x, t.transform.translation.y)
        return x, y, yaw

    def control_loop(self):
        if self.simulation_ready and not self.fetch_fsm.finished:
            self.fetch_fsm.execute()

    def on_change(self, configuration):
        self.get_logger().info(f'State: {configuration}')

    # pre-existing code

    def on_ready(self, msg):
        self.simulation_ready = msg.data

    def wait_for_simulation(self):
        """Spin until the simulation is ready."""
        self.get_logger().info('Waiting for the simulation')
        while rclpy.ok() and not self.simulation_ready:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('The simulation is ready')

    def start_nav2(self):
        """Wait until Nav2 is active, and localising the robot."""

        if self.localizer == 'amcl':
            # If using a map.
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            self.navigator.setInitialPose(pose)

        # With AMCL this also republishes the initial pose until AMCL takes it
        self.navigator.waitUntilNav2Active(localizer=self.localizer)
        self.get_logger().info('Nav2 is ready')


def main():
    rclpy.init()
    node = Task1()
    try:
        node.wait_for_simulation()
        node.start_nav2()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.navigator.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()