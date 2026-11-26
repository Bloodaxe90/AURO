"""
Sketch of a solution to task 2: the crane and the reactor.

wait_for_simulation() waits until the simulation monitor says, on /simulation_ready, that the
simulation is running.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, Float64MultiArray
from ros_gz_interfaces.srv import ControlWorld
from assessment_interfaces.srv import CellPose, RodPowerLevel
from sensor_msgs.msg import Image, JointState

from cv_bridge import CvBridge
from solution.move_rods_fsm import CranePlatform, MoveRods
from solution.cell_identifier import identify_pool_cells

import numpy as np

IDENTIFY_POOL_FRAMES = 5
CAMERA_TIMEOUT = 50 # Max time allowed for the camera to return frame (5s)


class Task2(Node):

    def __init__(self):
        super().__init__('task2')

        self.bridge = CvBridge()
        self.image = None

        self.crane_x = 0.0
        self.crane_y = 0.0

        self.current_power_value = None
        self.target_power_value = None

        control_rod_coefficient = float(self.declare_parameter('control_rod_coefficient', 0.2).value)
        storage_rod_level = float(self.declare_parameter('replacement_level', 100.0).value)
        maximum_power = float(self.declare_parameter('maximum_power_level', 0.0).value)
        if maximum_power <= 0.0:
            maximum_power = 100.0 * 93  # reactor_monitors default (93 fuel cells at level 100)
        full_fuel_rod_power = 100.0 * 100.0 / maximum_power  # % of maximum power for one fuel rod at level 100

        # subscribers
        self.create_subscription(Bool, '/crane/at_target', self.at_target_callback, 10)
        self.create_subscription(Image, '/crane/winch_camera/image', self.image_callback, 10)
        self.create_subscription(JointState, '/crane/joint_states', self.joint_callback, 10)
        self.create_subscription(Float64, '/current_power_level', self.current_power_callback, 10)
        self.create_subscription(Float64, '/target_power_level', self.target_power_callback, 10)

        # publishers
        self.target_pub = self.create_publisher(Float64MultiArray, '/crane/position_commands', 10)
        self.carrying_pub = self.create_publisher(Bool, '/crane/carrying', 10)

        # clients
        self.cell_client = self.create_client(CellPose, '/cell_pose')
        self.brake_release_client = self.create_client(ControlWorld, '/crane/winch/brake/release')
        self.brake_engage_client = self.create_client(ControlWorld, '/crane/winch/brake/engage')
        self.attach_client = self.create_client(ControlWorld, '/crane/winch/attach')
        self.detach_client = self.create_client(ControlWorld, '/crane/winch/detach')
        self.rod_level_client = self.create_client(RodPowerLevel, '/crane/winch/rod_power_level')

        # platform of state machine
        self.platform = CranePlatform(
            send_target=self.send_target, set_carrying=self.set_carrying,
            release_brake=self.release_brake, engage_brake=self.engage_brake, attach=self.attach, detach=self.detach,
            cell_pose=self.cell_pose, crane_pose=self.crane_pose, identify_pool=self.identify_pool,
            current_power=self.current_power, target_power=self.target_power, rod_level=self.rod_level)

        # state machine
        self.timer_period = 0.1
        self.time_unit = 0.1 # was 1 is now less so i can pass fractions of a second for wait()
        self.move_rod_fsm = MoveRods(self.get_logger(), self.timer_period, self.time_unit,
                                     parent=self.platform,
                                     control_rod_coefficient=control_rod_coefficient,
                                     full_fuel_rod_power=full_fuel_rod_power,
                                     storage_rod_level=storage_rod_level,
                                     on_change=self.on_change)
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # pre-existing code
        self.simulation_ready = False
        # Latched: the last value is received whenever the subscription is made
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/simulation_ready', self.on_ready, latched)

    # CALLBACKS

    def at_target_callback(self, msg):
        if msg.data:
            self.platform.at_target.set()

    def image_callback(self, msg):
        self.image = msg

    def joint_callback(self, msg):
        try:
            self.crane_x = float(msg.position[msg.name.index('bridge_slide')])
            self.crane_y = float(msg.position[msg.name.index('winch_block_slide')])
        except ValueError:
            pass

    def current_power_callback(self, msg):
        self.current_power_value = msg.data

    def target_power_callback(self, msg):
        self.target_power_value = msg.data

    # PUBLISHERS

    def send_target(self, x, y, z, tolerance):
        self.target_pub.publish(Float64MultiArray(data=[x, y, z, tolerance]))

    def set_carrying(self, carrying):
        self.carrying_pub.publish(Bool(data=carrying))

    # SERVICE CALLS

    def release_brake(self):
        response = yield from self.call_service(self.brake_release_client, ControlWorld.Request())
        return response.success

    def engage_brake(self):
        response = yield from self.call_service(self.brake_engage_client, ControlWorld.Request())
        return response.success

    def attach(self):
        response = yield from self.call_service(self.attach_client, ControlWorld.Request())
        return response.success

    def detach(self):
        response = yield from self.call_service(self.detach_client, ControlWorld.Request())
        return response.success

    def cell_pose(self, cell):
        response = yield from self.call_service(self.cell_client, CellPose.Request(cell=cell))
        return response.success, response.centre.x, response.centre.y

    def rod_level(self):
        # level of the fuel rod below the winch, if the winch is close enough above it
        response = yield from self.call_service(self.rod_level_client, RodPowerLevel.Request())
        return response.success, response.power_level

    def call_service(self, client, request):
        # check server is up
        while not client.service_is_ready():
            yield

        future = client.call_async(request)
        try:
            while not future.done():
                yield
        finally:
            if not future.done():
                client.remove_pending_request(future)
        return future.result()

    # other stuff

    def identify_pool(self, pool, x, y, z):
        # camera noise is random in each frame so use median of several frames
        frames = []
        while len(frames) < IDENTIFY_POOL_FRAMES:
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
        return identify_pool_cells(image, pool, x, y, z)

    def control_loop(self):
        if self.simulation_ready and not self.move_rod_fsm.finished:
            self.move_rod_fsm.execute()

    def on_change(self, configuration):
        self.get_logger().info(f'State: {configuration}')

    # TODO change to getters to avoild name overlap
    def crane_pose(self):
        return self.crane_x, self.crane_y

    def current_power(self):
        return self.current_power_value

    def target_power(self):
        return self.target_power_value

    # pre-existing code

    def on_ready(self, msg):
        self.simulation_ready = msg.data

    def wait_for_simulation(self):
        """Spin until the simulation is ready."""
        self.get_logger().info('Waiting for the simulation')
        while rclpy.ok() and not self.simulation_ready:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('The simulation is ready')


def main():
    rclpy.init()
    node = Task2()
    try:
        node.wait_for_simulation()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
