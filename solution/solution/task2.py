"""
Sketch of a solution to task 2: the crane and the reactor.

wait_for_simulation() waits until the simulation monitor says, on /simulation_ready, that the
simulation is running.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray
from ros_gz_interfaces.srv import ControlWorld
from assessment_interfaces.srv import CellPose
from sensor_msgs.msg import Image, JointState

from cv_bridge import CvBridge
from solution.move_rods_fsm import CranePlatform, MoveRods
from solution.cell_identifier import identify_pool_cells

class Task2(Node):

    def __init__(self):
        super().__init__('task2')

        self.bridge = CvBridge()
        self.image = None

        rods_to_move = self.declare_parameter('rods_to_move', 'M0:N8,N8:M0').value

        self.crane_x = 0.0
        self.crane_y = 0.0

        # subscribers
        self.create_subscription(Bool, '/crane/at_target', self.at_target_callback, 10)
        self.create_subscription(Image, '/crane/winch_camera/image', self.image_callback, 10)
        self.create_subscription(JointState, '/crane/joint_states', self.joint_callback, 10)


        # publishers
        self.target_pub = self.create_publisher(Float64MultiArray, '/crane/position_commands', 10)
        self.carrying_pub = self.create_publisher(Bool, '/crane/carrying', 10)

        # clients
        self.cell_client = self.create_client(CellPose, '/cell_pose')
        self.brake_release_client = self.create_client(ControlWorld, '/crane/winch/brake/release')
        self.brake_engage_client = self.create_client(ControlWorld, '/crane/winch/brake/engage')
        self.attach_client = self.create_client(ControlWorld, '/crane/winch/attach')
        self.detach_client = self.create_client(ControlWorld, '/crane/winch/detach')

        # platform of state machine
        self.platform = CranePlatform(
            send_target=self.send_target, set_carrying=self.set_carrying,
            release_brake=self.release_brake, engage_brake=self.engage_brake, attach=self.attach, detach=self.detach,
            cell_pose=self.cell_pose, crane_pose=self.crane_pose, identify_pool=self.identify_pool)

        # state machine
        self.timer_period = 0.1
        self.time_unit = 1
        self.move_rod_fsm = MoveRods(self.get_logger(), self.timer_period, self.time_unit,
                           parent=self.platform, rods_to_move=rods_to_move,
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

    # PUBLISHERS

    def send_target(self, x, y, z):
        self.target_pub.publish(Float64MultiArray(data=[x, y, z]))

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
        # forget older frames and wait for the next one
        self.image = None
        while self.image is None:
            yield
        image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        return identify_pool_cells(image, pool, x, y, z)

    def control_loop(self):
        if self.simulation_ready and not self.move_rod_fsm.finished:
            self.move_rod_fsm.execute()

    def on_change(self, configuration):
        self.get_logger().info(f'State: {configuration}')

    def crane_pose(self):
        return self.crane_x, self.crane_y

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
