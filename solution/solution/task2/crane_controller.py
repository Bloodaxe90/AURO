import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray

# Only the joint for the crane
JOINTS = ['bridge_slide', 'winch_block_slide', 'winch_lift'] # x y z
GRAVITY = 9.8
WINCH_MASS = 100.0
ROD_MASS = 100.0

class CraneController(Node):

    def __init__(self):
        super().__init__('crane_controller')

        # PD coefficients for x, y, z (tuned)
        self.Kp = [float(self.declare_parameter('Kp_x', 3400.0).value),
                   float(self.declare_parameter('Kp_y', 1400.0).value),
                   float(self.declare_parameter('Kp_z', 938.0).value)]
        self.Kd = [float(self.declare_parameter('Kd_x', 3300.0).value),
                   float(self.declare_parameter('Kd_y', 1350.0).value),
                   float(self.declare_parameter('Kd_z', 700.0).value)]

        # Tolerances for determining if the crane is at the target position and the target velocity is zero, Low since the rod has a 0.025 m play in the cell
        self.position_tolerance = float(self.declare_parameter('position_tolerance', 0.005).value)
        self.velocity_tolerance = float(self.declare_parameter('velocity_tolerance', 0.01).value)
        self.waypoint_tolerance = 0.0

        self.max_error = float(self.declare_parameter('max_error', 1.5).value) # ensuring the crane doesn't move too fast

        self.desired_joint_positions = None
        self.carrying = False

        self.joint_sub = self.create_subscription(
            JointState, '/crane/joint_states', self.joint_callback, 10)
        self.command_sub = self.create_subscription(
            Float64MultiArray, '/crane/position_commands', self.command_callback, 10)
        self.carrying_sub = self.create_subscription(
            Bool, '/crane/carrying', self.carrying_callback, 10)

        self.command_pub = self.create_publisher(
            Float64MultiArray, '/crane/effort_controller/commands', 10)
        self.at_target_pub = self.create_publisher(Bool, '/crane/at_target', 10)

    def carrying_callback(self, msg: Bool):
        self.carrying = msg.data

    def command_callback(self, msg: Float64MultiArray):
        if len(msg.data) not in (len(JOINTS), len(JOINTS) + 1):
            self.get_logger().error('Expected x, y, z and optionally a tolerance on '
                                    '/crane/position_commands')
            return
        self.desired_joint_positions = list(msg.data[:len(JOINTS)])
        # an optional 4th value makes the target use waypoint tolerance, this allows it to move on
        # from the target almost immediately without stopping if waypoint tolerance is large
        self.waypoint_tolerance = msg.data[3] if len(msg.data) > len(JOINTS) else 0.0

    def joint_callback(self, msg: JointState):

        try:
            joint_indices = [msg.name.index(joint) for joint in JOINTS]
        except ValueError:
            self.get_logger().error(f"Expected joint data from /crane/joint_states not recieved")
            return

        joint_positions = [float(msg.position[j]) for j in joint_indices]
        joint_velocities = [float(msg.velocity[j]) for j in joint_indices]

        # Until given a target hold the crane where it is
        if self.desired_joint_positions is None:
            self.desired_joint_positions = joint_positions

        # PD control
        efforts = []
        for i in range(len(JOINTS)):
            error = max(-self.max_error, min(self.max_error, self.desired_joint_positions[i] - joint_positions[i]))
            efforts.append(self.Kp[i] * error - self.Kd[i] * joint_velocities[i])

        # Gravity on the winch to hold its weight and thwe rods if necessary
        mass = WINCH_MASS + (ROD_MASS if self.carrying else 0.0)
        efforts[2] += mass * GRAVITY

        self.command_pub.publish(Float64MultiArray(data=efforts))

        errors = [abs(self.desired_joint_positions[i] - joint_positions[i])
                  for i in range(len(JOINTS))]
        if self.waypoint_tolerance > 0.0:
            # a waypoint
            at_target = all(error < self.waypoint_tolerance for error in errors)
        else:
            # an exact target
            at_target = all(errors[i] < self.position_tolerance
                            and abs(joint_velocities[i]) < self.velocity_tolerance
                            for i in range(len(JOINTS)))
        self.at_target_pub.publish(Bool(data=at_target))


def main():
    rclpy.init()
    node = CraneController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()