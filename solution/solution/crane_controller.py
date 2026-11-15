import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Only the joint for the crane
JOINTS = ['bridge_slide', 'winch_block_slide', 'winch_lift']
GRAVITY = 9.81
WINCH_MASS = 100.0
ROD_MASS = 100.0


class CraneController(Node):

    def __init__(self):
        super().__init__('crane_controller')

        # PD coefficients for x, y, z
        self.Kp = [float(self.declare_parameter('Kp_x', 300.0).value),
                   float(self.declare_parameter('Kp_y', 300.0).value),
                   float(self.declare_parameter('Kp_z', 300.0).value)]
        self.Kd = [float(self.declare_parameter('Kd_x', 300.0).value),
                   float(self.declare_parameter('Kd_y', 300.0).value),
                   float(self.declare_parameter('Kd_z', 300.0).value)]

        self.max_error = float(self.declare_parameter('max_error', 1.5).value) # ensuring the crane doesn't move too fast

        self.declare_parameter('carrying', False)

        self.desired_joint_positions = None

        self.joint_sub = self.create_subscription(
            JointState, '/crane/joint_states', self.joint_callback, 10)
        self.command_sub = self.create_subscription(
            Float64MultiArray, '/crane/position_commands', self.command_callback, 10)

        self.command_pub = self.create_publisher(
            Float64MultiArray, '/crane/effort_controller/commands', 10)

    def command_callback(self, msg: Float64MultiArray):
        if len(msg.data) != len(JOINTS):
            self.get_logger().error("Expected x, y, z for joints bridge_slide, winch_block_slide, winch_lift from /crane/position_commands")
            return
        self.desired_joint_positions = list(msg.data)

    def joint_callback(self, msg: JointState):

        for joint in JOINTS:
            if joint not in msg.name:
                self.get_logger().error(f"Expected joint {joint} data from /crane/joint_states not recieved")
                return

        joint_positions = [float(msg.position[i]) for i in range(len(JOINTS))]
        joint_velocities = [float(msg.velocity[i]) if len(msg.velocity) > i else 0.0 for i in range(len(JOINTS))]

        # Until given a target hold the crane where it is
        if self.desired_joint_positions is None:
            self.desired_joint_positions = joint_positions

        # PD control
        efforts = []
        for i in range(len(JOINTS)):
            error = max(-self.max_error, min(self.max_error, self.desired_joint_positions[i] - joint_positions[i]))
            efforts.append(self.Kp[i] * error - self.Kd[i] * joint_velocities[i])

        # Gravity on the winch to hold its weight and thwe rods if necessary
        mass = WINCH_MASS + (ROD_MASS if self.get_parameter('carrying').value else 0.0)
        efforts[2] += mass * GRAVITY

        self.command_pub.publish(Float64MultiArray(data=efforts))


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