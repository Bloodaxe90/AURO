import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class Joint2(Node):

    def __init__(self):
        super().__init__('joint2')

        self.declare_parameter('Kp', 10.0)
        self.declare_parameter('Kd', 1.0)

        self.target_q2 = 0.0

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/joint_controller/commands',
            10
        )

        self.subscriber = self.create_subscription(
            Float64MultiArray,
            '/position_controller/command',
            self.target_callback,
            10
        )

        self.subscriber2 = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

    def target_callback(self, msg: Float64MultiArray):
        self.target_q2 = float(msg.data[1])
        self.get_logger().info(f"Subscriber: {self.target_q2} rad")

    def joint_state_callback(self, msg: JointState):
        kp = self.get_parameter('Kp').get_parameter_value().double_value
        kd = self.get_parameter('Kd').get_parameter_value().double_value
        current_q2 = msg.position[1]
        current_v2 = msg.velocity[1]

        error = self.target_q2 - current_q2
        torque2 = (kp * error) - (kd * current_v2)

        new_msg = Float64MultiArray()
        new_msg.data = [0.0, float(torque2)]
        self.publisher_.publish(new_msg)


def main(args=None):
    try:
        with rclpy.init(args=args):
            joint2 = Joint2()
            rclpy.spin(joint2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()