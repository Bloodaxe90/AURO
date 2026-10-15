import rclpy
import math
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Vector3, PoseStamped
from sensor_msgs.msg import JointState
from transforms3d.euler import euler2quat

class KinematicsCompute(Node):

    def __init__(self):
        super().__init__('kinematics_compute')

        self.declare_parameter('L1', 0.4)
        self.declare_parameter('L2', 0.3)

        self.publisher1_ = self.create_publisher(
            Vector3,
            '/ee_pose2d',
            10
        )

        self.publisher2_ = self.create_publisher(
            PoseStamped,
            '/stamped',
            10
        )

        self.subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )

        self.yaw = 0


    def listener_callback(self, msg: JointState):
        # self.get_logger().info(f"Subscribed name: {msg.name}\nposition:{msg.position}\nvelocity: {msg.velocity}\neffort: {msg.effort}")

        new_msg1: Vector3 = self.calc1(msg)
        self.publisher1_.publish(new_msg1)
        # self.get_logger().info(f"Published1 x:{new_msg1.x} y: {new_msg1.y} z: {new_msg1.z}")

        new_msg2: PoseStamped = self.calc2(new_msg1)
        self.publisher2_.publish(new_msg2)
        p = new_msg2.pose.position
        q = new_msg2.pose.orientation
        self.get_logger().info(
            f"Published2:\n"
            f"Header: {new_msg2.header}\n"
            f"Pos x: {p.x:} y: {p.y} z={p.z}\n"
            f"Ori x:{q.x} y:{q.y} z:{q.z} w:{q.w}"
        )

    def calc1(self, msg: JointState) -> PoseStamped:

        l1 = self.get_parameter('L1').get_parameter_value().double_value
        l2 = self.get_parameter('L2').get_parameter_value().double_value

        q1 = msg.position[0]
        q2 = msg.position[1]
        self.yaw = q1 + q2
        x = l1 * math.cos(q1) + l2 * math.cos(q1 + q2)
        y = l1 * math.sin(q1) + l2 * math.sin(q1 + q2)

        new_msg = Vector3()
        new_msg.x = float(x)
        new_msg.y = float(y)
        new_msg.z = float(0.0)

        return new_msg

    def calc2(self, msg: Vector3) -> PoseStamped:
            q = euler2quat(0.0, 0.0, self.yaw)

            new_msg = PoseStamped()

            new_msg.header.stamp = self.get_clock().now().to_msg()
            new_msg.header.frame_id = 'base_link'
            new_msg.pose.position.x = msg.x
            new_msg.pose.position.y = msg.y
            new_msg.pose.position.z = msg.z

            new_msg.pose.orientation.w = float(q[0])
            new_msg.pose.orientation.x = float(q[1])
            new_msg.pose.orientation.y = float(q[2])
            new_msg.pose.orientation.z = float(q[3])

            return new_msg


def main(args=None):
    try:
        with rclpy.init(args=args):
            kinematics_compute = KinematicsCompute()
            rclpy.spin(kinematics_compute)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()