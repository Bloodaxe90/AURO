import rclpy
import math
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Vector3, PoseStamped
from sensor_msgs.msg import JointState

class KinematicsCompute(Node):

    def __init__(self):
        super().__init__('kinematics_compute')

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


    def listener_callback(self, msg: JointState):
        # self.get_logger().info(f"Subscribed name: {msg.name}\nposition:{msg.position}\nvelocity: {msg.velocity}\neffort: {msg.effort}")

        new_msg1: Vector3 = self.calc(msg)
        self.publisher1_.publish(new_msg1)
        # self.get_logger().info(f"Published1 x:{new_msg1.x} y: {new_msg1.y} z: {new_msg1.z}")

        new_msg2: PoseStamped = msg.stamp
        self.publisher2_.publish(new_msg2)
        self.get_logger().info(f"Published2 x:{new_msg2}")


    def calc(self, msg: JointState) -> Vector3:

        l1 = 0.4
        l2 = 0.3

        q1 = msg.position[0]
        q2 = msg.position[1]

        x = l1 * math.cos(q1) + l2 * math.cos(q1 + q2)
        y = l1 * math.sin(q1) + l2 * math.sin(q1 + q2)
        yaw = q1 + q2

        new_msg = Vector3()
        new_msg.x = float(x)
        new_msg.y = float(y)
        new_msg.z = float(yaw)

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