import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion


class Positioner(Node):

    def __init__(self):
        super().__init__('positioner')

        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10
        )

        # Previous position
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.previous_z = 0.0

        # Previous orientation
        self.previous_roll = 0.0
        self.previous_pitch = 0.0
        self.previous_yaw = 0.0

        # Previous time
        self.previous_time = None

    def listener_callback(self, msg: Odometry):

        new_msg = self.calc(msg)

        self.publisher_.publish(new_msg)

        self.get_logger().info(
            f'Linear: x={new_msg.linear.x:.3f}, '
            f'y={new_msg.linear.y:.3f}, '
            f'z={new_msg.linear.z:.3f} | '
            f'Angular: x={new_msg.angular.x:.3f}, '
            f'y={new_msg.angular.y:.3f}, '
            f'z={new_msg.angular.z:.3f}'
        )

    def calc(self, msg: Odometry) -> Twist:

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Current time
        current_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        new_msg = Twist()

        # Convert quaternion to Euler angles
        quaternion = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ]

        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # First message: no previous measurement
        if self.previous_time is None:
            self.previous_x = position.x
            self.previous_y = position.y
            self.previous_z = position.z

            self.previous_roll = roll
            self.previous_pitch = pitch
            self.previous_yaw = yaw

            self.previous_time = current_time

            return new_msg

        # Time difference
        dt = current_time - self.previous_time

        if dt > 0:

            # Linear velocity
            new_msg.linear.x = (
                position.x - self.previous_x
            ) / dt

            new_msg.linear.y = (
                position.y - self.previous_y
            ) / dt

            new_msg.linear.z = (
                position.z - self.previous_z
            ) / dt

            # Angular velocity
            new_msg.angular.x = (
                roll - self.previous_roll
            ) / dt

            new_msg.angular.y = (
                pitch - self.previous_pitch
            ) / dt

            new_msg.angular.z = (
                yaw - self.previous_yaw
            ) / dt

        # Save current values
        self.previous_x = position.x
        self.previous_y = position.y
        self.previous_z = position.z

        self.previous_roll = roll
        self.previous_pitch = pitch
        self.previous_yaw = yaw

        self.previous_time = current_time

        return new_msg


def main(args=None):
    try:
        with rclpy.init(args=args):
            positioner = Positioner()
            rclpy.spin(positioner)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
