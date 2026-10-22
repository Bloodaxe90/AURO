from enum import Enum
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class State(Enum):
    FORWARD = 0
    TURNING = 1


class TurtleBot3FSM(Node):

    def __init__(self):
        super().__init__('turtlebot3_fsm')

        self.declare_parameter('turn_time', 1.0)
        self.declare_parameter('forward_time', 4.0)

        self.state = State.FORWARD
        self.clock_C = 0

        self.publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info("TurtleBot3 FSM Node Started.")

    def control_loop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        turn_time = self.get_parameter('turn_time').get_parameter_value().double_value
        forward_time = self.get_parameter('forward_time').get_parameter_value().double_value

        match self.state:
            case State.FORWARD:
                msg.twist.linear.x = 0.3
                msg.twist.angular.z = 0.0

                # 4 seconds = 40 ticks at 10 Hz
                if self.clock_C >= int(forward_time / self.timer_period):
                    self.state = State.TURNING
                    self.clock_C = 0  # Reset counter!
                    self.get_logger().info("Switching to TURNING state")

            case State.TURNING:
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = 1.5

                # 1 second = 10 ticks at 10 Hz
                if self.clock_C >= int(turn_time / self.timer_period):
                    self.state = State.FORWARD
                    self.clock_C = 0  # Reset counter!
                    self.get_logger().info("Switching to FORWARD state")

            case _:
                pass

        self.publisher.publish(msg)
        self.clock_C += 1


def main(args=None):
    try:
        with rclpy.init(args=args):
            turtlebot3_fsm = TurtleBot3FSM()
            rclpy.spin(turtlebot3_fsm)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()