import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

from week_4.robochart_fsm import StateMachine


class TurtleBot3FSM(StateMachine, Node):

    def __init__(self):
        Node.__init__(self, 'turtlebot3_fsm_wait')
        StateMachine.__init__(self)

        # Declare time parameters (Task 5 requirement preserved)
        self.declare_parameter('forward_time', 4.0)
        self.declare_parameter('turn_time', 1.0)

        self.publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # Timer calls the StateMachine step function to advance generator execution
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.step)

        self.get_logger().info("TurtleBot3 FSM (Wait-based) Started.")

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.publisher.publish(msg)

    def execute(self):
        """State machine control flow using yield from self.wait()."""
        while rclpy.ok():
            # Get latest runtime parameter values
            forward_time = self.get_parameter('forward_time').get_parameter_value().double_value
            turn_time = self.get_parameter('turn_time').get_parameter_value().double_value

            # --- State: Forward ---
            self.get_logger().info("Entering State: FORWARD")
            self.publish_cmd_vel(linear_x=0.3, angular_z=0.0)
            
            # Entry action wait statement
            yield from self.wait(forward_time)

            # --- State: Turning ---
            self.get_logger().info("Entering State: TURNING")
            self.publish_cmd_vel(linear_x=0.0, angular_z=1.5)
            
            # Entry action wait statement
            yield from self.wait(turn_time)


def main(args=None):
    try:
        with rclpy.init(args=args):
            fsm_node = TurtleBot3FSM()
            rclpy.spin(fsm_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()