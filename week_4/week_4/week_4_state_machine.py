#!/usr/bin/env python3

import signal
from typing import Callable

from geometry_msgs.msg import Twist, TwistStamped, Vector3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Header
from week_4.robochart_fsm import Clock, Required, Scope, State, StateMachine

lvel = 0.3  # Linear velocity
avel = 1.5  # Angular velocity


class DrivingMachine(StateMachine):

    # RoboChart Clock
    clock_C: Clock

    # Operations required from platform
    cmd_vel: Required[Callable[[TwistStamped], None]]
    get_forward_time: Required[Callable[[], float]]
    get_turn_time: Required[Callable[[], float]]

    def initial(self):
        return self.Forward, self.clock_C.reset

    class Forward(State):

        def entry(self):
            self.cmd_vel(
                TwistStamped(
                    header=Header(),
                    twist=Twist(
                        linear=Vector3(x=lvel, y=0.0, z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=0.0)
                    )
                )
            )
            self.logger.info("Entering State: FORWARD")

        def transitions(self):
            # Transition condition [since(C) >= forward_time] / #C
            if self.clock_C.since() >= self.get_forward_time():
                return self.Turning, self.clock_C.reset

    class Turning(State):

        def entry(self):
            self.cmd_vel(
                TwistStamped(
                    header=Header(),
                    twist=Twist(
                        linear=Vector3(x=0.0, y=0.0, z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=avel)
                    )
                )
            )
            self.logger.info("Entering State: TURNING")

        def transitions(self):
            # Transition condition [since(C) >= turn_time] / #C
            if self.clock_C.since() >= self.get_turn_time():
                return self.Forward, self.clock_C.reset


# Robotic Platform Scope providing required operations
class TurtleBot3Platform(Scope):
    cmd_vel: Callable[[TwistStamped], None]
    get_forward_time: Callable[[], float]
    get_turn_time: Callable[[], float]


class TurtleBot3FSMNode(Node):

    def __init__(self):
        super().__init__('week_4_state_machine')

        # ROS 2 Parameters
        self.declare_parameter('forward_time', 4.0)
        self.declare_parameter('turn_time', 1.0)

        # Topic Publisher
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Timing config: 10 Hz loop frequency, 1 time_unit = 1.0s
        self.timer_period = 0.1
        self.time_unit = 1

        # Instantiate Platform and StateMachine
        self.platform = TurtleBot3Platform(
            cmd_vel=self.cmd_vel,
            get_forward_time=self.get_forward_time,
            get_turn_time=self.get_turn_time
        )
        self.stm = DrivingMachine(
            self.get_logger(),
            self.timer_period,
            self.time_unit,
            parent=self.platform
        )

        # Control loop timer
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.get_logger().info("TurtleBot3 FSM StateMachine Node Started.")

    def cmd_vel(self, msg: TwistStamped):
        self.publisher.publish(msg)

    def get_forward_time(self) -> float:
        return self.get_parameter('forward_time').get_parameter_value().double_value

    def get_turn_time(self) -> float:
        return self.get_parameter('turn_time').get_parameter_value().double_value

    def control_loop(self):
        # Drives the state machine steps
        self.stm.execute()


def main(args=None):
    # Disable default SIGINT handler so rclpy handles Ctrl+C cleanly
    signal.signal(signal.SIGINT, lambda signum, frame: None)

    rclpy.init(args=args)
    node = TurtleBot3FSMNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()