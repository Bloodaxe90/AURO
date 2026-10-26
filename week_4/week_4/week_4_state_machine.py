#!/usr/bin/env python3

import signal
from typing import Callable

from geometry_msgs.msg import Twist, TwistStamped, Vector3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Header
from week_4.robochart_fsm import (FINAL, Operation, Param, Required,
                                  Scope, State, StateMachine)

lvel = 0.3  # Linear velocity (m/s)
avel = 1.5  # Angular velocity (rad/s)


# Operation move(lvel: real, avel: real), defined by a state machine pattern
class Move(Operation):

    lvel: Param[float]
    avel: Param[float]

    # Required operation provided by platform
    cmd_vel: Required[Callable[[TwistStamped], None]]

    # Initial transition executes velocity command and finishes (FINAL)
    def initial(self):
        self.cmd_vel(
            TwistStamped(
                header=Header(),
                twist=Twist(
                    linear=Vector3(x=float(self.lvel), y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=float(self.avel))
                )
            )
        )
        return FINAL


class DrivingMachine(StateMachine):

    # Pass-through required operations for Move and timing
    cmd_vel: Required[Callable[[TwistStamped], None]]
    move: Required[Operation]
    get_forward_time: Required[Callable[[], int]]
    get_turn_time: Required[Callable[[], int]]

    def initial(self):
        return self.Forward

    class Forward(State):

        def entry(self):
            yield from self.move(lvel, 0.0)
            self.logger.info("Entering State: FORWARD")
            yield from self.wait(self.get_forward_time())

        def transitions(self):
            return self.Turning

    class Turning(State):

        def entry(self):
            yield from self.move(0.0, avel)
            self.logger.info("Entering State: TURNING")
            yield from self.wait(self.get_turn_time())

        def transitions(self):
            return self.Forward


# Robotic Platform Scope providing hardware operations
class TurtleBot3Platform(Scope):
    cmd_vel: Callable[[TwistStamped], None]
    get_forward_time: Callable[[], int]
    get_turn_time: Callable[[], int]


# Controller Scope binding the Move operation
class TurtleBot3Controller(Scope):
    move: type[Operation] = Move


class TurtleBot3FSMNode(Node):

    def __init__(self):
        super().__init__('week_4_state_machine')

        # ROS 2 Parameters
        self.declare_parameter('forward_time', 4)
        self.declare_parameter('turn_time', 1)

        # ROS 2 Publisher
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Timing config: 10 Hz loop frequency, 1 time_unit = 1.0s
        self.timer_period = 0.1
        self.time_unit = 1

        # Instantiate Platform, Controller Scope, and StateMachine
        self.platform = TurtleBot3Platform(
            cmd_vel=self.cmd_vel,
            get_forward_time=self.get_forward_time,
            get_turn_time=self.get_turn_time
        )
        self.controller = TurtleBot3Controller(parent=self.platform)

        self.stm = DrivingMachine(
            self.get_logger(),
            self.timer_period,
            self.time_unit,
            parent=self.controller
        )

        # Control loop timer
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.get_logger().info("TurtleBot3 FSM StateMachine Node Started.")

    def cmd_vel(self, msg: TwistStamped):
        self.get_logger().info(
            f'Publishing cmd_vel: linear.x={msg.twist.linear.x}, angular.z={msg.twist.angular.z}'
        )
        self.publisher.publish(msg)

    def get_forward_time(self) -> int:
        return self.get_parameter('forward_time').get_parameter_value().integer_value

    def get_turn_time(self) -> int:
        return self.get_parameter('turn_time').get_parameter_value().integer_value

    def control_loop(self):
        # Drives the state machine steps
        self.stm.execute()


def main(args=None):
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