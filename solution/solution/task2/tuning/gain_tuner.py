"""
Step test for tuning crane_controller gains

Run with the simulation and crane_controller running and the winch brake released:

python3 gain_tuner.py --ros-args -p use_sim_time:=true -p joint:=bridge_slide -p target:=5.0
    -p output_file:=example.csv
"""

from typing import Optional, TextIO

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = ['bridge_slide', 'winch_block_slide', 'winch_lift']  # x y z
TOLERANCE = 0.005  # position tolerance for settling time (m) (same as crane_controller)


class GainTuner(Node):

    def __init__(self) -> None:
        super().__init__('gain_tuner')

        self.joint: str = self.declare_parameter('joint', 'bridge_slide').value
        self.target: float = float(self.declare_parameter('target', 5.0).value)
        self.duration: float = float(self.declare_parameter('duration', 30.0).value)
        output_file: str = self.declare_parameter('output_file', 'step.csv').value

        self.start_time: Optional[float] = None
        self.start_position: Optional[float] = None
        self.samples: list[tuple[float, float, float]] = []  # (time, position, velocity)
        self.done = False

        self.create_subscription(JointState, '/crane/joint_states', self.joint_callback, 10)
        self.target_pub = self.create_publisher(Float64MultiArray, '/crane/position_commands', 10)

        self.log_file: TextIO = open(output_file, 'w')
        self.log_file.write('time,position,velocity\n')

    def joint_callback(self, msg: JointState) -> None:
        positions = [float(msg.position[msg.name.index(joint)]) for joint in JOINTS]
        velocities = [float(msg.velocity[msg.name.index(joint)]) for joint in JOINTS]
        now = self.get_clock().now().nanoseconds * 1e-9
        i = JOINTS.index(self.joint)

        if self.start_time is None:
            # wait until crane_controller is listening
            if self.target_pub.get_subscription_count() == 0:
                return
            # step this joint only
            command = list(positions)
            command[i] = self.target
            self.target_pub.publish(Float64MultiArray(data=command))
            self.start_time = now
            self.start_position = positions[i]

        time = now - self.start_time
        self.samples.append((time, positions[i], velocities[i]))
        self.log_file.write(f'{time},{positions[i]},{velocities[i]}\n')
        self.done = time >= self.duration

    def report(self) -> None:
        step = self.target - self.start_position
        errors = [self.target - position for _, position, _ in self.samples]

        # overshoot
        direction = 1.0 if step > 0 else -1.0
        overshoot = max(0.0, max(-error * direction for error in errors))

        # settling time
        settling = None
        if abs(errors[-1]) <= TOLERANCE:
            settling = 0.0
            for (time, _, _), error in zip(self.samples, errors):
                if abs(error) > TOLERANCE:
                    settling = time

        peak_speed = max(abs(velocity) for _, _, velocity in self.samples)

        self.get_logger().info(
            f'{self.joint} {self.start_position:.3f} -> {self.target:.3f}: '
            f'overshoot {overshoot * 1000:.1f} mm, '
            f'settling time {"not settled" if settling is None else f"{settling:.1f} s"} '
            f'(within {TOLERANCE * 1000:.0f} mm), peak speed {peak_speed:.2f} m/s')

    def destroy_node(self) -> None:
        self.log_file.close()
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GainTuner()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.report()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
