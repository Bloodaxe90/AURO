import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from argos3_ros2_bridge.msg import ProximityList


class ObstacleAvoidance(Node):
    def __init__(self, robot_name: str):
        super().__init__(f'obstacle_avoidance_{robot_name}')
        self.pub = self.create_publisher(Twist, f'{robot_name}/cmd_vel', 10)
        self.sub = self.create_subscription(
            ProximityList, f'{robot_name}/proximityList', self.callback, 10
        )

    def callback(self, msg):
        cmd = Twist()
        front_sensors = list(msg.proximities[:4]) + list(msg.proximities[-4:])
        front_values = [p.value for p in front_sensors]

        if max(front_values) > 0.2:
            cmd.angular.z = 0.8
        else:
            cmd.linear.x = 0.4

        self.pub.publish(cmd)


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()

    nodes = [ObstacleAvoidance(f'fb{i}') for i in range(10)]
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()