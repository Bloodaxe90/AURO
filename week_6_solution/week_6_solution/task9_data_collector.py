"""
Collect data relating the apparent size of a barrel to its distance from the robot.

Subscribes to /odom and /barrel_detections (as published by week_6's visual_sensor), and writes
the robot's position, the area of the barrel's bounding box (in pixels), and the distance
from the robot to the barrel's surface, to the CSV file given by the 'output_file' parameter
(task9_data_collection.csv by default).
"""

import math
from typing import Optional, TextIO

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


class DataCollector(Node):
    """Log (robot pose, barrel size, distance) samples for a single barrel."""

    def __init__(self) -> None:
        super().__init__('data_collector')

        # We need the position of the single barrel we'll be tracking as an input to this node.
        self.declare_parameter('barrel_x', 0.0)
        self.declare_parameter('barrel_y', 0.0)

        self.barrel_x: float = self.get_parameter('barrel_x').value
        self.barrel_y: float = self.get_parameter('barrel_y').value

        # The CSV file to write the data to.
        self.declare_parameter('output_file', 'task9_data_collection.csv')
        output_file: str = self.get_parameter('output_file').value

        # Radius of barrels
        self.barrel_radius = 0.2

        # There are several ways to capture the pose of the robot in the world,
        # either using the TF from world to odom, or assuming that odom is ground-truth.
        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        # Subscriber for the 'barrel_detections' topic published by the visual_sensor.
        self.detections_subscriber = self.create_subscription(
            Detection2DArray, 'barrel_detections', self.detections_callback, 10)

        self.robot_x: Optional[float] = None
        self.robot_y: Optional[float] = None
        self.distance: Optional[float] = None

        self.log_file: TextIO = open(output_file, 'w')
        self.log_file.write('robot_x,robot_y,size,distance\n')
        self.log_file.flush()

    def odom_callback(self, msg: Odometry) -> None:
        # We do not need yaw here, just (x, y)
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Calculate distance to obstacle and update it
        pure_distance = math.hypot(self.robot_x - self.barrel_x, self.robot_y - self.barrel_y)

        if pure_distance >= self.barrel_radius:
            self.distance = pure_distance - self.barrel_radius
        else:  # Somehow the robot is too close!
            self.distance = None

    def detections_callback(self, msg: Detection2DArray) -> None:
        # visual_sensor publishes for every frame, so an empty list means no barrel is in view.
        if len(msg.detections) > 0:
            # We take the first detection -- this experiment is to be carried out with just one
            # barrel visible.
            bbox = msg.detections[0].bbox
            size = bbox.size_x * bbox.size_y

            # We assume here that the position has been adjusted sufficiently close in time
            # to the publishing of the detection.
            self.write_out_data(size)

    def write_out_data(self, size: float) -> None:
        if not (self.robot_x is None or self.robot_y is None or self.distance is None):
            self.log_file.write(f'{self.robot_x},{self.robot_y},{size},{self.distance}\n')

    def destroy_node(self) -> None:
        self.log_file.close()
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DataCollector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
