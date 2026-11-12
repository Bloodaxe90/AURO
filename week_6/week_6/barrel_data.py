import csv
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

BARREL_POSITIONS = {
    'red': (1.50, -0.60),
}
BARREL_RADIUS = 0.1


class BarrelDataCollector(Node):

    def __init__(self):
        super().__init__('barrel_data_collector')

        self.robot_x = None
        self.robot_y = None

        self.csv_path = 'barrel_measurements.csv'
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp_sec',
            'colour',
            'robot_x',
            'robot_y',
            'bbox_center_x_px',
            'bbox_center_y_px',
            'bbox_width_px',
            'bbox_height_px',
            'surface_distance_m',
        ])

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(
            Detection2DArray, '/barrel_detections', self.detection_callback, 10
        )
        self.get_logger().info(f'Recording barrel data to {self.csv_path}')

    def odom_callback(self, msg: Odometry):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def detection_callback(self, msg: Detection2DArray):
        if self.robot_x is None or not msg.detections:
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        for detection in msg.detections:
            if not detection.results:
                continue

            colour = detection.results[0].hypothesis.class_id
            if colour not in BARREL_POSITIONS:
                continue

            barrel_x, barrel_y = BARREL_POSITIONS[colour]
            center_distance = math.hypot(
                barrel_x - self.robot_x, barrel_y - self.robot_y
            )
            surface_distance = center_distance - BARREL_RADIUS

            self.csv_writer.writerow([
                f'{stamp:.3f}',
                colour,
                f'{self.robot_x:.4f}',
                f'{self.robot_y:.4f}',
                f'{detection.bbox.center.position.x:.2f}',
                f'{detection.bbox.center.position.y:.2f}',
                f'{detection.bbox.size_x:.1f}',
                f'{detection.bbox.size_y:.1f}',
                f'{surface_distance:.4f}',
            ])

        self.csv_file.flush()

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BarrelDataCollector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()