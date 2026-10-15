import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt


class PosePlotter(Node):

    def __init__(self):
        super().__init__('pose_plotter')

        self.sub = self.create_subscription(
            Vector3,
            '/ee_pose2d',
            self.pose_callback,
            10
        )

        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.timestamps = []
        self.xs = []
        self.ys = []
        self.yaws = []

        self.get_logger().info("Recording /ee_pose2d data. Press Ctrl+C to stop and save/show plots.")

    def pose_callback(self, msg: Vector3):
        current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        
        self.timestamps.append(current_time)
        self.xs.append(msg.x)
        self.ys.append(msg.y)
        self.yaws.append(msg.z)

    def plot_data(self):
        if not self.timestamps:
            print("\n[WARNING] No data collected on /ee_pose2d! Make sure your FK node is running and publishing.")
            return

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))

        ax1.plot(self.timestamps, self.xs, label='$x$ (m)', color='red')
        ax1.plot(self.timestamps, self.ys, label='$y$ (m)', color='green')
        ax1.plot(self.timestamps, self.yaws, label=r'$\phi$ (rad)', color='blue')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Value')
        ax1.set_title('End-Effector Pose $(x, y, \\phi)$ over Time')
        ax1.grid(True)
        ax1.legend()

        ax2.plot(self.xs, self.ys, label='Path', color='purple')
        ax2.plot(self.xs[0], self.ys[0], 'go', label='Start')
        ax2.plot(self.xs[-1], self.ys[-1], 'ro', label='End')
        ax2.set_xlabel('$x$ (m)')
        ax2.set_ylabel('$y$ (m)')
        ax2.set_title('2D Spatial Trajectory $(x, y)$')
        ax2.grid(True)
        ax2.axis('equal')
        ax2.legend()

        plt.tight_layout()

        file_name = 'ee_pose_plot.png'
        plt.savefig(file_name)
        print(f"\n[SUCCESS] Plot saved to '{file_name}' in your workspace directory.")

        try:
            plt.show()
        except Exception as e:
            print(f"[NOTE] Could not open GUI display window ({e}). Image saved to file.")


def main(args=None):
    rclpy.init(args=args)
    node = PosePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nGenerating plot...")
        node.plot_data()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()