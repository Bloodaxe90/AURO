#!/usr/bin/python3

from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter


class PutAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('put', 0.5)
        self.progress_ = 0.0

    def do_work(self):
        # As long as the plan says it lasts
        rate = self.get_parameter('rate').value
        self.progress_ += 1.0 / max(1.0, self.get_planned_duration() * rate)
        if self.progress_ < 1.0 - 1e-3:
            self.send_feedback(self.progress_, 'Put running')
        else:
            self.finish(True, 1.0, 'Put completed')
            self.progress_ = 0.0

        self.get_logger().info('putting ... {}'.format(self.progress_))


def main(args=None):
    rclpy.init(args=args)

    node = PutAction()
    node.set_parameters([Parameter(name='action_name', value='put')])

    node.trigger_configure()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
