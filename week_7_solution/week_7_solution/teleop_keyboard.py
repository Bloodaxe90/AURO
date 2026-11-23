#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Darby Lim
#
# Changed by Pedro Ribeiro to incorporate picking and offloading services.
# Changed for week 7 to publish geometry_msgs/TwistStamped, as the Gazebo bridge expects, and to
# take its robot_id from its namespace.

"""
Drive one robot with the keyboard, and pick up and offload barrels with it.

Run it in the robot's namespace, which is also its robot_id for barrel_manager:

  ros2 run week_7 teleop_keyboard --ros-args -r __ns:=/robot1

It publishes on cmd_vel (so /robot1/cmd_vel), and calls /pick_up_item and /offload_item. The
parameter robot_id overrides the robot_id taken from the namespace.
"""

import os
import select
import sys

from auro_interfaces.srv import ItemRequest
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Waffle Pi : ~ 1.82)

p/o : pick/offload item in the vicinity

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))


def make_simple_profile(output, target, slop):
    if target > output:
        output = min(target, output + slop)
    elif target < output:
        output = max(target, output - slop)
    else:
        output = target

    return output


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    return constrain(velocity, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)


def call_item_service(node, client, robot_id, done, failed):
    """Call /pick_up_item or /offload_item for robot_id, and print the outcome."""
    rqt = ItemRequest.Request()
    rqt.robot_id = robot_id
    try:
        future = client.call_async(rqt)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        if response.success:
            print(done)
        else:
            print(failed + response.message)
    except Exception as e:
        print(e)


def twist_stamped(node, linear, angular):
    twist = TwistStamped()
    twist.header.stamp = node.get_clock().now().to_msg()
    twist.twist.linear.x = linear
    twist.twist.angular.z = angular
    return twist


def main(args=sys.argv):
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(TwistStamped, 'cmd_vel', qos)
    robot_id = node.declare_parameter('robot_id', '').value or node.get_namespace().strip('/')
    if not robot_id:
        print("No robot_id: run in the robot's namespace, e.g. --ros-args -r __ns:=/robot1")
        return
    print(f'robot_id: {robot_id}, publishing on {pub.topic_name}')

    pick_up_service = node.create_client(ItemRequest, '/pick_up_item')
    offload_service = node.create_client(ItemRequest, '/offload_item')

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        print(msg)
        while True:
            key = get_key(settings)
            if key == 'w':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'x':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'a':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'd':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'p':
                call_item_service(node, pick_up_service, robot_id,
                                  'Item picked up.', 'Unable to pick up item: ')
            elif key == 'o':
                call_item_service(node, offload_service, robot_id,
                                  'Item offloaded.', 'Unable to offload item: ')
            elif key == ' ' or key == 's':
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            pub.publish(twist_stamped(node, control_linear_velocity, control_angular_velocity))

    except Exception as e:
        print(e)

    finally:
        pub.publish(twist_stamped(node, 0.0, 0.0))

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
