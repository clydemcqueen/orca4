#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 Clyde McQueen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Generate /cmd_vel messages to move the sub around a bit.

Usage:
-- ros2 run orca_base test_base.py
"""

from geometry_msgs.msg import Twist, Vector3
import rclpy
import rclpy.logging
from rclpy.node import Node


class TestBase(Node):

    def __init__(self):
        super().__init__('test_base')

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._timer = self.create_timer(8.0, self.timer_callback)

        self._trajectory = [
            Twist(linear=Vector3(z=-0.1)),
            Twist(angular=Vector3(z=0.1)),
            Twist(linear=Vector3(x=0.1)),
            Twist(linear=Vector3(y=0.1)),
            Twist(linear=Vector3(x=-0.1)),
            Twist(linear=Vector3(y=-0.1)),
            Twist(linear=Vector3(z=0.1)),
            Twist(),
        ]
        self._segment = 0

    def timer_callback(self):
        print(self._trajectory[self._segment])
        self._cmd_vel_pub.publish(self._trajectory[self._segment])

        self._segment += 1
        if self._segment >= len(self._trajectory):
            print('reset')
            self._segment = 0


def main():
    rclpy.init()

    node = TestBase()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
