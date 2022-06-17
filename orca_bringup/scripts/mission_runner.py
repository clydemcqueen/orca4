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
Run the "huge loop" mission

Usage:
-- ros2 run orca_bringup mission_runner.py
"""

from threading import Thread

import rclpy
import rclpy.logging
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav2_msgs.action import FollowWaypoints
from orca_msgs.action import TargetMode
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Header


def make_pose(x: float, y: float, z: float):
    return PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=x, y=y, z=z)))


go_home = FollowWaypoints.Goal()
go_home.poses.append(make_pose(x=0.0, y=0.0, z=-1.0))

delay_loop = FollowWaypoints.Goal()
delay_loop.poses.append(make_pose(x=0.0, y=0.0, z=-7.0))
for _ in range(2):
    delay_loop.poses.append(make_pose(x=20.0, y=-13.0, z=-7.0))
    delay_loop.poses.append(make_pose(x=10.0, y=-23.0, z=-7.0))
    delay_loop.poses.append(make_pose(x=-10.0, y=-8.0, z=-7.0))
    delay_loop.poses.append(make_pose(x=0.0, y=0.0, z=-7.0))


# TODO(clyde): cancel goal when killed
class MissionRunner(Node):

    def __init__(self, mission: FollowWaypoints.Goal):
        super().__init__('mission_runner')

        self._mission = mission

        self._set_target_mode = ActionClient(self, TargetMode, '/set_target_mode')
        self._set_target_mode_goal_future = None
        self._set_target_mode_result_future = None

        self._follow_waypoints = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self._follow_waypoints_goal_future = None
        self._follow_waypoints_result_future = None

        self._thread = Thread(target=self.run)
        self._thread.start()

    def run(self):
        self.set_target_mode(TargetMode.Goal.ORCA_MODE_AUV)
        self.follow_waypoints()
        self.set_target_mode(TargetMode.Goal.ORCA_MODE_ROV)

    def set_target_mode(self, mode):
        goal_msg = TargetMode.Goal()
        goal_msg.target_mode = mode

        self._set_target_mode.wait_for_server()
        result = self._set_target_mode.send_goal(goal_msg)
        print(result)

    def follow_waypoints(self):
        self._follow_waypoints.wait_for_server()
        result = self._follow_waypoints.send_goal(self._mission)
        print(result)


def main():
    rclpy.init()

    # TODO(clyde): mission arg
    node = MissionRunner(delay_loop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
