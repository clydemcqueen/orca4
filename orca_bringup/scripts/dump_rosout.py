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
Read a ROS2 bag file and dump the /rosout topic.
"""

import sqlite3
import sys
import time
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from typing import List


class BagParser:
    def __init__(self, bag_file=''):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        topics_data = self.cursor.execute('SELECT id, name, type FROM topics').fetchall()
        self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {name_of: get_message(type_of) for id_of, name_of, type_of in
                                  topics_data}

    def __del__(self):
        self.conn.close()

    def get_messages(self, topic_name):
        topic_id = self.topic_id[topic_name]
        rows = self.cursor.execute(
            'SELECT timestamp, data FROM messages WHERE topic_id = {}'.format(topic_id)).fetchall()
        return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for
                timestamp, data in rows]


def print_usage():
    print('Print all messages:')
    print('    ros2 run orca_bringup dump_rosout.py /path/bag.db3')

    print('Print messages from only1, only2, ...:')
    print('    ros2 run orca_bringup dump_rosout.py /path/bag.db3 [only only1 [only2 ...]]')

    print('Ignore messages from ignore1, ignore2, ...:')
    print('    ros2 run orca_bringup dump_rosout.py /path/bag.db3 [ignore ignore1 [ignore2 ...]]')


def main(args: List):
    if len(args) < 2 or len(args) == 3:
        print_usage()
        return

    only = []
    ignore = []

    if len(args) > 2:
        modifier = args[2]
        if modifier != 'only' and modifier != 'ignore':
            print_usage()
            return

        for arg in args[3:]:
            if modifier == 'only':
                print('Only', arg)
                only.append(arg)
            else:
                print('Ignore', arg)
                ignore.append(arg)

    print('Open', args[1])
    print('----------------------------------------------------')
    parser = BagParser(args[1])
    rosout = parser.get_messages('/rosout')
    for msg in rosout:
        log: Log = msg[1]
        if (len(only) and log.name in only) or (not len(only) and log.name not in ignore):
            stamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(msg[0] / 1e9))
            print(stamp, ':', log.name, ':', log.msg)


if __name__ == '__main__':
    main(sys.argv)
