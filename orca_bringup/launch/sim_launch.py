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
Launch a simulation.

Includes Gazebo, ArduSub, RViz, mavros, all ROS nodes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')
    orca_description_dir = get_package_share_directory('orca_description')

    ardusub_params_file = os.path.join(orca_bringup_dir, 'cfg', 'sub.parm')
    mavros_params_file = os.path.join(orca_bringup_dir, 'params', 'sim_mavros_params.yaml')
    orca_params_file = os.path.join(orca_bringup_dir, 'params', 'sim_orca_params.yaml')
    rosbag2_record_qos_file = os.path.join(orca_bringup_dir, 'params', 'rosbag2_record_qos.yaml')
    rviz_file = os.path.join(orca_bringup_dir, 'cfg', 'sim_launch.rviz')
    world_file = os.path.join(orca_description_dir, 'worlds', 'sand.world')

    sim_left_ini = os.path.join(orca_bringup_dir, 'cfg', 'sim_left.ini')
    sim_right_ini = os.path.join(orca_bringup_dir, 'cfg', 'sim_right.ini')
    return LaunchDescription([
        DeclareLaunchArgument(
            'ardusub',
            default_value='True',
            description='Launch ArduSUB with SIM_JSON?'
        ),

        DeclareLaunchArgument(
            'bag',
            default_value='False',
            description='Bag interesting topics?',
        ),

        DeclareLaunchArgument(
            'base',
            default_value='True',
            description='Launch base controller?',
        ),

        DeclareLaunchArgument(
            'gzclient',
            default_value='True',
            description='Launch Gazebo UI?'
        ),

        DeclareLaunchArgument(
            'mavros',
            default_value='True',
            description='Launch mavros?',
        ),

        DeclareLaunchArgument(
            'nav',
            default_value='True',
            description='Launch navigation?',
        ),

        DeclareLaunchArgument(
            'qgc',
            default_value='False',
            description='Launch QGroundControl?',
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='True',
            description='Launch rviz?',
        ),

        DeclareLaunchArgument(
            'slam',
            default_value='True',
            description='Launch SLAM?',
        ),

        # Bag useful topics
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--qos-profile-overrides-path', rosbag2_record_qos_file,
                '--include-hidden-topics',
                '/cmd_vel',
                '/mavros/local_position/pose',
                '/mavros/rc/override',
                '/mavros/setpoint_position/global',
                '/mavros/state',
                '/mavros/vision_pose/pose',
                '/model/orca/odometry',
                '/motion',
                '/odom',
                '/orb_slam2_stereo_node/pose',
                '/orb_slam2_stereo_node/status',
                '/pid_z',
                '/rosout',
                '/tf',
                '/tf_static',
                '/world/sand/model/orca/link/base_link/sensor/imu_sensor/imu',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('bag')),
        ),

        # Launch QGroundControl
        ExecuteProcess(
            cmd=['QGroundControl.AppImage'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('qgc')),
        ),

        # Launch rviz
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),

        # Launch ArduSub w/ SIM_JSON (wipe eeprom w/ -w)
        # ardusub must be on the $PATH, see src/orca4/setup.bash
        ExecuteProcess(
            cmd=['ardusub', '-S', '-w', '-M', 'JSON', '--defaults', ardusub_params_file,
                 '-I0', '--home', '33.810313,-118.39386700000001,0.0,270.0'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('ardusub')),
        ),

        # Launch Ignition Gazebo
        # ign must be on the $PATH
        # libArduPilotPlugin.so must be on the IGN_GAZEBO_SYSTEM_PLUGIN_PATH
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '3', '-r', world_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gzclient')),
        ),

        # Launch Ignition Gazebo server-only
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '3', '-r', '-s', world_file],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('gzclient')),
        ),

        # Get images from Ignition Gazebo to ROS
        Node(
            package='ros_ign_image',
            executable='image_bridge',
            arguments=['stereo_left', 'stereo_right'],
            output='screen',
        ),

        # Ignition Gazebo doesn't publish camera info, so do that here
        Node(
            package='orca_base',
            executable='camera_info_publisher',
            name='left_info_publisher',
            output='screen',
            parameters=[{
                'camera_info_url': 'file://' + sim_left_ini,
                'camera_name': 'stereo_left',
                'frame_id': 'stereo_left_frame',
            }],
            remappings=[
                ('/camera_info', '/stereo_left/camera_info'),
            ],
        ),

        Node(
            package='orca_base',
            executable='camera_info_publisher',
            name='right_info_publisher',
            output='screen',
            parameters=[{
                'camera_info_url': 'file://' + sim_right_ini,
                'camera_name': 'stereo_right',
                'frame_id': 'stereo_right_frame',
            }],
            remappings=[
                ('/camera_info', '/stereo_right/camera_info'),
            ],
        ),

        # Publish ground truth poses from Ignition Gazebo
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/orca/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '/world/sand/model/orca/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            ],
            output='screen'
        ),

        # Bring up Orca and Nav2 nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(orca_bringup_dir, 'launch', 'bringup.py')),
            launch_arguments={
                'base': LaunchConfiguration('base'),
                'mavros': LaunchConfiguration('mavros'),
                'mavros_params_file': mavros_params_file,
                'nav': LaunchConfiguration('nav'),
                'orca_params_file': orca_params_file,
                'slam': LaunchConfiguration('slam'),
            }.items(),
        ),
    ])

