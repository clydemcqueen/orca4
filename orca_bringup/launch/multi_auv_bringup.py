#!/usr/bin/env python3

"""
Bring up all nodes for multi-AUV simulation

Coordinates MAVROS, base controllers, and SLAM for all three AUVs.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')

    # Parameters for each AUV
    auv1_mavros_params_file = LaunchConfiguration('auv1_mavros_params_file')
    auv2_mavros_params_file = LaunchConfiguration('auv2_mavros_params_file')
    auv3_mavros_params_file = LaunchConfiguration('auv3_mavros_params_file')
    
    auv1_orca_params_file = LaunchConfiguration('auv1_orca_params_file')
    auv2_orca_params_file = LaunchConfiguration('auv2_orca_params_file')
    auv3_orca_params_file = LaunchConfiguration('auv3_orca_params_file')

    nav2_bt_file = os.path.join(orca_bringup_dir, 'behavior_trees', 'orca4_bt.xml')
    nav2_params_file = os.path.join(orca_bringup_dir, 'params', 'nav2_params.yaml')

    # get_package_share_directory('orb_slam2_ros') will fail if orb_slam2_ros isn't installed
    orb_voc_file = os.path.join('install', 'orb_slam2_ros', 'share', 'orb_slam2_ros',
                                'orb_slam2', 'Vocabulary', 'ORBvoc.txt')

    # Rewrite to add the full path for each AUV
    auv1_configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'default_nav_to_pose_bt_xml': nav2_bt_file,
        },
        convert_types=True)

    auv2_configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'default_nav_to_pose_bt_xml': nav2_bt_file,
        },
        convert_types=True)

    auv3_configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'default_nav_to_pose_bt_xml': nav2_bt_file,
        },
        convert_types=True)

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'base',
            default_value='True',
            description='Launch base controller for all vehicles?',
        ),

        DeclareLaunchArgument(
            'mavros',
            default_value='True',
            description='Launch mavros for all vehicles?',
        ),

        DeclareLaunchArgument(
            'auv1_mavros_params_file',
            default_value=os.path.join(orca_bringup_dir, 'params', 'auv1_mavros_params.yaml'),
            description='Full path to the ROS2 parameters file to use for AUV1 mavros nodes',
        ),

        DeclareLaunchArgument(
            'auv2_mavros_params_file',
            default_value=os.path.join(orca_bringup_dir, 'params', 'auv2_mavros_params.yaml'),
            description='Full path to the ROS2 parameters file to use for AUV2 mavros nodes',
        ),

        DeclareLaunchArgument(
            'auv3_mavros_params_file',
            default_value=os.path.join(orca_bringup_dir, 'params', 'auv3_mavros_params.yaml'),
            description='Full path to the ROS2 parameters file to use for AUV3 mavros nodes',
        ),

        DeclareLaunchArgument(
            'nav',
            default_value='True',
            description='Launch navigation for all vehicles?',
        ),

        DeclareLaunchArgument(
            'auv1_orca_params_file',
            default_value=os.path.join(orca_bringup_dir, 'params', 'auv1_orca_params.yaml'),
            description='Full path to the ROS2 parameters file to use for AUV1 Orca nodes',
        ),

        DeclareLaunchArgument(
            'auv2_orca_params_file',
            default_value=os.path.join(orca_bringup_dir, 'params', 'auv2_orca_params.yaml'),
            description='Full path to the ROS2 parameters file to use for AUV2 Orca nodes',
        ),

        DeclareLaunchArgument(
            'auv3_orca_params_file',
            default_value=os.path.join(orca_bringup_dir, 'params', 'auv3_orca_params.yaml'),
            description='Full path to the ROS2 parameters file to use for AUV3 Orca nodes',
        ),

        DeclareLaunchArgument(
            'slam',
            default_value='True',
            description='Launch SLAM for all vehicles?',
        ),

        # ======= GLOBAL TF TRANSFORMS =======
        
        # Global to AUV transforms (connect each auvX/map to global map)
        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--x', '0', '--y', '0', '--z', '0',
                 '--frame-id', 'map',
                 '--child-frame-id', 'auv1/map'],
            output='screen',
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--x', '-2', '--y', '2', '--z', '0',
                 '--frame-id', 'map',
                 '--child-frame-id', 'auv2/map'],
            output='screen',
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--x', '-2', '--y', '-2', '--z', '0',
                 '--frame-id', 'map',
                 '--child-frame-id', 'auv3/map'],
            output='screen',
        ),

        # ======= AUV1 NODES =======

        # AUV1 MAVROS
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='auv1',
            output='screen',
            parameters=[auv1_mavros_params_file],
            condition=IfCondition(LaunchConfiguration('mavros')),
        ),

        # AUV1 Manager
        Node(
            package='orca_base',
            executable='manager',
            namespace='auv1',
            output='screen',
            name='auv1_manager',
            parameters=[auv1_orca_params_file],
            remappings=[
                ('/camera_pose', '/auv1/orb_slam2_stereo_node/pose'),
            ],
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        # AUV1 Base controller
        Node(
            package='orca_base',
            executable='base_controller',
            namespace='auv1',
            output='screen',
            name='auv1_base_controller',
            parameters=[auv1_orca_params_file],
            remappings=[
                ('/camera_pose', '/auv1/orb_slam2_stereo_node/pose'),
            ],
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        # AUV1 static transforms that are always needed
        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--x', '-0.15',
                 '--y', '0.18',
                 '--z', '-0.0675',
                 '--pitch', str(math.pi/2),
                 '--frame-id', 'auv1/base_link',
                 '--child-frame-id', 'auv1/left_camera_link'],
            output='screen',
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--pitch', str(math.pi/2),
                 '--frame-id', 'auv1/slam',
                 '--child-frame-id', 'auv1/down'],
            output='screen',
        ),

        # AUV1 initial transforms when base controller is running (will be overridden by dynamic transforms)
        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv1/map',
                 '--child-frame-id', 'auv1/slam'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv1/map',
                 '--child-frame-id', 'auv1/odom'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv1/odom',
                 '--child-frame-id', 'auv1/base_link'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        # AUV1 static transforms when base controller is NOT running
        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv1/map',
                 '--child-frame-id', 'auv1/slam'],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('base')),
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv1/map',
                 '--child-frame-id', 'auv1/odom'],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('base')),
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv1/odom',
                 '--child-frame-id', 'auv1/base_link'],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('base')),
        ),

        # AUV1 ORB-SLAM2
        Node(
            package='orb_slam2_ros',
            executable='orb_slam2_ros_stereo',
            namespace='auv1',
            output='screen',
            name='auv1_orb_slam2_stereo',
            parameters=[auv1_orca_params_file, {
                'voc_file': orb_voc_file,
            }],
            remappings=[
                ('/image_left/image_color_rect', '/auv1/stereo_left'),
                ('/image_right/image_color_rect', '/auv1/stereo_right'),
                ('/camera/camera_info', '/auv1/stereo_right/camera_info'),
            ],
            condition=IfCondition(LaunchConfiguration('slam')),
        ),

        # ======= AUV2 NODES =======
        
        # AUV2 MAVROS
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='auv2',
            output='screen',
            parameters=[auv2_mavros_params_file],
            condition=IfCondition(LaunchConfiguration('mavros')),
        ),

        # AUV2 Manager
        Node(
            package='orca_base',
            executable='manager',
            namespace='auv2',
            output='screen',
            name='auv2_manager',
            parameters=[auv2_orca_params_file],
            remappings=[
                ('/camera_pose', '/auv2/orb_slam2_stereo_node/pose'),
            ],
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        # AUV2 Base controller
        Node(
            package='orca_base',
            executable='base_controller',
            namespace='auv2',
            output='screen',
            name='auv2_base_controller',
            parameters=[auv2_orca_params_file],
            remappings=[
                ('/camera_pose', '/auv2/orb_slam2_stereo_node/pose'),
            ],
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        # AUV2 static transforms that are always needed
        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--x', '-0.15',
                 '--y', '0.18',
                 '--z', '-0.0675',
                 '--pitch', str(math.pi/2),
                 '--frame-id', 'auv2/base_link',
                 '--child-frame-id', 'auv2/left_camera_link'],
            output='screen',
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--pitch', str(math.pi/2),
                 '--frame-id', 'auv2/slam',
                 '--child-frame-id', 'auv2/down'],
            output='screen',
        ),

        # AUV2 initial transforms when base controller is running
        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv2/map',
                 '--child-frame-id', 'auv2/slam'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv2/map',
                 '--child-frame-id', 'auv2/odom'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv2/odom',
                 '--child-frame-id', 'auv2/base_link'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        # AUV2 static transforms when base controller is NOT running
        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv2/map',
                 '--child-frame-id', 'auv2/slam'],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('base')),
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv2/map',
                 '--child-frame-id', 'auv2/odom'],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('base')),
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv2/odom',
                 '--child-frame-id', 'auv2/base_link'],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('base')),
        ),

        # AUV2 ORB-SLAM2
        Node(
            package='orb_slam2_ros',
            executable='orb_slam2_ros_stereo',
            namespace='auv2',
            output='screen',
            name='auv2_orb_slam2_stereo',
            parameters=[auv2_orca_params_file, {
                'voc_file': orb_voc_file,
            }],
            remappings=[
                ('/image_left/image_color_rect', '/auv2/stereo_left'),
                ('/image_right/image_color_rect', '/auv2/stereo_right'),
                ('/camera/camera_info', '/auv2/stereo_right/camera_info'),
            ],
            condition=IfCondition(LaunchConfiguration('slam')),
        ),

        # ======= AUV3 NODES =======
        
        # AUV3 MAVROS
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='auv3',
            output='screen',
            parameters=[auv3_mavros_params_file],
            condition=IfCondition(LaunchConfiguration('mavros')),
        ),

        # AUV3 Manager
        Node(
            package='orca_base',
            executable='manager',
            namespace='auv3',
            output='screen',
            name='auv3_manager',
            parameters=[auv3_orca_params_file],
            remappings=[
                ('/camera_pose', '/auv3/orb_slam2_stereo_node/pose'),
            ],
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        # AUV3 Base controller
        Node(
            package='orca_base',
            executable='base_controller',
            namespace='auv3',
            output='screen',
            name='auv3_base_controller',
            parameters=[auv3_orca_params_file],
            remappings=[
                ('/camera_pose', '/auv3/orb_slam2_stereo_node/pose'),
            ],
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        # AUV3 static transforms that are always needed
        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--x', '-0.15',
                 '--y', '0.18',
                 '--z', '-0.0675',
                 '--pitch', str(math.pi/2),
                 '--frame-id', 'auv3/base_link',
                 '--child-frame-id', 'auv3/left_camera_link'],
            output='screen',
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--pitch', str(math.pi/2),
                 '--frame-id', 'auv3/slam',
                 '--child-frame-id', 'auv3/down'],
            output='screen',
        ),

        # AUV3 initial transforms when base controller is running
        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv3/map',
                 '--child-frame-id', 'auv3/slam'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv3/map',
                 '--child-frame-id', 'auv3/odom'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv3/odom',
                 '--child-frame-id', 'auv3/base_link'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        # AUV3 static transforms when base controller is NOT running
        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv3/map',
                 '--child-frame-id', 'auv3/slam'],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('base')),
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv3/map',
                 '--child-frame-id', 'auv3/odom'],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('base')),
        ),

        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'auv3/odom',
                 '--child-frame-id', 'auv3/base_link'],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('base')),
        ),

        # AUV3 ORB-SLAM2
        Node(
            package='orb_slam2_ros',
            executable='orb_slam2_ros_stereo',
            namespace='auv3',
            output='screen',
            name='auv3_orb_slam2_stereo',
            parameters=[auv3_orca_params_file, {
                'voc_file': orb_voc_file,
            }],
            remappings=[
                ('/image_left/image_color_rect', '/auv3/stereo_left'),
                ('/image_right/image_color_rect', '/auv3/stereo_right'),
                ('/camera/camera_info', '/auv3/stereo_right/camera_info'),
            ],
            condition=IfCondition(LaunchConfiguration('slam')),
        ),

        # Note: Navigation nodes would be added here for each AUV if needed
        # For now, focusing on basic functionality and visualization
    ])
