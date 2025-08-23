#!/usr/bin/env python3

"""
Launch a multi-AUV simulation with 3 vehicles in formation.

Includes Gazebo, ArduSub for each vehicle, RViz, mavros, all ROS nodes.
AUV1 is the leader, AUV2 and AUV3 are followers.
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

    # ArduSub parameters for each vehicle
    ardusub_params_file0 = os.path.join(orca_bringup_dir, 'cfg', 'sub0.parm')
    ardusub_params_file1 = os.path.join(orca_bringup_dir, 'cfg', 'sub1.parm')
    ardusub_params_file2 = os.path.join(orca_bringup_dir, 'cfg', 'sub2.parm')

    # MAVROS parameters for each vehicle
    auv1_mavros_params_file = os.path.join(orca_bringup_dir, 'params', 'auv1_mavros_params.yaml')
    auv2_mavros_params_file = os.path.join(orca_bringup_dir, 'params', 'auv2_mavros_params.yaml')
    auv3_mavros_params_file = os.path.join(orca_bringup_dir, 'params', 'auv3_mavros_params.yaml')
    
    # Orca parameters for each vehicle
    auv1_orca_params_file = os.path.join(orca_bringup_dir, 'params', 'auv1_orca_params.yaml')
    auv2_orca_params_file = os.path.join(orca_bringup_dir, 'params', 'auv2_orca_params.yaml')
    auv3_orca_params_file = os.path.join(orca_bringup_dir, 'params', 'auv3_orca_params.yaml')
    
    # Camera calibration files
    auv1_sim_left_ini = os.path.join(orca_bringup_dir, 'cfg', 'auv1_sim_left.ini')
    auv1_sim_right_ini = os.path.join(orca_bringup_dir, 'cfg', 'auv1_sim_right.ini')
    auv2_sim_left_ini = os.path.join(orca_bringup_dir, 'cfg', 'auv2_sim_left.ini')
    auv2_sim_right_ini = os.path.join(orca_bringup_dir, 'cfg', 'auv2_sim_right.ini')
    auv3_sim_left_ini = os.path.join(orca_bringup_dir, 'cfg', 'auv3_sim_left.ini')
    auv3_sim_right_ini = os.path.join(orca_bringup_dir, 'cfg', 'auv3_sim_right.ini')
    
    rosbag2_record_qos_file = os.path.join(orca_bringup_dir, 'params', 'rosbag2_record_qos.yaml')
    rviz_file = os.path.join(orca_bringup_dir, 'cfg', 'multi_auv_sim_launch.rviz')
    world_file = os.path.join(orca_description_dir, 'worlds', 'multi_auv_sand.world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ardusub',
            default_value='True',
            description='Launch ArduSUB for all vehicles?'
        ),

        DeclareLaunchArgument(
            'bag',
            default_value='False',
            description='Bag interesting topics?',
        ),

        DeclareLaunchArgument(
            'base',
            default_value='True',
            description='Launch base controller for all vehicles?',
        ),

        DeclareLaunchArgument(
            'gzclient',
            default_value='True',
            description='Launch Gazebo UI?'
        ),

        DeclareLaunchArgument(
            'mavros',
            default_value='True',
            description='Launch mavros for all vehicles?',
        ),

        DeclareLaunchArgument(
            'nav',
            default_value='True',
            description='Launch navigation for all vehicles?',
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='True',
            description='Launch rviz?',
        ),

        DeclareLaunchArgument(
            'slam',
            default_value='True',
            description='Launch SLAM for all vehicles?',
        ),

        # Bag useful topics from all vehicles
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--qos-profile-overrides-path', rosbag2_record_qos_file,
                '--include-hidden-topics',
                '/auv1/cmd_vel', '/auv2/cmd_vel', '/auv3/cmd_vel',
                '/auv1/mavros/local_position/pose', '/auv2/mavros/local_position/pose', '/auv3/mavros/local_position/pose',
                '/auv1/mavros/rc/override', '/auv2/mavros/rc/override', '/auv3/mavros/rc/override',
                '/auv1/mavros/setpoint_position/global', '/auv2/mavros/setpoint_position/global', '/auv3/mavros/setpoint_position/global',
                '/auv1/mavros/state', '/auv2/mavros/state', '/auv3/mavros/state',
                '/auv1/mavros/vision_pose/pose', '/auv2/mavros/vision_pose/pose', '/auv3/mavros/vision_pose/pose',
                '/model/auv1/odometry', '/model/auv2/odometry', '/model/auv3/odometry',
                '/auv1/motion', '/auv2/motion', '/auv3/motion',
                '/auv1/odom', '/auv2/odom', '/auv3/odom',
                '/auv1/orb_slam2_stereo_node/pose', '/auv2/orb_slam2_stereo_node/pose', '/auv3/orb_slam2_stereo_node/pose',
                '/auv1/orb_slam2_stereo_node/status', '/auv2/orb_slam2_stereo_node/status', '/auv3/orb_slam2_stereo_node/status',
                '/auv1/pid_z', '/auv2/pid_z', '/auv3/pid_z',
                '/rosout',
                '/tf',
                '/tf_static',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('bag')),
        ),

        # Launch rviz
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),

        # Launch ArduSub for AUV1 (I0) - Leader
        ExecuteProcess(
            cmd=['ardusub', '-S', '-w', '-M', 'JSON', '--defaults', ardusub_params_file0,
                 '-I0', '--home', '33.810313,-118.39386700000001,0.0,0',
                 '--serial0', '--out=tcp:127.0.0.1:5760'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('ardusub')),
        ),

        # Launch ArduSub for AUV2 (I1) - Follower
        ExecuteProcess(
            cmd=['ardusub', '-S', '-w', '-M', 'JSON', '--defaults', ardusub_params_file1,
                 '-I1', '--home', '33.810313,-118.39386700000001,0.0,0',
                 '--serial1', '--out=tcp:127.0.0.1:5770'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('ardusub')),
        ),

        # Launch ArduSub for AUV3 (I2) - Follower
        ExecuteProcess(
            cmd=['ardusub', '-S', '-w', '-M', 'JSON', '--defaults', ardusub_params_file2,
                 '-I2', '--home', '33.810313,-118.39386700000001,0.0,0',
                 '--serial2', '--out=tcp:127.0.0.1:5780'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('ardusub')),
        ),

        # Launch Gazebo Sim with UI
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', '-r', world_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gzclient')),
        ),

        # Launch Gazebo Sim server-only
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', '-r', '-s', world_file],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('gzclient')),
        ),

        # Get images from Gazebo Sim to ROS for all AUVs
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['auv1/stereo_left', 'auv1/stereo_right', 
                      'auv2/stereo_left', 'auv2/stereo_right',
                      'auv3/stereo_left', 'auv3/stereo_right'],
            output='screen',
        ),

        # Gazebo Sim doesn't publish camera info, so do that here for AUV1
        Node(
            package='orca_base',
            executable='camera_info_publisher',
            name='auv1_left_info_publisher',
            namespace='auv1',
            output='screen',
            parameters=[{
                'camera_info_url': 'file://' + auv1_sim_left_ini,
                'camera_name': 'stereo_left',
                'frame_id': 'auv1_stereo_left_frame',
                'timer_period_ms': 50,
            }],
            remappings=[
                ('/camera_info', '/auv1/stereo_left/camera_info'),
            ],
        ),

        Node(
            package='orca_base',
            executable='camera_info_publisher',
            name='auv1_right_info_publisher',
            namespace='auv1',
            output='screen',
            parameters=[{
                'camera_info_url': 'file://' + auv1_sim_right_ini,
                'camera_name': 'stereo_right',
                'frame_id': 'auv1_stereo_right_frame',
                'timer_period_ms': 50,
            }],
            remappings=[
                ('/camera_info', '/auv1/stereo_right/camera_info'),
            ],
        ),

        # Camera info publishers for AUV2
        Node(
            package='orca_base',
            executable='camera_info_publisher',
            name='auv2_left_info_publisher',
            namespace='auv2',
            output='screen',
            parameters=[{
                'camera_info_url': 'file://' + auv2_sim_left_ini,
                'camera_name': 'stereo_left',
                'frame_id': 'auv2_stereo_left_frame',
                'timer_period_ms': 50,
            }],
            remappings=[
                ('/camera_info', '/auv2/stereo_left/camera_info'),
            ],
        ),

        Node(
            package='orca_base',
            executable='camera_info_publisher',
            name='auv2_right_info_publisher',
            namespace='auv2',
            output='screen',
            parameters=[{
                'camera_info_url': 'file://' + auv2_sim_right_ini,
                'camera_name': 'stereo_right',
                'frame_id': 'auv2_stereo_right_frame',
                'timer_period_ms': 50,
            }],
            remappings=[
                ('/camera_info', '/auv2/stereo_right/camera_info'),
            ],
        ),

        # Camera info publishers for AUV3
        Node(
            package='orca_base',
            executable='camera_info_publisher',
            name='auv3_left_info_publisher',
            namespace='auv3',
            output='screen',
            parameters=[{
                'camera_info_url': 'file://' + auv3_sim_left_ini,
                'camera_name': 'stereo_left',
                'frame_id': 'auv3_stereo_left_frame',
                'timer_period_ms': 50,
            }],
            remappings=[
                ('/camera_info', '/auv3/stereo_left/camera_info'),
            ],
        ),

        Node(
            package='orca_base',
            executable='camera_info_publisher',
            name='auv3_right_info_publisher',
            namespace='auv3',
            output='screen',
            parameters=[{
                'camera_info_url': 'file://' + auv3_sim_right_ini,
                'camera_name': 'stereo_right',
                'frame_id': 'auv3_stereo_right_frame',
                'timer_period_ms': 50,
            }],
            remappings=[
                ('/camera_info', '/auv3/stereo_right/camera_info'),
            ],
        ),

        # Publish ground truth pose from Gazebo for each AUV
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/auv1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/auv2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/auv3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
            output='screen'
        ),

        # Formation controller for follower AUVs
        Node(
            package='orca_base',
            executable='formation_controller.py',
            name='formation_controller',
            output='screen',
        ),

        # Multi-AUV Path Publisher for trajectory visualization
        Node(
            package='orca_base',
            executable='multi_auv_path_publisher.py',
            name='multi_auv_path_publisher',
            output='screen',
        ),

        # Bring up Orca and Nav2 nodes for each AUV
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(orca_bringup_dir, 'launch', 'multi_auv_bringup.py')),
            launch_arguments={
                'base': LaunchConfiguration('base'),
                'mavros': LaunchConfiguration('mavros'),
                'auv1_mavros_params_file': auv1_mavros_params_file,
                'auv2_mavros_params_file': auv2_mavros_params_file,
                'auv3_mavros_params_file': auv3_mavros_params_file,
                'nav': LaunchConfiguration('nav'),
                'auv1_orca_params_file': auv1_orca_params_file,
                'auv2_orca_params_file': auv2_orca_params_file,
                'auv3_orca_params_file': auv3_orca_params_file,
                'slam': LaunchConfiguration('slam'),
            }.items(),
        ),

        # Auto-connect base controllers after a delay
        Node(
            package='orca_base',
            executable='auto_connector.py',
            name='auto_connector',
            output='screen',
        ),
    ])
