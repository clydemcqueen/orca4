#!/usr/bin/env python3

"""
Multi-AUV Path Publisher

Publishes path trajectories for all AUVs based on their odometry data.
This allows visualization of each AUV's trajectory in RViz.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class MultiAUVPathPublisher(Node):
    """
    Node that subscribes to each AUV's odometry and publishes path trajectories.
    """

    def __init__(self):
        super().__init__('multi_auv_path_publisher')
        
        # Path storage for each AUV
        self.auv1_path = Path()
        self.auv2_path = Path()
        self.auv3_path = Path()
        
        # Initialize path headers
        self.auv1_path.header.frame_id = "map"
        self.auv2_path.header.frame_id = "map"
        self.auv3_path.header.frame_id = "map"
        
        # Publishers for each AUV's path
        self.auv1_path_pub = self.create_publisher(Path, '/auv1/path', 10)
        self.auv2_path_pub = self.create_publisher(Path, '/auv2/path', 10)
        self.auv3_path_pub = self.create_publisher(Path, '/auv3/path', 10)
        
        # Subscribers to odometry
        self.auv1_odom_sub = self.create_subscription(
            Odometry,
            '/model/auv1/odometry',
            self.auv1_odom_callback,
            10
        )
        
        self.auv2_odom_sub = self.create_subscription(
            Odometry,
            '/model/auv2/odometry',
            self.auv2_odom_callback,
            10
        )
        
        self.auv3_odom_sub = self.create_subscription(
            Odometry,
            '/model/auv3/odometry',
            self.auv3_odom_callback,
            10
        )
        
        # Timer to publish paths regularly
        self.timer = self.create_timer(0.1, self.publish_paths)
        
        self.get_logger().info('Multi-AUV Path Publisher initialized')

    def auv1_odom_callback(self, msg):
        """Add new pose to AUV1 path"""
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = msg.pose.pose
        
        self.auv1_path.poses.append(pose_stamped)
        self.auv1_path.header.stamp = msg.header.stamp
        
        # Limit path length to prevent memory issues
        if len(self.auv1_path.poses) > 1000:
            self.auv1_path.poses.pop(0)

    def auv2_odom_callback(self, msg):
        """Add new pose to AUV2 path"""
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = msg.pose.pose
        
        self.auv2_path.poses.append(pose_stamped)
        self.auv2_path.header.stamp = msg.header.stamp
        
        # Limit path length to prevent memory issues
        if len(self.auv2_path.poses) > 1000:
            self.auv2_path.poses.pop(0)

    def auv3_odom_callback(self, msg):
        """Add new pose to AUV3 path"""
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = msg.pose.pose
        
        self.auv3_path.poses.append(pose_stamped)
        self.auv3_path.header.stamp = msg.header.stamp
        
        # Limit path length to prevent memory issues
        if len(self.auv3_path.poses) > 1000:
            self.auv3_path.poses.pop(0)

    def publish_paths(self):
        """Publish all AUV paths"""
        if len(self.auv1_path.poses) > 0:
            self.auv1_path_pub.publish(self.auv1_path)
        
        if len(self.auv2_path.poses) > 0:
            self.auv2_path_pub.publish(self.auv2_path)
        
        if len(self.auv3_path.poses) > 0:
            self.auv3_path_pub.publish(self.auv3_path)


def main(args=None):
    rclpy.init(args=args)
    
    path_publisher = MultiAUVPathPublisher()
    
    try:
        rclpy.spin(path_publisher)
    except KeyboardInterrupt:
        pass
    
    path_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
