#!/usr/bin/env python3

"""
Multi-AUV Formation Controller theo thuật toán MADDPG+RBF

Điều khiển đội hình 3 AUV với AUV1 làm leader và AUV2, AUV3 làm follower.
Thuật toán dựa trên MADDPG với mạng RBF được đơn giản hóa thành bộ điều khiển tỷ lệ.

Formation layout theo mô tả:
- AUV2 (follower 1): Δ₁₂ = (-5, 2) m (phía sau bên trái leader 5m, lệch trái 2m)
- AUV3 (follower 2): Δ₁₃ = (-5, -2) m (phía sau bên phải leader 5m, lệch phải 2m)

Thuật toán điều khiển: v_j(t) = v_leader(t) + K_p * (η_j,des(t) - η_j(t))
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import math


class FormationController(Node):
    """
    Formation controller implementing simplified MADDPG+RBF algorithm
    theo mô tả trong file mophong.txt
    """

    def __init__(self):
        super().__init__('formation_controller')
        
        # Formation parameters theo mô tả (Δ vector trong body frame của leader)
        # Δ₁₂ = (-5, 2) m: AUV2 ở phía sau bên trái
        # Δ₁₃ = (-5, -2) m: AUV3 ở phía sau bên phải
        self.delta_12 = np.array([-5.0, 2.0])   # Vector từ leader đến AUV2
        self.delta_13 = np.array([-5.0, -2.0])  # Vector từ leader đến AUV3
        
        # Current positions and velocities
        self.auv1_pose = None
        self.auv2_pose = None  
        self.auv3_pose = None
        self.auv1_velocity = None
        self.auv2_velocity = None
        self.auv3_velocity = None
        
        # Control gains (K_p trong công thức điều khiển)
        # Đặt nhỏ để tránh dao động như mô tả
        self.kp = 0.3  # Hệ số lợi ích chính
        self.max_linear_speed = 2.0  # Giả định tốc độ tối đa ≈2 m/s như leader
        self.max_angular_speed = 1.0
        
        # Subscribers to odometry (quan sát trạng thái đầu vào)
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
        
        # Publishers for velocity commands (điều khiển velocity cho follower)
        self.auv2_cmd_pub = self.create_publisher(Twist, '/auv2/cmd_vel', 10)
        self.auv3_cmd_pub = self.create_publisher(Twist, '/auv3/cmd_vel', 10)
        
        # Control timer (chạy thuật toán điều khiển định kỳ)
        self.timer = self.create_timer(0.05, self.control_formation)  # 20Hz như mô tả
        
        self.get_logger().info('Formation Controller theo thuật toán MADDPG+RBF initialized')
        self.get_logger().info('Delta vectors: AUV2=(-5,2), AUV3=(-5,-2) trong body frame của leader')

    def auv1_odom_callback(self, msg):
        """Lưu trạng thái AUV1 (leader)"""
        self.auv1_pose = msg.pose.pose
        self.auv1_velocity = msg.twist.twist

    def auv2_odom_callback(self, msg):
        """Lưu trạng thái AUV2 (follower 1)"""
        self.auv2_pose = msg.pose.pose
        self.auv2_velocity = msg.twist.twist

    def auv3_odom_callback(self, msg):
        """Lưu trạng thái AUV3 (follower 2)"""
        self.auv3_pose = msg.pose.pose
        self.auv3_velocity = msg.twist.twist

    def quaternion_to_yaw(self, q):
        """Chuyển quaternion thành góc yaw"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def calculate_desired_position(self, leader_pose, delta_vector):
        """
        Tính vị trí mong muốn của follower theo công thức:
        η_j,des(t) = η_1(t) + R(θ_1) * Δ_1j
        
        Args:
            leader_pose: Pose của AUV leader
            delta_vector: Vector Δ trong body frame của leader [dx, dy]
            
        Returns:
            tuple: (desired_x, desired_y) vị trí mong muốn trong world frame
        """
        if leader_pose is None:
            return None, None
            
        # Vị trí và hướng của leader
        leader_x = leader_pose.position.x
        leader_y = leader_pose.position.y
        leader_yaw = self.quaternion_to_yaw(leader_pose.orientation)
        
        # Ma trận xoay từ body frame sang world frame
        cos_yaw = math.cos(leader_yaw)
        sin_yaw = math.sin(leader_yaw)
        
        # Chuyển đổi delta vector sang world frame
        # R(θ) * Δ = [cos(θ) -sin(θ)] * [dx]
        #             [sin(θ)  cos(θ)]   [dy]
        delta_world_x = cos_yaw * delta_vector[0] - sin_yaw * delta_vector[1]
        delta_world_y = sin_yaw * delta_vector[0] + cos_yaw * delta_vector[1]
        
        # Vị trí mong muốn = vị trí leader + delta trong world frame
        desired_x = leader_x + delta_world_x
        desired_y = leader_y + delta_world_y
        
        return desired_x, desired_y

    def calculate_formation_error(self, follower_pose, desired_x, desired_y):
        """
        Tính sai số đội hình e_j(t) = |η_j(t) - η_j,des(t)|
        
        Args:
            follower_pose: Pose hiện tại của follower
            desired_x, desired_y: Vị trí mong muốn
            
        Returns:
            tuple: (error_x, error_y, error_magnitude) sai số vị trí và độ lớn
        """
        if follower_pose is None or desired_x is None or desired_y is None:
            return 0.0, 0.0, 0.0
            
        error_x = desired_x - follower_pose.position.x
        error_y = desired_y - follower_pose.position.y
        error_magnitude = math.sqrt(error_x*error_x + error_y*error_y)
        
        return error_x, error_y, error_magnitude

    def maddpg_rbf_control(self, follower_pose, leader_velocity, error_x, error_y):
        """
        Thuật toán điều khiển MADDPG+RBF được đơn giản hóa:
        v_j(t) = v_leader(t) + K_p * (η_j,des(t) - η_j(t))
        
        Args:
            follower_pose: Pose hiện tại của follower
            leader_velocity: Vận tốc của leader
            error_x, error_y: Sai số vị trí
            
        Returns:
            Twist: Lệnh điều khiển vận tốc
        """
        cmd = Twist()
        
        if leader_velocity is None:
            return cmd
            
        # Thành phần vận tốc từ leader (v_leader(t))
        leader_vx = leader_velocity.linear.x
        leader_vy = leader_velocity.linear.y
        leader_vyaw = leader_velocity.angular.z
        
        # Thành phần hiệu chỉnh tỷ lệ với sai số đội hình (K_p * error)
        correction_vx = self.kp * error_x
        correction_vy = self.kp * error_y
        
        # Vận tốc tổng hợp (trong world frame)
        total_vx = leader_vx + correction_vx
        total_vy = leader_vy + correction_vy
        
        # Chuyển đổi về body frame của follower để điều khiển
        if follower_pose is not None:
            follower_yaw = self.quaternion_to_yaw(follower_pose.orientation)
            cos_yaw = math.cos(follower_yaw)
            sin_yaw = math.sin(follower_yaw)
            
            # Chuyển từ world frame sang body frame
            cmd.linear.x = cos_yaw * total_vx + sin_yaw * total_vy
            cmd.linear.y = -sin_yaw * total_vx + cos_yaw * total_vy
        else:
            cmd.linear.x = total_vx
            cmd.linear.y = total_vy
            
        # Angular velocity (giữ góc yaw tương tự leader với điều chỉnh nhỏ)
        cmd.angular.z = leader_vyaw
        
        # Giới hạn vận tốc để đảm bảo an toàn
        cmd.linear.x = max(min(cmd.linear.x, self.max_linear_speed), -self.max_linear_speed)
        cmd.linear.y = max(min(cmd.linear.y, self.max_linear_speed), -self.max_linear_speed)
        cmd.angular.z = max(min(cmd.angular.z, self.max_angular_speed), -self.max_angular_speed)
        
        return cmd

    def control_formation(self):
        """
        Vòng lặp điều khiển chính theo thuật toán MADDPG+RBF
        Chạy ở tần số 20Hz để điều khiển đội hình theo thời gian thực
        """
        if self.auv1_pose is None or self.auv1_velocity is None:
            return  # Chờ thông tin từ leader
        
        # Điều khiển AUV2 (follower 1) với Δ₁₂ = (-5, 2)
        if self.auv2_pose is not None:
            # Tính vị trí mong muốn của AUV2
            desired_x, desired_y = self.calculate_desired_position(self.auv1_pose, self.delta_12)
            
            if desired_x is not None and desired_y is not None:
                # Tính sai số đội hình e_2(t)
                error_x, error_y, error_mag = self.calculate_formation_error(
                    self.auv2_pose, desired_x, desired_y)
                
                # Áp dụng thuật toán MADDPG+RBF
                cmd2 = self.maddpg_rbf_control(
                    self.auv2_pose, self.auv1_velocity, error_x, error_y)
                
                # Gửi lệnh điều khiển
                self.auv2_cmd_pub.publish(cmd2)
                
                # Log thông tin debug (có thể bỏ comment khi cần)
                # self.get_logger().debug(f'AUV2 - Error: {error_mag:.2f}m, Cmd: vx={cmd2.linear.x:.2f}, vy={cmd2.linear.y:.2f}')
        
        # Điều khiển AUV3 (follower 2) với Δ₁₃ = (-5, -2)
        if self.auv3_pose is not None:
            # Tính vị trí mong muốn của AUV3
            desired_x, desired_y = self.calculate_desired_position(self.auv1_pose, self.delta_13)
            
            if desired_x is not None and desired_y is not None:
                # Tính sai số đội hình e_3(t)
                error_x, error_y, error_mag = self.calculate_formation_error(
                    self.auv3_pose, desired_x, desired_y)
                
                # Áp dụng thuật toán MADDPG+RBF
                cmd3 = self.maddpg_rbf_control(
                    self.auv3_pose, self.auv1_velocity, error_x, error_y)
                
                # Gửi lệnh điều khiển
                self.auv3_cmd_pub.publish(cmd3)
                
                # Log thông tin debug (có thể bỏ comment khi cần)
                # self.get_logger().debug(f'AUV3 - Error: {error_mag:.2f}m, Cmd: vx={cmd3.linear.x:.2f}, vy={cmd3.linear.y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    
    formation_controller = FormationController()
    
    try:
        rclpy.spin(formation_controller)
    except KeyboardInterrupt:
        pass
    
    formation_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
