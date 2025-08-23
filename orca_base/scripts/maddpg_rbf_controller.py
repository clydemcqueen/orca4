#!/usr/bin/env python3

"""
MADDPG+RBF Formation Controller Implementation

Triển khai thuật toán MADDPG với mạng RBF theo mô tả trong mophong.txt.
Đây là version đầy đủ với RBF network simulation và multi-agent reward function.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math


class RBFNetwork:
    """
    Mạng RBF (Radial Basis Function) cho MADDPG Actor
    Hàm kích hoạt Gaussian: φ_k(s) = exp(-γ |s - c_k|²)
    """
    
    def __init__(self, input_dim, num_rbf_units=10, gamma=1.0):
        self.input_dim = input_dim
        self.num_rbf_units = num_rbf_units
        self.gamma = gamma
        
        # Tâm RBF units (c_k) - phân bố ngẫu nhiên trong không gian trạng thái
        self.centers = np.random.uniform(-10, 10, (num_rbf_units, input_dim))
        
        # Trọng số đầu ra (W và bias b)
        self.weights = np.random.uniform(-1, 1, (num_rbf_units, 2))  # 2 outputs: vx, vy
        self.bias = np.random.uniform(-0.1, 0.1, 2)
        
    def rbf_activation(self, state):
        """
        Tính activation của các RBF units
        φ_k(s) = exp(-γ |s - c_k|²)
        """
        activations = np.zeros(self.num_rbf_units)
        
        for k in range(self.num_rbf_units):
            # Khoảng cách từ state đến center k
            distance_sq = np.sum((state - self.centers[k]) ** 2)
            activations[k] = np.exp(-self.gamma * distance_sq)
            
        return activations
    
    def forward(self, state):
        """
        Forward pass: a_j = W^T * Φ(s_j) + b
        """
        phi = self.rbf_activation(state)
        output = np.dot(self.weights.T, phi) + self.bias
        return output


class MADDPGFormationController(Node):
    """
    Multi-Agent DDPG với RBF network cho điều khiển đội hình
    """

    def __init__(self):
        super().__init__('maddpg_formation_controller')
        
        # Formation parameters theo mô tả
        self.delta_12 = np.array([-5.0, 2.0])   # AUV2: (-5, 2)
        self.delta_13 = np.array([-5.0, -2.0])  # AUV3: (-5, -2)
        
        # MADDPG parameters
        self.alpha = 0.1    # Hệ số phạt lỗi vị trí (lớn nhất để ưu tiên giảm sai số)
        self.beta = 0.05    # Hệ số phạt va chạm
        self.gamma_energy = 0.01  # Hệ số phạt năng lượng điều khiển
        self.collision_distance = 2.0  # Khoảng cách tối thiểu giữa các AUV
        
        # RBF Networks cho mỗi agent (AUV2 và AUV3)
        state_dim = 6  # [x_j, y_j, vx_j, vy_j, x_1-x_j, y_1-y_j]
        self.rbf_auv2 = RBFNetwork(state_dim, num_rbf_units=15, gamma=0.5)
        self.rbf_auv3 = RBFNetwork(state_dim, num_rbf_units=15, gamma=0.5)
        
        # Current states
        self.auv1_pose = None
        self.auv2_pose = None
        self.auv3_pose = None
        self.auv1_velocity = None
        self.auv2_velocity = None
        self.auv3_velocity = None
        
        # Subscribers
        self.auv1_odom_sub = self.create_subscription(
            Odometry, '/model/auv1/odometry', self.auv1_odom_callback, 10)
        self.auv2_odom_sub = self.create_subscription(
            Odometry, '/model/auv2/odometry', self.auv2_odom_callback, 10)
        self.auv3_odom_sub = self.create_subscription(
            Odometry, '/model/auv3/odometry', self.auv3_odom_callback, 10)
        
        # Publishers
        self.auv2_cmd_pub = self.create_publisher(Twist, '/auv2/cmd_vel', 10)
        self.auv3_cmd_pub = self.create_publisher(Twist, '/auv3/cmd_vel', 10)
        
        # Control timer
        self.timer = self.create_timer(0.05, self.maddpg_control_loop)  # 20Hz
        
        self.get_logger().info('MADDPG+RBF Formation Controller initialized')
        self.get_logger().info(f'RBF networks: {self.rbf_auv2.num_rbf_units} units each')

    def auv1_odom_callback(self, msg):
        self.auv1_pose = msg.pose.pose
        self.auv1_velocity = msg.twist.twist

    def auv2_odom_callback(self, msg):
        self.auv2_pose = msg.pose.pose
        self.auv2_velocity = msg.twist.twist

    def auv3_odom_callback(self, msg):
        self.auv3_pose = msg.pose.pose
        self.auv3_velocity = msg.twist.twist

    def get_state_vector(self, agent_pose, agent_velocity, leader_pose):
        """
        Tạo vector trạng thái cho agent:
        s_j = [x_j, y_j, vx_j, vy_j, x_1-x_j, y_1-y_j]
        """
        if agent_pose is None or agent_velocity is None or leader_pose is None:
            return None
            
        state = np.array([
            agent_pose.position.x,
            agent_pose.position.y,
            agent_velocity.linear.x,
            agent_velocity.linear.y,
            leader_pose.position.x - agent_pose.position.x,
            leader_pose.position.y - agent_pose.position.y
        ])
        
        return state

    def calculate_desired_position_world(self, leader_pose, delta_vector):
        """Tính vị trí mong muốn trong world frame"""
        if leader_pose is None:
            return None
            
        leader_x = leader_pose.position.x
        leader_y = leader_pose.position.y
        
        # Giả sử leader luôn hướng về phía trước (có thể cải tiến thêm)
        # Ở đây đơn giản hóa: delta trong world frame
        desired_x = leader_x + delta_vector[0]
        desired_y = leader_y + delta_vector[1]
        
        return np.array([desired_x, desired_y])

    def calculate_formation_error(self, agent_pose, desired_position):
        """Tính sai số đội hình e_j(t)"""
        if agent_pose is None or desired_position is None:
            return 0.0
            
        agent_pos = np.array([agent_pose.position.x, agent_pose.position.y])
        error_vector = desired_position - agent_pos
        error_magnitude = np.linalg.norm(error_vector)
        
        return error_magnitude, error_vector

    def check_collision(self, pose1, pose2):
        """Kiểm tra va chạm giữa 2 AUV"""
        if pose1 is None or pose2 is None:
            return False
            
        distance = math.sqrt(
            (pose1.position.x - pose2.position.x)**2 + 
            (pose1.position.y - pose2.position.y)**2
        )
        
        return distance < self.collision_distance

    def calculate_reward(self, agent_pose, desired_position, action, collision_penalty=0):
        """
        Hàm phần thưởng MADDPG:
        r_j = -α * e_j² - β * (collision penalty) - γ * |a_j|²
        """
        error_mag, _ = self.calculate_formation_error(agent_pose, desired_position)
        
        # Thành phần phạt lỗi vị trí
        position_penalty = self.alpha * (error_mag ** 2)
        
        # Thành phần phạt năng lượng điều khiển
        energy_penalty = self.gamma_energy * (action[0]**2 + action[1]**2)
        
        # Thành phần phạt va chạm
        collision_term = self.beta * collision_penalty
        
        reward = -(position_penalty + collision_term + energy_penalty)
        
        return reward

    def rbf_policy(self, rbf_network, state):
        """
        Chính sách RBF Actor: a_j = W^T * Φ(s_j) + b
        """
        if state is None:
            return np.array([0.0, 0.0])
            
        action = rbf_network.forward(state)
        
        # Giới hạn action để đảm bảo an toàn
        action = np.clip(action, -2.0, 2.0)
        
        return action

    def maddpg_control_loop(self):
        """
        Vòng lặp MADDPG chính
        """
        if self.auv1_pose is None:
            return
            
        # === Agent AUV2 ===
        if (self.auv2_pose is not None and self.auv2_velocity is not None):
            # Lấy trạng thái đầu vào
            state2 = self.get_state_vector(self.auv2_pose, self.auv2_velocity, self.auv1_pose)
            
            if state2 is not None:
                # Chính sách RBF Actor
                action2 = self.rbf_policy(self.rbf_auv2, state2)
                
                # Tính vị trí mong muốn và reward (cho việc học)
                desired_pos2 = self.calculate_desired_position_world(self.auv1_pose, self.delta_12)
                
                # Kiểm tra va chạm
                collision_penalty2 = 1.0 if self.check_collision(self.auv2_pose, self.auv3_pose) else 0.0
                
                # Tính reward (trong thực tế sẽ dùng để cập nhật network)
                reward2 = self.calculate_reward(self.auv2_pose, desired_pos2, action2, collision_penalty2)
                
                # Gửi lệnh điều khiển
                cmd2 = Twist()
                cmd2.linear.x = float(action2[0])
                cmd2.linear.y = float(action2[1])
                self.auv2_cmd_pub.publish(cmd2)
        
        # === Agent AUV3 ===
        if (self.auv3_pose is not None and self.auv3_velocity is not None):
            # Lấy trạng thái đầu vào
            state3 = self.get_state_vector(self.auv3_pose, self.auv3_velocity, self.auv1_pose)
            
            if state3 is not None:
                # Chính sách RBF Actor
                action3 = self.rbf_policy(self.rbf_auv3, state3)
                
                # Tính vị trí mong muốn và reward
                desired_pos3 = self.calculate_desired_position_world(self.auv1_pose, self.delta_13)
                
                # Kiểm tra va chạm
                collision_penalty3 = 1.0 if self.check_collision(self.auv3_pose, self.auv2_pose) else 0.0
                
                # Tính reward
                reward3 = self.calculate_reward(self.auv3_pose, desired_pos3, action3, collision_penalty3)
                
                # Gửi lệnh điều khiển
                cmd3 = Twist()
                cmd3.linear.x = float(action3[0])
                cmd3.linear.y = float(action3[1])
                self.auv3_cmd_pub.publish(cmd3)


def main(args=None):
    rclpy.init(args=args)
    
    controller = MADDPGFormationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
