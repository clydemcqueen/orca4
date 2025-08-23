#!/usr/bin/env python3

"""
Leader Trajectory Controller theo thuật toán trong mophong.txt

Điều khiển AUV1 (leader) di chuyển theo quỹ đạo được mô tả:
1. Quỹ đạo hình sin: x_1(t) = v*t, y_1(t) = A*sin(ω*t)
2. Quỹ đạo waypoint tuần hoàn: (0,0) → (20,-13) → (10,-23) → (-10,-8) → (0,0)

Tốc độ di chuyển ≈2 m/s, đổi hướng đột ngột tại waypoints
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import time
import numpy as np


class LeaderTrajectoryController(Node):
    """
    Điều khiển quỹ đạo cho AUV leader theo thuật toán mô tả
    """

    def __init__(self):
        super().__init__('leader_trajectory_controller')
        
        # Trajectory parameters cho quỹ đạo hình sin
        self.v = 2.0        # Vận tốc tiến ≈2 m/s
        self.A = 5.0        # Biên độ dao động theo trục Y (m)
        self.omega = 0.2    # Tần số góc (rad/s)
        
        # Waypoint trajectory parameters
        self.waypoints = [
            (0.0, 0.0),      # Điểm bắt đầu
            (20.0, -13.0),   # Waypoint 1
            (10.0, -23.0),   # Waypoint 2
            (-10.0, -8.0),   # Waypoint 3
            (0.0, 0.0)       # Quay về điểm ban đầu
        ]
        self.current_waypoint_idx = 0
        self.cycles_completed = 0
        self.max_cycles = 2  # Lặp lại 2 vòng như mô tả
        
        # Control mode: 'sine' hoặc 'waypoint'
        self.trajectory_mode = 'waypoint'  # Có thể thay đổi để test
        
        # Current state
        self.current_pose = None
        self.start_time = time.time()
        self.waypoint_reached_tolerance = 1.0  # 1m tolerance để coi là đã đến waypoint
        
        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/auv1/cmd_vel', 10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/auv1/odometry', 
            self.odom_callback,
            10
        )
        
        # Control timer - chạy ở 20Hz
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(f'Leader Trajectory Controller khởi tạo - Mode: {self.trajectory_mode}')
        if self.trajectory_mode == 'sine':
            self.get_logger().info(f'Sine trajectory: v={self.v} m/s, A={self.A}m, ω={self.omega} rad/s')
        else:
            self.get_logger().info(f'Waypoint trajectory: {len(self.waypoints)} points, {self.max_cycles} cycles')

    def odom_callback(self, msg):
        """Cập nhật vị trí hiện tại của leader"""
        self.current_pose = msg.pose.pose

    def quaternion_to_yaw(self, q):
        """Chuyển quaternion thành góc yaw"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def sine_trajectory_control(self, t):
        """
        Điều khiển theo quỹ đạo hình sin:
        x_1(t) = v * t
        y_1(t) = A * sin(ω * t)
        """
        # Vị trí mong muốn
        desired_x = self.v * t
        desired_y = self.A * math.sin(self.omega * t)
        
        # Vận tốc mong muốn (đạo hàm của vị trí)
        desired_vx = self.v
        desired_vy = self.A * self.omega * math.cos(self.omega * t)
        
        cmd = Twist()
        cmd.linear.x = desired_vx
        cmd.linear.y = desired_vy
        
        return cmd

    def waypoint_trajectory_control(self):
        """
        Điều khiển theo các waypoint tuần hoàn
        Di chuyển theo đường thẳng giữa các waypoint với tốc độ ≈2 m/s
        """
        if self.current_pose is None:
            return Twist()
            
        if self.cycles_completed >= self.max_cycles:
            # Hoàn thành 2 vòng, dừng lại
            return Twist()
            
        # Waypoint hiện tại và tiếp theo
        current_wp = self.waypoints[self.current_waypoint_idx]
        
        # Vị trí hiện tại
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # Tính khoảng cách đến waypoint
        dx = current_wp[0] - current_x
        dy = current_wp[1] - current_y
        distance_to_wp = math.sqrt(dx*dx + dy*dy)
        
        # Kiểm tra xem đã đến waypoint chưa
        if distance_to_wp < self.waypoint_reached_tolerance:
            self.get_logger().info(f'Đã đến waypoint {self.current_waypoint_idx}: {current_wp}')
            
            # Chuyển sang waypoint tiếp theo
            self.current_waypoint_idx += 1
            
            if self.current_waypoint_idx >= len(self.waypoints):
                # Hoàn thành 1 vòng
                self.current_waypoint_idx = 1  # Bỏ qua điểm đầu, bắt đầu từ waypoint 1
                self.cycles_completed += 1
                self.get_logger().info(f'Hoàn thành vòng {self.cycles_completed}/{self.max_cycles}')
                
                if self.cycles_completed >= self.max_cycles:
                    self.get_logger().info('Hoàn thành toàn bộ mission!')
                    return Twist()
        
        # Tính hướng di chuyển đến waypoint tiếp theo
        if self.current_waypoint_idx < len(self.waypoints):
            target_wp = self.waypoints[self.current_waypoint_idx]
        else:
            target_wp = self.waypoints[1]  # Quay lại waypoint đầu tiên của vòng mới
            
        # Vector hướng đến target
        dx_target = target_wp[0] - current_x
        dy_target = target_wp[1] - current_y
        distance_to_target = math.sqrt(dx_target*dx_target + dy_target*dy_target)
        
        if distance_to_target < 0.1:  # Tránh chia cho 0
            return Twist()
        
        # Normalize vector hướng và nhân với vận tốc mong muốn
        vx = (dx_target / distance_to_target) * self.v
        vy = (dy_target / distance_to_target) * self.v
        
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        
        # Đổi hướng đột ngột tại waypoint (không giảm tốc)
        # Angular velocity để hướng về target
        if self.current_pose is not None:
            current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            desired_yaw = math.atan2(dy_target, dx_target)
            yaw_error = desired_yaw - current_yaw
            
            # Normalize yaw error
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi
                
            cmd.angular.z = 2.0 * yaw_error  # Hệ số cao để đổi hướng nhanh
        
        return cmd

    def control_loop(self):
        """Vòng lặp điều khiển chính"""
        if self.trajectory_mode == 'sine':
            # Quỹ đạo hình sin
            t = time.time() - self.start_time
            cmd = self.sine_trajectory_control(t)
        else:
            # Quỹ đạo waypoint
            cmd = self.waypoint_trajectory_control()
            
        # Giới hạn vận tốc để đảm bảo an toàn
        max_vel = 3.0
        cmd.linear.x = max(min(cmd.linear.x, max_vel), -max_vel)
        cmd.linear.y = max(min(cmd.linear.y, max_vel), -max_vel)
        cmd.angular.z = max(min(cmd.angular.z, 2.0), -2.0)
        
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    
    controller = LeaderTrajectoryController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
