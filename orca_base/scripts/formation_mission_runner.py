#!/usr/bin/env python3

"""
Multi-AUV Formation Mission Runner cho thuật toán MADDPG+RBF

Mission script điều khiển AUV1 (leader) theo các quỹ đạo được định nghĩa.
AUV2 và AUV3 sẽ tự động bám theo sử dụng thuật toán MADDPG+RBF.

Integration với:
- formation_controller.py: Điều khiển đội hình tự động
- leader_trajectory_controller.py: Quỹ đạo phức tạp
- Trajectory từ mophong.txt: (0,0) → (20,-13) → (10,-23) → (-10,-8) → (0,0)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from mavros_msgs.msg import State
import time
import math


class FormationMissionRunner(Node):
    """
    Mission runner that sends simple movement commands to the leader AUV.
    """

    def __init__(self):
        super().__init__('formation_mission_runner')
        
        # Publisher for AUV1 (leader) commands
        self.auv1_cmd_pub = self.create_publisher(Twist, '/auv1/cmd_vel', 10)
        
        # Service clients for base controller connections
        self.auv1_conn_client = self.create_client(SetBool, '/auv1/conn')
        self.auv2_conn_client = self.create_client(SetBool, '/auv2/conn')
        self.auv3_conn_client = self.create_client(SetBool, '/auv3/conn')
        
        # MAVROS state subscribers to check connection status
        self.auv1_mavros_state = None
        self.auv2_mavros_state = None
        self.auv3_mavros_state = None
        
        self.auv1_state_sub = self.create_subscription(State, '/auv1/mavros/state', self.auv1_state_callback, 100)
        self.auv2_state_sub = self.create_subscription(State, '/auv2/mavros/state', self.auv2_state_callback, 100)
        self.auv3_state_sub = self.create_subscription(State, '/auv3/mavros/state', self.auv3_state_callback, 100)
        
        self.get_logger().info('Formation Mission Runner initialized')
        self.get_logger().info('Controlling AUV1 (leader), followers will maintain formation')

    def auv1_state_callback(self, msg):
        self.auv1_mavros_state = msg

    def auv2_state_callback(self, msg):
        self.auv2_mavros_state = msg

    def auv3_state_callback(self, msg):
        self.auv3_mavros_state = msg

    def wait_for_connections(self, timeout=150.0):
        """Wait for base controller connection services to be available"""
        self.get_logger().info('Waiting for base controller connection services...')
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if (self.auv1_conn_client.service_is_ready() and 
                self.auv2_conn_client.service_is_ready() and 
                self.auv3_conn_client.service_is_ready()):
                self.get_logger().info('All base controller connection services ready!')
                return True
            time.sleep(0.5)
        
        self.get_logger().warn('Timeout waiting for base controller services - continuing anyway')
        return False

    def enable_base_controllers(self):
        """Enable all base controllers"""
        self.get_logger().info('Enabling base controllers...')
        
        # Create service request
        request = SetBool.Request()
        request.data = True
        
        # Enable AUV1 base controller
        if self.auv1_conn_client.service_is_ready():
            try:
                future = self.auv1_conn_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.result() is not None:
                    self.get_logger().info(f'AUV1 base controller: {future.result().message}')
                else:
                    self.get_logger().warn('AUV1 base controller service call failed')
            except Exception as e:
                self.get_logger().warn(f'AUV1 base controller service error: {e}')
        else:
            self.get_logger().warn('AUV1 base controller connection service not available')
        
        # Enable AUV2 base controller
        if self.auv2_conn_client.service_is_ready():
            try:
                future = self.auv2_conn_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.result() is not None:
                    self.get_logger().info(f'AUV2 base controller: {future.result().message}')
                else:
                    self.get_logger().warn('AUV2 base controller service call failed')
            except Exception as e:
                self.get_logger().warn(f'AUV2 base controller service error: {e}')
        else:
            self.get_logger().warn('AUV2 base controller connection service not available')
        
        # Enable AUV3 base controller
        if self.auv3_conn_client.service_is_ready():
            try:
                future = self.auv3_conn_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.result() is not None:
                    self.get_logger().info(f'AUV3 base controller: {future.result().message}')
                else:
                    self.get_logger().warn('AUV3 base controller service call failed')
            except Exception as e:
                self.get_logger().warn(f'AUV3 base controller service error: {e}')
        else:
            self.get_logger().warn('AUV3 base controller connection service not available')

    def check_mavros_connections(self):
        """Check MAVROS connection status for all AUVs"""
        self.get_logger().info('Checking MAVROS connections...')
        
        # Wait a bit for state messages
        time.sleep(0.5)
        
        if self.auv1_mavros_state:
            self.get_logger().info(f'AUV1 MAVROS - Connected: {self.auv1_mavros_state.connected}, Armed: {self.auv1_mavros_state.armed}, Mode: {self.auv1_mavros_state.mode}')
        else:
            self.get_logger().warn('AUV1 MAVROS state not received')
        
        if self.auv2_mavros_state:
            self.get_logger().info(f'AUV2 MAVROS - Connected: {self.auv2_mavros_state.connected}, Armed: {self.auv2_mavros_state.armed}, Mode: {self.auv2_mavros_state.mode}')
        else:
            self.get_logger().warn('AUV2 MAVROS state not received')
        
        if self.auv3_mavros_state:
            self.get_logger().info(f'AUV3 MAVROS - Connected: {self.auv3_mavros_state.connected}, Armed: {self.auv3_mavros_state.armed}, Mode: {self.auv3_mavros_state.mode}')
        else:
            self.get_logger().warn('AUV3 MAVROS state not received')

    def move_forward(self, speed=0.3, duration=5.0):
        """Move forward at specified speed for given duration"""
        self.get_logger().info(f'Moving forward at {speed} m/s for {duration} seconds')
        
        cmd = Twist()
        cmd.linear.x = speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.auv1_cmd_pub.publish(cmd)
            time.sleep(0.1)
        
        # Stop
        cmd.linear.x = 0.0
        self.auv1_cmd_pub.publish(cmd)

    def turn(self, angular_speed=0.3, duration=3.0):
        """Turn at specified angular speed for given duration"""
        self.get_logger().info(f'Turning at {angular_speed} rad/s for {duration} seconds')
        
        cmd = Twist()
        cmd.angular.z = angular_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.auv1_cmd_pub.publish(cmd)
            time.sleep(0.1)
        
        # Stop
        cmd.angular.z = 0.0
        self.auv1_cmd_pub.publish(cmd)

    def stop(self, duration=2.0):
        """Stop for specified duration"""
        self.get_logger().info(f'Stopping for {duration} seconds')
        
        cmd = Twist()
        start_time = time.time()
        while time.time() - start_time < duration:
            self.auv1_cmd_pub.publish(cmd)
            time.sleep(0.1)

    def run_formation_mission(self):
        """Execute formation mission - Updated for MADDPG+RBF compatibility"""
        self.get_logger().info('Starting formation mission!')
        self.get_logger().info('Choose mission type:')
        self.get_logger().info('1. Basic maneuvers (current)')
        self.get_logger().info('2. MADDPG waypoint trajectory (call maddpg_formation_mission)')
        self.get_logger().info('3. Sine wave trajectory (call sine_wave_trajectory)')
        
        # Wait for connections and enable base controllers
        self.wait_for_connections()
        self.enable_base_controllers()
        self.check_mavros_connections()
        
        # Wait a bit for everything to initialize
        self.stop(5.0)
        
        # Basic mission sequence (compatible với formation_controller.py)
        self.move_forward(speed=0.2, duration=8.0)  # Move forward slowly
        self.stop(2.0)
        
        self.turn(angular_speed=0.2, duration=5.0)  # Turn right
        self.stop(2.0)
        
        self.move_forward(speed=0.25, duration=6.0)  # Move forward
        self.stop(2.0)
        
        self.turn(angular_speed=-0.2, duration=5.0)  # Turn left
        self.stop(2.0)
        
        self.move_forward(speed=0.2, duration=8.0)  # Move forward
        self.stop(2.0)
        
        self.turn(angular_speed=0.2, duration=10.0)  # Complete a circle
        self.stop(2.0)
        
        self.get_logger().info('Formation mission completed!')
        self.get_logger().info('Followers should have maintained formation using MADDPG+RBF')

    def maddpg_formation_mission(self):
        """
        Thực hiện mission theo thuật toán MADDPG+RBF với quỹ đạo từ mophong.txt
        Waypoints: (0,0) → (20,-13) → (10,-23) → (-10,-8) → (0,0) x 2 vòng
        """
        self.get_logger().info('Starting MADDPG Formation Mission!')
        self.get_logger().info('Following waypoint trajectory from mophong.txt')
        
        # Initialize system
        self.wait_for_connections()
        self.enable_base_controllers()
        self.check_mavros_connections()
        self.stop(3.0)  # Initial stabilization
        
        # Waypoints theo mô tả trong mophong.txt
        waypoints = [
            (0.0, 0.0),      # Start point
            (20.0, -13.0),   # Waypoint 1  
            (10.0, -23.0),   # Waypoint 2
            (-10.0, -8.0),   # Waypoint 3
            (0.0, 0.0)       # Return to start
        ]
        
        cycles = 2  # 2 vòng như mô tả
        speed = 1.5  # ≈2 m/s theo yêu cầu
        
        for cycle in range(cycles):
            self.get_logger().info(f'Starting cycle {cycle + 1}/{cycles}')
            
            for i, (target_x, target_y) in enumerate(waypoints[1:], 1):  # Bỏ qua điểm đầu
                self.get_logger().info(f'Moving to waypoint {i}: ({target_x}, {target_y})')
                
                # Đổi hướng đột ngột (không giảm tốc) như mô tả
                self.move_to_waypoint(target_x, target_y, speed, sudden_turn=True)
                self.stop(1.0)  # Dừng ngắn tại waypoint
                
        self.get_logger().info('MADDPG Formation Mission completed!')
        self.stop(3.0)

    def move_to_waypoint(self, target_x, target_y, speed=1.5, sudden_turn=True):
        """
        Di chuyển đến waypoint với tốc độ không đổi và đổi hướng đột ngột
        Implement theo mô tả: "đổi hướng đột ngột (không giảm tốc)"
        """
        # Tính hướng di chuyển (góc yaw target)
        angle_to_target = math.atan2(target_y, target_x)
        
        if sudden_turn:
            # Đổi hướng đột ngột trước khi di chuyển
            self.rotate_to_angle(angle_to_target, angular_speed=2.0)  # Quay nhanh
            
        # Tính khoảng cách và thời gian di chuyển
        distance = math.sqrt(target_x*target_x + target_y*target_y)
        duration = distance / speed
        
        # Di chuyển thẳng với vận tốc không đổi
        self.move_straight(speed, duration, angle_to_target)

    def rotate_to_angle(self, target_angle, angular_speed=2.0):
        """Quay đến góc mục tiêu với tốc độ góc cao (đổi hướng đột ngột)"""
        self.get_logger().info(f'Rotating to angle: {math.degrees(target_angle):.1f}°')
        
        # Estimate rotation duration (rough calculation)
        duration = abs(target_angle) / angular_speed
        duration = max(0.5, min(duration, 3.0))  # Limit duration
        
        cmd = Twist()
        cmd.angular.z = angular_speed if target_angle >= 0 else -angular_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.auv1_cmd_pub.publish(cmd)
            time.sleep(0.05)  # High frequency control
            
        # Stop rotation
        cmd.angular.z = 0.0
        self.auv1_cmd_pub.publish(cmd)

    def move_straight(self, speed, duration, direction_angle=0.0):
        """Di chuyển thẳng với vận tốc không đổi theo hướng đã định"""
        self.get_logger().info(f'Moving straight: speed={speed:.1f}m/s, duration={duration:.1f}s')
        
        cmd = Twist()
        cmd.linear.x = speed * math.cos(direction_angle)
        cmd.linear.y = speed * math.sin(direction_angle)
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.auv1_cmd_pub.publish(cmd)
            time.sleep(0.05)  # High frequency for smooth trajectory
            
        # Stop
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        self.auv1_cmd_pub.publish(cmd)

    def sine_wave_trajectory(self, duration=60.0):
        """
        Thực hiện quỹ đạo hình sin theo mô tả:
        x₁(t) = v * t
        y₁(t) = A * sin(ω * t)
        """
        self.get_logger().info('Starting sine wave trajectory')
        
        v = 2.0      # Vận tốc tiến 2 m/s
        A = 5.0      # Biên độ 5m
        omega = 0.2  # Tần số góc
        
        start_time = time.time()
        while time.time() - start_time < duration:
            t = time.time() - start_time
            
            # Tính vận tốc theo công thức
            vx = v
            vy = A * omega * math.cos(omega * t)
            
            cmd = Twist()
            cmd.linear.x = vx
            cmd.linear.y = vy
            
            self.auv1_cmd_pub.publish(cmd)
            time.sleep(0.05)
            
        # Stop
        self.stop(2.0)
        self.get_logger().info('Sine wave trajectory completed')


def main(args=None):
    rclpy.init(args=args)
    
    mission_runner = FormationMissionRunner()
    
    try:
        # Run the formation mission
        print("\n" + "="*60)
        print("  MADDPG+RBF Formation Mission Options")
        print("="*60)
        print("Available missions:")
        print("1. Basic Formation Mission (default)")
        print("2. MADDPG Waypoint Trajectory")
        print("3. Sine Wave Trajectory")
        print("="*60)
        
        # Default: run basic formation mission
        mission_runner.run_formation_mission()
        
        # To run other missions, uncomment below:
        # mission_runner.maddpg_formation_mission()      # Waypoint trajectory
        # mission_runner.sine_wave_trajectory(60.0)      # Sine wave for 60s
        
        # Keep the node alive
        rclpy.spin(mission_runner)
        
    except KeyboardInterrupt:
        mission_runner.get_logger().info('Mission interrupted by user')
    
    mission_runner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
