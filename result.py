import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation
from datetime import datetime
import os
import math
import shutil
from enum import Enum

# Thiết lập một số thông số hiển thị cho matplotlib
plt.rcParams['figure.figsize'] = (12, 8)
plt.rcParams['font.size'] = 10
plt.rcParams['axes.grid'] = True
plt.rcParams['grid.alpha'] = 0.3
plt.rcParams['font.family'] = 'DejaVu Sans'  # hỗ trợ hiển thị ký tự Unicode (ví dụ: chỉ số dưới)

class ControllerType(Enum):
    BASIC = "Basic Formation"
    MADDPG_RBF = "MADDPG+RBF"
    LEADER_CONTROLLED = "Leader Trajectory"

class MissionType(Enum):
    BASIC_MANEUVERS = "Basic Maneuvers"
    WAYPOINT_TRAJECTORY = "Waypoint Trajectory"
    SINE_WAVE = "Sine Wave"

class ScenarioAnalyzer:
    def __init__(self):
        pass

    def simulate_basic_formation_controller(self, mission_type, duration=300):
        """Mô phỏng Basic Formation Controller"""
        dt = 0.05  # tần số 20Hz
        t = np.arange(0, duration, dt)
        # Độ lệch đội hình mong muốn
        delta_12 = np.array([-5, 2])    # AUV2 so với AUV1
        delta_13 = np.array([-5, -2])   # AUV3 so với AUV1
        # Khởi tạo mảng vị trí cho các AUV
        auv1_pos = np.zeros((len(t), 2))
        auv2_pos = np.zeros((len(t), 2))
        auv3_pos = np.zeros((len(t), 2))
        # Quỹ đạo leader (AUV1) tùy theo loại nhiệm vụ
        if mission_type == MissionType.BASIC_MANEUVERS:
            for i, time_step in enumerate(t):
                if time_step < 60:
                    # Đi thẳng
                    auv1_pos[i] = [1.0 * time_step, 0.0]
                elif time_step < 120:
                    # Rẽ phải (quỹ đạo tròn)
                    angle = 0.5 * (time_step - 60)
                    radius = 20
                    auv1_pos[i] = [60 + radius * np.sin(angle), -radius * (1 - np.cos(angle))]
                elif time_step < 180:
                    # Đi thẳng tiếp
                    auv1_pos[i] = [60 + 20 * np.sin(np.pi/2), -20 + 1.0 * (time_step - 120)]
                else:
                    # Lượn vòng tròn
                    angle = 0.2 * (time_step - 180)
                    auv1_pos[i] = [80 + 15 * np.cos(angle), -20 + 15 * np.sin(angle)]
        elif mission_type == MissionType.WAYPOINT_TRAJECTORY:
            # Lộ trình waypoint (0,0) → (20,-13) → (10,-23) → (-10,-8) → (0,0)
            waypoints = np.array([[0, 0], [20, -13], [10, -23], [-10, -8], [0, 0]])
            cycle_duration = duration / 2  # chạy 2 chu kỳ waypoints
            for i, time_step in enumerate(t):
                cycle_time = time_step % cycle_duration
                progress = cycle_time / cycle_duration * (len(waypoints) - 1)
                segment = int(progress)
                segment = min(segment, len(waypoints) - 2)
                alpha = progress - segment
                auv1_pos[i] = (1 - alpha) * waypoints[segment] + alpha * waypoints[segment + 1]
        elif mission_type == MissionType.SINE_WAVE:
            # Quỹ đạo hình sin
            velocity = 2.0      # vận tốc chuyển động theo trục x
            amplitude = 5.0
            frequency = 0.1
            for i, time_step in enumerate(t):
                auv1_pos[i] = [velocity * time_step, amplitude * np.sin(frequency * time_step)]
        # Bộ điều khiển cơ bản (PID) để duy trì đội hình
        Kp_pos = 1.5
        Kd_pos = 0.3
        # Tốc độ của các follower
        auv2_vel = np.zeros((len(t), 2))
        auv3_vel = np.zeros((len(t), 2))
        formation_noise = 0.2  # nhiễu nhỏ mô phỏng sai lệch
        for i in range(1, len(t)):
            # Vị trí mong muốn của follower
            desired_auv2 = auv1_pos[i] + delta_12
            desired_auv3 = auv1_pos[i] + delta_13
            # Sai số đội hình
            error_auv2 = desired_auv2 - auv2_pos[i-1] + np.random.normal(0, formation_noise, 2)
            error_auv3 = desired_auv3 - auv3_pos[i-1] + np.random.normal(0, formation_noise, 2)
            # Điều khiển PID tính vận tốc
            auv2_vel[i] = Kp_pos * error_auv2 + Kd_pos * (error_auv2 - (desired_auv2 - auv2_pos[max(0, i-2)]))
            auv3_vel[i] = Kp_pos * error_auv3 + Kd_pos * (error_auv3 - (desired_auv3 - auv3_pos[max(0, i-2)]))
            # Tích phân vận tốc để cập nhật vị trí
            auv2_pos[i] = auv2_pos[i-1] + auv2_vel[i] * dt
            auv3_pos[i] = auv3_pos[i-1] + auv3_vel[i] * dt
        return {
            'time': t,
            'auv1_pos': auv1_pos,
            'auv2_pos': auv2_pos,
            'auv3_pos': auv3_pos,
            'auv2_vel': auv2_vel,
            'auv3_vel': auv3_vel,
            'formation_errors': {
                'auv2': np.linalg.norm(auv2_pos - (auv1_pos + delta_12), axis=1),
                'auv3': np.linalg.norm(auv3_pos - (auv1_pos + delta_13), axis=1)
            }
        }

    def simulate_maddpg_rbf_controller(self, mission_type, duration=300):
        """Mô phỏng MADDPG+RBF Controller (thuật toán nâng cao)"""
        dt = 0.05
        t = np.arange(0, duration, dt)
        # Tham số đội hình
        delta_12 = np.array([-5, 2])
        delta_13 = np.array([-5, -2])
        # Tham số mạng RBF và thuật toán MADDPG
        n_rbf_centers = 10
        rbf_sigma = 2.0
        learning_rate = 0.001
        exploration_noise = 0.1
        # Khởi tạo vị trí (AUV1 sẽ dùng quỹ đạo leader giống bộ điều khiển cơ bản)
        auv1_pos = np.zeros((len(t), 2))
        auv2_pos = np.zeros((len(t), 2))
        auv3_pos = np.zeros((len(t), 2))
        basic_result = self.simulate_basic_formation_controller(mission_type, duration)
        auv1_pos = basic_result['auv1_pos']
        # Khởi tạo trọng số mạng RBF ngẫu nhiên
        rbf_weights_auv2 = np.random.normal(0, 0.1, (n_rbf_centers, 2))
        rbf_weights_auv3 = np.random.normal(0, 0.1, (n_rbf_centers, 2))
        rbf_centers = np.random.uniform(-10, 10, (n_rbf_centers, 4))
        # Mảng lưu reward
        rewards_auv2 = []
        rewards_auv3 = []
        # Tham số PID nâng cao
        Kp_rbf = 2.0
        Ki_rbf = 0.1
        Kd_rbf = 0.5
        integral_error_auv2 = np.zeros(2)
        integral_error_auv3 = np.zeros(2)
        prev_error_auv2 = np.zeros(2)
        prev_error_auv3 = np.zeros(2)
        formation_noise = 0.1  # nhiễu nhỏ hơn do điều khiển tốt hơn
        # Khởi tạo mảng tốc độ follower
        auv2_vel = np.zeros((len(t), 2))
        auv3_vel = np.zeros((len(t), 2))
        for i in range(1, len(t)):
            # Vị trí mong muốn của follower tại thời điểm i
            desired_auv2 = auv1_pos[i] + delta_12
            desired_auv3 = auv1_pos[i] + delta_13
            # Trạng thái hiện tại (đầu vào mạng RBF)
            state_auv2 = np.concatenate([auv1_pos[i], auv2_pos[i-1]])
            state_auv3 = np.concatenate([auv1_pos[i], auv3_pos[i-1]])
            # Kích hoạt RBF
            rbf_output_auv2 = np.zeros(2)
            rbf_output_auv3 = np.zeros(2)
            for j in range(n_rbf_centers):
                dist2 = np.linalg.norm(state_auv2 - rbf_centers[j])
                dist3 = np.linalg.norm(state_auv3 - rbf_centers[j])
                phi2 = np.exp(-dist2**2 / (2 * rbf_sigma**2))
                phi3 = np.exp(-dist3**2 / (2 * rbf_sigma**2))
                rbf_output_auv2 += rbf_weights_auv2[j] * phi2
                rbf_output_auv3 += rbf_weights_auv3[j] * phi3
            # Sai số đội hình (có bù RBF)
            error_auv2 = desired_auv2 - auv2_pos[i-1] + 0.5 * rbf_output_auv2
            error_auv3 = desired_auv3 - auv3_pos[i-1] + 0.5 * rbf_output_auv3
            # Bộ điều khiển PID (với thành phần tích phân và đạo hàm)
            integral_error_auv2 += error_auv2 * dt
            integral_error_auv3 += error_auv3 * dt
            derivative_error_auv2 = (error_auv2 - prev_error_auv2) / dt
            derivative_error_auv3 = (error_auv3 - prev_error_auv3) / dt
            control_auv2 = Kp_rbf * error_auv2 + Ki_rbf * integral_error_auv2 + Kd_rbf * derivative_error_auv2
            control_auv3 = Kp_rbf * error_auv3 + Ki_rbf * integral_error_auv3 + Kd_rbf * derivative_error_auv3
            # Thêm nhiễu khám phá cho MADDPG
            control_auv2 += np.random.normal(0, exploration_noise, 2)
            control_auv3 += np.random.normal(0, exploration_noise, 2)
            # Giới hạn độ lớn điều khiển (saturation)
            max_control = 3.0
            control_auv2 = np.clip(control_auv2, -max_control, max_control)
            control_auv3 = np.clip(control_auv3, -max_control, max_control)
            # Lưu tốc độ điều khiển áp dụng
            auv2_vel[i] = control_auv2
            auv3_vel[i] = control_auv3
            # Cập nhật vị trí follower
            auv2_pos[i] = auv2_pos[i-1] + control_auv2 * dt + np.random.normal(0, formation_noise, 2)
            auv3_pos[i] = auv3_pos[i-1] + control_auv3 * dt + np.random.normal(0, formation_noise, 2)
            # Tính reward tức thời
            formation_error_2 = np.linalg.norm(error_auv2)
            formation_error_3 = np.linalg.norm(error_auv3)
            reward_2 = -formation_error_2 - 0.1 * np.linalg.norm(control_auv2)
            reward_3 = -formation_error_3 - 0.1 * np.linalg.norm(control_auv3)
            rewards_auv2.append(reward_2)
            rewards_auv3.append(reward_3)
            # Cập nhật trọng số RBF đơn giản (mô phỏng học của MADDPG)
            if i % 20 == 0:
                rbf_weights_auv2 += learning_rate * np.outer(np.random.normal(0, 0.1, n_rbf_centers), error_auv2)
                rbf_weights_auv3 += learning_rate * np.outer(np.random.normal(0, 0.1, n_rbf_centers), error_auv3)
            prev_error_auv2 = error_auv2
            prev_error_auv3 = error_auv3
        # Chèn giá trị reward ban đầu = 0 để đồng bộ độ dài
        rewards_auv2.insert(0, 0.0)
        rewards_auv3.insert(0, 0.0)
        rewards_arr_auv2 = np.array(rewards_auv2)
        rewards_arr_auv3 = np.array(rewards_auv3)
        return {
            'time': t,
            'auv1_pos': auv1_pos,
            'auv2_pos': auv2_pos,
            'auv3_pos': auv3_pos,
            'auv2_vel': auv2_vel,
            'auv3_vel': auv3_vel,
            'formation_errors': {
                'auv2': np.linalg.norm(auv2_pos - (auv1_pos + delta_12), axis=1),
                'auv3': np.linalg.norm(auv3_pos - (auv1_pos + delta_13), axis=1)
            },
            'rewards': {
                'auv2': rewards_arr_auv2,
                'auv3': rewards_arr_auv3
            }
        }

    def simulate_leader_trajectory_controller(self, mission_type, duration=300):
        """Mô phỏng Leader Trajectory Controller"""
        dt = 0.05
        t = np.arange(0, duration, dt)
        # Tham số đội hình (để tính sai số)
        delta_12 = np.array([-5, 2])
        delta_13 = np.array([-5, -2])
        # Khởi tạo vị trí các AUV
        auv1_pos = np.zeros((len(t), 2))
        auv2_pos = np.zeros((len(t), 2))
        auv3_pos = np.zeros((len(t), 2))
        # Quỹ đạo leader
        if mission_type == MissionType.SINE_WAVE:
            # Quỹ đạo sine nâng cao với nhiều tần số
            v_base = 2.0
            A1, A2 = 3.0, 1.5
            omega1, omega2 = 0.1, 0.3
            for i, time_step in enumerate(t):
                x = v_base * time_step + A2 * math.cos(omega2 * time_step)
                y = A1 * math.sin(omega1 * time_step) + 0.5 * A2 * math.sin(omega2 * time_step)
                auv1_pos[i] = [x, y]
        else:
            # Sử dụng cùng quỹ đạo như bộ điều khiển khác để so sánh công bằng
            basic_result = self.simulate_basic_formation_controller(mission_type, duration)
            auv1_pos = basic_result['auv1_pos']
        # Điều khiển leader-follower mang tính dự báo
        prediction_horizon = 10  # số bước dự báo trước
        Kp_leader = 3.0
        Kv_leader = 1.0
        formation_noise = 0.05
        for i in range(1, len(t)):
            # Dự đoán vị trí leader trong tương lai (sau prediction_horizon bước)
            if i + prediction_horizon < len(t):
                future_leader_pos = auv1_pos[i + prediction_horizon]
            else:
                future_leader_pos = auv1_pos[-1]
            # Vị trí mong muốn hiện tại
            desired_auv2 = auv1_pos[i] + delta_12
            desired_auv3 = auv1_pos[i] + delta_13
            # Vị trí mong muốn trong tương lai (để điều khiển dự báo)
            future_desired_auv2 = future_leader_pos + delta_12
            future_desired_auv3 = future_leader_pos + delta_13
            # Luật điều khiển dự báo (sử dụng sai số hiện tại và tương lai)
            error_auv2 = desired_auv2 - auv2_pos[i-1]
            error_auv3 = desired_auv3 - auv3_pos[i-1]
            future_error_auv2 = future_desired_auv2 - desired_auv2
            future_error_auv3 = future_desired_auv3 - desired_auv3
            control_auv2 = Kp_leader * error_auv2 + Kv_leader * future_error_auv2
            control_auv3 = Kp_leader * error_auv3 + Kv_leader * future_error_auv3
            # Cập nhật vị trí follower (có thêm nhiễu nhỏ)
            auv2_pos[i] = auv2_pos[i-1] + control_auv2 * dt + np.random.normal(0, formation_noise, 2)
            auv3_pos[i] = auv3_pos[i-1] + control_auv3 * dt + np.random.normal(0, formation_noise, 2)
        return {
            'time': t,
            'auv1_pos': auv1_pos,
            'auv2_pos': auv2_pos,
            'auv3_pos': auv3_pos,
            'formation_errors': {
                'auv2': np.linalg.norm(auv2_pos - (auv1_pos + delta_12), axis=1),
                'auv3': np.linalg.norm(auv3_pos - (auv1_pos + delta_13), axis=1)
            }
        }

    def run_scenario_comparison(self, mission_type, duration=300):
        """Chạy mô phỏng cả 3 controller cho một loại nhiệm vụ và trả về kết quả"""
        print(f"\n🔄 Running scenario comparison for: {mission_type.value}")
        # Chạy cả ba controller
        basic_result = self.simulate_basic_formation_controller(mission_type, duration)
        maddpg_result = self.simulate_maddpg_rbf_controller(mission_type, duration)
        leader_result = self.simulate_leader_trajectory_controller(mission_type, duration)
        results = {
            ControllerType.BASIC: basic_result,
            ControllerType.MADDPG_RBF: maddpg_result,
            ControllerType.LEADER_CONTROLLED: leader_result
        }
        # Tính một số chỉ số hiệu suất (metrics)
        metrics = {}
        for controller_type, result in results.items():
            err2 = result['formation_errors']['auv2']
            err3 = result['formation_errors']['auv3']
            metrics[controller_type] = {
                'mean_formation_error': (np.mean(err2) + np.mean(err3)) / 2,
                'max_formation_error': max(np.max(err2), np.max(err3)),
                'std_formation_error': (np.std(err2) + np.std(err3)) / 2,
                'convergence_time': self.calculate_convergence_time(err2, err3),
                'steady_state_error': self.calculate_steady_state_error(err2, err3)
            }
        return results, metrics

    def calculate_convergence_time(self, error_auv2, error_auv3, threshold=1.0):
        """Tính thời gian để sai số trung bình < ngưỡng (mặc định 1.0m)"""
        combined_error = (error_auv2 + error_auv3) / 2
        for i in range(len(combined_error)):
            if i > 100:  # bỏ qua 5s đầu
                if np.all(combined_error[i:i+50] < threshold):
                    return i * 0.05  # quy đổi index thành giây
        return float('inf')

    def calculate_steady_state_error(self, error_auv2, error_auv3):
        """Tính sai số trạng thái ổn định (trung bình sai số 20% thời gian cuối)"""
        last_20_percent = int(0.8 * len(error_auv2))
        steady_error_2 = np.mean(error_auv2[last_20_percent:])
        steady_error_3 = np.mean(error_auv3[last_20_percent:])
        return (steady_error_2 + steady_error_3) / 2

    def plot_scenario_comparison(self, mission_type, results, metrics):
        """Vẽ biểu đồ so sánh các controller cho một scenario (9 ô đồ thị)"""
        fig = plt.figure(figsize=(20, 16))
        scenario_folder = mission_type.value.lower().replace(' ', '_')
        scenario_dir = os.path.join("final_results", scenario_folder)
        os.makedirs(scenario_dir, exist_ok=True)
        # 1. So sánh quỹ đạo
        ax1 = plt.subplot(3, 3, 1)
        colors = {'Basic Formation': 'blue', 'MADDPG+RBF': 'red', 'Leader Trajectory': 'green'}
        for controller_type, result in results.items():
            label = controller_type.value
            color = colors[label]
            plt.plot(result['auv1_pos'][:, 0], result['auv1_pos'][:, 1], color=color, linewidth=2,
                     label=f'{label} - Leader', alpha=0.8)
            plt.plot(result['auv2_pos'][:, 0], result['auv2_pos'][:, 1], color=color, linewidth=1,
                     linestyle='--', label=f'{label} - AUV2', alpha=0.6)
            plt.plot(result['auv3_pos'][:, 0], result['auv3_pos'][:, 1], color=color, linewidth=1,
                     linestyle=':', label=f'{label} - AUV3', alpha=0.6)
        plt.title(f'Trajectory Comparison - {mission_type.value}')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        # 2. So sánh sai số đội hình theo thời gian
        ax2 = plt.subplot(3, 3, 2)
        for controller_type, result in results.items():
            label = controller_type.value
            color = colors[label]
            combined_error = (result['formation_errors']['auv2'] + result['formation_errors']['auv3']) / 2
            plt.plot(result['time'], combined_error, color=color, linewidth=2, label=label)
        plt.title('Formation Error Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Average Formation Error (m)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        # 3. So sánh sai số trung bình (biểu đồ cột)
        ax3 = plt.subplot(3, 3, 3)
        controllers = list(metrics.keys())
        mean_errors = [metrics[ctrl]['mean_formation_error'] for ctrl in controllers]
        controller_names = [ctrl.value for ctrl in controllers]
        bars = plt.bar(controller_names, mean_errors,
                       color=[colors[name] for name in controller_names], alpha=0.7)
        plt.title('Mean Formation Error Comparison')
        plt.ylabel('Mean Formation Error (m)')
        plt.xticks(rotation=45)
        for bar, value in zip(bars, mean_errors):
            plt.text(bar.get_x() + bar.get_width()/2, value + 0.01, f'{value:.3f}', ha='center', va='bottom')
        # 4. So sánh thời gian hội tụ
        ax4 = plt.subplot(3, 3, 4)
        conv_times = [metrics[ctrl]['convergence_time'] for ctrl in controllers]
        bars = plt.bar(controller_names, conv_times,
                       color=[colors[name] for name in controller_names], alpha=0.7)
        plt.title('Convergence Time Comparison')
        plt.ylabel('Convergence Time (s)')
        plt.xticks(rotation=45)
        for bar, value in zip(bars, conv_times):
            label = 'No Conv.' if value == float('inf') else f'{value:.1f}s'
            plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1, label, ha='center', va='bottom')
        # 5. Phân bố sai số (biểu đồ boxplot)
        ax5 = plt.subplot(3, 3, 5)
        error_data = []
        labels = []
        for controller_type, result in results.items():
            combined_error = (result['formation_errors']['auv2'] + result['formation_errors']['auv3']) / 2
            error_data.append(combined_error)
            labels.append(controller_type.value)
        plt.boxplot(error_data, tick_labels=labels)
        plt.title('Formation Error Distribution')
        plt.ylabel('Formation Error (m)')
        plt.xticks(rotation=45)
        plt.grid(True, alpha=0.3)
        # 6. Reward của MADDPG+RBF theo thời gian
        ax6 = plt.subplot(3, 3, 6)
        if ControllerType.MADDPG_RBF in results and 'rewards' in results[ControllerType.MADDPG_RBF]:
            rewards = results[ControllerType.MADDPG_RBF]['rewards']
            plt.plot(rewards['auv2'], label='AUV2 Reward', alpha=0.7)
            plt.plot(rewards['auv3'], label='AUV3 Reward', alpha=0.7)
            plt.title('MADDPG+RBF Learning Progress')
            plt.xlabel('Time Steps')
            plt.ylabel('Reward')
            plt.legend()
            plt.grid(True, alpha=0.3)
        else:
            plt.text(0.5, 0.5, 'No reward data\navailable', ha='center', va='center', transform=ax6.transAxes)
            plt.title('MADDPG+RBF Rewards')
        # 7. Bảng tổng hợp chỉ số hiệu suất
        ax7 = plt.subplot(3, 3, 7)
        ax7.axis('tight')
        ax7.axis('off')
        metric_names = ['Mean Error (m)', 'Max Error (m)', 'Std Error (m)', 'Conv. Time (s)', 'SS Error (m)']
        table_data = []
        for controller_type in controllers:
            m = metrics[controller_type]
            conv_time = m['convergence_time']
            conv_str = f"{conv_time:.1f}" if conv_time != float('inf') else "∞"
            row = [f"{m['mean_formation_error']:.3f}",
                   f"{m['max_formation_error']:.3f}",
                   f"{m['std_formation_error']:.3f}",
                   conv_str,
                   f"{m['steady_state_error']:.3f}"]
            table_data.append(row)
        table = ax7.table(cellText=table_data,
                          rowLabels=[ctrl.value for ctrl in controllers],
                          colLabels=metric_names,
                          cellLoc='center', loc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1.2, 1.5)
        plt.title('Performance Metrics Summary', pad=20)
        # 8. Vị trí đội hình cuối cùng
        ax8 = plt.subplot(3, 3, 8)
        final_idx = -1
        for controller_type, result in results.items():
            label = controller_type.value
            color = colors[label]
            auv1_final = result['auv1_pos'][final_idx]
            auv2_final = result['auv2_pos'][final_idx]
            auv3_final = result['auv3_pos'][final_idx]
            plt.scatter(auv1_final[0], auv1_final[1], color=color, s=100, marker='s', label=f'{label} - Leader')
            plt.scatter(auv2_final[0], auv2_final[1], color=color, s=80, marker='^', alpha=0.7)
            plt.scatter(auv3_final[0], auv3_final[1], color=color, s=80, marker='v', alpha=0.7)
            plt.plot([auv1_final[0], auv2_final[0]], [auv1_final[1], auv2_final[1]],
                     color=color, linestyle='--', alpha=0.5)
            plt.plot([auv1_final[0], auv3_final[0]], [auv1_final[1], auv3_final[1]],
                     color=color, linestyle='--', alpha=0.5)
        plt.title('Final Formation Positions')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        # 9. Hồ sơ tốc độ AUV2 theo thời gian
        ax9 = plt.subplot(3, 3, 9)
        for controller_type, result in results.items():
            if 'auv2_vel' in result:
                auv2_speed = np.linalg.norm(result['auv2_vel'], axis=1)
                plt.plot(result['time'], auv2_speed, color=colors[controller_type.value],
                         label=f'{controller_type.value} - AUV2', alpha=0.7)
        plt.title('AUV2 Speed Profile')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (m/s)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        # Lưu hình ảnh so sánh scenario
        plt.tight_layout()
        filename = os.path.join(scenario_dir, f"scenario_comparison_{mission_type.value.lower().replace(' ', '_')}.png")
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"💾 Scenario comparison saved: {filename}")
        plt.close(fig)
        return fig

    def run_comprehensive_analysis(self):
        """Chạy phân tích toàn diện cho cả 3 kịch bản"""
        print("🚀 Starting Comprehensive Multi-Scenario Analysis")
        print("=" * 60)
        all_results = {}
        all_metrics = {}
        mission_types = [MissionType.BASIC_MANEUVERS, MissionType.WAYPOINT_TRAJECTORY, MissionType.SINE_WAVE]
        for mission_type in mission_types:
            results, metrics = self.run_scenario_comparison(mission_type)
            all_results[mission_type] = results
            all_metrics[mission_type] = metrics
            # Lưu biểu đồ so sánh cho scenario này
            self.plot_scenario_comparison(mission_type, results, metrics)
            # Sinh biểu đồ chi tiết và dữ liệu cho controller MADDPG+RBF
            advanced_result = results[ControllerType.MADDPG_RBF]
            generate_advanced_plots_and_data(mission_type, advanced_result)
        # Tạo biểu đồ tổng hợp so sánh giữa các controller trên mọi kịch bản
        self.create_comprehensive_summary(all_results, all_metrics)
        print("\n✅ Comprehensive analysis complete!")
        return all_results, all_metrics

    def create_comprehensive_summary(self, all_results, all_metrics):
        """Tạo biểu đồ tổng hợp so sánh các controller trên tất cả các kịch bản"""
        print("\n📊 Creating comprehensive summary across scenarios...")
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        mission_types = list(all_metrics.keys())
        controller_types = list(all_metrics[mission_types[0]].keys())
        colors = {'Basic Formation': 'blue', 'MADDPG+RBF': 'red', 'Leader Trajectory': 'green'}
        # 1. So sánh sai số trung bình trên các kịch bản
        ax = axes[0, 0]
        for i, controller in enumerate(controller_types):
            errors = [all_metrics[mission][controller]['mean_formation_error'] for mission in mission_types]
            x_pos = np.arange(len(mission_types)) + i * 0.25
            ax.bar(x_pos, errors, 0.25, label=controller.value, color=colors[controller.value], alpha=0.7)
        ax.set_xlabel('Mission Types')
        ax.set_ylabel('Mean Formation Error (m)')
        ax.set_title('Mean Formation Error Comparison')
        ax.set_xticks(np.arange(len(mission_types)) + 0.25)
        ax.set_xticklabels([m.value for m in mission_types], rotation=45)
        ax.legend()
        ax.grid(True, alpha=0.3)
        # 2. So sánh thời gian hội tụ trên các kịch bản
        ax = axes[0, 1]
        for i, controller in enumerate(controller_types):
            conv_times = []
            for mission in mission_types:
                t_val = all_metrics[mission][controller]['convergence_time']
                conv_times.append(t_val if t_val != float('inf') else 0)
            x_pos = np.arange(len(mission_types)) + i * 0.25
            ax.bar(x_pos, conv_times, 0.25, label=controller.value, color=colors[controller.value], alpha=0.7)
        ax.set_xlabel('Mission Types')
        ax.set_ylabel('Convergence Time (s)')
        ax.set_title('Convergence Time Comparison')
        ax.set_xticks(np.arange(len(mission_types)) + 0.25)
        ax.set_xticklabels([m.value for m in mission_types], rotation=45)
        ax.legend()
        ax.grid(True, alpha=0.3)
        # 3. So sánh sai số trạng thái ổn định trên các kịch bản
        ax = axes[0, 2]
        for i, controller in enumerate(controller_types):
            ss_errors = [all_metrics[mission][controller]['steady_state_error'] for mission in mission_types]
            x_pos = np.arange(len(mission_types)) + i * 0.25
            ax.bar(x_pos, ss_errors, 0.25, label=controller.value, color=colors[controller.value], alpha=0.7)
        ax.set_xlabel('Mission Types')
        ax.set_ylabel('Steady State Error (m)')
        ax.set_title('Steady State Error Comparison')
        ax.set_xticks(np.arange(len(mission_types)) + 0.25)
        ax.set_xticklabels([m.value for m in mission_types], rotation=45)
        ax.legend()
        ax.grid(True, alpha=0.3)
        # 4. Xếp hạng hiệu suất tổng thể (thấp hơn là tốt hơn)
        ax = axes[1, 0]
        overall_scores = {}
        for controller in controller_types:
            score = 0
            for mission in mission_types:
                m = all_metrics[mission][controller]
                score += (m['mean_formation_error'] * 0.4 +
                          m['steady_state_error'] * 0.3 +
                          (m['convergence_time'] if m['convergence_time'] != float('inf') else 300) * 0.001 * 0.3)
            overall_scores[controller] = score / len(mission_types)
        controllers_sorted = sorted(overall_scores.keys(), key=lambda x: overall_scores[x])
        scores_sorted = [overall_scores[c] for c in controllers_sorted]
        bars = ax.bar([c.value for c in controllers_sorted], scores_sorted,
                      color=[colors[c.value] for c in controllers_sorted], alpha=0.7)
        ax.set_ylabel('Overall Performance Score')
        ax.set_title('Overall Performance Ranking\n(Lower is Better)')
        ax.grid(True, alpha=0.3)
        for bar, score in zip(bars, scores_sorted):
            ax.text(bar.get_x() + bar.get_width()/2, score + 0.01, f'{score:.3f}', ha='center', va='bottom')
        # 5. Heatmap hiệu suất
        ax = axes[1, 1]
        metrics_names = ['Mean Error', 'Max Error', 'Std Error', 'Conv. Time', 'SS Error']
        performance_matrix = []
        for controller in controller_types:
            row = []
            for mission in mission_types:
                m = all_metrics[mission][controller]
                norm_mean = min(1.0, m['mean_formation_error'] / 2.0)
                norm_max = min(1.0, m['max_formation_error'] / 5.0)
                norm_std = min(1.0, m['std_formation_error'] / 1.0)
                norm_conv = min(1.0, (m['convergence_time'] if m['convergence_time'] != float('inf') else 300) / 300)
                norm_ss = min(1.0, m['steady_state_error'] / 1.0)
                avg_perf = (norm_mean + norm_max + norm_std + norm_conv + norm_ss) / 5
                row.append(avg_perf)
            performance_matrix.append(row)
        im = ax.imshow(performance_matrix, cmap='RdYlGn_r', aspect='auto')
        ax.set_xticks(np.arange(len(mission_types)))
        ax.set_yticks(np.arange(len(controller_types)))
        ax.set_xticklabels([m.value for m in mission_types], rotation=45)
        ax.set_yticklabels([c.value for c in controller_types])
        ax.set_title('Performance Heatmap\n(Green=Better, Red=Worse)')
        for i in range(len(controller_types)):
            for j in range(len(mission_types)):
                ax.text(j, i, f'{performance_matrix[i][j]:.2f}', ha="center", va="center", color="black", fontweight='bold')
        plt.colorbar(im, ax=ax)
        # 6. Bảng tổng kết
        ax = axes[1, 2]
        ax.axis('tight')
        ax.axis('off')
        categories = ['Best Mean Error', 'Best Convergence', 'Best Steady State', 'Most Consistent']
        summary_data = []
        for mission in mission_types:
            m_metrics = all_metrics[mission]
            best_mean = min(controller_types, key=lambda x: m_metrics[x]['mean_formation_error']).value
            best_conv = min(controller_types, key=lambda x: m_metrics[x]['convergence_time']).value
            best_ss = min(controller_types, key=lambda x: m_metrics[x]['steady_state_error']).value
            best_std = min(controller_types, key=lambda x: m_metrics[x]['std_formation_error']).value
            summary_data.append([best_mean, best_conv, best_ss, best_std])
        table = ax.table(cellText=summary_data,
                         rowLabels=[m.value for m in mission_types],
                         colLabels=categories,
                         cellLoc='center', loc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(8)
        table.scale(1.0, 1.8)
        ax.set_title('Best Performers by Category', pad=20)
        plt.tight_layout()
        # Lưu hình ảnh tổng hợp
        summary_dir = os.path.join("final_results", "comprehensive_summary")
        os.makedirs(summary_dir, exist_ok=True)
        summary_path = os.path.join(summary_dir, "comprehensive_scenario_analysis.png")
        plt.savefig(summary_path, dpi=300, bbox_inches='tight')
        print(f"💾 Comprehensive summary saved: {summary_path}")
        plt.close(fig)

def generate_advanced_plots_and_data(mission_type, advanced_result):
    """Sinh biểu đồ chi tiết và lưu dữ liệu CSV cho thuật toán MADDPG+RBF ở từng kịch bản"""
    scenario_folder = mission_type.value.lower().replace(' ', '_')
    scenario_dir = os.path.join("final_results", scenario_folder)
    os.makedirs(scenario_dir, exist_ok=True)
    # Trích xuất dữ liệu từ kết quả mô phỏng MADDPG+RBF
    time = advanced_result['time']
    x1 = advanced_result['auv1_pos'][:, 0]; y1 = advanced_result['auv1_pos'][:, 1]
    x2 = advanced_result['auv2_pos'][:, 0]; y2 = advanced_result['auv2_pos'][:, 1]
    x3 = advanced_result['auv3_pos'][:, 0]; y3 = advanced_result['auv3_pos'][:, 1]
    err2 = advanced_result['formation_errors']['auv2']; err3 = advanced_result['formation_errors']['auv3']
    vel2 = advanced_result['auv2_vel']; vel3 = advanced_result['auv3_vel']
    reward2 = advanced_result['rewards']['auv2']; reward3 = advanced_result['rewards']['auv3']
    # Tính khoảng cách giữa các AUV và năng lượng điều khiển
    dist12 = np.linalg.norm(advanced_result['auv1_pos'] - advanced_result['auv2_pos'], axis=1)
    dist13 = np.linalg.norm(advanced_result['auv1_pos'] - advanced_result['auv3_pos'], axis=1)
    dist23 = np.linalg.norm(advanced_result['auv2_pos'] - advanced_result['auv3_pos'], axis=1)
    energy2 = np.linalg.norm(vel2, axis=1)**2
    energy3 = np.linalg.norm(vel3, axis=1)**2
    dt = time[1] - time[0] if len(time) > 1 else 0.05
    cumulative_energy2 = np.cumsum(energy2) * dt
    cumulative_energy3 = np.cumsum(energy3) * dt
    # Lưu dữ liệu mô phỏng vào CSV
    data_df = pd.DataFrame({
        'Time': time,
        'AUV1_X': x1, 'AUV1_Y': y1,
        'AUV2_X': x2, 'AUV2_Y': y2,
        'AUV3_X': x3, 'AUV3_Y': y3,
        'Error_AUV2': err2, 'Error_AUV3': err3,
        'Distance_12': dist12, 'Distance_13': dist13, 'Distance_23': dist23,
        'Energy_AUV2': energy2, 'Energy_AUV3': energy3,
        'Reward_AUV2': reward2, 'Reward_AUV3': reward3
    })
    csv_path = os.path.join(scenario_dir, "simulation_data.csv")
    data_df.to_csv(csv_path, index=False)
    # 1. Biểu đồ quỹ đạo
    fig1 = plt.figure(figsize=(15, 10))
    plt.plot(x1, y1, 'b-', linewidth=3, label='AUV1 (Leader)', alpha=0.8)
    plt.plot(x2, y2, 'r-', linewidth=2, label='AUV2 (Follower)', alpha=0.7)
    plt.plot(x3, y3, 'g-', linewidth=2, label='AUV3 (Follower)', alpha=0.7)
    # Đánh dấu điểm đầu và cuối
    plt.plot(x1[0], y1[0], 'go', markersize=10, label='Start', markerfacecolor='lightgreen')
    plt.plot(x1[-1], y1[-1], 'rs', markersize=10, label='End', markerfacecolor='salmon')
    # Tam giác đội hình tại một số mốc thời gian
    N = len(time)
    key_points = [0, N//4, N//2, 3*N//4, N-1]
    for i, idx in enumerate(key_points):
        alpha = 0.3 + 0.4 * i / len(key_points)
        tri_x = [x1[idx], x2[idx], x3[idx], x1[idx]]
        tri_y = [y1[idx], y2[idx], y3[idx], y1[idx]]
        plt.plot(tri_x, tri_y, 'k--', alpha=alpha, linewidth=1)
        if idx == key_points[-1]:
            plt.plot(tri_x, tri_y, 'k-', alpha=0.8, linewidth=2, label='Formation Shape')
    plt.grid(True, alpha=0.3)
    plt.xlabel('X Position (m)', fontsize=12)
    plt.ylabel('Y Position (m)', fontsize=12)
    plt.title('Quỹ đạo chuyển động Multi-AUV với MADDPG+RBF Formation Control', fontsize=14, fontweight='bold')
    plt.legend(fontsize=11)
    plt.axis('equal')
    # Thông tin cấu hình đội hình (delta và Kp)
    textstr = 'Formation Configuration:\nAUV2: Δ₁₂ = (-5, 2) m\nAUV3: Δ₁₃ = (-5, -2) m\nControl Gain: Kp = 2.0'
    plt.text(0.02, 0.98, textstr, transform=plt.gca().transAxes, fontsize=10,
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    plt.tight_layout()
    traj_path = os.path.join(scenario_dir, "1_trajectories.png")
    plt.savefig(traj_path, dpi=300, bbox_inches='tight')
    plt.close(fig1)
    # 2. Biểu đồ sai số đội hình theo thời gian và phân bố sai số
    fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 10))
    ax1.plot(time, err2, 'r-', linewidth=2, label='AUV2 Formation Error')
    ax1.plot(time, err3, 'g-', linewidth=2, label='AUV3 Formation Error')
    ax1.axhline(y=1.0, color='orange', linestyle='--', alpha=0.7, label='Acceptable Error (1m)')
    ax1.axhline(y=0.5, color='blue', linestyle='--', alpha=0.7, label='Good Error (0.5m)')
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Formation Error (m)', fontsize=12)
    ax1.set_title('Sai số đội hình theo thời gian', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=11)
    mean_errors = [np.mean(err2), np.mean(err3)]
    max_errors = [np.max(err2), np.max(err3)]
    ax1.text(0.02, 0.98,
             f'Mean Errors: AUV2={mean_errors[0]:.3f}m, AUV3={mean_errors[1]:.3f}m\n' +
             f'Max Errors: AUV2={max_errors[0]:.3f}m, AUV3={max_errors[1]:.3f}m',
             transform=ax1.transAxes, fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    ax2.hist(err2, bins=50, alpha=0.7, color='red', label='AUV2 Error Distribution', density=True)
    ax2.hist(err3, bins=50, alpha=0.7, color='green', label='AUV3 Error Distribution', density=True)
    ax2.set_xlabel('Formation Error (m)', fontsize=12)
    ax2.set_ylabel('Probability Density', fontsize=12)
    ax2.set_title('Phân bố sai số đội hình', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=11)
    plt.tight_layout()
    err_path = os.path.join(scenario_dir, "2_formation_errors.png")
    plt.savefig(err_path, dpi=300, bbox_inches='tight')
    plt.close(fig2)
    # 3. Biểu đồ khoảng cách giữa các AUV
    fig3 = plt.figure(figsize=(15, 8))
    plt.plot(time, dist12, 'b-', linewidth=2, label='Distance AUV1-AUV2')
    plt.plot(time, dist13, 'r-', linewidth=2, label='Distance AUV1-AUV3')
    plt.plot(time, dist23, 'g-', linewidth=2, label='Distance AUV2-AUV3')
    expected_d12 = math.sqrt(5**2 + 2**2)  # ≈ 5.39m
    expected_d13 = expected_d12
    expected_d23 = 4.0
    plt.axhline(y=expected_d12, color='blue', linestyle='--', alpha=0.7, label=f'Expected d₁₂ = {expected_d12:.2f}m')
    plt.axhline(y=expected_d13, color='red', linestyle='--', alpha=0.7, label=f'Expected d₁₃ = {expected_d13:.2f}m')
    plt.axhline(y=expected_d23, color='green', linestyle='--', alpha=0.7, label=f'Expected d₂₃ = {expected_d23:.2f}m')
    plt.axhline(y=2.0, color='orange', linestyle=':', linewidth=3, alpha=0.8, label=f'Collision Threshold = 2m')
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Distance (m)', fontsize=12)
    plt.title('Khoảng cách giữa các AUV trong đội hình', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=11)
    mean_distances = [np.mean(dist12), np.mean(dist13), np.mean(dist23)]
    std_distances = [np.std(dist12), np.std(dist13), np.std(dist23)]
    stats_text = (f'Mean Distances:\n'
                  f'd₁₂ = {mean_distances[0]:.3f} ± {std_distances[0]:.3f}m\n'
                  f'd₁₃ = {mean_distances[1]:.3f} ± {std_distances[1]:.3f}m\n'
                  f'd₂₃ = {mean_distances[2]:.3f} ± {std_distances[2]:.3f}m')
    plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, fontsize=10,
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
    plt.tight_layout()
    dist_path = os.path.join(scenario_dir, "3_inter_distances.png")
    plt.savefig(dist_path, dpi=300, bbox_inches='tight')
    plt.close(fig3)
    # 4. Biểu đồ năng lượng điều khiển
    fig4, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 10))
    ax1.plot(time, energy2, 'r-', linewidth=2, label='AUV2 Control Energy')
    ax1.plot(time, energy3, 'g-', linewidth=2, label='AUV3 Control Energy')
    ax1_twin = ax1.twinx()
    ax1_twin.plot(time, cumulative_energy2, 'r--', linewidth=1.5, alpha=0.7, label='AUV2 Cumulative Energy')
    ax1_twin.plot(time, cumulative_energy3, 'g--', linewidth=1.5, alpha=0.7, label='AUV3 Cumulative Energy')
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Instantaneous Energy (J/s)', fontsize=12, color='black')
    ax1_twin.set_ylabel('Cumulative Energy (J)', fontsize=12, color='gray')
    ax1.set_title('Năng lượng điều khiển theo thời gian', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax1_twin.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, fontsize=11)
    total_energy = cumulative_energy2[-1] + cumulative_energy3[-1]
    mean_formation_error = (np.mean(err2) + np.mean(err3)) / 2
    efficiency = 1.0 / (total_energy * mean_formation_error) if total_energy * mean_formation_error != 0 else 0.0
    ax1.text(0.02, 0.98,
             f'Total Energy Consumption: {total_energy:.2f}J\nEnergy Efficiency: {efficiency:.4f} 1/(J·m)',
             transform=ax1.transAxes, fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
    # Phân bổ năng lượng theo các giai đoạn nhiệm vụ
    N = len(time)
    phase1_e2 = np.sum(energy2[:int(0.25*N)])
    phase1_e3 = np.sum(energy3[:int(0.25*N)])
    phase2_e2 = np.sum(energy2[int(0.25*N):int(0.5*N)])
    phase2_e3 = np.sum(energy3[int(0.25*N):int(0.5*N)])
    phase3_e2 = np.sum(energy2[int(0.5*N):int(0.75*N)])
    phase3_e3 = np.sum(energy3[int(0.5*N):int(0.75*N)])
    phase4_e2 = np.sum(energy2[int(0.75*N):])
    phase4_e3 = np.sum(energy3[int(0.75*N):])
    phases = ['Phase 1\n(0-25%)', 'Phase 2\n(25-50%)', 'Phase 3\n(50-75%)', 'Phase 4\n(75-100%)']
    auv2_phases = [phase1_e2, phase2_e2, phase3_e2, phase4_e2]
    auv3_phases = [phase1_e3, phase2_e3, phase3_e3, phase4_e3]
    x = np.arange(len(phases))
    width = 0.35
    ax2.bar(x - width/2, auv2_phases, width, label='AUV2', color='red', alpha=0.7)
    ax2.bar(x + width/2, auv3_phases, width, label='AUV3', color='green', alpha=0.7)
    ax2.set_xlabel('Mission Phase', fontsize=12)
    ax2.set_ylabel('Energy Consumption (J)', fontsize=12)
    ax2.set_title('Năng lượng tiêu thụ theo giai đoạn nhiệm vụ', fontsize=14, fontweight='bold')
    ax2.set_xticks(x)
    ax2.set_xticklabels(phases)
    ax2.legend(fontsize=11)
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    energy_path = os.path.join(scenario_dir, "4_control_energy.png")
    plt.savefig(energy_path, dpi=300, bbox_inches='tight')
    plt.close(fig4)
    # 5. Phân tích hàm reward
    fig5, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 10))
    ax1.plot(time, reward2, 'r-', linewidth=2, label='AUV2 Reward', alpha=0.8)
    ax1.plot(time, reward3, 'g-', linewidth=2, label='AUV3 Reward', alpha=0.8)
    window = 100
    if len(reward2) < window:
        window = max(1, len(reward2) // 10)
    if window > 1:
        ma_reward2 = np.convolve(reward2, np.ones(window)/window, mode='same')
        ma_reward3 = np.convolve(reward3, np.ones(window)/window, mode='same')
        ax1.plot(time, ma_reward2, 'r--', linewidth=3, label='AUV2 Trend (Moving Average)', alpha=0.9)
        ax1.plot(time, ma_reward3, 'g--', linewidth=3, label='AUV3 Trend (Moving Average)', alpha=0.9)
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Reward Value', fontsize=12)
    ax1.set_title('MADDPG Reward Function theo thời gian', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=11)
    mean_rewards = [np.mean(reward2), np.mean(reward3)]
    std_rewards = [np.std(reward2), np.std(reward3)]
    ax1.text(0.02, 0.02,
             f'Mean Rewards:\nAUV2: {mean_rewards[0]:.6f} ± {std_rewards[0]:.6f}\n' +
             f'AUV3: {mean_rewards[1]:.6f} ± {std_rewards[1]:.6f}',
             transform=ax1.transAxes, fontsize=10, verticalalignment='bottom',
             bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.8))
    # Thành phần reward (penalty)
    avg_formation_penalty = - (np.mean(err2) + np.mean(err3)) / 2
    avg_energy_penalty = -0.1 * (np.mean(np.linalg.norm(vel2, axis=1)) + np.mean(np.linalg.norm(vel3, axis=1))) / 2
    total_avg_reward = (np.mean(reward2) + np.mean(reward3)) / 2
    components = ['Formation\nError Penalty', 'Energy\nPenalty', 'Total\nAverage Reward']
    values = [avg_formation_penalty, avg_energy_penalty, total_avg_reward]
    colors = ['red', 'blue', 'green']
    bars = ax2.bar(components, values, color=colors, alpha=0.7)
    ax2.set_ylabel('Reward Value', fontsize=12)
    ax2.set_title('Phân tích thành phần Reward Function', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    for bar, value in zip(bars, values):
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height + (0.01 if height >= 0 else -0.01),
                 f'{value:.6f}', ha='center', va='bottom' if height >= 0 else 'top', fontsize=11)
    plt.tight_layout()
    reward_path = os.path.join(scenario_dir, "5_rewards.png")
    plt.savefig(reward_path, dpi=300, bbox_inches='tight')
    plt.close(fig5)
    # 6. Biểu đồ radar hiệu suất
    fig6 = plt.figure(figsize=(6, 6))
    ax = fig6.add_subplot(111, polar=True)
    metrics = ['Accuracy', 'Stability', 'Energy\nEfficiency', 'Smoothness', 'Robustness']
    accuracy = 1 - np.mean([np.mean(err2), np.mean(err3)]) / 10.0
    stability = 1 - np.mean([np.std(err2), np.std(err3)]) / 5.0
    total_energy = cumulative_energy2[-1] + cumulative_energy3[-1]
    energy_eff = 1 - total_energy / 1000.0
    smoothness = 1 - np.mean([np.var(err2), np.var(err3)]) / 10.0
    min_distances = np.minimum(np.minimum(dist12, dist13), dist23)
    collision_count = np.sum(min_distances < 2.0)
    robustness = max(0.0, 1 - collision_count / len(time))
    values = [accuracy, stability, energy_eff, smoothness, robustness]
    values = [max(0, min(1, v)) for v in values]
    angles = [n / float(len(metrics)) * 2 * math.pi for n in range(len(metrics))]
    angles += angles[:1]
    values += values[:1]
    ax.plot(angles, values, 'o-', linewidth=2, label='MADDPG+RBF', color='blue')
    ax.fill(angles, values, alpha=0.25, color='blue')
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(metrics, fontsize=9)
    ax.set_ylim(0, 1)
    ax.set_yticks([0.2, 0.4, 0.6, 0.8, 1.0])
    ax.set_title('Radar Hiệu suất Thuật toán', fontsize=12, fontweight='bold')
    ax.grid(True)
    radar_path = os.path.join(scenario_dir, "6_performance_radar.png")
    plt.savefig(radar_path, dpi=300, bbox_inches='tight')
    plt.close(fig6)
    # Animation quá trình di chuyển của các AUV
    fig_anim, ax_anim = plt.subplots(figsize=(12, 8))
    ax_anim.set_xlim(np.min(x1) - 10, np.max(x1) + 10)
    ax_anim.set_ylim(np.min(y1) - 10, np.max(y1) + 10)
    ax_anim.set_aspect('equal')
    ax_anim.grid(True, alpha=0.3)
    ax_anim.set_xlabel('X Position (m)', fontsize=12)
    ax_anim.set_ylabel('Y Position (m)', fontsize=12)
    ax_anim.set_title(f'MADDPG+RBF Formation Control - {mission_type.value} (Animation)', fontsize=14, fontweight='bold')
    line1, = ax_anim.plot([], [], 'b-', linewidth=2, alpha=0.5, label='AUV1 Path')
    line2, = ax_anim.plot([], [], 'r-', linewidth=2, alpha=0.5, label='AUV2 Path')
    line3, = ax_anim.plot([], [], 'g-', linewidth=2, alpha=0.5, label='AUV3 Path')
    point1, = ax_anim.plot([], [], 'bo', markersize=8, label='AUV1 (Leader)')
    point2, = ax_anim.plot([], [], 'ro', markersize=8, label='AUV2 (Follower)')
    point3, = ax_anim.plot([], [], 'go', markersize=8, label='AUV3 (Follower)')
    formation_line, = ax_anim.plot([], [], 'k--', linewidth=1, alpha=0.7)
    time_text = ax_anim.text(0.02, 0.98, '', transform=ax_anim.transAxes, fontsize=12,
                             bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    error_text = ax_anim.text(0.02, 0.88, '', transform=ax_anim.transAxes, fontsize=10,
                              bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    ax_anim.legend(loc='upper right', fontsize=10)
    skip = 5
    N_frames = len(time) // skip if len(time) // skip > 0 else 1
    def animate(frame):
        idx = frame * skip
        if idx >= len(time):
            idx = len(time) - 1
        line1.set_data(x1[:idx+1], y1[:idx+1])
        line2.set_data(x2[:idx+1], y2[:idx+1])
        line3.set_data(x3[:idx+1], y3[:idx+1])
        point1.set_data([x1[idx]], [y1[idx]])
        point2.set_data([x2[idx]], [y2[idx]])
        point3.set_data([x3[idx]], [y3[idx]])
        formation_line.set_data([x1[idx], x2[idx], x3[idx], x1[idx]], [y1[idx], y2[idx], y3[idx], y1[idx]])
        time_text.set_text(f'Time: {time[idx]:.1f}s')
        error_text.set_text(f'Formation Errors:\nAUV2: {err2[idx]:.2f}m\nAUV3: {err3[idx]:.2f}m')
        return line1, line2, line3, point1, point2, point3, formation_line, time_text, error_text
    anim = FuncAnimation(fig_anim, animate, frames=N_frames, interval=50, blit=False, repeat=True)
    
    try:
        gif_path = os.path.join(scenario_dir, "formation_animation.gif")
        anim.save(gif_path, writer='pillow', fps=20)
        print(f"💾 Animation saved: {gif_path}")
    except Exception as e:
        print(f"⚠️  Could not save animation: {e}")
    
    plt.close(fig_anim)

# Thực thi chính
if os.path.exists("final_results"):
    shutil.rmtree("final_results")
os.makedirs("final_results", exist_ok=True)

if __name__ == "__main__":
    analyzer = ScenarioAnalyzer()
    print("🌊 MADDPG+RBF Formation Control - Simulation and Analysis")
    print("Running simulations for Basic Maneuvers, Waypoint Trajectory, and Sine Wave scenarios...")
    all_results, all_metrics = analyzer.run_comprehensive_analysis()
    print("\n📁 All results have been saved in the 'final_results' directory.")
    print("Each scenario has its own subfolder with charts (PNG), data (CSV), and animation (GIF).")
    print("A comprehensive summary is available in final_results/comprehensive_summary.\n")
