import numpy as np
from typing import List, Tuple
import time

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, output_limits: Tuple[float, float]):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
    def compute(self, error: float) -> float:
        # 计算时间间隔
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 计算积分项
        self.integral += error * dt
        
        # 计算微分项
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        
        # 计算输出
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        # 限制输出范围
        output = np.clip(output, *self.output_limits)
        
        return output
    
    def reset(self):
        """重置控制器状态"""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

class Controller:
    def __init__(self, config: dict):
        # 创建速度和转向的PID控制器
        self.speed_pid = PIDController(
            config['controller']['speed_pid']['kp'],
            config['controller']['speed_pid']['ki'],
            config['controller']['speed_pid']['kd'],
            (-config['controller']['max_speed'], config['controller']['max_speed'])
        )
        
        self.steering_pid = PIDController(
            config['controller']['steering_pid']['kp'],
            config['controller']['steering_pid']['ki'],
            config['controller']['steering_pid']['kd'],
            (-config['controller']['max_steering'], config['controller']['max_steering'])
        )
        
        self.max_speed = config['controller']['max_speed']
        self.max_steering = config['controller']['max_steering']
        
        # 更新避障参数
        self.obstacle_influence = config['controller']['obstacle_avoidance']['influence_distance']
        self.safe_distance = config['controller']['obstacle_avoidance']['safe_distance']
        self.obstacle_gain = config['controller']['obstacle_avoidance']['obstacle_gain']
        self.target_gain = config['controller']['obstacle_avoidance']['target_gain']
        self.multi_obstacle_factor = config['controller']['obstacle_avoidance']['multi_obstacle_factor']
        
        # 添加路径跟踪增益
        self.path_following_gain = 2.0  # 增加路径跟踪的权重
        
    def compute_control(self, 
                       current_pos: List[float],
                       current_ori: List[float],
                       target_pos: Tuple[float, float],
                       lidar_data: List[float] = None) -> Tuple[float, float]:
        """计算控制输入（包含避障）"""
        # 计算到目标的矢量
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        distance = np.sqrt(dx**2 + dy**2)
        
        # 计算目标方向
        target_angle = np.arctan2(dy, dx)
        current_angle = current_ori[2]  # yaw angle
        
        if lidar_data:
            # 分析障碍物情况
            obstacle_analysis = self._analyze_obstacles(lidar_data, current_angle)
            
            if obstacle_analysis['num_zones'] > 0:
                # 计算避障向量
                avoid_x, avoid_y = self._compute_avoidance_vector(obstacle_analysis, current_angle)
                avoid_angle = np.arctan2(avoid_y, avoid_x) if (avoid_x != 0 or avoid_y != 0) else current_angle
                
                # 根据障碍物情况调整权重
                obstacle_weight = self._compute_obstacle_weight(
                    obstacle_analysis['min_distance'],
                    obstacle_analysis['num_zones']
                )
                path_weight = self._compute_path_weight(distance)
                
                # 确保权重和为1
                total_weight = obstacle_weight + path_weight
                obstacle_weight /= total_weight
                path_weight /= total_weight
                
                # 合并避障方向和目标方向
                desired_angle = (obstacle_weight * avoid_angle + 
                               path_weight * target_angle)
                
                # 计算速度因子
                speed_factor = self._compute_speed_factor(
                    obstacle_analysis['min_distance'],
                    distance,
                    obstacle_analysis['critical']
                )
            else:
                desired_angle = target_angle
                speed_factor = 1.0
        else:
            desired_angle = target_angle
            speed_factor = 1.0
        
        # 计算转向角误差
        angle_error = np.arctan2(np.sin(desired_angle - current_angle), 
                                np.cos(desired_angle - current_angle))
        
        # 使用PID控制器计算速度和转向
        speed = self.speed_pid.compute(distance) * speed_factor
        steering = self.steering_pid.compute(angle_error)
        
        return speed, steering
    
    def _analyze_obstacles(self, lidar_data: List[float], current_angle: float) -> dict:
        """分析障碍物分布情况"""
        num_rays = len(lidar_data)
        angle_step = 2 * np.pi / num_rays
        
        # 检测危险区域
        danger_zones = []
        current_zone = None
        
        for i, distance in enumerate(lidar_data):
            angle = current_angle + i * angle_step
            
            if distance < self.obstacle_influence:
                if current_zone is None:
                    current_zone = {'start_idx': i, 'min_distance': distance, 'angles': [angle]}
                else:
                    current_zone['angles'].append(angle)
                    current_zone['min_distance'] = min(current_zone['min_distance'], distance)
            elif current_zone is not None:
                current_zone['end_idx'] = i - 1
                danger_zones.append(current_zone)
                current_zone = None
        
        # 处理跨越360度的情况
        if current_zone is not None:
            if len(danger_zones) > 0 and danger_zones[0]['start_idx'] == 0:
                # 合并首尾区域
                danger_zones[0]['angles'].extend(current_zone['angles'])
                danger_zones[0]['min_distance'] = min(danger_zones[0]['min_distance'], 
                                                    current_zone['min_distance'])
            else:
                current_zone['end_idx'] = num_rays - 1
                danger_zones.append(current_zone)
        
        return {
            'zones': danger_zones,
            'num_zones': len(danger_zones),
            'min_distance': min(lidar_data),
            'critical': any(zone['min_distance'] < self.safe_distance for zone in danger_zones)
        }
    
    def _compute_avoidance_vector(self, obstacle_analysis: dict, current_angle: float) -> Tuple[float, float]:
        """计算避障向量"""
        if not obstacle_analysis['zones']:
            return 0, 0
            
        avoid_x = 0
        avoid_y = 0
        
        for zone in obstacle_analysis['zones']:
            # 计算该区域的中心角度
            zone_angles = zone['angles']
            mean_angle = np.mean(zone_angles)
            
            # 计算避障力
            force = self._compute_repulsive_force(zone['min_distance'])
            
            # 多障碍物情况下增加力度
            if obstacle_analysis['num_zones'] > 1:
                force *= self.multi_obstacle_factor
            
            # 计算避障方向
            avoid_x -= force * np.cos(mean_angle)
            avoid_y -= force * np.sin(mean_angle)
        
        return avoid_x, avoid_y
    
    def _compute_path_weight(self, target_distance: float) -> float:
        """计算路径跟踪权重"""
        # 距离目标越近，路径跟踪权重越大
        base_weight = self.path_following_gain
        distance_factor = np.clip(1.0 - target_distance / 5.0, 0.2, 1.0)
        return base_weight * distance_factor
    
    def _compute_speed_factor(self, 
                            obstacle_distance: float, 
                            target_distance: float,
                            is_critical: bool) -> float:
        """计算速度因子"""
        # 在危险情况下显著降低速度
        if is_critical:
            return 0.2
        
        # 基于障碍物距离的速度因子
        obstacle_factor = np.clip(obstacle_distance / self.obstacle_influence, 0.2, 1.0)
        
        # 基于目标距离的速度因子
        target_factor = np.clip(target_distance / 2.0, 0.5, 1.0)
        
        return min(obstacle_factor, target_factor)
    
    def _compute_obstacle_weight(self, min_distance: float, num_zones: int) -> float:
        """计算障碍物权重"""
        if min_distance >= self.obstacle_influence:
            return 0.0
        
        # 基础权重
        base_weight = 1.0 / (1.0 + np.exp(5 * (min_distance / self.obstacle_influence - 0.5)))
        
        # 多障碍物情况下增加权重
        if num_zones > 1:
            base_weight *= (1 + (num_zones - 1) * 0.2)
        
        return np.clip(base_weight, 0, 1)
    
    def _compute_repulsive_force(self, distance: float) -> float:
        """计算斥力大小"""
        if distance >= self.obstacle_influence:
            return 0.0
        
        # 使用平滑的指数衰减
        normalized_distance = distance / self.obstacle_influence
        return self.obstacle_gain * np.exp(-5 * normalized_distance)
    
    def reset(self):
        """重置控制器状态"""
        self.speed_pid.reset()
        self.steering_pid.reset()