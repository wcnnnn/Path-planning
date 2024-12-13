import pybullet as p
import pybullet_data
import time
import yaml
import os
import numpy as np
from src.robot.car import Car
from src.robot.controller import Controller
from src.planner.trajectory import TrajectoryPlanner
from src.utils.visualization import Visualizer
from src.env.environment import Environment

def load_config(config_path: str) -> dict:
    """加载配置文件"""
    # 获取当前文件的目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # 构建配置文件的完整路径
    config_file = os.path.join(current_dir, config_path)
    
    with open(config_file, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def main():
    # 加载配置
    config = load_config('config/config.yaml')
    
    # 初始化物理引擎
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    # 设置摄像机
    p.resetDebugVisualizerCamera(
        cameraDistance=10.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0]
    )
    
    # 加载地面
    p.loadURDF("plane.urdf")
    
    # 创建小车
    car = Car(config)
    car.create()
    
    # 创建轨迹规划器和控制器
    planner = TrajectoryPlanner(config)
    controller = Controller(config)
    
    # 生成轨迹
    trajectory = planner.generate_trajectory()
    current_target_index = 0
    
    # 可视化轨迹
    for i in range(len(trajectory)-1):
        p.addUserDebugLine(
            [trajectory[i][0], trajectory[i][1], 0.1],
            [trajectory[i+1][0], trajectory[i+1][1], 0.1],
            [0, 1, 0]
        )
    
    # 创建调试文本ID
    debug_text_id = p.addUserDebugText("", [0, 0, 2], [1, 1, 1])
    
    # 创建可视化器
    visualizer = Visualizer(config)
    
    # 记录开始时间
    start_time = time.time()
    
    # 创建环境
    env = Environment(config)
    
    # 添加动态障碍物
    env.add_random_obstacles(config['environment']['num_obstacles'])
    
    # 添加传感器
    env.add_lidar_sensor(car.robot_id, config['environment']['lidar_rays'])
    
    # 主循环
    while True:
        current_time = time.time() - start_time
        
        # 更新障碍物位置
        env.update_obstacles()
        
        # 获取传感器数据
        lidar_data = env.get_lidar_data()
        
        # 添加环境干扰
        env.add_environmental_forces(car.robot_id)
        
        # 获取当前状态
        pos, ori = car.get_state()
        
        # 获取目标点
        target = trajectory[current_target_index]
        
        # 计算到目标点的距离
        distance = ((pos[0] - target[0])**2 + (pos[1] - target[1])**2)**0.5
        
        # 如果达到当前目标点，切换到下一个目标点
        if distance < 0.5:
            current_target_index = (current_target_index + 1) % len(trajectory)
        
        # 计算控制输入
        speed, steering = controller.compute_control(
            pos, 
            ori, 
            target,
            lidar_data  # 传入激光雷达数据
        )
        
        # 更新小车状态
        new_pos = [
            pos[0] + speed * 0.01 * np.cos(ori[2] + steering),
            pos[1] + speed * 0.01 * np.sin(ori[2] + steering),
            pos[2]
        ]
        new_ori = p.getQuaternionFromEuler([0, 0, ori[2] + steering * 0.1])
        car.set_state(new_pos, new_ori)
        
        # 计算位置误差
        position_error = np.sqrt(
            (pos[0] - target[0])**2 + 
            (pos[1] - target[1])**2
        )
        
        # 更新可视化
        visualizer.update(
            current_time,
            position_error,
            speed,
            steering
        )
        
        # 显示误差标记
        visualizer.show_error_marker(pos, target)
        
        # 显示调试信息
        closest_obstacle = min(lidar_data) if lidar_data else float('inf')
        debug_text = (f"Target: {current_target_index}\n"
                     f"Speed: {speed:.1f}\n"
                     f"Steering: {steering:.2f}\n"
                     f"Closest Obstacle: {closest_obstacle:.2f}")
        p.addUserDebugText(
            debug_text,
            [0, 0, 2],
            [1, 1, 1],
            replaceItemUniqueId=debug_text_id
        )
        
        p.stepSimulation()
        time.sleep(1./240.)

if __name__ == "__main__":
    main() 