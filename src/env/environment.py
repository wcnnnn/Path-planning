import pybullet as p
import numpy as np
from typing import List, Tuple, Dict
import random

class Environment:
    def __init__(self, config: dict):
        self.config = config
        self.obstacles = []  # 存储障碍物ID
        self.sensors = {}    # 存储传感器
        
    def add_random_obstacles(self, num_obstacles: int):
        """添加随机动态障碍物"""
        for _ in range(num_obstacles):
            size = [random.uniform(0.3, 0.4) for _ in range(3)]  # 减小障碍物尺寸
            
            # 确保障碍物初始位置不会太靠近小车
            while True:
                position = [
                    random.uniform(-5, 5),
                    random.uniform(-5, 5),
                    size[2]/2
                ]
                # 检查是否离原点（小车初始位置）太近
                if np.sqrt(position[0]**2 + position[1]**2) > 2.0:
                    break
            
            collision_box = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=size
            )
            
            visual_box = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=size,
                rgbaColor=[0.8, 0.2, 0.2, 0.8]  # 红色半透明
            )
            
            obstacle_id = p.createMultiBody(
                baseMass=1,
                baseCollisionShapeIndex=collision_box,
                baseVisualShapeIndex=visual_box,
                basePosition=position
            )
            
            self.obstacles.append({
                'id': obstacle_id,
                'velocity': [random.uniform(-self.config['environment']['obstacle_speed'],
                                         self.config['environment']['obstacle_speed']), 
                            random.uniform(-self.config['environment']['obstacle_speed'],
                                         self.config['environment']['obstacle_speed']), 
                            0],
                'bounds': {'min': -5, 'max': 5}
            })
    
    def update_obstacles(self):
        """更新障碍物位置"""
        for obstacle in self.obstacles:
            pos, _ = p.getBasePositionAndOrientation(obstacle['id'])
            new_pos = [
                pos[0] + obstacle['velocity'][0],
                pos[1] + obstacle['velocity'][1],
                pos[2]
            ]
            
            # 碰到边界就反弹
            for i in range(2):
                if new_pos[i] > obstacle['bounds']['max'] or \
                   new_pos[i] < obstacle['bounds']['min']:
                    obstacle['velocity'][i] *= -1
                    new_pos[i] = pos[i]
            
            p.resetBasePositionAndOrientation(
                obstacle['id'],
                new_pos,
                p.getQuaternionFromEuler([0, 0, 0])
            )
    
    def add_lidar_sensor(self, robot_id: int, num_rays: int = 16):
        """添加激光雷达传感器"""
        self.sensors['lidar'] = {
            'robot_id': robot_id,
            'num_rays': num_rays,
            'max_distance': 5.0,
            'debug_lines': []
        }
    
    def get_lidar_data(self) -> List[float]:
        """获取激光雷达数据"""
        if 'lidar' not in self.sensors:
            return []
            
        sensor = self.sensors['lidar']
        robot_pos, robot_ori = p.getBasePositionAndOrientation(sensor['robot_id'])
        
        # 清除旧的调试线
        for line in sensor['debug_lines']:
            p.removeUserDebugItem(line)
        sensor['debug_lines'].clear()
        
        distances = []
        angle_step = 2 * np.pi / sensor['num_rays']
        
        for i in range(sensor['num_rays']):
            angle = i * angle_step
            ray_dir = [
                np.cos(angle),
                np.sin(angle),
                0
            ]
            
            end_pos = [
                robot_pos[0] + ray_dir[0] * sensor['max_distance'],
                robot_pos[1] + ray_dir[1] * sensor['max_distance'],
                robot_pos[2]
            ]
            
            result = p.rayTest(robot_pos, end_pos)[0]
            hit_distance = result[2] * sensor['max_distance']
            distances.append(hit_distance)
            
            # 可视化射线
            color = [1, 0, 0] if result[0] != -1 else [0, 1, 0]
            line_id = p.addUserDebugLine(
                robot_pos,
                [robot_pos[0] + ray_dir[0] * hit_distance,
                 robot_pos[1] + ray_dir[1] * hit_distance,
                 robot_pos[2]],
                color,
                lifeTime=0.1
            )
            sensor['debug_lines'].append(line_id)
        
        return distances
    
    def add_environmental_forces(self, robot_id: int):
        """添加环境干扰力"""
        # 模拟风力或其他外部干扰
        force_magnitude = random.uniform(0, 10)
        force_direction = [
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            0
        ]
        
        p.applyExternalForce(
            robot_id,
            -1,  # 作用于基座
            [force_magnitude * d for d in force_direction],
            [0, 0, 0],  # 作用点（相对于质心）
            p.WORLD_FRAME
        ) 