import pybullet as p
import numpy as np
from typing import List, Tuple

class Car:
    def __init__(self, config: dict):
        """初始化小车类
        
        Args:
            config (dict): 配置参数字典
        """
        self.length = config['robot']['length']
        self.width = config['robot']['width']
        self.height = config['robot']['height']
        self.mass = config['robot']['mass']
        
        self.position = [0, 0, self.height/2]
        self.orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robot_id = None
        
    def create(self) -> int:
        """创建小车实体
        
        Returns:
            int: 小车的唯一ID
        """
        # 创建碰撞形状
        collision_box = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[self.length/2, self.width/2, self.height/2]
        )
        
        # 创建视觉形状
        visual_box = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[self.length/2, self.width/2, self.height/2],
            rgbaColor=[1, 0, 0, 1]  # 红色
        )
        
        # 创建多体
        self.robot_id = p.createMultiBody(
            baseMass=self.mass,
            baseCollisionShapeIndex=collision_box,
            baseVisualShapeIndex=visual_box,
            basePosition=self.position,
            baseOrientation=self.orientation
        )
        
        return self.robot_id
    
    def get_state(self) -> Tuple[List[float], List[float]]:
        """获取小车当前状态
        
        Returns:
            Tuple[List[float], List[float]]: (位置, 方向)
        """
        if self.robot_id is None:
            raise ValueError("Car not created yet. Call create() first.")
            
        pos, ori = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(ori)
        return list(pos), list(euler)
    
    def set_state(self, position: List[float], orientation: List[float]) -> None:
        """设置小车状态
        
        Args:
            position (List[float]): [x, y, z]位置
            orientation (List[float]): [roll, pitch, yaw]方向或四元数
        """
        if self.robot_id is None:
            raise ValueError("Car not created yet. Call create() first.")
            
        # 如果输入的是欧拉角，转换为四元数
        if len(orientation) == 3:
            orientation = p.getQuaternionFromEuler(orientation)
            
        p.resetBasePositionAndOrientation(self.robot_id, position, orientation) 