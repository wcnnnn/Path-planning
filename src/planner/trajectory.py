import numpy as np
from typing import List, Tuple

class TrajectoryPlanner:
    def __init__(self, config: dict):
        self.config = config
        self.trajectory_type = config['trajectory']['type']
        self.trajectory_points = []
        
    def generate_trajectory(self) -> List[Tuple[float, float]]:
        """生成轨迹点"""
        if self.trajectory_type == "circle":
            return self._generate_circle_trajectory()
        elif self.trajectory_type == "square":
            return self._generate_square_trajectory()
        elif self.trajectory_type == "eight":
            return self._generate_figure_eight_trajectory()
        elif self.trajectory_type == "spiral":
            return self._generate_spiral_trajectory()
        elif self.trajectory_type == "custom":
            return self.config['trajectory']['points']
        else:
            raise ValueError(f"Unknown trajectory type: {self.trajectory_type}")
    
    def _generate_circle_trajectory(self) -> List[Tuple[float, float]]:
        """生成圆形轨迹"""
        radius = self.config['trajectory']['radius']
        points = []
        for theta in np.linspace(0, 2*np.pi, 100):
            x = radius * np.cos(theta)
            y = radius * np.sin(theta)
            points.append((x, y))
        return points
    
    def _generate_square_trajectory(self) -> List[Tuple[float, float]]:
        """生成方形轨迹"""
        size = self.config['trajectory']['radius']
        points = []
        # 生成正方形的四个边
        for x in np.linspace(-size, size, 25):
            points.append((x, -size))
        for y in np.linspace(-size, size, 25):
            points.append((size, y))
        for x in np.linspace(size, -size, 25):
            points.append((x, size))
        for y in np.linspace(size, -size, 25):
            points.append((-size, y))
        return points 
    
    def _generate_figure_eight_trajectory(self) -> List[Tuple[float, float]]:
        """生成8字形轨迹"""
        points = []
        radius = self.config['trajectory']['radius']
        num_points = 100
        
        for t in np.linspace(0, 2*np.pi, num_points):
            x = radius * np.sin(t)
            y = radius * np.sin(t) * np.cos(t)
            points.append((x, y))
        
        return points
    
    def _generate_spiral_trajectory(self) -> List[Tuple[float, float]]:
        """生成螺旋形轨迹"""
        points = []
        radius = self.config['trajectory']['radius']
        num_points = 100
        
        for t in np.linspace(0, 6*np.pi, num_points):
            r = radius * t / (6*np.pi)
            x = r * np.cos(t)
            y = r * np.sin(t)
            points.append((x, y))
        
        return points