import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import numpy as np
from typing import List, Tuple
import pybullet as p
import matplotlib.font_manager as fm
import os
import warnings
warnings.filterwarnings('ignore')
# 字体文件夹路径
font_dir = r'C:\Users\30766\anaconda3\envs\Tensorflow\Lib\site-packages\matplotlib\mpl-data\fonts\ttf'
# 获取字体文件夹中的所有字体文件
font_files = fm.findSystemFonts(fontpaths=[font_dir])
# 将字体文件添加到字体管理器中
for font_file in font_files:
    fm.fontManager.addfont(font_file)
# 手动创建 FontProperties 实例来获取字体名称
fonts = [fm.FontProperties(fname=f).get_name() for f in font_files]
import matplotlib.pyplot as plt
import scienceplots  
# 加载样式
plt.style.use(['science', 'bright','no-latex','grid','cjk-sc-font'])

class Visualizer:
    def __init__(self, config: dict):
        # 创建Tkinter窗口
        self.root = tk.Tk()
        self.root.title("Robot Simulation Metrics")
        
        # 创建图表
        self.fig = Figure(figsize=(12, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        
        # 创建子图
        self.error_plot = self.fig.add_subplot(311)
        self.speed_plot = self.fig.add_subplot(312)
        self.steering_plot = self.fig.add_subplot(313)
        
        # 初始化数据
        self.max_points = config['visualization']['max_data_points']
        self.times = []
        self.position_errors = []
        self.speeds = []
        self.steerings = []
        
        # 配置图表
        self._setup_plots()
        
        # 显示图表
        self.canvas.draw()
        self.canvas.get_tk_widget().pack()
        
        # 误差显示比例
        self.error_scale = config['visualization']['error_scale']
        
    def _setup_plots(self):
        """配置图表样式"""
        self.error_plot.set_title('Position Error')
        self.error_plot.set_ylabel('Error (m)')
        
        self.speed_plot.set_title('Speed')
        self.speed_plot.set_ylabel('Speed (m/s)')
        
        self.steering_plot.set_title('Steering Angle')
        self.steering_plot.set_ylabel('Angle (rad)')
        self.steering_plot.set_xlabel('Time (s)')
        
    def update(self, 
              current_time: float,
              position_error: float,
              speed: float,
              steering: float):
        """更新图表数据"""
        # 更新数据列表
        self.times.append(current_time)
        self.position_errors.append(position_error)
        self.speeds.append(speed)
        self.steerings.append(steering)
        
        # 保持数据点数量在限制范围内
        if len(self.times) > self.max_points:
            self.times.pop(0)
            self.position_errors.pop(0)
            self.speeds.pop(0)
            self.steerings.pop(0)
        
        # 更新图表
        self.error_plot.clear()
        self.speed_plot.clear()
        self.steering_plot.clear()
        
        self._setup_plots()
        
        self.error_plot.plot(self.times, self.position_errors)
        self.speed_plot.plot(self.times, self.speeds)
        self.steering_plot.plot(self.times, self.steerings)
        
        self.fig.tight_layout()
        self.canvas.draw()
        
        # 处理Tkinter事件
        self.root.update()
    
    def show_error_marker(self, 
                         current_pos: List[float],
                         target_pos: Tuple[float, float]):
        """显示误差标记"""
        # 绘制从当前位置到目标位置的线
        p.addUserDebugLine(
            [current_pos[0], current_pos[1], 0.1],
            [target_pos[0], target_pos[1], 0.1],
            [1, 0, 0],  # 红色
            1.0,
            lifeTime=0.1
        ) 