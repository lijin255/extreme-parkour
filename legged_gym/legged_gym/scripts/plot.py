
import cv2
import os

from isaacgym import gymapi
import numpy as np
import torch
import matplotlib.pyplot as plt
from PIL import Image as im
from rsl_rl.env.vec_env import VecEnv
def plot_debug(env:VecEnv):
    # 初始化实时绘图
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 6))
    base_line, = ax.plot([], [], label='base_height', color='blue')
    cmd_line, = ax.plot([], [], '--', label='command_height', color='orange')
    ax.set_xlabel('timestamp')
    ax.set_ylabel('m')
    ax.set_title('base height track')
    ax.legend(loc='upper right')
    ax.grid(True)
    
    # 数据缓存
    data = {
        'timesteps': [],
        'base_heights': [],
        'command_heights': []
    }
    
    def update_plot():
        # 从VecEnv获取最新数据（支持多环境）
        commands = env.commands[:, 4].cpu().numpy()           # 所有环境的指令高度
        root_states = env.root_states[:, 2].cpu().numpy()     # 所有环境的根高度
        measured_heights = env.measured_heights.cpu().numpy()# 所有环境的测量高度
        
        # 计算实际高度（假设单个环境）
        base_height = np.mean(root_states[0] - measured_heights[0])
        
        # 更新数据
        data['timesteps'].append(len(data['timesteps']))
        data['base_heights'].append(base_height)
        data['command_heights'].append(commands[0])
        
        # 更新曲线数据
        base_line.set_data(data['timesteps'], data['base_heights'])
        cmd_line.set_data(data['timesteps'], data['command_heights'])
        
        # 自动缩放视图
        ax.relim()
        ax.autoscale_view()
        
        # 渲染更新
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.001)
        
        return fig  # 返回figure对象以便外部控制
    
    return update_plot  # 返回更新函数