# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
from action_provider.action_base import ActionProvider
from typing import Optional
import torch
from typing import Callable


def create_trajectory_generator():
    """创建轨迹生成器"""
    def trajectory_generator(env, step_count):
        """生成动态双臂轨迹 - 挥手动作"""
        all_joint_names = env.scene["robot"].data.joint_names
        full_action = torch.zeros(len(all_joint_names), device=env.device)
        
        # 获取仿真时间
        current_time = env.sim.current_time
        
        # 运动参数
        period = 3.0  # 周期3秒
        amplitude = 0.8  # 增大幅度到0.8弧度 (约45度)
        
        # 计算时间相关的运动
        t = torch.tensor(current_time, device=env.device)
        
        # 定义手臂关节映射
        arm_joint_mapping = {
            "left_shoulder_pitch_joint": 0,
            "left_shoulder_roll_joint": 1, 
            "left_shoulder_yaw_joint": 2,
            "left_elbow_joint": 3,
            "left_wrist_roll_joint": 4,
            "left_wrist_pitch_joint": 5,
            "left_wrist_yaw_joint": 6,
            "right_shoulder_pitch_joint": 7,
            "right_shoulder_roll_joint": 8,
            "right_shoulder_yaw_joint": 9,
            "right_elbow_joint": 10,
            "right_wrist_roll_joint": 11,
            "right_wrist_pitch_joint": 12,
            "right_wrist_yaw_joint": 13
        }
        
        # 生成动态角度
        sin_wave = amplitude * torch.sin(2 * torch.pi * t / period)
        cos_wave = amplitude * torch.cos(2 * torch.pi * t / period)
        
        # 左臂挥手动作
        left_angles = [
            0.5 + sin_wave,     # left_shoulder_pitch_joint - 前后摆动
            0.8 + cos_wave,     # left_shoulder_roll_joint - 上下挥动
            0.3 * sin_wave,     # left_shoulder_yaw_joint - 轻微旋转
            1.0 + 0.5 * cos_wave, # left_elbow_joint - 肘部弯曲
            0.0,                # left_wrist_roll_joint
            0.2 * sin_wave,     # left_wrist_pitch_joint - 手腕摆动
            0.0,                # left_wrist_yaw_joint
        ]
        
        # 右臂相反方向的挥手动作
        right_angles = [
            0.5 - sin_wave,     # right_shoulder_pitch_joint - 相反方向
            0.8 - cos_wave,     # right_shoulder_roll_joint - 相反方向
            -0.3 * sin_wave,    # right_shoulder_yaw_joint - 相反旋转
            1.0 + 0.5 * sin_wave, # right_elbow_joint - 不同相位
            0.0,                # right_wrist_roll_joint
            -0.2 * sin_wave,    # right_wrist_pitch_joint - 相反摆动
            0.0,                # right_wrist_yaw_joint
        ]
        
        # 合并左右臂角度
        target_angles = left_angles + right_angles
        
        # 填充关节角度
        for i, name in enumerate(all_joint_names):
            if name in arm_joint_mapping:
                idx = arm_joint_mapping[name]
                if idx < len(target_angles):
                    full_action[i] = target_angles[idx]
        
        return full_action.unsqueeze(0)
    
    return trajectory_generator

class TrajectoryActionProvider(ActionProvider):
    """基于轨迹生成的Action提供者"""
    
    def __init__(self, trajectory_generator: Callable):
        super().__init__("TrajectoryActionProvider")
        self.trajectory_generator = trajectory_generator
        self.step_count = 0
    
    def get_action(self, env) -> Optional[torch.Tensor]:
        """从轨迹生成器获取action"""
        try:
            action = self.trajectory_generator(env, self.step_count)
            self.step_count += 1
            return action
        except Exception as e:
            print(f"[{self.name}] 获取轨迹action失败: {e}")
            return None