# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
gripper state
"""      

from __future__ import annotations

import torch
from typing import TYPE_CHECKING
import sys
import os
if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


import torch

def get_robot_girl_joint_names() -> list[str]:
    return [
        "R_pinky_proximal_joint",
        "R_ring_proximal_joint",
        "R_middle_proximal_joint",
        "R_index_proximal_joint",
        "R_thumb_proximal_pitch_joint",
        "R_thumb_proximal_yaw_joint",
        "L_pinky_proximal_joint",
        "L_ring_proximal_joint",
        "L_middle_proximal_joint",
        "L_index_proximal_joint",
        "L_thumb_proximal_pitch_joint",
        "L_thumb_proximal_yaw_joint",
    ]

# global variable to cache the DDS instance
_gripper_dds = None
_dds_initialized = False

def _get_inspire_dds_instance():
    """get the DDS instance, delay initialization"""
    global _inspire_dds, _dds_initialized
    
    if not _dds_initialized:
        try:
            # dynamically import the DDS module
            sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), 'dds'))
            from dds.inspire_dds    import get_inspire_dds
            
            _inspire_dds = get_inspire_dds()
            _inspire_dds.start_communication(enable_publish=True, enable_subscribe=False)
            print("[Observations] DDS communication instance obtained")
            
            # register the cleanup function
            import atexit
            def cleanup_dds():
                try:
                    if _inspire_dds:
                        _inspire_dds.stop_communication()
                        _inspire_dds.cleanup()
                        print("[gripper_state] DDS communication closed correctly")
                except Exception as e:
                    print(f"[gripper_state] Error closing DDS: {e}")
            atexit.register(cleanup_dds)
            
        except Exception as e:
            print(f"[Observations] Failed to get DDS instances: {e}")
            _inspire_dds = None
        
        _dds_initialized = True
    
    return _inspire_dds

def initialize_inspire_dds():
    """explicitly initialize the DDS communication
    
    this function can be called manually to initialize the DDS communication,
    instead of relying on delayed initialization
    """
    return _get_inspire_dds_instance()


def get_robot_inspire_joint_states(
    env: ManagerBasedRLEnv,
    enable_dds: bool = True,
) -> torch.Tensor:
    """get the robot gripper joint states and publish them to DDS
    
    Args:
        env: ManagerBasedRLEnv - reinforcement learning environment instance
        enable_dds: bool - whether to enable the DDS publish function
    
    返回:
        torch.Tensor
    """
    # get the gripper joint states
    joint_pos = env.scene["robot"].data.joint_pos
    joint_vel = env.scene["robot"].data.joint_vel  
    joint_torque = env.scene["robot"].data.applied_torque
    
    # get the gripper joint indices (last 2 joints)
    # gripper_joint_names = get_robot_girl_joint_names()
    # all_joint_names = env.scene["robot"].data.joint_names
    # # print(f"all_joint_names: {all_joint_names}")
    # gripper_joint_indices = [all_joint_names.index(name) for name in gripper_joint_names if name in all_joint_names]
    # print(f"gripper_joint_indices: {gripper_joint_indices}")
    inspire_joint_indices = [36, 37, 35, 34, 48, 38, 31, 32, 30, 29, 43, 33]
    if len(inspire_joint_indices) >= 12:
        # extract the gripper joint states in the specified order
        inspire_positions = joint_pos[:, inspire_joint_indices]
        inspire_velocities = joint_vel[:, inspire_joint_indices]  
        inspire_torques = joint_torque[:, inspire_joint_indices]
        # publish to DDS (only publish the data of the first environment)
        if enable_dds and len(inspire_positions) > 0:
            try:

                inspire_dds = _get_inspire_dds_instance()
                if inspire_dds:
                    pos = inspire_positions[0].cpu().numpy()
                    vel = inspire_velocities[0].cpu().numpy()
                    torque = inspire_torques[0].cpu().numpy()
                    # write the gripper state to shared memory
                    inspire_dds.write_inspire_state(pos, vel, torque)
            except Exception as e:
                print(f"[gripper_state] Failed to write to shared memory: {e}")
        
        return inspire_positions
    else:
        # if the gripper joints are not found, return a zero tensor
        # print(f"[gripper_state] Warning: no gripper joints found, expected: {gripper_joint_names}, available: {all_joint_names}")
        print(f"[gripper_state] Warning: no gripper joints found")
        return torch.zeros((joint_pos.shape[0], 2))

