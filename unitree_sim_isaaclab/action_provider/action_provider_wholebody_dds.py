# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
from action_provider.action_base import ActionProvider
from typing import Optional
import torch
import time
import threading
class DDSWholebodyActionProvider(ActionProvider):
    """Action provider based on DDS"""
    
    def __init__(self,env, robot_type="g129", enable_gripper=False, enable_dex3=False, enable_inspire=False):
        super().__init__("DDSWholebodyActionProvider")
        self.enable_robot = robot_type
        self.enable_gripper = enable_gripper
        self.enable_dex3 = enable_dex3
        self.enable_inspire = enable_inspire
        self.env = env
        # Initialize DDS communication
        self.robot_dds = None
        self.gripper_dds = None
        self.dex3_dds = None
        self.inspire_dds = None
        # self._is_running = True
        # self._action_thread = None
        self._setup_dds()
        self._setup_joint_mapping()
        # self.start_action_thread()
    def _setup_dds(self):
        """Setup DDS communication"""
        print(f"enable_robot: {self.enable_robot}")
        print(f"enable_gripper: {self.enable_gripper}")
        print(f"enable_dex3: {self.enable_dex3}")
        try:
            if self.enable_robot == "g129":
                print(f"Starting G1 robot DDS subscriber...")
                from dds.g1_robot_dds import start_g1_robot_subscriber_only
                self.robot_dds = start_g1_robot_subscriber_only()
                print(f"start_g1_robot_subscriber_only success")
            if self.enable_gripper:
                from dds.gripper_dds import start_gripper_subscriber_only
                self.gripper_dds = start_gripper_subscriber_only()
                print(f"gripper_dds start_gripper_subscriber_only success")
            if self.enable_dex3:
                from dds.dex3_dds import start_hand_subscriber_only
                self.dex3_dds = start_hand_subscriber_only()
                print(f"dex3_dds start_hand_subscriber_only success")
            if self.enable_inspire:
                from dds.inspire_dds import start_inspire_hand_subscriber_only
                self.inspire_dds = start_inspire_hand_subscriber_only()
                print(f"inspire_dds start_inspire_hand_subscriber_only success")
            print(f"[{self.name}] DDS communication initialized")
        except Exception as e:
            print(f"[{self.name}] DDS initialization failed: {e}")
    
    def _setup_joint_mapping(self):
        """Setup joint mapping"""
        if self.enable_robot == "g129":
            self.boy_joint_names_urdf = {'left_hip_pitch_joint':0, 
                                    'left_hip_roll_joint':1, 
                                    'left_hip_yaw_joint':2, 
                                    'left_knee_joint':3, 
                                    'left_ankle_pitch_joint':4, 
                                    'left_ankle_roll_joint':5, 
                                    'right_hip_pitch_joint':6, 
                                    'right_hip_roll_joint':7, 
                                    'right_hip_yaw_joint':8, 
                                    'right_knee_joint':9, 
                                    'right_ankle_pitch_joint':10, 
                                    'right_ankle_roll_joint':11, 
                                    'waist_yaw_joint':12, 
                                    'waist_roll_joint':13, 
                                    'waist_pitch_joint':14, 
                                    'left_shoulder_pitch_joint':15, 
                                    'left_shoulder_roll_joint':16, 
                                    'left_shoulder_yaw_joint':17, 
                                    'left_elbow_joint':18, 
                                    'left_wrist_roll_joint':19, 
                                    'left_wrist_pitch_joint':20, 
                                    'left_wrist_yaw_joint':21, 
                                    'right_shoulder_pitch_joint':22, 
                                    'right_shoulder_roll_joint':23, 
                                    'right_shoulder_yaw_joint':24, 
                                    'right_elbow_joint':25, 
                                    'right_wrist_roll_joint':26, 
                                    'right_wrist_pitch_joint':27, 
                                    'right_wrist_yaw_joint':28
                                    }

        if self.enable_gripper:
            self.gripper_joint_mapping = {
                "left_hand_Joint1_1": 1,
                "left_hand_Joint2_1": 1,
                "right_hand_Joint1_1": 0,
                "right_hand_Joint2_1": 0,
            }
        if self.enable_dex3:
            self.left_hand_joint_mapping = {
                "left_hand_thumb_0_joint":0,
                "left_hand_thumb_1_joint":1,
                "left_hand_thumb_2_joint":2,
                "left_hand_middle_0_joint":3,
                "left_hand_middle_1_joint":4,
                "left_hand_index_0_joint":5,
                "left_hand_index_1_joint":6}
            self.right_hand_joint_mapping = {
                "right_hand_thumb_0_joint":0,     
                "right_hand_thumb_1_joint":1,
                "right_hand_thumb_2_joint":2,
                "right_hand_middle_0_joint":3,
                "right_hand_middle_1_joint":4,
                "right_hand_index_0_joint":5,
                "right_hand_index_1_joint":6}
        self.all_joint_names = self.env.scene["robot"].data.joint_names
        self.joint_to_index = {name: i for i, name in enumerate(self.all_joint_names)}

    # def start_action_thread(self):
    #     """启动动作执行线程"""
    #     if self._action_thread is None or not self._action_thread.is_alive():
    #         self._is_running = True
    #         self._action_thread = threading.Thread(target=self._apply_action_loop, daemon=True)
    #         self._action_thread.start()
    #         print(f"[{self.name}] Action thread started")
    #     else:
    #         print(f"[{self.name}] Action thread is already running")
    
    # def stop_action_thread(self):
    #     """停止动作执行线程"""
    #     if self._is_running:
    #         self._is_running = False
    #         if self._action_thread and self._action_thread.is_alive():
    #             self._action_thread.join(timeout=5.0)  # 等待最多5秒
    #             if self._action_thread.is_alive():
    #                 print(f"[{self.name}] Warning: Action thread did not stop gracefully")
    #             else:
    #                 print(f"[{self.name}] Action thread stopped")
    #         self._action_thread = None
    
    # def _apply_action_loop(self):
    #     """线程中执行的动作循环"""
    #     print(f"[{self.name}] Action loop started in thread")
    #     try:
    #         start_time = time.time()
    #         while self._is_running:
    #             action = self.get_action(self.env)
    #             if action is not None:
    #                 self.env.scene["robot"].set_joint_position_target(action)
    #                 self.env.scene.write_data_to_sim()
    #                 self.env.scene.update(dt=self.env.physics_dt)
    #                 self.env.sim.step(render=False)
    #                 time.sleep(self.env.physics_dt)start_action_thread
    #             print(f"[{self.name}] Action loop time: {time.time() - start_time}")
    #             start_time = time.time()
    #     except Exception as e:
    #         print(f"[{self.name}] Action loop error: {e}")
    #     finally:
    #         print(f"[{self.name}] Action loop ended")        
    def get_action(self, env) -> Optional[torch.Tensor]:
        """Get action from DDS"""
        try:

            full_action = torch.zeros(len(self.all_joint_names), device=self.env.device)
            # Get robot command
            if self.enable_robot == "g129" and self.robot_dds:
                cmd_data = self.robot_dds.get_robot_command()
                if cmd_data and 'motor_cmd' in cmd_data:
                    positions = cmd_data['motor_cmd']['positions']
                    if len(positions) >= 29:
                        # arm_action = torch.tensor(positions[15:29], device=env.device, dtype=torch.float32)
                        for joint_name, arm_idx in self.boy_joint_names_urdf.items():
                            if joint_name in self.joint_to_index:
                                full_action[self.joint_to_index[joint_name]] = positions[arm_idx]
            
            # Get gripper command
            if self.gripper_dds:
                gripper_cmd = self.gripper_dds.get_gripper_command()
                if gripper_cmd and 'positions' in gripper_cmd:
                    gripper_positions = gripper_cmd['positions']
                    # print(f"gripper_positions: {gripper_positions}")
                    if len(gripper_positions) >= 2:
                        for joint_name, gripper_idx in self.gripper_joint_mapping.items():
                            if joint_name in self.joint_to_index:
                                # Convert gripper range
                                gripper_value = gripper_positions[gripper_idx]
                                # print(f"gripper_value: {gripper_value}")
                                full_action[self.joint_to_index[joint_name]] = gripper_value
            
            # Get hand command
            if self.dex3_dds:
                hand_cmds = self.dex3_dds.get_hand_commands()
                if hand_cmds:
                    left_hand_cmd = hand_cmds.get('left_hand_cmd', {})
                    right_hand_cmd = hand_cmds.get('right_hand_cmd', {})
                    
                    if left_hand_cmd and right_hand_cmd:
                        left_positions = left_hand_cmd.get('positions', [])
                        right_positions = right_hand_cmd.get('positions', [])
                        for joint_name, left_idx in self.left_hand_joint_mapping.items():
                            if joint_name in self.joint_to_index:
                                full_action[self.joint_to_index[joint_name]] = left_positions[left_idx]
                        for joint_name, right_idx in self.right_hand_joint_mapping.items():
                            if joint_name in self.joint_to_index:
                                full_action[self.joint_to_index[joint_name]] = right_positions[right_idx]
                        
            # print(f"full_action: {full_action}")
            return full_action.unsqueeze(0)
            
        except Exception as e:
            print(f"[{self.name}] Get DDS action failed: {e}")
            return None
    
    def _convert_to_joint_range(self, value):
        """Convert gripper control value to joint angle"""
        input_min, input_max = 0.0, 5.6
        output_min, output_max = 0.03, -0.02
        value = max(input_min, min(input_max, value))
        return output_min + (output_max - output_min) * (value - input_min) / (input_max - input_min)
    
    def cleanup(self):
        """Clean up DDS resources"""
        try:
            # self.stop_action_thread()
            if self.robot_dds:
                self.robot_dds.stop_communication()
            if self.gripper_dds:
                self.gripper_dds.stop_communication()
            if self.dex3_dds:
                self.dex3_dds.stop_communication()
        except Exception as e:
            print(f"[{self.name}] Clean up DDS resources failed: {e}")