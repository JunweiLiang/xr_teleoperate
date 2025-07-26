# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
from action_provider.action_base import ActionProvider
from typing import Optional
import torch

class PolicyActionProvider(ActionProvider):
    """Action provider based on policy model, future use"""
    
    def __init__(self, policy_model, observation_processor=None):
        super().__init__("PolicyActionProvider")
        self.policy_model = policy_model
        self.observation_processor = observation_processor
    
    def get_action(self, env) -> Optional[torch.Tensor]:
        """Get action from policy model"""
        try:
            # Get observation
            obs = env.observation_manager.compute()
            
            # Process observation (if needed)
            if self.observation_processor:
                obs = self.observation_processor(obs)
            
            # Use policy to generate action
            with torch.no_grad():
                action = self.policy_model(obs)
            
            return action
            
        except Exception as e:
            print(f"[{self.name}] Get policy action failed: {e}")
            return None