# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
Sim state DDS communication class
Specialized in publishing and receiving sim state data
"""

import threading
import torch
from typing import Any, Dict, Optional
from dds.dds_base import BaseDDSNode, node_manager
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_

import json

class SimStateDDS(BaseDDSNode):
    """Sim state DDS node (singleton pattern)"""
    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls, env=None, task_name=None):
        """Singleton pattern implementation"""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(SimStateDDS, cls).__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    
    def __init__(self, env, task_name):
        """Initialize the sim state DDS node"""
        # avoid duplicate initialization
        if hasattr(self, '_initialized') and self._initialized:
            return
            
        super().__init__("SimStateDDS")
        self.env = env
        self.task_name = task_name
        self._initialized = True
        self.sim_state = std_msgs_msg_dds__String_()

        # setup the shared memory
        self.setup_shared_memory(
            input_shm_name="isaac_sim_state",  # read sim state data for publishing
            input_size=4096,
            outputshm_flag=False
        )

        print(f"[{self.node_name}] Sim state DDS node initialized")

    
    def setup_publisher(self) -> bool:
        """Setup the publisher of the sim state"""
        try:
            self.publisher = ChannelPublisher("rt/sim_state", String_)
            self.publisher.Init()
            
            print(f"[{self.node_name}] Sim state publisher initialized")
            return True
        except Exception as e:
            print(f"sim_state_dds [{self.node_name}] Sim state publisher initialization failed: {e}")
            return False
    
    def setup_subscriber(self) -> bool:
        """Setup the subscriber of the sim state"""
        try:
            self.subscriber = ChannelSubscriber("rt/sim_state_cmd", String_)
            self.subscriber.Init(self.process_subscribe_data, 10)
            
            print(f"[{self.node_name}] Sim state subscriber initialized")
            return True
        except Exception as e:
            print(f"sim_state_dds [{self.node_name}] Sim state subscriber initialization failed: {e}")
            return False
    
    
    def process_publish_data(self, data: Dict[str, Any]) -> Any:
        """Process the publish data"""
        try:
            # get sim state from environment
            sim_state = json.dumps(data)
            self.sim_state.data = sim_state
            return self.sim_state
        except Exception as e:
            print(f"sim_state_dds [{self.node_name}] Error processing publish data: {e}")
            return None
    
    def process_subscribe_data(self, msg: String_) -> Dict[str, Any]:
        """Process the subscribe data"""
        try:
            # Parse received sim state command
            data = json.loads(msg.data)
            
            # Process the command (implement according to your needs)
            # For example, you might want to apply the received state to the environment
            return data
        except Exception as e:
            print(f"sim_state_dds [{self.node_name}] Error processing subscribe data: {e}")
            return None

    def tensors_to_list(self, obj):
        if isinstance(obj, torch.Tensor):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {k: self.tensors_to_list(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [self.tensors_to_list(i) for i in obj]
        return obj

    def sim_state_to_json(self,data):
        data_serializable = self.tensors_to_list(data)
        json_str = json.dumps(data_serializable)
        return json_str

    def write_sim_state_data(self, sim_state_data=None):
        """Write sim state data to shared memory to trigger publishing
        
        Args:
            sim_state_data: Optional sim state data. If None, will get current state from environment
        """
        try:
            if sim_state_data is None:
                # Get current sim state from environment
                sim_state_data = {"trigger": "publish_sim_state"}
            
            # write to the input shared memory for publishing
            if self.input_shm:
                self.input_shm.write_data(sim_state_data)
                
        except Exception as e:
            print(f"sim_state_dds [{self.node_name}] Error writing sim state data: {e}")

    def get_sim_state_command(self) -> Optional[Dict[str, Any]]:
        """Get the sim state control command
        
        Returns:
            Dict: the sim state command, return None if there is no new command
        """
        if self.output_shm:
            return self.output_shm.read_data()
        return None
    
    def start_communication(self, enable_publish: bool = True, enable_subscribe: bool = False):
        """Start the sim state DDS communication
        
        Args:
            enable_publish: whether to enable the publish function (publish the sim state)
            enable_subscribe: whether to enable the subscribe function (receive the sim state control command)
        """
        try:
            # register the node
            node_manager.register_node(self)
            
            # if the node is already running, dynamically add new features
            if self.running:
                print(f"[{self.node_name}] Node is already running, dynamically add features...")
                
                # if the publish function is enabled but the publisher is not set, then set the publisher
                if enable_publish and not self.publisher:
                    print(f"[{self.node_name}] Add publish function...")
                    if self.setup_publisher():
                        if not self.publish_thread or not self.publish_thread.is_alive():
                            import threading
                            self.publish_thread = threading.Thread(target=self._publish_loop)
                            self.publish_thread.daemon = True
                            self.publish_thread.start()
                            print(f"[{self.node_name}] Publish thread started")
                
                # if the subscribe function is enabled but the subscriber is not set, then set the subscriber
                if enable_subscribe and not self.subscriber:
                    print(f"[{self.node_name}] Add subscribe function...")
                    if self.setup_subscriber():
                        if not self.subscribe_thread or not self.subscribe_thread.is_alive():
                            import threading
                            self.subscribe_thread = threading.Thread(target=self._subscribe_loop)
                            self.subscribe_thread.daemon = True
                            self.subscribe_thread.start()
                            print(f"[{self.node_name}] Subscribe thread started")
            else:
                # the node is not running, start normally
                self.start(enable_publish=enable_publish, enable_subscribe=enable_subscribe)
            
            status_msg = f"[{self.node_name}] Sim state DDS communication started"
            if enable_publish and enable_subscribe:
                status_msg += " (publish + subscribe)"
            elif enable_publish:
                status_msg += " (publish only)"
            elif enable_subscribe:
                status_msg += " (subscribe only)"
            else:
                status_msg += " (no function enabled)"
            
            print(status_msg)
            
        except Exception as e:
            print(f"sim_state_dds [{self.node_name}] Failed to start communication: {e}")
    
    def stop_communication(self):
        """Stop the sim state DDS communication"""
        try:
            self.stop()
            print(f"[{self.node_name}] Sim state DDS communication stopped")
        except Exception as e:
            print(f"sim_state_dds [{self.node_name}] Failed to stop communication: {e}")
    
    @classmethod
    def get_instance(cls, env, task_name) -> 'SimStateDDS':
        """Get the singleton instance"""
        return cls(env, task_name)


# convenient functions
def get_sim_state_dds(env, task_name) -> SimStateDDS:    
    """Get the sim state DDS instance"""
    return SimStateDDS.get_instance(env, task_name)


def start_sim_state_communication(env, task_name, enable_publish: bool = True, enable_subscribe: bool = True):
    """Start the sim state DDS communication
    
    Args:
        enable_publish: whether to enable the publish function (publish the sim state)
        enable_subscribe: whether to enable the subscribe function (receive the sim state control command)
    
    Returns:
        SimStateDDS: the DDS instance
    """
    sim_state_dds = get_sim_state_dds(env, task_name)
    sim_state_dds.start_communication(enable_publish=enable_publish, enable_subscribe=enable_subscribe)
    return sim_state_dds


def start_sim_state_publisher_only(env, task_name):
    """Start the sim state publish function only
    
    Applicable scenarios: only need to publish the sim state, no need to receive the external control command
    """
    return start_sim_state_communication(env, task_name, enable_publish=True, enable_subscribe=False)


def start_sim_state_subscriber_only(env, task_name):
    """Start the sim state command subscribe function only
    
    Applicable scenarios: only need to receive the external sim state control command, no need to publish the sim state
    """
    return start_sim_state_communication(env, task_name, enable_publish=False, enable_subscribe=True)


def stop_sim_state_communication(env, task_name):
    """Stop the sim state DDS communication"""     
    sim_state_dds = get_sim_state_dds(env, task_name)
    sim_state_dds.stop_communication()


def publish_sim_state(env, task_name, sim_state_data=None):
    """Manually trigger sim state publishing
    
    Args:
        env: Isaac Lab environment
        task_name: task name
        sim_state_data: Optional sim state data to publish
    """
    sim_state_dds = get_sim_state_dds(env, task_name)
    sim_state_dds.write_sim_state_data(sim_state_data)


