# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
Reset pose DDS communication class
Specialized in receiving the reset pose command
"""

import threading
from typing import Any, Dict, Optional
from dds.dds_base import BaseDDSNode, node_manager
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_


class ResetPoseCmdDDS(BaseDDSNode):
    """Reset pose command DDS node (singleton pattern)"""
    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        """Singleton pattern implementation"""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(ResetPoseCmdDDS, cls).__new__(cls)
        return cls._instance

    
    def __init__(self):
        """Initialize the reset pose DDS node"""
        # avoid duplicate initialization
        if hasattr(self, '_initialized'):
            return
        super().__init__("ResetPoseCmdDDS")
        
        self._initialized = True
        
        # setup the shared memory
        self.setup_shared_memory(
            output_shm_name="isaac_reset_pose_cmd", 
            output_size=512, 
            outputshm_flag=True,
            inputshm_flag=False,
        )
        print(f"[{self.node_name}] Reset pose DDS node initialized")

    
    def setup_publisher(self) -> bool:
        """Setup the reset pose command publisher (this node is mainly used for subscribe, the publisher is optional)"""
        pass
    
    def setup_subscriber(self) -> bool:
        """Setup the reset pose command subscriber"""
        try:
            self.subscriber = ChannelSubscriber("rt/reset_pose/cmd", String_)
            self.subscriber.Init(self._subscribe_message_handler, 1)
            
            print(f"[{self.node_name}] Reset pose command subscriber initialized")
            return True
        except Exception as e:
            print(f"reset_pose_dds [{self.node_name}] Failed to initialize the reset pose command subscriber: {e}")
            return False
    
    
    def process_publish_data(self, data: Dict[str, Any]) -> Any:
        """Process the publish data (this node is mainly used for subscribe, the publish function is optional)"""
        pass
    
    def process_subscribe_data(self, msg: String_) -> Dict[str, Any]:
        """Process the subscribe data"""
        try:
            cmd_data = {
                "reset_category": msg.data
            }
            return cmd_data
        except Exception as e:
            print(f"reset_pose_dds [{self.node_name}] Failed to process the subscribe data: {e}")
            return {}
    
    def get_reset_pose_command(self) -> Optional[Dict[str, Any]]:
        """Get the reset pose command
        
        Returns:
            Dict: the reset pose command, if no command return None
        """
        if self.output_shm:
            return self.output_shm.read_data()
        return None
    
    def write_reset_pose_command(self, flag_category):
        """Write the reset pose command to the shared memory
        
        Args:
            positions: the reset pose command, if no command return None
        """
        try:
            # prepare the reset pose data
            cmd_data = {
                "reset_category":flag_category
            }
            
            # write the reset pose data to the shared memory
            if self.output_shm:
                self.output_shm.write_data(cmd_data)
                
        except Exception as e:
            print(f"reset_pose_dds [{self.node_name}] Failed to write the reset pose command: {e}")

    
    def start_communication(self, enable_publish: bool = True, enable_subscribe: bool = True):
        """Start the reset pose DDS communication
        
        Args:
            enable_publish: whether to enable the publish function (publish the reset pose state)
            enable_subscribe: whether to enable the subscribe function (receive the reset pose control command)
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
            
            status_msg = f"[{self.node_name}] Reset pose DDS communication started"
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
            print(f"reset_pose_dds [{self.node_name}] Failed to start communication: {e}")
    
    def stop_communication(self):
        """Stop the reset pose DDS communication"""
        try:
            self.stop()
            print(f"[{self.node_name}] Reset pose DDS communication stopped")
        except Exception as e:
            print(f"reset_pose_dds [{self.node_name}] Failed to stop communication: {e}")
    
    @classmethod
    def get_instance(cls) -> 'ResetPoseCmdDDS':
        """Get the singleton instance"""
        return cls()


# convenient functions
def get_reset_pose_dds() -> ResetPoseCmdDDS:
    """Get the reset pose DDS instance"""
    return ResetPoseCmdDDS.get_instance()


def start_reset_pose_communication(enable_publish: bool = True, enable_subscribe: bool = True):
    """Start the reset pose DDS communication
    
    Args:
        enable_publish: whether to enable the publish function (publish the reset pose state)
        enable_subscribe: whether to enable the subscribe function (receive the reset pose control command)
    
    Returns:
        ResetPoseCmdDDS: the DDS instance
    """
    reset_pose_dds = get_reset_pose_dds()
    reset_pose_dds.start_communication(enable_publish=enable_publish, enable_subscribe=enable_subscribe)
    return reset_pose_dds


def start_reset_pose_publisher_only():
    """Start the reset pose state publish function only
    
    Applicable scenarios: only need to publish the reset pose state, no need to receive the external control command
    """
    return start_reset_pose_communication(enable_publish=True, enable_subscribe=False)


def start_reset_pose_subscriber_only():
    """Start the reset pose command subscribe function only
    
    Applicable scenarios: only need to receive the external reset pose control command, no need to publish the reset pose state
    """
    return start_reset_pose_communication(enable_publish=False, enable_subscribe=True)


def stop_reset_pose_communication():
    """Stop the reset pose DDS communication"""
    reset_pose_dds = get_reset_pose_dds()
    reset_pose_dds.stop_communication()


if __name__ == "__main__":
    # test the singleton pattern
    dds1 = ResetPoseCmdDDS()
    dds2 = ResetPoseCmdDDS()
    print(f"Singleton test: {dds1 is dds2}")  # should be True
    
    # start
    try:
        start_reset_pose_communication()
        
        import time
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:   
        print("Program interrupted...")
    finally:
        stop_reset_pose_communication() 