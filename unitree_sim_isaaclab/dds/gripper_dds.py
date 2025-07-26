# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
Gripper DDS communication class
Handle the state publishing and command receiving of the gripper
"""

import threading
from typing import Any, Dict, Optional
from dds.dds_base import BaseDDSNode, node_manager
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_, unitree_go_msg_dds__MotorState_


class GripperDDS(BaseDDSNode):
    """Gripper DDS communication class - singleton pattern
    
    Features:
    - Publish the state of the gripper to DDS (rt/unitree_actuator/state)
    - Receive the control command of the gripper (rt/unitree_actuator/cmd)
    """
    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        """Singleton pattern implementation"""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(GripperDDS, cls).__new__(cls)
        return cls._instance
    
    def __init__(self):
        """Initialize the gripper DDS node"""
        # avoid duplicate initialization
        if hasattr(self, '_initialized'):
            return
            
        super().__init__("GripperDDS")
        
        # initialize the gripper state message (2 grippers)
        self.gripper_state = MotorStates_()
        # initialize 2 motor states
        self.gripper_state.states = []
        for _ in range(2):
            motor_state = unitree_go_msg_dds__MotorState_()
            self.gripper_state.states.append(motor_state)
        
        self._initialized = True
        
        # setup the shared memory
        self.setup_shared_memory(
            input_shm_name="isaac_gripper_state",  # read the state of the gripper from Isaac Lab
            input_size=512,
            output_shm_name="isaac_gripper_cmd",  # output the command to Isaac Lab
            output_size=512,  # output the command to Isaac Lab
        )
        
        print(f"[{self.node_name}] Gripper DDS node initialized")
    
    def setup_publisher(self) -> bool:
        """Setup the publisher of the gripper"""
        try:
            self.publisher = ChannelPublisher("rt/unitree_actuator/state", MotorStates_)
            self.publisher.Init()
            
            print(f"[{self.node_name}] Gripper state publisher initialized")
            return True
        except Exception as e:
            print(f"gripper_dds [{self.node_name}] Gripper state publisher initialization failed: {e}")
            return False
    
    def setup_subscriber(self) -> bool:
        """Setup the subscriber of the gripper"""
        try:
            self.subscriber = ChannelSubscriber("rt/unitree_actuator/cmd", MotorCmds_)
            self.subscriber.Init(self._subscribe_message_handler, 1)
            
            print(f"[{self.node_name}] Gripper command subscriber initialized")
            return True
        except Exception as e:
            print(f"gripper_dds [{self.node_name}] Gripper command subscriber initialization failed: {e}")
            return False
    
    def process_publish_data(self, data: Dict[str, Any]) -> Any:
        """Process the publish data: convert the Isaac Lab state to the DDS message
        
        Expected data format:
        {
            "positions": [2 gripper joint positions] (Isaac Lab joint angle range [-0.02, 0.03])
            "velocities": [2 gripper joint velocities],
            "torques": [2 gripper joint torques]
        }
        """
        try:
            if all(key in data for key in ["positions", "velocities", "torques"]):
                positions = data["positions"]
                velocities = data["velocities"]
                torques = data["torques"]
                for i in range(min(2, len(positions))):
                    if i < len(self.gripper_state.states):
                        # convert the Isaac Lab joint angle to the gripper control value    
                        gripper_value = self.convert_to_gripper_range(float(positions[i]))
                        self.gripper_state.states[i].q = gripper_value
                        if i < len(velocities):
                            self.gripper_state.states[i].dq = float(velocities[i])
                        if i < len(torques):
                            self.gripper_state.states[i].tau_est = float(torques[i])
            
            return self.gripper_state
            
        except Exception as e:
            print(f"gripper_dds [{self.node_name}] Error processing publish data: {e}")    
            return None
    
    def process_subscribe_data(self, msg: MotorCmds_) -> Dict[str, Any]:
        """Process the subscribe data: convert the DDS command to the Isaac Lab format
        
        Returns:
            Dict: the gripper command, format as follows:
            {
                "positions": [2 gripper joint position target values] (Isaac Lab joint angle)
                "velocities": [2 gripper joint velocity target values],
                "torques": [2 gripper joint torque target values],
                "kp": [2 gripper position gains],
                "kd": [2 gripper speed gains]
            }
        """
        try:
            cmd_data = {
                "positions": [],
                "velocities": [],
                "torques": [],
                "kp": [],
                "kd": []
            }
            # process the gripper command (at most 2 grippers)
            for i in range(min(2, len(msg.cmds))):
                # convert the gripper control value to the Isaac Lab joint angle
                joint_angle = self.convert_to_joint_range(float(msg.cmds[i].q))
                
                cmd_data["positions"].append(joint_angle)
                cmd_data["velocities"].append(float(msg.cmds[i].dq))
                cmd_data["torques"].append(float(msg.cmds[i].tau))
                cmd_data["kp"].append(float(msg.cmds[i].kp))
                cmd_data["kd"].append(float(msg.cmds[i].kd))
            
            return cmd_data
            
        except Exception as e:
            print(f"gripper_dds [{self.node_name}] Error processing subscribe data: {e}")
            return {}
    
    def get_gripper_command(self) -> Optional[Dict[str, Any]]:
        """Get the gripper control command
        
        Returns:
            Dict: the gripper command, return None if there is no new command
        """
        if self.output_shm:
            return self.output_shm.read_data()
        return None
    
    def write_gripper_state(self, positions, velocities, torques):
        """Write the gripper state to the shared memory
        
        Args:
            positions: the gripper joint position list or torch.Tensor (Isaac Lab joint angle)
            velocities: the gripper joint velocity list or torch.Tensor  
            torques: the gripper joint torque list or torch.Tensor
        """
        try:
            # prepare the gripper data
            gripper_data = {
                "positions": positions.tolist() if hasattr(positions, 'tolist') else positions,
                "velocities": velocities.tolist() if hasattr(velocities, 'tolist') else velocities,
                "torques": torques.tolist() if hasattr(torques, 'tolist') else torques
            }
            
            # write the input shared memory for publishing
            if self.input_shm:
                self.input_shm.write_data(gripper_data)
                
        except Exception as e:
            print(f"gripper_dds [{self.node_name}] Error writing gripper state: {e}")
    
    def convert_to_joint_range(self, value):
        """Convert the command value to the Isaac Lab joint angle [5.6, 0] -> [-0.02, 0.03]
        
        Args:
            value: the input value, range in [5.6, 0]
                  5.6: fully open
                  0.0: fully closed
            
        Returns:
            float: the converted value, range in [-0.02, 0.03]
                  -0.02: fully open
                  0.03: fully closed
        """
        # input range (gripper control value)
        input_min = 0.0    # fully closed
        input_max = 5.6    # fully open
        
        # output range (joint angle)
        output_min = 0.03  # fully closed
        output_max = -0.02 # fully open
        
        # ensure the input value is in the valid range
        value = max(input_min, min(input_max, value))
        
        # linear mapping conversion
        converted_value = output_min + (output_max - output_min) * (value - input_min) / (input_max - input_min)
        
        return converted_value

    def convert_to_gripper_range(self, value):
        """Convert the Isaac Lab joint angle to the gripper control value [-0.02, 0.03] -> [5.6, 0]
        
        Args:
            value: the input value, range in [-0.02, 0.03]
                  -0.02: fully open
                  0.03: fully closed
            
        Returns:
            float: the converted value, range in [5.6, 0]
                  5.6: fully open
                  0.0: fully closed
        """
        # input range (joint angle)
        input_min = 0.03   # fully closed
        input_max = -0.02  # fully open
        
        # output range (gripper control value)
        output_min = 0.0   # fully closed
        output_max = 5.6   # fully open
        
        # ensure the input value is in the valid range
        value = max(input_max, min(input_min, value))
        
        # linear mapping conversion
        converted_value = output_min + (output_max - output_min) * (input_min - value) / (input_min - input_max)
        
        return converted_value
    
    def start_communication(self, enable_publish: bool = True, enable_subscribe: bool = True):
        """Start the gripper DDS communication
        
        Args:
            enable_publish: whether to enable the publish function (publish the gripper state)
            enable_subscribe: whether to enable the subscribe function (receive the gripper control command)
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
            
            status_msg = f"[{self.node_name}] Gripper DDS communication started"
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
            print(f"gripper_dds [{self.node_name}] Failed to start communication: {e}")
    
    def stop_communication(self):
        """Stop the gripper DDS communication"""
        try:
            self.stop()
            print(f"[{self.node_name}] Gripper DDS communication stopped")  
        except Exception as e:
            print(f"gripper_dds [{self.node_name}] Failed to stop communication: {e}")
    
    @classmethod
    def get_instance(cls) -> 'GripperDDS':
        """Get the singleton instance"""
        return cls()


# convenient functions
def get_gripper_dds() -> GripperDDS:
    """Get the gripper DDS instance"""
    return GripperDDS.get_instance()


def start_gripper_communication(enable_publish: bool = True, enable_subscribe: bool = True):
    """Start the gripper DDS communication
    
    Args:
        enable_publish: whether to enable the publish function (publish the gripper state)
        enable_subscribe: whether to enable the subscribe function (receive the gripper control command)
    
    Returns:
        GripperDDS: the DDS instance
    """
    gripper_dds = get_gripper_dds()
    gripper_dds.start_communication(enable_publish=enable_publish, enable_subscribe=enable_subscribe)
    return gripper_dds


def start_gripper_publisher_only():
    """Start the gripper state publish function only
    
    Applicable scenarios: only need to publish the gripper state, no need to receive the external control command
    """
    return start_gripper_communication(enable_publish=True, enable_subscribe=False)


def start_gripper_subscriber_only():
    """Start the gripper command subscribe function only
    
    Applicable scenarios: only need to receive the external control command, no need to publish the gripper state
    """
    return start_gripper_communication(enable_publish=False, enable_subscribe=True)


def stop_gripper_communication():
    """Stop the gripper DDS communication"""
    gripper_dds = get_gripper_dds()
    gripper_dds.stop_communication()


if __name__ == "__main__":
    # test the singleton pattern
    dds1 = GripperDDS()
    dds2 = GripperDDS()
    print(f"Singleton test: {dds1 is dds2}")  # should be True
    
    # start the communication
    try:
        start_gripper_communication()
        
        import time
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("Program interrupted...")
    finally:
        stop_gripper_communication() 