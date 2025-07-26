# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Dex3 DDS communication class
Handle the state publishing and command receiving of the hand (left and right)
"""

import threading
from typing import Any, Dict, Optional, Tuple
from dds.dds_base import BaseDDSNode, node_manager
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_, HandCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandState_, unitree_hg_msg_dds__HandCmd_


class Dex3DDS(BaseDDSNode):
    """Hand DDS communication class - singleton pattern
    
    Features:
    - Publish the state of the hand to DDS (rt/dex3/left/state, rt/dex3/right/state)
    - Receive the control command of the hand (rt/dex3/left/cmd, rt/dex3/right/cmd)
    """
    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        """Singleton pattern implementation"""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(Dex3DDS, cls).__new__(cls)
        return cls._instance
    
    def __init__(self):
        """Initialize the hand DDS node"""
        # avoid duplicate initialization
        if hasattr(self, '_initialized'):
            return
            
        super().__init__("Dex3DDS")
        
        # initialize the state message of the hand
        self.left_hand_state = unitree_hg_msg_dds__HandState_()
        self.right_hand_state = unitree_hg_msg_dds__HandState_()
        
        # initialize the publisher and subscriber
        self.left_state_publisher = None
        self.right_state_publisher = None
        self.left_cmd_subscriber = None
        self.right_cmd_subscriber = None
        
        self._initialized = True
        
        # setup shared memory
        self.setup_shared_memory(
            input_shm_name="isaac_dex3_state",  # read the state of the hand from Isaac Lab
            input_size=1180,
            output_shm_name="isaac_dex3_cmd",  # output the command to Isaac Lab
            output_size=1180,  # output the command to Isaac Lab
        )
        
        print(f"[{self.node_name}] Hand DDS node initialized")
    
    def setup_publisher(self) -> bool:
        """Setup the publisher of the hand"""
        try:
            # left hand state publisher
            self.left_state_publisher = ChannelPublisher("rt/dex3/left/state", HandState_)
            self.left_state_publisher.Init()
            
            # right hand state publisher
            self.right_state_publisher = ChannelPublisher("rt/dex3/right/state", HandState_)
            self.right_state_publisher.Init()
            
            print(f"[{self.node_name}] Hand state publisher initialized")
            return True
        except Exception as e:
            print(f"dex3_dds [{self.node_name}] Hand state publisher initialization failed: {e}")
            return False
    
    def setup_subscriber(self) -> bool:
        """Setup the subscriber of the hand"""
        try:
            # left hand command subscriber
            self.left_cmd_subscriber = ChannelSubscriber("rt/dex3/left/cmd", HandCmd_)
            self.left_cmd_subscriber.Init(
                lambda msg: self._handle_hand_command(msg, "left"), 1
            )
            
            # right hand command subscriber
            self.right_cmd_subscriber = ChannelSubscriber("rt/dex3/right/cmd", HandCmd_)
            self.right_cmd_subscriber.Init(
                lambda msg: self._handle_hand_command(msg, "right"), 1
            )
            
            print(f"[{self.node_name}] Hand command subscriber initialized")
            return True
        except Exception as e:
            print(f"dex3_dds [{self.node_name}] Hand command subscriber initialization failed: {e}")
            return False
    
    def _handle_hand_command(self, msg: HandCmd_, hand_side: str):
        """Handle the command of the hand"""
        try:
            # process the command of the hand and write to the shared memory
            cmd_data = self._process_hand_command(msg, hand_side)
            if cmd_data and self.output_shm:
                # read the existing data or create new data
                existing_data = self.output_shm.read_data() or {}
                existing_data[f"{hand_side}_hand_cmd"] = cmd_data
                self.output_shm.write_data(existing_data)
        except Exception as e:
            print(f"dex3_dds [{self.node_name}] Error handling {hand_side} hand command: {e}")
    
    def _process_hand_command(self, msg: HandCmd_, hand_side: str) -> Dict[str, Any]:
        """Process the command of the hand"""
        try:
            cmd_data = {
                "positions": [float(msg.motor_cmd[i].q) for i in range(len(msg.motor_cmd))],
                "velocities": [float(msg.motor_cmd[i].dq) for i in range(len(msg.motor_cmd))],
                "torques": [float(msg.motor_cmd[i].tau) for i in range(len(msg.motor_cmd))],
                "kp": [float(msg.motor_cmd[i].kp) for i in range(len(msg.motor_cmd))],
                "kd": [float(msg.motor_cmd[i].kd) for i in range(len(msg.motor_cmd))]
            }
            return cmd_data
        except Exception as e:
            print(f"dex3_dds [{self.node_name}] Error processing {hand_side} hand command data: {e}")
            return {}
    
    def process_publish_data(self, data: Dict[str, Any]) -> Any:
        """Process the publish data: convert the hand state of Isaac Lab to DDS message
        
        Expected data format:
        {
            "left_hand": {
                "positions": [7 left hand joint positions],
                "velocities": [7 left hand joint velocities],
                "torques": [7 left hand joint torques]
            },
            "right_hand": {
                "positions": [7 right hand joint positions],
                "velocities": [7 right hand joint velocities],
                "torques": [7 right hand joint torques]
            }
        }
        """
        try:
            # process the left hand data
            if "left_hand" in data:
                left_data = data["left_hand"]
                self._update_hand_state(self.left_hand_state, left_data)
                if self.left_state_publisher:
                    self.left_state_publisher.Write(self.left_hand_state)
            
            # process the right hand data
            if "right_hand" in data:
                right_data = data["right_hand"]
                self._update_hand_state(self.right_hand_state, right_data)
                if self.right_state_publisher:
                    self.right_state_publisher.Write(self.right_hand_state)
            
            # since we need to publish to two different topics, handle the publishing here
            # return None to tell the base class not to publish again
            return None
        except Exception as e:
            print(f"dex3_dds [{self.node_name}] Error processing publish data: {e}")
            return None
    
    def _update_hand_state(self, hand_state, hand_data: Dict[str, Any]):
        """Update the hand state"""
        try:
            if all(key in hand_data for key in ["positions", "velocities", "torques"]):
                positions = hand_data["positions"]
                velocities = hand_data["velocities"]
                torques = hand_data["torques"]
                
                for i in range(min(7, len(positions))):  # at most 7 fingers
                    if i < len(positions):
                        hand_state.motor_state[i].q = float(positions[i])
                    if i < len(velocities):
                        hand_state.motor_state[i].dq = float(velocities[i])
                    if i < len(torques):
                        hand_state.motor_state[i].tau_est = float(torques[i])
        except Exception as e:
            print(f"dex3_dds [{self.node_name}] Error updating hand state: {e}")
    
    def process_subscribe_data(self, msg: Any) -> Dict[str, Any]:
        """This method is handled by _handle_hand_command, not directly used"""
        return {}
    
    def get_hand_commands(self) -> Optional[Dict[str, Any]]:
        """Get the hand control commands
        
        Returns:
            Dict: the dictionary containing the commands of the left and right hands, the format is as follows:
            {
                "left_hand_cmd": {left hand command},
                "right_hand_cmd": {right hand command}
            }
        """
        if self.output_shm:
            return self.output_shm.read_data()
        return None
    
    def get_left_hand_command(self) -> Optional[Dict[str, Any]]:
        """Get the left hand command"""
        commands = self.get_hand_commands()
        if commands and "left_hand_cmd" in commands:
            return commands["left_hand_cmd"]
        return None
    
    def get_right_hand_command(self) -> Optional[Dict[str, Any]]:
        """Get the right hand command"""
        commands = self.get_hand_commands()
        if commands and "right_hand_cmd" in commands:
            return commands["right_hand_cmd"]
        return None
    
    def publish_hand_states(self, left_hand_data: Dict[str, Any], right_hand_data: Dict[str, Any]):
        """Publish the left and right hand states
        
        Args:
            left_hand_data: the data of the left hand
            right_hand_data: the data of the right hand
        """
        try:
            combined_data = {
                "left_hand": left_hand_data,
                "right_hand": right_hand_data
            }
            
            # write to the input shared memory for publishing
            if self.input_shm:
                self.input_shm.write_data(combined_data)
                
        except Exception as e:
            print(f"dex3_dds [{self.node_name}] Error publishing hand states: {e}")
    
    def write_hand_states(self, left_positions, left_velocities, left_torques, 
                         right_positions, right_velocities, right_torques):
        """Write the hand states to the shared memory directly
        
        Args:
            left_positions: the list or torch.Tensor of the left hand joint positions
            left_velocities: the list or torch.Tensor of the left hand joint velocities
            left_torques: the list or torch.Tensor of the left hand joint torques
            right_positions: the list or torch.Tensor of the right hand joint positions
            right_velocities: the list or torch.Tensor of the right hand joint velocities
            right_torques: the list or torch.Tensor of the right hand joint torques
        """
        try:
            # prepare the left hand data
            left_hand_data = {
                "positions": left_positions.tolist() if hasattr(left_positions, 'tolist') else left_positions,
                "velocities": left_velocities.tolist() if hasattr(left_velocities, 'tolist') else left_velocities,
                "torques": left_torques.tolist() if hasattr(left_torques, 'tolist') else left_torques
            }
            
            # prepare the right hand data
            right_hand_data = {
                "positions": right_positions.tolist() if hasattr(right_positions, 'tolist') else right_positions,
                "velocities": right_velocities.tolist() if hasattr(right_velocities, 'tolist') else right_velocities,
                "torques": right_torques.tolist() if hasattr(right_torques, 'tolist') else right_torques
            }
            
            # publish the states
            self.publish_hand_states(left_hand_data, right_hand_data)
            
        except Exception as e:
            print(f"dex3_dds [{self.node_name}] Error writing hand states: {e}")
    
    def write_single_hand_state(self, hand_side: str, positions, velocities, torques):
        """Write the single hand state
        
        Args:
            hand_side: the side of the hand ("left" or "right")
            positions: the list or torch.Tensor of the hand joint positions
            velocities: the list or torch.Tensor of the hand joint velocities
            torques: the list or torch.Tensor of the hand joint torques
        """
        try:
            hand_data = {
                "positions": positions.tolist() if hasattr(positions, 'tolist') else positions,
                "velocities": velocities.tolist() if hasattr(velocities, 'tolist') else velocities,
                "torques": torques.tolist() if hasattr(torques, 'tolist') else torques
            }
            
            # decide how to publish based on the hand side
            if hand_side == "left":
                # get the existing right hand data or use the default value
                existing_data = self.input_shm.read_data() if self.input_shm else {}
                right_data = existing_data.get("right_hand", {"positions": [0], "velocities": [0], "torques": [0]})
                self.publish_hand_states(hand_data, right_data)
            elif hand_side == "right":
                # get the existing left hand data or use the default value
                existing_data = self.input_shm.read_data() if self.input_shm else {}
                left_data = existing_data.get("left_hand", {"positions": [0], "velocities": [0], "torques": [0]})
                self.publish_hand_states(left_data, hand_data)
            else:
                print(f"dex3_dds [{self.node_name}] Invalid hand side: {hand_side}")
                
        except Exception as e:
            print(f"dex3_dds [{self.node_name}] Error writing {hand_side} hand state: {e}")
    
    
    def start(self, enable_publish: bool = True, enable_subscribe: bool = True):
        """Rewrite the start method to handle multiple publishers and subscribers"""
        if self.running:
            print(f"[{self.node_name}] Node already running")
            return
        
        try:
            # initialize DDS (only initialize once globally)
            from dds.dds_base import dds_init_manager
            if not dds_init_manager.initialize():
                print(f"[{self.node_name}] Failed to initialize DDS factory")
                return
            
            # setup the publisher
            if enable_publish:
                if not self.setup_publisher():
                    print(f"[{self.node_name}] Failed to setup publisher")
                    return
            
            # setup the subscriber  
            if enable_subscribe:
                print(f"[{self.node_name}] Setting up subscriber...")
                if not self.setup_subscriber():
                    print(f"[{self.node_name}] Failed to setup subscriber")
                    return
                else:
                    print(f"[{self.node_name}] Subscriber setup successfully")
            
            self.running = True
            
            # start the publish thread (check if there is any publisher)
            if enable_publish and (self.left_state_publisher or self.right_state_publisher):
                self.publisher=True
                self.publish_thread = threading.Thread(target=self._publish_loop)
                self.publish_thread.daemon = True
                self.publish_thread.start()

            # start the subscribe thread (check if there is any subscriber)
            if enable_subscribe and (self.left_cmd_subscriber or self.right_cmd_subscriber):
                self.subscriber = True
                print(f"[{self.node_name}] Starting subscribe thread...")
                self.subscribe_thread = threading.Thread(target=self._subscribe_loop)
                self.subscribe_thread.daemon = True
                self.subscribe_thread.start()
                print(f"[{self.node_name}] Subscribe thread started")
            elif enable_subscribe:
                print(f"[{self.node_name}] Warning: subscriber is empty, cannot start subscribe thread")
            
            print(f"[{self.node_name}] Node started successfully")
            
        except Exception as e:
            print(f"[{self.node_name}] Failed to start node: {e}")
            self.stop()
    
    def start_communication(self, enable_publish: bool = True, enable_subscribe: bool = True):
        """Start the hand DDS communication
        
        Args:
            enable_publish: whether to enable the publish function (publish the left and right hand states)
            enable_subscribe: whether to enable the subscribe function (receive the left and right hand control commands)
        """
        try:
            # register the node
            node_manager.register_node(self)
            
            # if the node is already running, dynamically add new features
            if self.running:
                print(f"[{self.node_name}] Node is already running, dynamically add features...")
                # if the publish function is enabled but the publisher is not set, then set the publisher
                if enable_publish and not self.left_state_publisher and not self.right_state_publisher:
                    print(f"[{self.node_name}] Add publish function...")
                    if self.setup_publisher():
                        if not self.publish_thread or not self.publish_thread.is_alive():
                            import threading
                            self.publisher=True
                            self.publish_thread = threading.Thread(target=self._publish_loop)
                            self.publish_thread.daemon = True
                            self.publish_thread.start()
                            print(f"[{self.node_name}] Publish thread started")
                
                # if the subscribe function is enabled but the subscriber is not set, then set the subscriber
                if enable_subscribe and not self.left_cmd_subscriber and not self.right_cmd_subscriber:
                    print(f"[{self.node_name}] Add subscribe function...")
                    if self.setup_subscriber():
                        if not self.subscribe_thread or not self.subscribe_thread.is_alive():
                            self.subscriber = True
                            import threading
                            self.subscribe_thread = threading.Thread(target=self._subscribe_loop)
                            self.subscribe_thread.daemon = True
                            self.subscribe_thread.start()
                            print(f"[{self.node_name}] Subscribe thread started")
            else:
                # the node is not running, start normally
                self.start(enable_publish=enable_publish, enable_subscribe=enable_subscribe)
            
            status_msg = f"[{self.node_name}] Hand DDS communication started"
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
            print(f"[{self.node_name}] Failed to start communication: {e}")
    
    def stop_communication(self):
        """Stop the hand DDS communication"""
        try:
            self.stop()
            print(f"[{self.node_name}] Hand DDS communication stopped")
        except Exception as e:
            print(f"[{self.node_name}] Failed to stop communication: {e}")
    
    @classmethod
    def get_instance(cls) -> 'Dex3DDS':
        """Get the singleton instance"""
        return cls()

# 便捷函数
def get_dex3_dds() -> Dex3DDS:
    """Get the hand DDS instance"""
    return Dex3DDS.get_instance()


def start_hand_communication(enable_publish: bool = True, enable_subscribe: bool = True):
    """Start the hand DDS communication
    
    Args:
        enable_publish: whether to enable the publish function (publish the left and right hand states)
        enable_subscribe: whether to enable the subscribe function (receive the left and right hand control commands)
    
    Returns:
        Dex3DDS: the DDS instance
    """
    dex3_dds = get_dex3_dds()
    dex3_dds.start_communication(enable_publish=enable_publish, enable_subscribe=enable_subscribe)
    return dex3_dds


def start_hand_publisher_only():
    """Start the hand state publish function only
    
    Applicable scenarios: only need to publish the left and right hand states, no need to receive the external control commands
    """
    return start_hand_communication(enable_publish=True, enable_subscribe=False)


def start_hand_subscriber_only():
    """Start the hand command subscribe function only
    
    Applicable scenarios: only need to receive the external left and right hand control commands, no need to publish the hand states
    """
    return start_hand_communication(enable_publish=False, enable_subscribe=True)


def stop_hand_communication():
    """Stop the hand DDS communication"""
    dex3_dds = get_dex3_dds()
    dex3_dds.stop_communication()


if __name__ == "__main__":
    # test the singleton pattern
    dds1 = Dex3DDS()
    dds2 = Dex3DDS()
    print(f"Singleton test: {dds1 is dds2}")  # should be True
    
    # start the communication
    try:
        start_hand_communication()
        
        import time
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("Program interrupted...")
    finally:
        stop_hand_communication() 