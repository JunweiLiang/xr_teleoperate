# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
G1 robot DDS communication class
Handle the state publishing and command receiving of the G1 robot
"""

import threading
from typing import Any, Dict, Optional
from dds.dds_base import BaseDDSNode, node_manager
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.utils.crc import CRC


class G1RobotDDS(BaseDDSNode):
    """G1 robot DDS communication class - singleton pattern
    
    Features:
    - Publish the state of the G1 robot to DDS (rt/lowstate)
    - Receive the control command of the G1 robot (rt/lowcmd)
    """
    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        """Singleton pattern implementation"""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(G1RobotDDS, cls).__new__(cls)
        return cls._instance
    
    def __init__(self):
        """Initialize the G1 robot DDS node"""
        # avoid duplicate initialization
        if hasattr(self, '_initialized'):
            return
            
        super().__init__("G1RobotDDS")
        self.crc = CRC()
        self.low_state = unitree_hg_msg_dds__LowState_()
        self._initialized = True
        
        # setup the shared memory
        self.setup_shared_memory(
            input_shm_name="isaac_robot_state",  # read the state of the G1 robot from Isaac Lab
            output_shm_name="dds_robot_cmd",  # output the command to Isaac Lab
            input_size=3072,
            output_size=3072  # output the command to Isaac Lab
        )
        
        print(f"[{self.node_name}] G1 robot DDS node initialized")
    
    def setup_publisher(self) -> bool:
        """Setup the publisher of the G1 robot"""
        try:
            self.publisher = ChannelPublisher("rt/lowstate", LowState_)
            self.publisher.Init()
            print(f"[{self.node_name}] State publisher initialized (rt/lowstate)")
            return True
        except Exception as e:
            print(f"g1_robot_dds [{self.node_name}] State publisher initialization failed: {e}")    
            return False
    
    def setup_subscriber(self) -> bool:
        """Setup the subscriber of the G1 robot"""
        try:
            print(f"[{self.node_name}] Create ChannelSubscriber...")
            self.subscriber = ChannelSubscriber("rt/lowcmd", LowCmd_)
            self.subscriber.Init(self._subscribe_message_handler, 1)
            return True
        except Exception as e:
            print(f"g1_robot_dds [{self.node_name}] Command subscriber initialization failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def process_publish_data(self, data: Dict[str, Any]) -> Any:
        """Process the publish data: convert the Isaac Lab state to the DDS message
        
        Expected data format:
        {
            "joint_positions": [29 joint positions],
            "joint_velocities": [29 joint velocities],
            "joint_torques": [29 joint torques],
            "imu_data": [13 IMU data: pos(3) + quat(4) + vel(3) + angvel(3)]
        }
        """
        try:
            # update the joint state
            if all(key in data for key in ["joint_positions", "joint_velocities", "joint_torques"]):
                positions = data["joint_positions"]
                velocities = data["joint_velocities"]
                torques = data["joint_torques"]
                
                for i in range(min(35, len(positions))):
                    if i < len(positions):
                        self.low_state.motor_state[i].q = float(positions[i])
                    if i < len(velocities):
                        self.low_state.motor_state[i].dq = float(velocities[i])
                    if i < len(torques):
                        self.low_state.motor_state[i].tau_est = float(torques[i])
            
            # update the IMU data
            if "imu_data" in data:
                imu = data["imu_data"]
                if len(imu) >= 13:
                    # quaternion (w, x, y, z) 
                    self.low_state.imu_state.quaternion[0] = float(imu[4])  # x
                    self.low_state.imu_state.quaternion[1] = float(imu[5])  # y
                    self.low_state.imu_state.quaternion[2] = float(imu[6])  # z
                    self.low_state.imu_state.quaternion[3] = float(imu[3])  # w
                    
                    # angular velocity
                    for i in range(3):
                        self.low_state.imu_state.gyroscope[i] = float(imu[10+i])
                    
                    # linear velocity as the accelerometer data
                    for i in range(3):
                        self.low_state.imu_state.accelerometer[i] = float(imu[7+i])
            
            # update the timestamp and CRC
            self.low_state.tick = int(self.low_state.tick + 1)
            self.low_state.crc = self.crc.Crc(self.low_state)
            
            return self.low_state
            
        except Exception as e:
            print(f"g1_robot_dds [{self.node_name}] Error processing publish data: {e}")
            return None
    
    def process_subscribe_data(self, msg: LowCmd_) -> Dict[str, Any]:
        """Process the subscribe data: convert the DDS command to the Isaac Lab format
        
        Return data format:
        {
            "mode_pr": int,
            "mode_machine": int,
            "motor_cmd": {
                "positions": [35 joint position commands],
                "velocities": [35 joint velocity commands],
                "torques": [35 joint torque commands],
                "kp": [35 position gains],
                "kd": [35 speed gains]
            }
        }
        """
        try:
            # verify the CRC
            if self.crc.Crc(msg) != msg.crc:
                print(f"g1_robot_dds [{self.node_name}] Warning: CRC verification failed!")
                return {}
            
            # extract the command data
            cmd_data = {
                "mode_pr": int(msg.mode_pr),
                "mode_machine": int(msg.mode_machine),
                "motor_cmd": {
                    "positions": [float(msg.motor_cmd[i].q) for i in range(35)],
                    "velocities": [float(msg.motor_cmd[i].dq) for i in range(35)],
                    "torques": [float(msg.motor_cmd[i].tau) for i in range(35)],
                    "kp": [float(msg.motor_cmd[i].kp) for i in range(35)],
                    "kd": [float(msg.motor_cmd[i].kd) for i in range(35)]
                }
            }
            # print(f"cmd_data: {cmd_data}")
            return cmd_data
            
        except Exception as e:
            print(f"g1_robot_dds [{self.node_name}] Error processing subscribe data: {e}")
            return {}
    
    def get_robot_command(self) -> Optional[Dict[str, Any]]:
        """Get the robot control command
        
        Returns:
            Dict: the robot control command, return None if there is no new command
        """
        if self.output_shm:
            return self.output_shm.read_data()
        return None
    
    def write_robot_state(self, joint_positions, joint_velocities, joint_torques, imu_data):
        """Write the robot state to the shared memory
        
        Args:
            joint_positions: the joint position list or torch.Tensor
            joint_velocities: the joint velocity list or torch.Tensor
            joint_torques: the joint torque list or torch.Tensor
            imu_data: the IMU data list or torch.Tensor
        """
        if self.input_shm is None:
            return
            
        try:
            state_data = {
                "joint_positions": joint_positions.tolist() if hasattr(joint_positions, 'tolist') else joint_positions,
                "joint_velocities": joint_velocities.tolist() if hasattr(joint_velocities, 'tolist') else joint_velocities,
                "joint_torques": joint_torques.tolist() if hasattr(joint_torques, 'tolist') else joint_torques,
                "imu_data": imu_data.tolist() if hasattr(imu_data, 'tolist') else imu_data
            }
            self.input_shm.write_data(state_data)
        except Exception as e:
            print(f"g1_robot_dds [{self.node_name}] Error writing robot state: {e}")
    
    def start_communication(self,enable_publish: bool = True, enable_subscribe: bool = True):
        """Start the G1 robot DDS communication"""
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
            
            print(f"[{self.node_name}] G1 robot DDS communication started")
            
        except Exception as e:
            print(f"[{self.node_name}] Failed to start communication: {e}")
    
    def stop_communication(self):
        """Stop the G1 robot DDS communication"""
        try:
            self.stop()
            print(f"[{self.node_name}] G1 robot DDS communication stopped")
        except Exception as e:
            print(f"g1_robot_dds [{self.node_name}] Failed to stop communication: {e}")
    
    @classmethod
    def get_instance(cls) -> 'G1RobotDDS':
        """Get the singleton instance"""
        return cls()


# convenient functions
def get_g1_robot_dds() -> G1RobotDDS:
    """Get the G1 robot DDS instance"""
    return G1RobotDDS.get_instance()


def start_g1_robot_communication(enable_publish: bool = True, enable_subscribe: bool = True):
    """Start the G1 robot DDS communication
    
    Args:
        enable_publish: whether to enable the publish function (publish the robot state)
        enable_subscribe: whether to enable the subscribe function (receive the control command)
    
    Returns:
        G1RobotDDS: the DDS instance
    """
    g1_dds = get_g1_robot_dds()
    g1_dds.start_communication(enable_publish=enable_publish, enable_subscribe=enable_subscribe)
    return g1_dds


def start_g1_robot_publisher_only():
    """Start the G1 robot state publish function only
    
    Applicable scenarios: only need to publish the robot state, no need to receive the external control command
    """
    return start_g1_robot_communication(enable_publish=True, enable_subscribe=False)


def start_g1_robot_subscriber_only():
    """Start the G1 robot command subscribe function only
    
    Applicable scenarios: only need to receive the external control command, no need to publish the robot state
    """
    return start_g1_robot_communication(enable_publish=False, enable_subscribe=True)


def stop_g1_robot_communication():
    """Stop the G1 robot DDS communication"""
    g1_dds = get_g1_robot_dds()
    g1_dds.stop_communication()


if __name__ == "__main__":
    # test the singleton pattern
    dds1 = G1RobotDDS()
    dds2 = G1RobotDDS()
    print(f"Singleton test: {dds1 is dds2}")  # should be True
    
    # start the communication
    try:
        start_g1_robot_communication()
        
        import time
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("Program interrupted...")
    finally:
        stop_g1_robot_communication() 