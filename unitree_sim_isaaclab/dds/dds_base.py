# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
DDS communication base class module
Provide unified publish/subscribe interface and shared memory management
"""

import json
import time
import threading
import numpy as np
from abc import ABC, abstractmethod
from multiprocessing import shared_memory
from typing import Any, Dict, Optional, Callable
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize


# Global DDS initialization manager
class DDSInitManager:
    """DDS initialization manager, ensure ChannelFactoryInitialize is called only once"""
    
    def __init__(self):
        self._initialized = False
        self._lock = threading.Lock()
    
    def initialize(self) -> bool:
        """Initialize DDS factory
        
        Returns:
            bool: initialization success or not
        """
        with self._lock:
            if self._initialized:
                return True
            
            try:
                ChannelFactoryInitialize(1)
                self._initialized = True
                print("[DDSInitManager] DDS factory initialized successfully")
                return True
            except Exception as e:
                print(f"[DDSInitManager] Failed to initialize DDS factory: {e}")
                return False
    
    def is_initialized(self) -> bool:
        """Check if DDS factory is initialized"""
        return self._initialized


# Global DDS initialization manager instance
dds_init_manager = DDSInitManager()


class SharedMemoryManager:
    """Shared memory manager"""
    
    def __init__(self, name: str = None, size: int = 512):
        """Initialize shared memory manager
        
        Args:
            name: shared memory name, if None, create new one
            size: shared memory size (bytes)
        """
        self.size = size
        self.lock = threading.RLock()  # reentrant lock
        
        if name:
            try:
                self.shm = shared_memory.SharedMemory(name=name)
                self.shm_name = name
                self.created = False
            except FileNotFoundError:
                self.shm = shared_memory.SharedMemory(create=True, size=size)
                self.shm_name = self.shm.name
                self.created = True
        else:
            self.shm = shared_memory.SharedMemory(create=True, size=size)
            self.shm_name = self.shm.name
            self.created = True
    
    def write_data(self, data: Dict[str, Any]) -> bool:
        """Write data to shared memory
        
        Args:
            data: data to write
            
        Returns:
            bool: write success or not
        """
        try:
            with self.lock:
                json_str = json.dumps(data)
                json_bytes = json_str.encode('utf-8')
                
                if len(json_bytes) > self.size - 8:  # reserve 8 bytes for length and timestamp
                    print(f"Warning: Data too large for shared memory ({len(json_bytes)} > {self.size - 8})")
                    return False
                
                # write timestamp (4 bytes) and data length (4 bytes)
                timestamp = int(time.time()) & 0xFFFFFFFF  # 32-bit timestamp, use bitmask to ensure in range
                self.shm.buf[0:4] = timestamp.to_bytes(4, 'little')
                self.shm.buf[4:8] = len(json_bytes).to_bytes(4, 'little')
                
                # write data
                self.shm.buf[8:8+len(json_bytes)] = json_bytes
                return True
                
        except Exception as e:
            print(f"Error writing to shared memory: {e}")
            return False
    
    def read_data(self) -> Optional[Dict[str, Any]]:
        """Read data from shared memory
        
        Returns:
            Dict[str, Any]: read data dictionary, return None if failed
        """
        try:
            with self.lock:
                # read timestamp and data length
                timestamp = int.from_bytes(self.shm.buf[0:4], 'little')
                data_len = int.from_bytes(self.shm.buf[4:8], 'little')
                
                if data_len == 0:
                    return None
                
                # read data
                json_bytes = bytes(self.shm.buf[8:8+data_len])
                data = json.loads(json_bytes.decode('utf-8'))
                data['_timestamp'] = timestamp  # add timestamp information
                return data
                
        except Exception as e:
            print(f"Error reading from shared memory: {e}")
            return None
    
    def get_name(self) -> str:
        """Get shared memory name"""
        return self.shm_name
    
    def cleanup(self):
        """Clean up shared memory"""
        if hasattr(self, 'shm') and self.shm:
            self.shm.close()
            if self.created:
                try:
                    self.shm.unlink()
                except:
                    pass
    
    def __del__(self):
        """Destructor"""
        self.cleanup()


class BaseDDSNode(ABC):
    """DDS node base class"""
    
    def __init__(self, node_name: str):
        """Initialize DDS node
        
        Args:
            node_name: node name
        """
        self.node_name = node_name
        self.running = False
        self.publish_started = False
        self.subscribe_started = False
        # thread management
        self.publish_thread = None
        self.subscribe_thread = None
        
        # shared memory manager
        self.input_shm = None   # for reading input data (publish)
        self.output_shm = None  # for writing output data (subscribe)
        
        # DDS components
        self.publisher = None
        self.subscriber = None
        
        # callback functions
        self.publish_callback = None
        self.subscribe_callback = None
        
        print(f"[{self.node_name}] DDS Node initialized")
    
    def setup_shared_memory(self, input_shm_name: str = None, output_shm_name: str = None, 
                           input_size: int = 4096, output_size: int = 4096,inputshm_flag:bool=True,outputshm_flag:bool=True):
        """Setup shared memory
        
        Args:
            input_shm_name: input shared memory name
            output_shm_name: output shared memory name
            input_size: input shared memory size
            output_size: output shared memory size
        """
        if inputshm_flag:
            if input_shm_name:
                self.input_shm = SharedMemoryManager(input_shm_name, input_size)
                print(f"[{self.node_name}] Input shared memory: {self.input_shm.get_name()}")
            else:
                self.input_shm = SharedMemoryManager(size=input_size)
                print(f"[{self.node_name}] Input shared memory: {self.input_shm.get_name()}")
        if outputshm_flag:
            if output_shm_name:
                self.output_shm = SharedMemoryManager(output_shm_name, output_size)
                print(f"[{self.node_name}] Output shared memory: {self.output_shm.get_name()}")
            else:
                self.output_shm = SharedMemoryManager(size=output_size)
                print(f"[{self.node_name}] Output shared memory: {self.output_shm.get_name()}")
    
    @abstractmethod
    def setup_publisher(self) -> bool:
        """Setup publisher (subclass implementation)
        
        Returns:
            bool: setup success or not
        """
        pass
    
    @abstractmethod
    def setup_subscriber(self) -> bool:
        """Setup subscriber (subclass implementation)
        
        Returns:
            bool: setup success or not
        """
        pass
    
    @abstractmethod
    def process_publish_data(self, data: Dict[str, Any]) -> Any:
        """Process publish data (subclass implementation)
        
        Args:
            data: data read from shared memory
            
        Returns:
            Any: processed DDS message object
        """
        pass
    
    @abstractmethod
    def process_subscribe_data(self, msg: Any) -> Dict[str, Any]:
        """Process subscribe data (subclass implementation)
        
        Args:
            msg: received DDS message
            
        Returns:
            Dict[str, Any]: data to write to shared memory after processing
        """
        pass
    
    def _publish_loop(self):
        """Publish loop thread"""
        print(f"[{self.node_name}] Publish thread started")
        
        while self.running:
            try:
                if self.input_shm and self.publisher:
                    # read data from shared memory
                    data = self.input_shm.read_data()
                    if data:
                        # process data and publish
                        msg = self.process_publish_data(data)
                        if msg:
                            self.publisher.Write(msg)
                            
                        # call callback function
                        if self.publish_callback:
                            self.publish_callback(data, msg)
                elif self.publisher:
                    msg = self.process_publish_data("")
                    if msg:
                        self.publisher.Write(msg)
                        
                    # call callback function
                    if self.publish_callback:
                        self.publish_callback("", msg)
                elif self.publisher is None:
                    print(f"[{self.node_name}] Warning: publisher is empty, cannot start publish thread")
                else:
                    print(f"[{self.node_name}] Warning: publisher is not set, cannot start publish thread")
                time.sleep(0.001)  # 1ms循环
                
            except Exception as e:
                print(f"[{self.node_name}] Error in publish loop: {e}")
                time.sleep(0.01)
        
        print(f"[{self.node_name}] Publish thread stopped")
    
    def _subscribe_message_handler(self, msg: Any):
        """Subscribe message handler"""
        try:
            # process received message
            # print(f"[{self.node_name}] 收到DDS消息: {msg}")
            data = self.process_subscribe_data(msg)
            if data and self.output_shm:
                # write to shared memory
                self.output_shm.write_data(data)
            
            # call callback function
            if self.subscribe_callback:
                self.subscribe_callback(msg, data)
                
        except Exception as e:
            print(f"[{self.node_name}] Error processing subscribe message: {e}")
    
    def _subscribe_loop(self):
        """Subscribe loop thread (keep subscriber alive)"""
        print(f"[{self.node_name}] Subscribe thread started")
        
        loop_count = 0
        while self.running:
            try:
                time.sleep(0.001)  # keep thread alive
                loop_count += 1
            except Exception as e:
                print(f"[{self.node_name}] Error in subscribe loop: {e}")
                time.sleep(0.01)
        
        print(f"[{self.node_name}] Subscribe thread stopped")
    
    def start(self, enable_publish: bool = True, enable_subscribe: bool = True):
        """Start DDS node
        
        Args:
            enable_publish: whether to enable publish
            enable_subscribe: whether to enable subscribe
        """
        if self.running and self.publisher is not None and self.subscriber is not None:
            print(f"[{self.node_name}] Node already running")
            return
        
        try:
            # initialize DDS (only initialize once globally)
            if not dds_init_manager.initialize():
                print(f"[{self.node_name}] Failed to initialize DDS factory")
                return
            
            # setup publisher
            if enable_publish and self.publisher is None and not self.publish_started:
                if not self.setup_publisher():
                    print(f"[{self.node_name}] Failed to setup publisher")
                    return
            
            # setup subscriber  
            if enable_subscribe and self.subscriber is None and not self.subscribe_started:
                print(f"[{self.node_name}] Setting up subscriber...")
                if not self.setup_subscriber():
                    print(f"[{self.node_name}] Failed to setup subscriber")
                    return
                else:
                    print(f"[{self.node_name}] Subscriber setup successfully")
            
            self.running = True
            
            # start publish thread
            if enable_publish and self.publisher and not self.publish_started:
                print(f"[{self.node_name}] Starting publish thread...")
                self.publish_thread = threading.Thread(target=self._publish_loop)
                self.publish_thread.daemon = True
                self.publish_thread.start()
                self.publish_started = True
                print(f"[{self.node_name}] Publish thread started")
            else:
                print(f"[{self.node_name}] Warning: publish is started")
            # start subscribe thread
            if enable_subscribe and self.subscriber and not self.subscribe_started:
                print(f"[{self.node_name}] Starting subscribe thread...")
                self.subscribe_thread = threading.Thread(target=self._subscribe_loop)
                self.subscribe_thread.daemon = True
                self.subscribe_thread.start()
                self.subscribe_started = True
                print(f"[{self.node_name}] Subscribe thread started")
            else:
                print(f"[{self.node_name}] Warning: subscribe is started")
            print(f"[{self.node_name}] Node started successfully")
            
        except Exception as e:
            print(f"[{self.node_name}] Failed to start node: {e}")
            self.stop()
    
    def stop(self):
        """Stop DDS node""" 
        if not self.running:
            return
        
        print(f"[{self.node_name}] Stopping node...")
        self.running = False
        
        # wait for threads to finish
        if self.publish_thread:
            self.publish_thread.join(timeout=1.0)
        if self.subscribe_thread:
            self.subscribe_thread.join(timeout=1.0)
        
        print(f"[{self.node_name}] Node stopped")
    
    def set_publish_callback(self, callback: Callable[[Dict[str, Any], Any], None]):
        """Set publish callback function
        
        Args:
            callback: callback function, parameters are (input_data, published_msg)
        """
        self.publish_callback = callback
    
    def set_subscribe_callback(self, callback: Callable[[Any, Dict[str, Any]], None]):
        """Set subscribe callback function
        
        Args:
            callback: callback function, parameters are (received_msg, output_data)
        """
        self.subscribe_callback = callback
    
    def get_input_shm_name(self) -> Optional[str]:
        """Get input shared memory name"""
        return self.input_shm.get_name() if self.input_shm else None
    
    def get_output_shm_name(self) -> Optional[str]:
        """Get output shared memory name"""
        return self.output_shm.get_name() if self.output_shm else None
    
    def cleanup(self):
        """Clean up resources"""
        self.stop()
        if self.input_shm:
            self.input_shm.cleanup()
        if self.output_shm:
            self.output_shm.cleanup()
    
    def __del__(self):
        """Destructor"""
        self.cleanup()


class DDSNodeManager:
    """DDS node manager
    
    Manage the lifecycle of multiple DDS nodes, ensure correct initialization and cleanup.
    Use global DDSInitManager to ensure ChannelFactoryInitialize is called only once.
    """
    
    def __init__(self):
        """Initialize node manager"""
        self.nodes = {}
        print("[DDSNodeManager] Manager initialized")
    
    def register_node(self, node: BaseDDSNode):
        """Register node
        
        Args:
            node: DDS node instance
        """
        self.nodes[node.node_name] = node
        print(f"[DDSNodeManager] Node '{node.node_name}' registered")
    
    def start_all_nodes(self):
        """Start all nodes"""
        for name, node in self.nodes.items():
            try:
                node.start()
                print(f"[DDSNodeManager] Node '{name}' started")
            except Exception as e:
                print(f"[DDSNodeManager] Failed to start node '{name}': {e}")
    
    def stop_all_nodes(self):
        """Stop all nodes"""
        for name, node in self.nodes.items():
            try:
                node.stop()
                print(f"[DDSNodeManager] Node '{name}' stopped")
            except Exception as e:
                print(f"[DDSNodeManager] Failed to stop node '{name}': {e}")
    
    def get_node(self, name: str) -> Optional[BaseDDSNode]:
        """Get node
        
        Args:
            name: node name
            
        Returns:
            BaseDDSNode: node instance, return None if not found
        """
        return self.nodes.get(name)
    
    def list_nodes(self) -> Dict[str, str]:
        """List all nodes
        
        Returns:
            Dict[str, str]: node name to state mapping
        """
        return {name: "running" if node.running else "stopped" 
                for name, node in self.nodes.items()}
    
    def cleanup(self):
        """Clean up all nodes"""
        self.stop_all_nodes()
        for node in self.nodes.values():
            node.cleanup()
        self.nodes.clear()
        print("[DDSNodeManager] All nodes cleaned up")
    
    def __del__(self):
        """Destructor"""
        self.cleanup()


# Global node manager instance
node_manager = DDSNodeManager()

# Clean up resources when program exits
import atexit
atexit.register(node_manager.cleanup) 