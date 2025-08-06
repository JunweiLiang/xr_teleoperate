from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_                           # idl
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_

from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
import numpy as np
from enum import IntEnum
import threading
import time
from multiprocessing import Process, Array

import logging_mp
logger_mp = logging_mp.get_logger(__name__)

Inspire_Num_Motors = 6
kTopicInspireCommand = "rt/inspire/cmd"
kTopicInspireState = "rt/inspire/state"

class Inspire_Controller:
    def __init__(self, left_hand_array, right_hand_array, dual_hand_data_lock = None, dual_hand_state_array = None,
                       dual_hand_action_array = None, fps = 100.0, Unit_Test = False, simulation_mode = False):
        logger_mp.info("Initialize Inspire_Controller...")
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
        else:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND_Unit_Test)

        if self.simulation_mode:
            ChannelFactoryInitialize(1)
        else:
            ChannelFactoryInitialize(0)

        # initialize handcmd publisher and handstate subscriber
        self.HandCmb_publisher = ChannelPublisher(kTopicInspireCommand, MotorCmds_)
        self.HandCmb_publisher.Init()

        self.HandState_subscriber = ChannelSubscriber(kTopicInspireState, MotorStates_)
        self.HandState_subscriber.Init()

        # Shared Arrays for hand states
        self.left_hand_state_array  = Array('d', Inspire_Num_Motors, lock=True)  
        self.right_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)

        # initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        while True:
            if any(self.right_hand_state_array): # any(self.left_hand_state_array) and 
                break
            time.sleep(0.01)
            logger_mp.warning("[Inspire_Controller] Waiting to subscribe dds...")
        logger_mp.info("[Inspire_Controller] Subscribe dds ok.")

        hand_control_process = Process(target=self.control_process, args=(left_hand_array, right_hand_array,  self.left_hand_state_array, self.right_hand_state_array,
                                                                          dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array))
        hand_control_process.daemon = True
        hand_control_process.start()

        logger_mp.info("Initialize Inspire_Controller OK!\n")

    def _subscribe_hand_state(self):
        while True:
            hand_msg  = self.HandState_subscriber.Read()
            if hand_msg is not None:
                for idx, id in enumerate(Inspire_Left_Hand_JointIndex):
                    self.right_hand_state_array[idx] = hand_msg.states[id].q
                for idx, id in enumerate(Inspire_Right_Hand_JointIndex):
                    self.left_hand_state_array[idx] = hand_msg.states[id].q
            time.sleep(0.002)

    def ctrl_dual_hand(self, left_q_target, right_q_target):
        """
        Set current left, right hand motor state target q
        """
        for idx, id in enumerate(Inspire_Left_Hand_JointIndex):             
            self.hand_msg.cmds[id].q = left_q_target[idx]         
        for idx, id in enumerate(Inspire_Right_Hand_JointIndex):             
            self.hand_msg.cmds[id].q = right_q_target[idx] 

        self.HandCmb_publisher.Write(self.hand_msg)
        # logger_mp.debug("hand ctrl publish ok.")
    
    def control_process(self, left_hand_array, right_hand_array, left_hand_state_array, right_hand_state_array,
                              dual_hand_data_lock = None, dual_hand_state_array = None, dual_hand_action_array = None):
        self.running = True
        # 这个遥操作一开始就在跑了，最高100 Hz检查left_hand_pos_array, 这个是从OpenXR获取到的手

        left_q_target  = np.full(Inspire_Num_Motors, 1.0)
        right_q_target = np.full(Inspire_Num_Motors, 1.0)

        # initialize inspire hand's cmd msg
        self.hand_msg  = MotorCmds_()
        self.hand_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(len(Inspire_Right_Hand_JointIndex) + len(Inspire_Left_Hand_JointIndex))]

        for idx, id in enumerate(Inspire_Left_Hand_JointIndex):
            self.hand_msg.cmds[id].q = 1.0
        for idx, id in enumerate(Inspire_Right_Hand_JointIndex):
            self.hand_msg.cmds[id].q = 1.0

        try:
            while self.running:
                start_time = time.time()
                # get dual hand state
                with left_hand_array.get_lock():
                    left_hand_data  = np.array(left_hand_array[:]).reshape(25, 3).copy()
                with right_hand_array.get_lock():
                    right_hand_data = np.array(right_hand_array[:]).reshape(25, 3).copy()

                # Read left and right q_state from shared arrays
                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                if not np.all(right_hand_data == 0.0) and not np.all(left_hand_data[4] == np.array([-1.13, 0.3, 0.15])): # if hand data has been initialized.
                    # # left_indices (2, 15)? # assets/inspire_hand/inspire_hand.xml -> target_link_human_indices_dexpilot
                    ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                    ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                    # retarget() from teleop/robot_control/dex-retargeting/src/dex_retargeting/retargeting_config.py -> build(), also see seq_retarget.py
                    # retarget() function的时候返回robot_qpos
                    # self.hand_retargeting.left_dex_retargeting_to_hardware -> [5, 4, 3, 2, 1, 0]
                    # 所以这个q_target，长度是6，顺序就是 self.left_inspire_api_joint_names from teleop/robot_control/hand_retargeting.py
                    """
                    self.left_inspire_api_joint_names  = [
                        'L_pinky_proximal_joint',
                        'L_ring_proximal_joint',
                        'L_middle_proximal_joint',
                        'L_index_proximal_joint',
                        'L_thumb_proximal_pitch_joint',
                        'L_thumb_proximal_yaw_joint' ]
                    """
                    left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                    # In website https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand, you can find
                    #     In the official document, the angles are in the range [0, 1] ==> 0.0: fully closed  1.0: fully open
                    # The q_target now is in radians, ranges:
                    #     - idx 0~3: 0~1.7 (1.7 = closed)
                    #     - idx 4:   0~0.5
                    #     - idx 5:  -0.1~1.3
                    # We normalize them using (max - value) / range
                    # so this will reverse the 0.0 fully open to fully close
                    def normalize(val, min_val, max_val):
                        return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

                    # 把原本 URDF joint的关节目标角度，转化成[0, 1], 这个才能输出给因时的API? 而且1.0是开手，URDF是0.0才是开手
                    for idx in range(Inspire_Num_Motors):
                        if idx <= 3:
                            # pinky_proximal_joint, limits: [0.000, 1.700]
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.7)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)

                        #joint id: 41, name: R_thumb_proximal_yaw_joint, limits: [-0.100, 1.300]
                        #joint id: 42, name: R_thumb_proximal_pitch_joint, limits: [-0.100, 0.600]
                        elif idx == 4: # 'L_thumb_proximal_pitch_joint',
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 0.5)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)

                        elif idx == 5: # 'L_thumb_proximal_yaw_joint'
                            left_q_target[idx]  = normalize(left_q_target[idx], -0.1, 1.3)
                            right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)

                # get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target))    
                if dual_hand_state_array and dual_hand_action_array:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                self.ctrl_dual_hand(left_q_target, right_q_target)
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("Inspire_Controller has been closed.")

class Inspire_Gripper_Controller:
    def __init__(self, left_gripper_value_in, right_gripper_value_in, dual_hand_data_lock = None, dual_hand_state_array = None,
                       dual_hand_action_array = None, fps = 100.0, Unit_Test = False, simulation_mode = False):
        logger_mp.info("Initialize Inspire_Controller...")
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode

        if self.simulation_mode:
            ChannelFactoryInitialize(1)
        else:
            ChannelFactoryInitialize(0)

        # initialize handcmd publisher and handstate subscriber
        self.HandCmb_publisher = ChannelPublisher(kTopicInspireCommand, MotorCmds_)
        self.HandCmb_publisher.Init()

        self.HandState_subscriber = ChannelSubscriber(kTopicInspireState, MotorStates_)
        self.HandState_subscriber.Init()

        # Shared Arrays for hand states
        self.left_hand_state_array  = Array('d', Inspire_Num_Motors, lock=True)
        self.right_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)

        # initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        while True:
            if any(self.right_hand_state_array): # any(self.left_hand_state_array) and
                break
            time.sleep(0.01)
            logger_mp.warning("[Inspire_Controller] Waiting to subscribe dds...")
        logger_mp.info("[Inspire_Controller] Subscribe dds ok.")

        # 以下thread就会从一开始获取OpenXR数据，控制手，并且存下手的state和action
        hand_control_process = Process(
            target=self.control_process,
            args=(
                left_gripper_value_in, right_gripper_value_in,
                self.left_hand_state_array, self.right_hand_state_array,
                dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array))
        hand_control_process.daemon = True
        hand_control_process.start()

        logger_mp.info("Initialize Inspire_Controller OK!\n")

    def _subscribe_hand_state(self):
        while True:
            hand_msg  = self.HandState_subscriber.Read()
            if hand_msg is not None:
                for idx, id in enumerate(Inspire_Left_Hand_JointIndex):
                    self.right_hand_state_array[idx] = hand_msg.states[id].q
                for idx, id in enumerate(Inspire_Right_Hand_JointIndex):
                    self.left_hand_state_array[idx] = hand_msg.states[id].q
            time.sleep(0.002)

    def ctrl_dual_hand(self, left_q_target, right_q_target):
        """
        Set current left, right hand motor state target q
        """
        for idx, id in enumerate(Inspire_Left_Hand_JointIndex):
            self.hand_msg.cmds[id].q = left_q_target[idx]
        for idx, id in enumerate(Inspire_Right_Hand_JointIndex):
            self.hand_msg.cmds[id].q = right_q_target[idx]

        self.HandCmb_publisher.Write(self.hand_msg)
        # logger_mp.debug("hand ctrl publish ok.")

     # We normalize them using (max - value) / range
    # so this will reverse the 0.0 fully open to fully close
    def normalize(self, val, min_val, max_val):
        return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

    def control_process(self, left_gripper_value_in, right_gripper_value_in, left_hand_state_array, right_hand_state_array,
                              dual_hand_data_lock = None, dual_hand_state_array = None, dual_hand_action_array = None):
        self.running = True
        # 这个遥操作一开始就在跑了，最高100 Hz检查left_hand_pos_array, 这个是从OpenXR获取到的手



        # initialize inspire hand's cmd msg
        self.hand_msg  = MotorCmds_()
        self.hand_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(len(Inspire_Right_Hand_JointIndex) + len(Inspire_Left_Hand_JointIndex))]

        for idx, id in enumerate(Inspire_Left_Hand_JointIndex):
            self.hand_msg.cmds[id].q = 1.0
        for idx, id in enumerate(Inspire_Right_Hand_JointIndex):
            self.hand_msg.cmds[id].q = 1.0

        # 构建手全打开的初始状态
        #left_q_target  = np.full(Inspire_Num_Motors, 1.0) # (6,)
        #right_q_target = np.full(Inspire_Num_Motors, 1.0)
        # 顺序
        """
            self.left_inspire_api_joint_names  = [
                'L_pinky_proximal_joint',
                'L_ring_proximal_joint',
                'L_middle_proximal_joint',
                'L_index_proximal_joint',
                'L_thumb_proximal_pitch_joint',
                'L_thumb_proximal_yaw_joint' ]
        """


        # 注意，以上的q值，是inspire API定义的，0.0是关闭，1.0是打开

        # 以下都是根据URDF中定义的弧度， 0.0是打开
        # 定义 gripper 全打开状态，拇指对齐食指方向：
            # L_thumb_proximal_yaw_joint: 1.3
            # R_thumb_proximal_yaw_joint: 1.3
            # 如[图](./g1_inspire_open.png)
        # so
        THUMB_YAW = 1.3  # 打开，关闭，手拇指的yaw都不变，保持和食指对齐

        # 定义 gripper 全关闭状态，以下设置了主动关节，留有些空间，实机为连杆结构，应该拇指尖和食指尖接触：
            # R_thumb_proximal_yaw_joint: 1.3
            # R_thumb_proximal_pitch_joint: 0.6
            # R_index_proximal_joint: 0.756
            # R_middle_proximal_joint: 0.756
            # R_ring_proximal_joint: 0.756
            # R_pinky_proximal_joint: 0.756
            # L的数值也是一样的
            # 如[图](./g1_inspire_close.png)

        CLOSE_THUMB_PITCH = 0.6 # open is 0.0
        CLOSE_OTHER_JOINT = 0.756 # open is 0.0

        try:
            while self.running:
                start_time = time.time()
                # 构建手全打开的初始状态作为默认
                left_q_target  = np.full(Inspire_Num_Motors, 1.0) # (6,)
                right_q_target = np.full(Inspire_Num_Motors, 1.0)

                # get dual trigger command from XR device
                with left_gripper_value_in.get_lock():
                    left_gripper_value  = left_gripper_value_in.value
                with right_gripper_value_in.get_lock():
                    right_gripper_value = right_gripper_value_in.value
                logger_mp.info("right gripper value: %s" % right_gripper_value)
                # in the following, we map the gripper value [0.0, 1.0] to the hand action

                # Read left and right q_state from shared arrays
                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))


                if left_gripper_value != 0.0 or right_gripper_value != 0.0: # if input data has been initialized.
                    # Linear mapping from [0, 1.0] to hand action range

                    left_q_target[4] = np.interp(left_gripper_value, [0.0, 1.0], [0.0, CLOSE_THUMB_PITCH])
                    left_q_target[:4] = np.interp(left_gripper_value, [0.0, 1.0], [0.0, CLOSE_OTHER_JOINT])

                    right_q_target[4] = np.interp(right_gripper_value, [0.0, 1.0], [0.0, CLOSE_THUMB_PITCH])
                    right_q_target[:4] = np.interp(right_gripper_value, [0.0, 1.0], [0.0, CLOSE_OTHER_JOINT])


                    # In website https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand, you can find
                    #     In the official document, the angles are in the range [0, 1] ==> 0.0: fully closed  1.0: fully open
                    # The q_target now is in radians, ranges:
                    #     - idx 0~3: 0~1.7 (1.7 = closed)
                    #     - idx 4:   0~0.5
                    #     - idx 5:  -0.1~1.3


                    # 把原本 URDF joint的关节目标角度，转化成[0, 1], 这个才能输出给因时的API? 而且1.0是开手，URDF是0.0才是开手
                    for idx in range(Inspire_Num_Motors):
                        if idx <= 3:
                            # pinky_proximal_joint, limits: [0.000, 1.700]
                            left_q_target[idx]  = self.normalize(left_q_target[idx], 0.0, 1.7)
                            right_q_target[idx] = self.normalize(right_q_target[idx], 0.0, 1.7)

                        #joint id: 41, name: R_thumb_proximal_yaw_joint, limits: [-0.100, 1.300]
                        #joint id: 42, name: R_thumb_proximal_pitch_joint, limits: [-0.100, 0.600]
                        elif idx == 4: # 'L_thumb_proximal_pitch_joint',
                            left_q_target[idx]  = self.normalize(left_q_target[idx], 0.0, 0.6)
                            right_q_target[idx] = self.normalize(right_q_target[idx], 0.0, 0.6)


                # 无论手打开关闭，拇指保持和食指对齐
                # 'L_thumb_proximal_yaw_joint'
                left_q_target[5] = THUMB_YAW
                right_q_target[5] = THUMB_YAW
                left_q_target[5]  = self.normalize(left_q_target[idx], -0.1, 1.3)
                right_q_target[5] = self.normalize(right_q_target[idx], -0.1, 1.3)

                # get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target))
                logger_mp.info("action data: %s" % action_data)
                """
                0.55529 0.55529 0.55529   0.55529 0.      0.
                 0.55529 0.55529 0.55529   0.55529 0.      0.
                1.      1.  1.      1.      1.      0.
                """
                if dual_hand_state_array and dual_hand_action_array:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                self.ctrl_dual_hand(left_q_target, right_q_target)
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("Inspire_Gripper_Controller has been closed.")


# Update hand state, according to the official documentation, https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand
# the state sequence is as shown in the table below
# ┌──────┬───────┬──────┬────────┬────────┬────────────┬────────────────┬───────┬──────┬────────┬────────┬────────────┬────────────────┐
# │ Id   │   0   │  1   │   2    │   3    │     4      │       5        │   6   │  7   │   8    │   9    │    10      │       11       │
# ├──────┼───────┼──────┼────────┼────────┼────────────┼────────────────┼───────┼──────┼────────┼────────┼────────────┼────────────────┤
# │      │                    Right Hand                                │                   Left Hand                                  │
# │Joint │ pinky │ ring │ middle │ index  │ thumb-bend │ thumb-rotation │ pinky │ ring │ middle │ index  │ thumb-bend │ thumb-rotation │
# └──────┴───────┴──────┴────────┴────────┴────────────┴────────────────┴───────┴──────┴────────┴────────┴────────────┴────────────────┘
class Inspire_Right_Hand_JointIndex(IntEnum):
    kRightHandPinky = 0
    kRightHandRing = 1
    kRightHandMiddle = 2
    kRightHandIndex = 3
    kRightHandThumbBend = 4
    kRightHandThumbRotation = 5

class Inspire_Left_Hand_JointIndex(IntEnum):
    kLeftHandPinky = 6
    kLeftHandRing = 7
    kLeftHandMiddle = 8
    kLeftHandIndex = 9
    kLeftHandThumbBend = 10
    kLeftHandThumbRotation = 11
