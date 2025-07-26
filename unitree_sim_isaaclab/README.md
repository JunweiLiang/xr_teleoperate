<div align="center">
  <h1 align="center"> unitree_sim_isaaclab </h1>
  <h3 align="center"> Unitree Robotics </h3>
  <p align="center">
    <a> English </a> | <a href="README_zh-CN.md">中文</a> 
  </p>
</div>

## Important Notes First
- Please use the [officially recommended](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html) hardware resources for deployment
- The simulator may take some time to load resources during its first startup, and the waiting time depends on hardware performance and network environment
- After the simulator starts running, it will send/receive the same DDS topics as the real robot (Please note to distinguish between the simulator and real robot if there is a real robot running on the same network). For specific DDS usage, please refer to[G1 Control](https://github.com/unitreerobotics/unitree_sdk2_python/tree/master/example/g1) and [Dex3 Dexterous Hand Control](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/dex3/g1_dex3_example.cpp)
- After the virtual scene starts up, please click PerspectiveCamera -> Cameras -> PerspectiveCamera to view the main view scene. The operation steps are shown below:
<table align="center">
    <tr>
    <td align="center">
        <img src="./img/mainview.png" width="300" alt="G1-gripper-cylinder"/>
      <br/>
      <code>Main View Finding Steps</code>
    </td>
    </tr>
</table>

## 1、 📖 Introduction

This project is built on **Isaac Lab** to simulate **Unitree robots** in various tasks, facilitating data collection, playback, generation, and model validation. It can be used in conjunction with the [xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate) repository for dataset collection. The project adopts the same DDS communication protocol as the real robot to enhance code generality and ease of use.


Currently, this project uses Unitree G1 with gripper (G1-29dof-gripper) and Unitree G1 with three-finger dexterous hand (G1-29dof-dex3) to build simulation scenarios for different tasks. The specific task scene names and illustrations are shown in the table below:

<table align="center">
  <tr>
    <th>G1-29dof-gripper</th>
    <th>G1-29dof-dex3</th>
    <th>G1-29dof-inspire</th>
  </tr>
  <tr>
    <td align="center">
      <img src="./img/pickplace_clinder_g129_dex1.png" width="300" alt="G1-gripper-cylinder"/>
      <br/>
      <code>Isaac-PickPlace-Cylinder-G129-Dex1-Joint</code>
    </td>
    <td align="center">
      <img src="./img/pickplace_clinder_g129_dex3.png" width="300" alt="G1-dex3-cylinder"/>
      <br/>
      <code>Isaac-PickPlace-Cylinder-G129-Dex3-Joint</code>
    </td>
    <td align="center">
      <img src="./img/Isaac-PickPlace-Cylinder-G129-Inspire-Joint.png" width="300" alt="G1-dex3-cylinder"/>
      <br/>
      <code>Isaac-PickPlace-Cylinder-G129-Inspire-Joint</code>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="./img/pickplace_redblock_g129_dex1.png" width="300" alt="G1-gripper-redblock"/>
      <br/>
      <code>Isaac-PickPlace-RedBlock-G129-Dex1-Joint</code>
    </td>
    <td align="center">
      <img src="./img/pickplace_redblock_g129_dex3.png" width="300" alt="G1-dex3-redblock"/>
      <br/>
      <code>Isaac-PickPlace-RedBlock-G129-Dex3-Joint</code>
    </td>
    <td align="center">
      <img src="./img/Isaac-PickPlace-RedBlock-G129-Inspire-Joint.png" width="300" alt="G1-dex3-redblock"/>
      <br/>
      <code>Isaac-PickPlace-RedBlock-G129-Inspire-Joint</code>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="./img/stack_rgyblock_g129_dex1.png" width="300" alt="G1-gripper-redblock"/>
      <br/>
      <code>Isaac-Stack-RgyBlock-G129-Dex1-Joint</code>
    </td>
    <td align="center">
      <img src="./img/stack_rgyblock_g129_dex3.png" width="300" alt="G1-dex3-redblock"/>
      <br/>
      <code>Isaac-Stack-RgyBlock-G129-Dex3-Joint</code>
    </td>
    <td align="center">
      <img src="./img/Isaac-Stack-RgyBlock-G129-Inspire-Joint.png" width="300" alt="G1-dex3-redblock"/>
      <br/>
      <code>Isaac-Stack-RgyBlock-G129-Inspire-Joint</code>
    </td>
  </tr>
</table>

## 2、⚙️ Environment Setup and Running

This project requires Isaac Sim 4.5.0 and Isaac Lab. You can refer to the [official installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html)  or follow the steps below. The installation steps differ between Ubuntu 20.04 and Ubuntu 22.04 or later, so please follow the appropriate instructions based on your system version.

### 2.1 Installation on Ubuntu 22.04 and Later

- **Create Virtual Environment**

```
conda create -n unitree_sim_env python=3.10
conda activate unitree_sim_env
```
- **Install Pytorch**

This needs to be installed according to your CUDA version. Please refer to the [official PyTorch installation guide](https://pytorch.org/get-started/locally/). The following example uses CUDA 12:

```
pip install torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu121
```
- **Install Isaac Sim 4.5.0**

```
pip install --upgrade pip

pip install 'isaacsim[all,extscache]==4.5.0' --extra-index-url https://pypi.nvidia.com
```
Verify successful installation:
```
isaacsim
```
First execution will show: Do you accept the EULA? (Yes/No):  Yes

-  **Install Isaac Lab**

The current IsaacLab version used is 91ad4944f2b7fad29d52c04a5264a082bcaad71d

```
git clone git@github.com:isaac-sim/IsaacLab.git

sudo apt install cmake build-essential

cd IsaacLab

./isaaclab.sh --install 

```

Verify successful installation:
```
python scripts/tutorials/00_sim/create_empty.py
or
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
```

- **Install unitree_sdk2_python**

```
git clone https://github.com/unitreerobotics/unitree_sdk2_python

cd unitree_sdk2_python

pip3 install -e .
```

- **Install other dependencies**
```
pip install -r requirements.txt
```

### 2.2 Installation on Ubuntu 20.04

- **Download Isaac Sim Binary**

Download the [Isaac Sim 4.5.0 binary](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html) and extract it.

Assume the path to Isaac Sim is ``/home/unitree/tools/isaac-sim``. Follow the steps below:

- **Set environment variables**

Please replace with your own path

```
export ISAACSIM_PATH="${HOME}/tools/isaac-sim"            
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"  
```
Verify the setup:
```
${ISAACSIM_PATH}/isaac-sim.sh
# or
${ISAACSIM_PYTHON_EXE} -c "print('Isaac Sim configuration is now complete.')"

Note: All conda environments (including base) must be deactivated before running this.
```
**Note:** You can add the above commands to your ~/.bashrc file for convenience.

- **Install Isaac Lab**

Using IsaacLab commit `91ad4944f2b7fad29d52c04a5264a082bcaad71d`

```
git clone git@github.com:isaac-sim/IsaacLab.git

sudo apt install cmake build-essential

cd IsaacLab

ln -s ${HOME}/tools/isaac-sim/ _isaac_sim     (Please replace with your own path)

./isaaclab.sh --conda unitree_sim_env

conda activate unitree_sim_env

./isaaclab.sh --install

```

- **Install unitree_sdk2_python**

```
git clone https://github.com/unitreerobotics/unitree_sdk2_python

cd unitree_sdk2_python

pip3 install -e .

```

- **Install other dependencies**

```
pip install -r requirements.txt
```
### 2.3 Run Program

#### 2.3.1 Teleoperation

```
python sim_main.py --device cpu  --enable_cameras  --task  Isaac-PickPlace-Cylinder-G129-Dex1-Joint    --enable_gripper_dds --robot_type g129
```

- --task: Task name, corresponding to the task names in the table above
- --enable_gripper_dds/--enable_dex3_dds: Represent enabling DDS for two-finger gripper/three-finger dexterous hand respectively  
- --robot_type: Robot type, currently has 29-DOF unitree g1 (g129)

#### 2.3.2 Data Replay

```
python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-G129-Dex1-Joint     --enable_gripper_dds --robot_type g129 --replay  --file_path "/home/unitree/Code/xr_teleoperate/teleop/utils/data" 
```
- --replay: Specifies whether to perform data replay.

- --file_path: Directory where the dataset is stored (please update this to your own dataset path).


**Note:** The dataset format used here is consistent with the one recorded via teleoperation in [xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate) .

#### 2.3.3 Data Generation
During data replay, by modifying lighting conditions and camera parameters and re-capturing image data, more diverse visual features can be generated for data augmentation, thereby improving the model’s generalization ability.
```
python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-G129-Dex1-Joint     --enable_gripper_dds --robot_type g129 --replay  --file_path "/home/unitree/Code/xr_teleoperate/teleop/utils/data" --generate_data --generate_data_dir "./data2"
```

- --generate_data: Enables generation of new data.

- --generate_data_dir: Directory to store the newly generated data.

- --rerun_log: Enables logging during data generation.

- --modify_light: Enables modification of lighting conditions (you need to adjust the update_light function in main accordingly).

- --modify_camera: Enables modification of camera parameters (you need to adjust the batch_augment_cameras_by_name function in main accordingly).

**Note:**
If you wish to modify lighting or camera parameters, please tune and test the parameters carefully before performing large-scale data generation.

## 3、Task Scene Construction

### 3.1 Code Structure

```
unitree_sim_isaaclab/
│
├── action_provider                   [Action providers, provides interfaces for reading file actions, receiving DDS actions, policy-generated actions, etc. Currently mainly uses DDS-based action acquisition]
│
├── dds                               [DDS communication module, implements DDS communication for g1, gripper, and three-finger dexterous hand]
│
├── image_server                      [Image publishing service, uses ZMQ for image publishing]
│
├── layeredcontrol                    [Low-level control module, gets actions and sets them in virtual environment]
│
├── robots                            [Basic robot configurations]
│
├── tasks                             [Task-related files]
│   ├── common_config
│   │     ├── camera_configs.py       [Camera placement related configurations]
│   │     ├── robot_configs.py        [Robot setup related configurations]
│   │
│   ├── common_event
│   │      ├── event_manager.py       [Event registration management]  
│   │
│   ├── common_observations
│   │      ├── camera_state.py        [Camera data acquisition]  
│   │      ├── dex3_state.py          [Three-finger dexterous hand data acquisition]
│   │      ├── g1_29dof_state.py      [Robot state data acquisition]
│   │      ├── gripper_state.py       [Gripper data acquisition]
│   │
│   ├── common_scene                
│   │      ├── base_scene_pickplace_cylindercfg.py         [Common scene for cylinder grasping task]  
│   │      ├── base_scene_pickplace_redblock.py            [Common scene for red block grasping task] 
│   │
│   ├── common_termination                                 [Judgment of whether objects in different tasks exceed specified working range]
│   │      ├── base_termination_pick_place_cylinder         
│   │      ├── base_termination_pick_place_redblock 
│   │
│   ├── g1_tasks                                            [All g1-related tasks]
│   │      ├── pick_place_cylinder_g1_29dof_dex1            [Cylinder grasping task]
│   │      │     ├── mdp                                      
│   │      │     │     ├── observations.py                  [Observation data]
│   │      │     │     ├── terminations.py                  [Termination judgment conditions]
│   │      │     ├── __init__.py                            [Task name registration]  
│   │      │     ├── pickplace_cylinder_g1_29dof_dex1_joint_env_cfg.py           [Task-specific scene import and related class initialization]
│   │      ├── ...
│   │      ├── __init__.py                                  [Display all task names existing in g1]
│   ├── utils                                               [Utility functions]
├── tools                                                   [USD conversion and modification related tools]
├── usd                                                     [USD model files]
├── sim_main.py                                             [Main function] 
├── reset_pose_test.py                                      [Test function for object position reset] 
```

### 3.2 Task Scene Construction Steps
If using existing robot configurations (G1-29dof-gripper, G1-29dof-dex3) to build new task scenes, just follow the steps below:

#### 3.2.1、Build Common Parts of Task Scene (i.e., scenes other than the robot)
According to existing task configurations, add new task common scene configurations in the common_scene directory. You can refer to existing task common configuration files.
#### 3.2.2 Termination or Object Reset Condition Judgment
Add termination or object reset judgment conditions according to your scene needs in the common_termination directory
#### 3.2.3 Add and Register Tasks
Add new task directories in the g1_tasks directory and modify related files following existing tasks. Taking the pick_place_cylinder_g1_29dof_dex1 task as an example:

- observations.py: Add corresponding observation functions, just import the corresponding files as needed
 ```

# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
from tasks.common_observations.g1_29dof_state import get_robot_boy_joint_states
from tasks.common_observations.gripper_state import get_robot_gipper_joint_states
from tasks.common_observations.camera_state import get_camera_image

# ensure functions can be accessed by external modules
__all__ = [
    "get_robot_boy_joint_states",
    "get_robot_gipper_joint_states", 
    "get_camera_image"
]

 ```
- terminations.py: Add corresponding condition judgment functions, import corresponding files from common_termination
 ```
 from tasks.common_termination.base_termination_pick_place_cylinder import reset_object_estimate
__all__ = [
"reset_object_estimate"
]
 ```

- pick_place_cylinder_g1_29dof_dex1/```__init__.py ```

Add ```__init__.py``` in the new task directory and add task name, as shown in the ```__init__.py``` under pick_place_cylinder_g1_29dof_dex1:

```
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  

import gymnasium as gym

from . import pickplace_cylinder_g1_29dof_dex1_joint_env_cfg


gym.register(
    id="Isaac-PickPlace-Cylinder-G129-Dex1-Joint",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": pickplace_cylinder_g1_29dof_dex1_joint_env_cfg.PickPlaceG129DEX1BaseFixEnvCfg,
    },
    disable_env_checker=True,
)


```
- Write the environment configuration file corresponding to the task, such as pickplace_cylinder_g1_29dof_dex1_joint_env_cfg.py

Import common scenes, set robot positions, and add camera configurations

- Modify g1_tasks/```__init__.py```

Add the new task configuration class to the ```__init__.py``` file in the g1_tasks directory as follows:

```

# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""Unitree G1 robot task module
contains various task implementations for the G1 robot, such as pick and place, motion control, etc.
"""

# use relative import
from . import pick_place_cylinder_g1_29dof_dex3
from . import pick_place_cylinder_g1_29dof_dex1
from . import pick_place_redblock_g1_29dof_dex1
from . import pick_place_redblock_g1_29dof_dex3
# export all modules
__all__ = ["pick_place_cylinder_g1_29dof_dex3", "pick_place_cylinder_g1_29dof_dex1", "pick_place_redblock_g1_29dof_dex1", "pick_place_redblock_g1_29dof_dex3"]

```
### 📋 TODO List

- ⬜ Continue adding new task scenes
- ⬜ Continue code optimization 

## 🙏 Acknowledgement

This code builds upon following open-source code-bases. Please visit the URLs to see the respective LICENSES:

1. https://github.com/isaac-sim/IsaacLab
2. https://github.com/isaac-sim/IsaacSim
3. https://github.com/zeromq/pyzmq
4. https://github.com/unitreerobotics/unitree_sdk2_python