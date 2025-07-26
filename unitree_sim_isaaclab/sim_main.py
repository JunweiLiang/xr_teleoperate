
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
#!/usr/bin/env python3
# main.py
import os

project_root = os.path.dirname(os.path.abspath(__file__))
os.environ["PROJECT_ROOT"] = project_root

import argparse
import contextlib
import time
import sys
import signal
import torch
import gymnasium as gym
from pathlib import Path

# Isaac Lab AppLauncher
from isaaclab.app import AppLauncher

from image_server.image_server import ImageServer

# add command line arguments
parser = argparse.ArgumentParser(description="Unitree Simulation")
parser.add_argument("--task", type=str, default="Isaac-PickPlace-G129-Head-Waist-Fix", help="task name")
parser.add_argument("--action_source", type=str, default="dds", 
                   choices=["dds", "file", "trajectory", "policy", "replay","dds_wholebody"], 
                   help="Action source")
parser.add_argument("--file_path", type=str, default="./data", help="file path (when action_source=file)")

parser.add_argument("--robot_type", type=str, default="g129", help="robot type")
parser.add_argument("--enable_gripper_dds", action="store_true", help="enable gripper DDS")
parser.add_argument("--enable_dex3_dds", action="store_true", help="enable dexterous hand DDS")
parser.add_argument("--enable_inspire_dds", action="store_true", help="enable inspire hand DDS")
parser.add_argument("--stats_interval", type=float, default=10.0, help="statistics print interval (seconds)")

parser.add_argument("--generate_data_dir", type=str, default="./data", help="save data dir")
parser.add_argument("--generate_data", action="store_true", default=False, help="generate data")
parser.add_argument("--rerun_log", action="store_true", default=False, help="rerun log")
parser.add_argument("--replay_data",  action="store_true", default=False, help="replay data")

parser.add_argument("--modify_light",  action="store_true", default=False, help="modify light")
parser.add_argument("--modify_camera",  action="store_true", default=False,    help="modify camera")

# performance analysis parameters
parser.add_argument("--step_hz", type=int, default=500, help="control frequency")
parser.add_argument("--enable_profiling", action="store_true", default=True, help="enable performance analysis")
parser.add_argument("--profile_interval", type=int, default=500, help="performance analysis report interval (steps)")


# add AppLauncher parameters
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()


if args_cli.enable_dex3_dds and args_cli.enable_gripper_dds and args_cli.enable_inspire_dds:
    print("Error: enable_dex3_dds and enable_gripper_dds and enable_inspire_dds cannot be enabled at the same time")
    print("Please select one of the options")
    sys.exit(1)


import pinocchio 
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from layeredcontrol.robot_control_system import (
    RobotController, 
    ControlConfig,
)

from dds.reset_pose_dds import get_reset_pose_dds
import tasks
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

from tools.augmentation_utils import (
    update_light,
    batch_augment_cameras_by_name,
)

from tools.data_json_load import sim_state_to_json
from dds.sim_state_dds import get_sim_state_dds
from action_provider.create_action_provider import create_action_provider



def setup_signal_handlers(controller):
    """set signal handlers"""
    def signal_handler(signum, frame):
        print(f"\nreceived signal {signum}, stopping controller...")
        try:
            controller.stop()
        except Exception as e:
            print(f"Failed to stop controller: {e}")

    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)


def main():
    """main function"""
    import os
    import atexit
    try:
        os.setpgrp()
        current_pgid = os.getpgrp()
        print(f"Setting process group: {current_pgid}")
        
        def cleanup_process_group():
            try:
                print(f"Cleaning up process group: {current_pgid}")
                import signal
                os.killpg(current_pgid, signal.SIGTERM)
            except Exception as e:
                print(f"Failed to clean up process group: {e}")
        
        atexit.register(cleanup_process_group)
        
    except Exception as e:
        print(f"Failed to set process group: {e}")
    print("=" * 60)
    print("robot control system started")
    print(f"Task: {args_cli.task}")
    print(f"Action source: {args_cli.action_source}")
    print("=" * 60)

    # parse environment configuration
    try:
        env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
        env_cfg.env_name = args_cli.task
    except Exception as e:
        print(f"Failed to parse environment configuration: {e}")
        return
    
    # create environment
    print("\ncreate environment...")
    try:
        env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
        print(f"\ncreate environment success ...")
    except Exception as e:
        print(f"\nFailed to create environment: {e}")
        return
    
    print("\n")
    print("***  Please left-click on the Sim window to activate rendering. ***")
    print("\n")
    # reset environment
    if args_cli.modify_light:
        update_light(
            prim_path="/World/light",
            color=(1.0, 0.8, 0.6),
            intensity=20000.0,
            position=(1.0, 2.0, 3.0),
            radius=0.1,
            enabled=True,
            cast_shadows=True
        )
    if args_cli.modify_camera:
        batch_augment_cameras_by_name(
            names=["front_cam"],
            focal_length=3.0,
            horizontal_aperture=22.0,
            vertical_aperture=16.0,
            exposure=0.8,
            focus_distance=1.2
        )
    env.sim.reset()
    env.reset()
    

    
    # create simplified control configuration
    try:    
        control_config = ControlConfig(
            step_hz=args_cli.step_hz,
            replay_mode=args_cli.replay_data
        )
    except Exception as e:
        print(f"Failed to create control configuration: {e}")
        return
    
    # create controller

    if not args_cli.replay_data:
        print("========= create image server =========")
        try:
            server = ImageServer(fps=30, Unit_Test=False)
        except Exception as e:
            print(f"Failed to create image server: {e}")
            return
        print("========= create image server success =========")
        print("========= create dds =========")
        try:
            reset_pose_dds = get_reset_pose_dds()
            reset_pose_dds.start_communication(enable_publish=False, enable_subscribe=True)
            sim_state_dds = get_sim_state_dds(env,args_cli.task)
            sim_state_dds.start_communication(enable_publish=True, enable_subscribe=False)
        except Exception as e:
            print(f"Failed to create dds: {e}")
            return
        print("========= create dds success =========")
    else:
        from tools.data_json_load import get_data_json_list
        print("========= get data json list =========")
        data_idx=0
        data_json_list = get_data_json_list(args_cli.file_path)
        if args_cli.action_source != "replay":
            args_cli.action_source = "replay"
        print("========= get data json list success =========")
    # create action provider
    
    print(f"\ncreate action provider: {args_cli.action_source}...")
    try:
        action_provider = create_action_provider(env,args_cli)
        if action_provider is None:
            print("action provider creation failed, exiting")
            return
    except Exception as e:
        print(f"Failed to create action provider: {e}")
        return
    
    # set action provider
    print("========= create controller =========")
    controller = RobotController(env, control_config)
    controller.set_action_provider(action_provider)
    print("========= create controller success =========")
    
    # configure performance analysis
    if args_cli.enable_profiling:
        controller.set_profiling(True, args_cli.profile_interval)
        print(f"performance analysis enabled, report every {args_cli.profile_interval} steps")
    else:
        controller.set_profiling(False)
        print("performance analysis disabled")


    # set signal handlers
    setup_signal_handlers(controller)
    print("Note: The DDS in Sim transmits messages on channel 1. Please ensure that other DDS instances use the same channel for message exchange by setting: ChannelFactoryInitialize(1).")
    try:
        # start controller - start asynchronous components
        print("========= start controller =========")
        controller.start()
        print("========= start controller success =========")
        
        # main loop - execute in main thread to support rendering
        last_stats_time = time.time()
        loop_start_time = time.time()
        loop_count = 0
        last_loop_time = time.time()
        recent_loop_times = []  # for calculating moving average frequency

        # use torch.inference_mode() and exception suppression
        with contextlib.suppress(KeyboardInterrupt), torch.inference_mode():
            while simulation_app.is_running() and controller.is_running:
                current_time = time.time()
                loop_count += 1
                if not args_cli.replay_data:
                    env_state = env.scene.get_state()
                    env_state_json =  sim_state_to_json(env_state)
                    sim_state = {"init_state":env_state_json,"task_name":args_cli.task}
                    # sim_state = json.dumps(sim_state)
                    sim_state_dds.write_sim_state_data(sim_state)
                    # print(f"reset_pose_dds: {reset_pose_dds}")
                    reset_pose_cmd = reset_pose_dds.get_reset_pose_command()
                    # # print(f"reset_pose_cmd: {reset_pose_cmd}")
                    if reset_pose_cmd is not None:
                        reset_category = reset_pose_cmd.get("reset_category")
                        # print(f"reset_category: {reset_category}")
                        if reset_category == '1':
                            print("reset object")
                            env_cfg.event_manager.trigger("reset_object_self", env)
                            reset_pose_dds.write_reset_pose_command(-1)
                        elif reset_category == '2':
                            print("reset all")
                            env_cfg.event_manager.trigger("reset_all_self", env)
                            reset_pose_dds.write_reset_pose_command(-1)
                else:
                    if action_provider.get_start_loop() and data_idx<len(data_json_list):
                        print(f"data_idx: {data_idx}")
                        sim_state,task_name = action_provider.load_data(data_json_list[data_idx])
                        if task_name!=args_cli.task:
                            raise ValueError(f" The {task_name} in the dataset is different from the {args_cli.task} being executed .")
                        env.reset_to(sim_state, torch.tensor([0], device=env.device), is_relative=True)
                        env.sim.reset()
                        time.sleep(0.2)
                        action_provider.start_replay()
                        data_idx+=1
                # print(f"env_state: {env_state}")
                # calculate instantaneous loop time
                loop_dt = current_time - last_loop_time
                last_loop_time = current_time
                recent_loop_times.append(loop_dt)
                
                # keep recent 100 loop times
                if len(recent_loop_times) > 100:
                    recent_loop_times.pop(0)
                
                # execute control step (in main thread, support rendering)
                controller.step()

                # print statistics and loop frequency periodically
                if current_time - last_stats_time >= args_cli.stats_interval:
                    # calculate while loop execution frequency
                    elapsed_time = current_time - loop_start_time
                    loop_frequency = loop_count / elapsed_time if elapsed_time > 0 else 0
                    
                    # calculate moving average frequency (based on recent loop times)
                    if recent_loop_times:
                        avg_loop_time = sum(recent_loop_times) / len(recent_loop_times)
                        moving_avg_frequency = 1.0 / avg_loop_time if avg_loop_time > 0 else 0
                        min_loop_time = min(recent_loop_times)
                        max_loop_time = max(recent_loop_times)
                        max_freq = 1.0 / min_loop_time if min_loop_time > 0 else 0
                        min_freq = 1.0 / max_loop_time if max_loop_time > 0 else 0
                    else:
                        moving_avg_frequency = 0
                        min_freq = max_freq = 0
                    
                    print(f"\n=== While loop execution frequency statistics ===")
                    print(f"loop execution count: {loop_count}")
                    print(f"running time: {elapsed_time:.2f} seconds")
                    print(f"overall average frequency: {loop_frequency:.2f} Hz")
                    print(f"moving average frequency: {moving_avg_frequency:.2f} Hz (last {len(recent_loop_times)} times)")
                    print(f"frequency range: {min_freq:.2f} - {max_freq:.2f} Hz")
                    print(f"average loop time: {(elapsed_time/loop_count*1000):.2f} ms")
                    if recent_loop_times:
                        print(f"recent loop time: {(avg_loop_time*1000):.2f} ms")
                    print(f"=============================")
                    
                    # print_stats(controller)
                    last_stats_time = current_time
       
                # check environment state
                if env.sim.is_stopped():
                    print("\nenvironment stopped")
                    break
                # rate_limiter.sleep(env)
    except KeyboardInterrupt:
        print("\nuser interrupted program")
    
    except Exception as e:
        print(f"\nprogram exception: {e}")
    
    finally:
        # clean up resources
        print("\nclean up resources...")
        controller.cleanup()
        
        env.close()
        print("cleanup completed")


if __name__ == "__main__":
    try:
        main()
    finally:
        print("Performing final cleanup...")
        
        # Get current process information
        import os
        import subprocess
        import signal
        import time
        
        current_pid = os.getpid()
        print(f"Current main process PID: {current_pid}")
        
        try:
            # Find all related Python processes
            result = subprocess.run(['pgrep', '-f', 'sim_main.py'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                pids = result.stdout.strip().split('\n')
                print(f"Found related processes: {pids}")
                
                for pid in pids:
                    if pid and pid != str(current_pid):
                        try:
                            print(f"Terminating child process: {pid}")
                            os.kill(int(pid), signal.SIGTERM)
                        except ProcessLookupError:
                            print(f"Process {pid} does not exist")
                        except Exception as e:
                            print(f"Failed to terminate process {pid}: {e}")
                
                # Wait for processes to exit
                time.sleep(2)
                
                # Check if there are any remaining processes, force kill them
                result2 = subprocess.run(['pgrep', '-f', 'sim_main.py'], 
                                       capture_output=True, text=True)
                if result2.returncode == 0:
                    remaining_pids = result2.stdout.strip().split('\n')
                    for pid in remaining_pids:
                        if pid and pid != str(current_pid):
                            try:
                                print(f"Force killing process: {pid}")
                                os.kill(int(pid), signal.SIGKILL)
                            except Exception as e:
                                print(f"Failed to force kill process {pid}: {e}")
                                
        except Exception as e:
            print(f"Error during process cleanup: {e}")
        
        try:
            simulation_app.close()
        except Exception as e:
            print(f"Failed to close simulation application: {e}")
            
        print("Program exit completed")
        
        # Force exit
        os._exit(0)

# python sim_main.py --device cpu  --enable_cameras  --task  Isaac-PickPlace-Cylinder-G129-Dex1-Joint    --enable_gripper_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint    --enable_dex3_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-Cylinder-G129-Inspire-Joint    --enable_inspire_dds --robot_type g129

# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-G129-Dex1-Joint     --enable_gripper_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-G129-Dex3-Joint    --enable_dex3_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task  Isaac-PickPlace-RedBlock-G129-Inspire-Joint    --enable_inspire_dds --robot_type g129


# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-G129-Dex1-Joint     --enable_gripper_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-G129-Dex3-Joint     --enable_dex3_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-G129-Inspire-Joint     --enable_inspire_dds --robot_type g129
