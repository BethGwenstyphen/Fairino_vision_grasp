import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    # 核心字典序列 (你的原始定义)
    LAUNCH_SEQUENCE = [
        {"name": "机械臂底层", "cmd": ["ros2", "run", "fairino_hardware", "ros2_cmd_server"], "delay": 3.0},
        {"name": "图漾相机",   "cmd": ["ros2", "launch", "percipio_camera", "percipio_camera.launch.py"], "delay": 5.0},
        {"name": "肌肉控制",   "cmd": ["ros2", "run", "handle_grasp_project", "control_node"], "delay": 3.0},
        {"name": "视觉处理",   "cmd": ["ros2", "run", "handle_grasp_project", "vision_node"], "delay": 10.0},
        {"name": "行为树大脑", "cmd": ["ros2", "run", "bt_grasp_brain", "bt_main_node"], "delay": 0.0},
    ]

    current_delay = 0.0

    for step in LAUNCH_SEQUENCE:
        current_delay += step["delay"]
        
        # 封装执行进程，强制 output='screen'
        process = ExecuteProcess(
            cmd=step["cmd"],
            output='screen',
            name=step["name"]
        )
        
        # 封装定时器动作
        timer_action = TimerAction(
            period=current_delay,
            actions=[process]
        )
        
        ld.add_action(timer_action)

    return ld