# -*- coding: utf-8 -*-
"""
依据 grap.py 的指令格式，演示灵巧手 + 双臂 + 升降轴的简单动作。

与 grap.py 一致的下发方式:
  - 灵巧手: atom.hand_control(hand_angle_target=[0~1000 共6个], arm_type=...)（与 grap.py 相同）
  - 双臂同步: atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=...)
      plan_info["targrt"] = np.deg2rad([度]) 或弧度列表（与 grap 相同）
  - 单臂: atom.movJ(targetJoint=np.deg2rad([度]), sacle=..., arm_type=...)
  - 升降轴: 已暂时注释（恢复时取消 demo_lift_cycle / take·put 内升降调用）

运行（在 py_viewer 目录，且已配置 CYCLONEDDS_URI）:
    python grap_simple_motion_demo.py
"""

from __future__ import annotations

import os
import signal
import sys
import time

import numpy as np
from spatialmath import SE3, SO3

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from atom import robot_upper_control as robot_control_handle
from atom.atom_api import Arm_type_strucrt, Atom
from atom.robot_model import Arm_IK as robot_model

# ---------- 与 grap.py 一致的运动参数 ----------
CYCLE = 0.01  # 控制周期
JOINT_VEL = 2.0  # 关节速度
JOINT_ACC = 80.0  # 关节加速度
TCP_VEL = 2.0  # TCP末端速度
TCP_ACC = 80.0  # TCP末端加速度

SIMULATE = False  # 是否仿真模式

# 灵巧手（0~1000，与 grap.py control_init / take 相同）
HAND_OPEN = [1000, 1000, 1000, 1000, 1000, 1000]
HAND_HALF = [1000, 1000, 1000, 1000, 1000, 500]
HAND_GRASP = [600, 600, 600, 600, 600, 500]

# 升降轴路点（弧度，来自 grap.py main / take）— 暂不使用
# LIFT_HIGH = np.array([-0.54, 1.36, -0.83])
# LIFT_MID = np.array([-0.70, 1.57, -0.87])
# LIFT_LOW = np.array([-0.88, 2.24, -0.80])
# LIFT_CARRY = np.array([-0.76, 1.35, -0.59])

# 双臂待机（度 → np.deg2rad，同 grap control_init）
Q_HOME_LEFT_DEG = [27.0201, 11.9929, -2.6176, -37.0925, 3.7974, -9.2938, -5.7138]
Q_HOME_RIGHT_DEG = [32.2010, -8.4353, -13.1994, -40.4444, 14.2924, -4.0446, 4.5302]

# 单臂示教点（grap 右臂拍照位）
Q_RIGHT_PHOTO_DEG = [35.2243, -7.6056, -24.7194, -42.4094, -42.1045, -16.1803, -17.6314]

# grap take() 抱箱预备（度；原弧度 0.17/1.48 等已换算）
Q_TAKE_READY_LEFT_DEG = [0.0, 9.7388, 0.0, 84.7769, 0.0, 0.0, 0.0]  # 左臂抱箱预备位
Q_TAKE_READY_RIGHT_DEG = [0.0, -9.7388, 0.0, 84.7769, 0.0, 0.0, 0.0]  # 右臂抱箱预备位

flag_exit = False  # 退出标志


def handle_sigint(signum, frame) -> None: # 处理SIGINT信号
    global flag_exit
    print("\nCtrl+C 退出") # 打印退出信息
    flag_exit = True
    sys.exit(0) # 退出程序      


def plan_info_deg(q_deg, vel: float = JOINT_VEL, acc: float = JOINT_ACC, cp: float = 0.0) -> dict: # 将关节角转换为弧度
    return {
        "targrt": np.deg2rad(q_deg), # 将关节角转换为弧度
        "vel": vel, # 关节速度
        "acc": acc, # 关节加速度    
        "CP": cp * 0.01, # 插补速度     
    }


def plan_info_rad(q_rad, vel: float = JOINT_VEL, acc: float = JOINT_ACC, cp: float = 0.0) -> dict:      # 将关节角转换为弧度    
    return {
        "targrt": list(q_rad), # 将关节角转换为弧度
        "vel": vel, # 关节速度
        "acc": acc, # 关节加速度    
        "CP": cp * 0.01, # 插补速度     
    }


def hand_both(atom: Atom, targets) -> None:
    """双手相同目标；targets 为 6 维 0~1000（与 grap.py 一致）。"""
    atom.hand_control(hand_angle_target=list(targets), arm_type=Arm_type_strucrt.left_arm)
    atom.hand_control(hand_angle_target=list(targets), arm_type=Arm_type_strucrt.right_arm)


def two_arm_cp(atom: Atom, q_left, q_right, *, use_deg: bool, sacle: float) -> None:  # 双臂同步 → 待机位
    if use_deg:
        traj_l = [plan_info_deg(q_left)] # 左臂轨迹
        traj_r = [plan_info_deg(q_right)] # 右臂轨迹
    else:
        traj_l = [plan_info_rad(q_left)] # 左臂轨迹
        traj_r = [plan_info_rad(q_right)] # 右臂轨迹
    atom.TwoArm_movJ_CP(traj_l, traj_r, sacle=sacle) # 双臂同步 → 待机位


def create_atom(simulate: bool = SIMULATE) -> tuple[Atom, object | None]:  # 创建机器人实例
    robot = robot_model()  # 创建机器人模型
    real_robot = None if simulate else robot_control_handle.UpperControl() # 真实机器人控制实例

    tool_left = SE3() # 左臂工具
    tool_right = SE3(0.21995386, 0.05015792, 0.03143192) # 右臂工具
    tool_right.A[:3, :3] = SO3.RPY(0, 90, -90, unit="deg").R # 右臂工具旋转

    atom = Atom(  # 创建机器人实例  
        robot=robot,
        real_robot=real_robot,
        tcp_vel=TCP_VEL,
        tcp_acc=TCP_ACC,  # TCP末端加速度
        joint_vel=JOINT_VEL,  # 关节速度
        joint_acc=JOINT_ACC,  # 关节加速度
        cycle=CYCLE,  # 控制周期
        simulate=simulate,  # 是否仿真模式
        isdrag=False,  # 是否拖拽
        isVisual=False,  # 是否可视化
        tool_left=tool_left,  # 左臂工具
        tool_right=tool_right,  # 右臂工具
        real_robot_q_left_dir=[1, 1, 1, 1, 1, 1, 1],  # 左臂关节方向
        real_robot_q_right_dir=[1, 1, 1, 1, 1, 1, 1],  # 右臂关节方向
        real_robot_q_left_offset=np.deg2rad([0, 0, 0, 0, 0, 0, 0]),  # 左臂零位偏移
        real_robot_q_right_offset=np.deg2rad([0, 0, 0, 0, 0, 0, 0]),  # 右臂零位偏移
    )
    atom.sync_allJoint()  # 同步所有关节        
    return atom, real_robot  # 返回机器人实例和真实机器人控制实例


def demo_control_init(atom: Atom, lift_robot) -> None:  # 控制初始化    
    """对应 grap.py control_init：手张开 + 双臂回待机。"""
    print("[1] control_init: 手张开 + 双臂待机")
    hand_both(atom, HAND_OPEN)  # 双手张开
    two_arm_cp(atom, Q_HOME_LEFT_DEG, Q_HOME_RIGHT_DEG, use_deg=True, sacle=0.3)  # 双臂同步 → 待机位


def demo_lift_cycle(lift_robot) -> None:
    """对应 grap.py main 中升降轴两段 + take 起落（已暂时关闭）。"""
    print("[2] 升降轴: 已跳过（未启用）")
    return
    # if lift_robot is None:
    #     print("  (simulate) 跳过升降轴")
    #     return
    # print("[2] 升降轴: 中高 → 中 → 低位")
    # lift_robot.dynamic_lift_approach(LIFT_HIGH)
    # lift_robot.dynamic_lift_approach(LIFT_MID)
    # lift_robot.dynamic_lift_approach(LIFT_LOW)


def demo_single_arm(atom: Atom) -> None:  # 右臂 movJ → 拍照示教点  
    """对应 grap.py 单臂 movJ（targetJoint=np.deg2rad([...])）。"""
    print("[3] 右臂 movJ → 拍照示教点")
    atom.movJ(
        targetJoint=np.deg2rad(Q_RIGHT_PHOTO_DEG),  # 右臂 → 拍照示教点
        sacle=0.5,
        arm_type=Arm_type_strucrt.right_arm,  # 右臂
    )
 

def demo_take_ready(atom: Atom, lift_robot) -> None:
    """对应 grap.py take() 前半：手张开 → 抱箱预备姿 → 手半握 → 升降抬起。"""
    print("[4] take 简化: 预备姿 + 抓取手型（升降已跳过）")
    hand_both(atom, HAND_OPEN)
    two_arm_cp(
        atom,
        Q_TAKE_READY_LEFT_DEG,
        Q_TAKE_READY_RIGHT_DEG,
        use_deg=True,
        sacle=0.7,
    )
    hand_both(atom, HAND_GRASP)
    time.sleep(0.5)
    # if lift_robot is not None:
    #     lift_robot.dynamic_lift_approach(LIFT_CARRY)


def demo_put_simple(atom: Atom, lift_robot) -> None:
    """对应 grap.py put() 简化：下降 → 手张开 → 回待机。"""
    print("[5] put 简化: 松手 + 回待机（升降已跳过）")
    # if lift_robot is not None:
    #     lift_robot.dynamic_lift_approach(LIFT_LOW)
    hand_both(atom, HAND_OPEN)
    two_arm_cp(atom, Q_HOME_LEFT_DEG, Q_HOME_RIGHT_DEG, use_deg=True, sacle=0.5)


def run_simple_demo(atom: Atom, lift_robot) -> None:
    demo_control_init(atom, lift_robot)  # 控制初始化
    time.sleep(0.3)
    demo_lift_cycle(lift_robot)  # 升降轴两段 + take 起落
    time.sleep(0.3)
    demo_single_arm(atom)  # 右臂 movJ → 拍照示教点
    time.sleep(0.3)
    demo_take_ready(atom, lift_robot)  # take 预备姿 + 抓取手型 + 抬起  
    time.sleep(0.3)
    demo_put_simple(atom, lift_robot)  # 下降 → 手张开 → 回待机 
    print("[完成] grap 风格简单动作演示结束")


def main() -> int:
    signal.signal(signal.SIGINT, handle_sigint) # 处理SIGINT信号
    simulate = "--simulate" in sys.argv
    atom, lift_robot = create_atom(simulate=simulate)
    lift_robot = None  # 升降轴控制已暂时关闭
    # if not simulate and lift_robot is None:
    #     lift_robot = robot_control_handle.UpperControl()
    try:
        run_simple_demo(atom, lift_robot)  # 运行简单动作演示
    except KeyboardInterrupt:
        print("中断")  # 打印中断信息
    return 0


if __name__ == "__main__":
    raise SystemExit(main())  # 退出程序
