# -*- coding: utf-8 -*-
"""
ATOM机器人运动控制API使用示例
演示如何在其他程序中使用 Atom API
"""

import numpy as np
import sys
import os
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)
import robot_upper_control

# 添加父目录到路径，以便导入（当作为脚本直接运行时）
if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from atom.robot_model import Arm_IK
    from atom.atom_api import Atom, Arm_type_strucrt
else:
    # 作为模块导入时使用相对导入
    from .robot_model import Arm_IK
    from .atom_api import Atom, Arm_type_strucrt
# from robot_upper_control import UpperControl  # 如果使用真实机器人，取消注释

# 手柄相关导入
try:
    if __name__ == "__main__":
        from atom.robot_control_dds.Joystick.JoystickState import JoystickButtonState
    else:
        from .robot_control_dds.Joystick.JoystickState import JoystickButtonState
except ImportError:
    # 如果导入失败，尝试其他路径
    try:
        from robot_control_dds.Joystick.JoystickState import JoystickButtonState
    except ImportError:
        JoystickButtonState = None
        print("警告: 无法导入 JoystickButtonState，手柄功能将不可用")


def example_real_robot_usage():
    """真实机器人使用示例"""
    # 1. 初始化机器人模型
    robot = Arm_IK()
    
    # 2. 初始化真实机器人（必须成功）
    real_robot = robot_upper_control.UpperControl()

    # 3. 创建运动控制器实例（强制真实机器人模式）
    atom = Atom(
        robot=robot,                         # Arm_IK 模型实例
        real_robot=real_robot,               # 真实机器人控制器，如 UpperControl
        tcp_vel=0.7,                         # TCP 末端最大线速度 (m/s)
        tcp_acc=20,                          # TCP 末端最大加速度 (m/s^2)
        joint_vel=1.4,                       # 单关节最大角速度 (rad/s)
        joint_acc=20,                        # 单关节最大角加速度 (rad/s^2)
        cycle=0.01,                          # 控制周期，单位秒
        simulate=False,                      # 禁止回退仿真，确保真机模式
        isdrag=False,                        # 是否启用拖拽示教
        isVisual=False,                      # 是否启用可视化
        # 真实机器人参数：各轴方向与零位偏移
        real_robot_q_left_dir=[1, 1, 1, 1, 1, 1, 1],   # 左臂关节方向
        real_robot_q_right_dir=[1, 1, 1, 1, 1, 1, 1],  # 右臂关节方向
        real_robot_q_left_offset=np.deg2rad([0, 0, 0, 0, 0, 0, 0]),   # 左臂零位偏移
        real_robot_q_right_offset=np.deg2rad([0, 0, 0, 0, 0, 0, 0])   # 右臂零位偏移
    )
    
    # 4. 同步实际关节角度
    atom.sync_allJoint()
    
    # 5. 执行运动（与仿真模式相同）
    # 方式1: 直接指定关节角度
    # target_joint = np.deg2rad([30, 20, -10, -40, 5, -10, -5])
    # atom.movJ(target_joint, sacle=0.5, arm_type=Arm_type_strucrt.left_arm)

    plan_info = {
        "targrt": [0,0.17,0,1.48,0,0,0],
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_left = [plan_info]
    
    plan_info = {
        "targrt": [0,-0.17,0,1.48,0,0,0],
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_right = [plan_info]
    # 双手同时执行动作
    atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=1)

    plan_info = {
        "targrt": [0,0.17,0,1.48,0,0,0],
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_left = [plan_info]
    
    plan_info = {
        "targrt": [0,-0.17,0,1.48,0,0,0],
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_right = [plan_info]
    # 双手同时执行动作
    atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=1)



    # 方式2: 使用保存的点位（推荐，点位数据会自动持久化到文件）
    # 点位数据保存在 robot_dds-develop/py_viewer/atom/saved_points.json
    # 程序重启后仍可使用之前保存的点位
    from atom.saved_points import get_point_joint, get_point_pose, list_points
    
    # 查看所有已保存的点位
    saved_point_names = list_points()
    print(f"已保存的点位: {saved_point_names}")
    
    # 使用保存的点位执行运动
    # P1 = get_point_joint('P1', 'right')  # 获取P1点位的右臂关节角度
    # P2 = get_point_joint('P2', 'right')  # 获取P2点位的右臂关节角度

    # print("移动到P1点位...")
    # atom.movJ(P1, sacle=0.5, arm_type=Arm_type_strucrt.right_arm)
    # print("移动到P2点位...")
    # atom.movJ(P2, sacle=0.5, arm_type=Arm_type_strucrt.right_arm)
   
    # 也可以使用笛卡尔坐标点位
    # P1_pose = get_point_pose('P1', 'right')
    # if P1_pose is not None:
    #     atom.movL(P1_pose, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)

    # # 6. 腰部控制示例（可指定持续时间）
    # torso_angle_target = np.deg2rad(0)  # 目标腰部角度
    # atom.torsor_control(torso_angle_target, duration=1.0)
    
    # # 7. 升降轴控制示例
    # # 获取当前升降轴状态
    # current_lift_state = real_robot.get_lift_state()
    # print(f"当前升降轴位置: {current_lift_state}")
      
    # 方法2: 缓慢移动到目标位置（推荐，更安全）
    # 3个关节的目标位置（弧度）,直接从机器人的/dobot/debug/bin/showLowerState中获取
    # target_lift = np.array([-0.88, 2.24, -0.8])
    # target_lift1 = [-0.48,1.48,-0.96] 
    # target_lift2 = [-0.32,1.21,-0.88]
    # real_robot.dynamic_lift_approach(target_lift1)
    # real_robot.dynamic_lift_approach(target_lift2)

    # 8.头部控制
    head_angle_target = np.deg2rad([0, -15])
    # atom.head_control(head_angle_target)  # 使用默认2.0秒
    atom.head_control(head_angle_target, duration=0.5)  # 快速移动，0.5秒完成
    
    # 9. TTS语音播报示例（简化版，完整示例请参考 example_tts_voice() 函数）
    # try:
    #     from robot_control_dds.voice_sdk.dobot_voice import RpcClient
    #     rpc = RpcClient()
    #     # 必须先开启 TTS 服务
    #     rpc.pc2_switch_tts(True)
    #     time.sleep(1)  # 等待服务初始化
    #     # 播报文本
    #     code, result = rpc.pc2_play_tts("语音功能测试")
    #     if code == 0:
    #         print("TTS语音播报: 语音功能测试")
    #     else:
    #         print(f"TTS语音播报失败: {result}")
    # except ImportError:
    #     print("警告: 无法导入 RpcClient，跳过TTS语音播报示例")
    # except Exception as e:
    #     print(f"TTS语音播报失败: {str(e)}")
    
    # 10. 底盘ARM控制示例
    # try:
    #     from robot_control_dds.amr.amr_sdk import AMR_SDK
        
    #     # 初始化AMR SDK
    #     amr = AMR_SDK()
    #     time.sleep(2)  # 等待SDK初始化完成
        
    #     # 获取当前底盘状态
    #     state = amr.get_amr_state()
    #     if state:
    #         print(f"底盘位置: x={state['position']['x']:.2f}, y={state['position']['y']:.2f}, theta={state['position']['theta']:.2f}")
    #         print(f"导航状态: {state['navigation_status']}")
    #         print(f"电池电量: {state['basic_status']['battery_level']}%")
        
    #     # 控制底盘移动到指定标签点
    #     result = amr.amr_move(tag_id=1008, theta=180.0)
    #     if result is True:
    #         print("底盘移动成功完成")
    #     elif result == "A":
    #         print("底盘移动被按钮A中断")
    #     else:
    #         print("底盘移动失败或超时")
    # except ImportError:
    #     print("警告: 无法导入 AMR_SDK，跳过底盘控制示例")


def example_lift_and_chassis():
    """升降轴和底盘控制示例"""
    # 1. 初始化真实机器人（包含升降轴控制）
    real_robot = robot_upper_control.UpperControl()
    
    # 2. 升降轴控制示例
    print("=" * 50)
    print("升降轴控制示例")
    print("=" * 50)
    
    # 获取当前升降轴状态
    current_lift_state = real_robot.get_lift_state()
    print(f"当前升降轴位置: {current_lift_state}")
    
    # 方法1: 直接控制升降轴到指定位置（快速）
    target_lift = np.array([-0.79, 1.71, -0.91])  # 3个关节的目标位置（弧度）
    print(f"方法1: 直接控制升降轴到位置 {target_lift}")
    # real_robot.command_lift_state(target_lift, kp=800, kd=40)
    
    # 方法2: 缓慢移动到目标位置（推荐，更安全）
    target_lift_slow = np.array([-0.88, 2.24, -0.8])
    print(f"方法2: 缓慢移动升降轴到位置 {target_lift_slow}")
    # real_robot.dynamic_lift_approach(target_lift_slow)
    
    # 3. 底盘ARM控制示例
    print("\n" + "=" * 50)
    print("底盘ARM控制示例")
    print("=" * 50)
    
    try:
        from robot_control_dds.amr.amr_sdk import AMR_SDK
        
        # 初始化AMR SDK
        amr = AMR_SDK()
        print("等待AMR SDK初始化...")
        time.sleep(2)  # 等待SDK初始化完成
        
        # 获取当前底盘状态
        state = amr.get_amr_state()
        if state:
            print(f"底盘位置: x={state['position']['x']:.2f}, y={state['position']['y']:.2f}, theta={state['position']['theta']:.2f}")
            print(f"导航状态: {state['navigation_status']}")
            print(f"电池电量: {state['basic_status']['battery_level']}%")
        
        # 控制底盘移动到指定标签点
        print("\n示例：移动到标签点 1008，角度 180°")
        # result = amr.amr_move(tag_id=1008, theta=180.0)
        # if result is True:
        #     print("底盘移动成功完成")
        # elif result == "A":
        #     print("底盘移动被按钮A中断")
        # else:
        #     print("底盘移动失败或超时")
        
        # 示例：移动到取料点
        # print("\n示例：移动到取料点 1001")
        # amr.amr_move(tag_id=1001, theta=0.0)
        
        # 示例：移动到放置点
        # print("\n示例：移动到放置点 1002")
        # amr.amr_move(tag_id=1002, theta=0.0)
        
    except ImportError:
        print("警告: 无法导入 AMR_SDK，跳过底盘控制示例")
        print("请确保 robot_control_dds.amr.amr_sdk 模块可用")


def get_button_state(real_robot, simulate=False):
    """获取无线手柄按钮状态
    
    Args:
        real_robot: UpperControl 实例
        simulate: 是否为仿真模式
        
    Returns:
        JoystickButtonState: 手柄按钮状态对象，如果无法获取则返回 None
    """
    if simulate:
        return None
    
    if JoystickButtonState is None:
        print("警告: JoystickButtonState 未导入，手柄功能不可用")
        return None

    # 检查 real_robot 是否已初始化
    if not hasattr(real_robot, "robot") or real_robot == 0:
        return None

    try:
        wireless_remote = None

        # 优先使用上肢状态中的无线手柄数据
        upper_msg = getattr(real_robot.robot, "upper_msg", None)
        if upper_msg is not None:
            wireless_remote_attr = getattr(upper_msg, "wireless_remote", None)
            if wireless_remote_attr is not None:
                try:
                    wireless_remote = list(wireless_remote_attr)
                except (TypeError, ValueError):
                    pass

        # 如果上肢状态暂未更新，尝试低位消息
        if wireless_remote is None or len(wireless_remote) == 0:
            low_msg = getattr(real_robot.robot, "lower_msg", None)
            if low_msg is not None:
                wireless_remote_attr = getattr(low_msg, "wireless_remote", None)
                if wireless_remote_attr is not None:
                    try:
                        wireless_remote = list(wireless_remote_attr)
                    except (TypeError, ValueError):
                        pass

        if wireless_remote and len(wireless_remote) > 0:
            return JoystickButtonState(wireless_remote)

    except Exception as e:
        print(f"获取无线手柄数据时出错: {e}")
    
    return None


def example_joystick_control():
    """手柄控制使用示例
    
    演示如何读取手柄按钮状态，检测到按钮按下时打印信息
    
    注意：此功能需要真实机器人，仿真模式下无法使用
    """
    if JoystickButtonState is None:
        print("错误: JoystickButtonState 未导入，无法使用手柄功能")
        print("请确保 robot_control_dds.Joystick.JoystickState 模块可用")
        return
    
    # 初始化真实机器人（手柄功能需要真实机器人）
    real_robot = robot_upper_control.UpperControl()
    
    print("=" * 50)
    print("手柄控制示例")
    print("=" * 50)
    
    # 循环监听手柄按钮
    try:
        while True:
            js = get_button_state(real_robot, simulate=False)
            
            if js is not None:
                # 检查所有按钮状态并打印
                if js.button_R1_ == True:
                    print("RB 按钮被按下")
                    time.sleep(0.3)  # 防抖
                
                if js.button_R2_ == True:
                    print("RT 按钮被按下")
                    time.sleep(0.3)
                
                if js.button_L1_ == True:
                    print("LB 按钮被按下")
                    time.sleep(0.3)
                
                if js.button_L2_ == True:
                    print("LT 按钮被按下")
                    time.sleep(0.3)
                
                if js.button_START_ == True:
                    print("START 按钮被按下")
                    time.sleep(0.3)
                
                if js.button_SELECT_ == True:
                    print("SELECT 按钮被按下")
                    time.sleep(0.3)
                
                if js.button_A_ == True:
                    print("A 按钮被按下")
                    time.sleep(0.3)
                
                if js.button_B_ == True:
                    print("B 按钮被按下")
                    time.sleep(0.3)
                
                if js.button_X_ == True:
                    print("X 按钮被按下")
                    time.sleep(0.3)
                
                if js.button_Y_ == True:
                    print("Y 按钮被按下")
                    time.sleep(0.3)
                
                if js.button_UP_ == True:
                    print("UP 方向键被按下")
                    time.sleep(0.3)
                
                if js.button_DOWN_ == True:
                    print("DOWN 方向键被按下")
                    time.sleep(0.3)
                
                if js.button_LEFT_ == True:
                    print("LEFT 方向键被按下")
                    time.sleep(0.3)
                
                if js.button_RIGHT_ == True:
                    print("RIGHT 方向键被按下")
                    time.sleep(0.3)
            
            # 短暂延时，避免CPU占用过高
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n手柄控制示例结束")


def example_tts_voice():
    """TTS语音播报使用示例
    
    演示如何使用 RpcClient 进行语音克隆播报（pc2_play_tts）
    
    注意：
    - 需要确保语音服务已启动（默认IP: 192.168.8.234, Port: 51235）
    - 使用前需要先开启 TTS 服务
    - 支持中文和英文文本播报
    """
    print("TTS语音播报使用示例")
    
    try:
        # 导入语音SDK（尝试多种导入方式）
        try:
            # 优先尝试从 atom.robot_control_dds 导入
            from atom.robot_control_dds.voice_sdk.dobot_voice import RpcClient
        except ImportError:
            try:
                # 尝试从 robot_control_dds 导入
                from robot_control_dds.voice_sdk.dobot_voice import RpcClient
            except ImportError:
                try:
                    # 尝试相对导入（作为模块导入时）
                    from .robot_control_dds.voice_sdk.dobot_voice import RpcClient
                except ImportError:
                    raise ImportError("无法导入 RpcClient，请检查 robot_control_dds.voice_sdk.dobot_voice 模块")
        
        # 1. 初始化 RpcClient
        print("\n1. 初始化 RpcClient...")
        rpc = RpcClient()
        print("RpcClient 初始化成功")
        
        # 2. 开启 TTS 服务（必须先开启才能使用语音播报）
        print("\n2. 开启 TTS 服务...")
        try:
            code, result = rpc.pc2_switch_tts(True)
            if code == 0:
                print(f"TTS 服务已开启: {result}")
            else:
                print(f"警告: 开启 TTS 服务失败: {result}")
                print("请检查语音服务是否正常运行")
        except Exception as e:
            print(f"开启 TTS 服务时出错: {e}")
            return
        
        # 等待服务初始化完成
        time.sleep(1)
        
        # 3. 基础语音播报示例
        print("\n3. 基础语音播报示例")
        print("-" * 30)
        
        # 示例1: 简单中文播报
        print("示例1: 播报中文文本")
        try:
            code, result = rpc.pc2_play_tts("您好，我是Atom机器人，很高兴为您服务")
            if code == 0:
                print(f"播报成功: {result}")
            else:
                print(f"播报失败: {result}")
        except Exception as e:
            print(f"播报时出错: {e}")
        
        time.sleep(2)  # 等待播报完成
        
        
        # 示例3: 状态播报
        print("\n示例3: 机器人状态播报")
        status_messages = [
            "机器人初始化完成",
            "开始执行任务",
            "任务执行中",
            "任务完成",
            "检测到障碍物",
            "电量不足，请充电"
        ]
        
        for i, message in enumerate(status_messages, 1):
            print(f"  播报状态 {i}/{len(status_messages)}: {message}")
            try:
                code, result = rpc.pc2_play_tts(message)
                if code == 0:
                    print(f"    ✓ 播报成功")
                else:
                    print(f"    ✗ 播报失败: {result}")
            except Exception as e:
                print(f"    ✗ 播报时出错: {e}")
            time.sleep(1.5)  # 等待播报完成
        
        # 5. 音量控制示例（可选）
        print("\n5. 音量控制示例")
        print("-" * 30)
        try:
            # 获取当前音量
            code, result = rpc.get_volume()
            if code == 0:
                current_volume = result.get("result", "未知")
                print(f"当前音量: {current_volume}")
            
            # 设置音量（0-100）
            # volume_level = 50  # 50% 音量
            # code, result = rpc.set_volume(volume_level)
            # if code == 0:
            #     print(f"音量已设置为: {volume_level}%")
            
        except Exception as e:
            print(f"音量控制时出错: {e}")
        
        # 6. 关闭 TTS 服务（可选，如果不再使用）
        print("\n6. 关闭 TTS 服务（可选）")
        print("-" * 30)
        # 取消注释下面的代码来关闭 TTS 服务
        # try:
        #     code, result = rpc.pc2_switch_tts(False)
        #     if code == 0:
        #         print(f"TTS 服务已关闭: {result}")
        #     else:
        #         print(f"关闭 TTS 服务失败: {result}")
        # except Exception as e:
        #     print(f"关闭 TTS 服务时出错: {e}")
        
        print("\n" + "=" * 50)
        print("TTS语音播报示例完成")
        print("=" * 50)
        
    except ImportError:
        print("错误: 无法导入 RpcClient")
        print("请确保 robot_control_dds.voice_sdk.dobot_voice 模块可用")
        print("检查路径: robot_dds-develop/robot_control_dds/voice_sdk/dobot_voice.py")
    except Exception as e:
        print(f"TTS语音播报示例执行出错: {e}")
        import traceback
        traceback.print_exc()


def example_with_visualization():
    """带可视化的使用示例"""
    from pinocchio.visualize import MeshcatVisualizer
    
    # 1. 初始化机器人模型
    robot = Arm_IK()
    
    # 2. 设置可视化
    viz = MeshcatVisualizer(
        robot.interface_model,
        robot.interface_geom_model,
        robot.interface_geom_model,
    )
    viz.initViewer(loadModel=True, zmq_url="tcp://192.168.8.123:6000")
    viz.loadViewerModel()
    viz.displayVisuals(True)
    viz.displayCollisions(False)
    
    # 3. 创建运动控制器实例（启用可视化）
    atom = Atom(
        robot=robot,
        real_robot=None,
        simulate=True,
        isVisual=True,  # 启用可视化
        viz=viz  # 传入可视化器
    )
    
    # 4. 执行运动（会自动显示在可视化界面中）
    target_joint = np.deg2rad([30, 20, -10, -40, 5, -10, -5])
    atom.movJ(target_joint, sacle=0.5, arm_type=Arm_type_strucrt.left_arm)




if __name__ == "__main__":
    print("机器人运动控制器使用示例")
    print("=" * 50)
    
    # print("\n2. 真实机器人使用示例")
    # example_real_robot_usage()
    
    # print("\n3. 升降轴和底盘控制示例")
    # example_lift_and_chassis()
    
    # print("\n4. 手柄控制示例")
    # example_joystick_control()
    
    # print("\n5. TTS语音播报示例")
    example_tts_voice()
    
    # print("\n6. 带可视化的使用示例")
    # example_with_visualization()
    
    # 默认运行真实机器人示例
    # example_real_robot_usage()
    
    # print("\n示例代码已展示，取消注释相应函数即可运行")

