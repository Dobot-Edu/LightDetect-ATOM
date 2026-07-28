# 此文件是用于视觉循环触发拍照识别抓取

#  export CYCLONEDDS_URI=/home/dobotpc2/Documents/robot_dds-develop/cyclonedds.xml
import signal
from spatialmath import SE3, SO3
import numpy as np
import time
import socket
from colorama import Fore, Back, Style, init
from spatialmath.base import b
init()
import sys
import os
import traceback
from atom.robot_model import Arm_IK as robot_model
import pinocchio as pin
from atom import robot_upper_control as robot_control_handle
from pinocchio.visualize import MeshcatVisualizer
from atom.robot_control_dds.Joystick.JoystickState import JoystickButtonState
from atom.robot_control_dds.voice_sdk.dobot_voice import RpcClient

# 从 atom_api 导入相关类和函数
from atom.atom_api import Atom, Arm_type_strucrt

# 音频文件路径定义
AUDIO_BASE_PATH = "/home/dobotpc2/Documents/robot_dds-develop_20251208/robot_dds-develop/py_viewer/atom/robot_control_dds/voice_sdk/yinpin"

# 全局变量（用于兼容现有代码）
joint_angles_left = np.zeros((7))  # 全局左手关节角度
joint_angles_right = np.zeros((7))  # 全局右手关节角度
joint_angles_handle_left = np.zeros((6))  # 全局灵巧手角度
joint_angles_handle_right = np.zeros((6))  # 全局灵巧手角度
head_angle = np.zeros((2))  # 全局头部角度
torso_angle = 0  # 全局腰部角度
pose_left = SE3()  # flange系
pose_right = SE3()  # flange系

CYCLE = 0.01 # 控制周期

simulate = False  # 是否仿真
isdrag = False  # 是否拖拽
isVisual = False #是否可视化

tcp_vel = 2  #末端速度
tcp_acc = 80 #末端加速度

joint_vel = 1 #关节速度
joint_acc = 80 #关节加速度

robot = 0 #求解器模型
real_robot = 0 #真实机器人模型

flag_exit = False #ctrl+c信号

last_wireless_log_time = 0.0

# 全局 atom 实例（将在主程序中初始化）
atom = None
viz = None  # 可视化器

# 工具变换
tool_left = SE3()  # left 工具
tool_right=SE3(0.21995386 ,0.05015792 ,0.03143192)#工具
tool_right.A[:3,:3] =SO3.RPY(0,90,-90,unit='deg')

# 真实机器人参数
real_robot_q_left_dir = [1, 1, 1, 1, 1, 1, 1]
real_robot_q_right_dir = [1, 1, 1, 1, 1, 1, 1]
real_robot_q_left_offset = np.deg2rad([0, 0, 0, 0, 0, 0, 0])
real_robot_q_right_offset = np.deg2rad([0, 0, 0, 0, 0, 0, 0])


# ==================== 业务流程函数 ====================
# 运动之前先全部回到初始位置
def control_init():
    
    
    atom.head_control(np.deg2rad([0,30]), duration=1.0)
    
    atom.hand_control(
            hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000],
            arm_type=Arm_type_strucrt.left_arm,
        )

    atom.hand_control(
            hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000],
            arm_type=Arm_type_strucrt.right_arm,
        )

    # 左手起始点
    plan_info = {
        "targrt": np.deg2rad([27.0201,11.9929,-2.6176,-37.0925,3.7974,-9.2938,-5.7138]),
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_left = []
    planning_traj_left.append(plan_info)

    # 右手起始点
    plan_info = {
        "targrt": np.deg2rad([32.2010,-8.4353,-13.1994,-40.4444,14.2924,-4.0446,4.5302]),
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_right = []
    planning_traj_right.append(plan_info)
    
    # 左右手同时动作
    atom.TwoArm_movJ_CP(planning_traj_left,planning_traj_right, sacle=0.3)
    # amr.amr_move(tag_id=1001, theta=0.0)    # amr移动到抓取点
    # torsor_control(np.deg2rad(0))           # 腰部控制
    
# 商业运动之前先全部回到初始位置
def businesstask_init():
    
    
    atom.head_control(np.deg2rad([0,10]), duration=1.0)
    
    atom.hand_control(
            hand_angle_target=[1000, 1000, 1000, 1000, 1000, 500],
            arm_type=Arm_type_strucrt.left_arm,
        )

    atom.hand_control(
            hand_angle_target=[1000, 1000, 1000, 1000, 1000, 500],
            arm_type=Arm_type_strucrt.right_arm,
        )

    # 左手起始点
    plan_info = {
        "targrt": np.deg2rad([31.9883,2.8949,3.4523,-30.8486,-3.8958,-6.4122,4.0498]),
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_left = []
    planning_traj_left.append(plan_info)

    # 右手起始点
    plan_info = {
        "targrt": np.deg2rad([31.9883,-2.8949,-3.4523,-30.8486,3.8958,-6.4122,4.0498]),
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_right = []
    planning_traj_right.append(plan_info)
    
    # 左右手同时动作
    atom.TwoArm_movJ_CP(planning_traj_left,planning_traj_right, sacle=0.8)
# 定义一个信号处理函数
def handle_sigint(signum, frame):
    global flag_exit
    print("\n捕获到 Ctrl+C,程序将优雅地退出。")
    flag_exit = True
    sys.exit(0)


def get_button_state():
    """获取无线手柄按钮状态"""
    global last_wireless_log_time
    if simulate:
        return None

    # 检查 real_robot 是否已初始化
    if not hasattr(real_robot, "robot") or real_robot == 0:
        if time.time() - last_wireless_log_time > 5.0:  # 增加打印间隔到5秒
            print("无线手柄数据未获取到: real_robot 未初始化或 robot 属性不存在")
            last_wireless_log_time = time.time()
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

        # 只有在确实没有数据时才打印（增加间隔，减少日志）
        if time.time() - last_wireless_log_time > 5.0:
            # 诊断信息
            upper_msg_status = "存在" if upper_msg is not None else "None"
            lower_msg_status = "存在" if getattr(real_robot.robot, "lower_msg", None) is not None else "None"
            print(f"无线手柄数据未获取到: upper_msg={upper_msg_status}, lower_msg={lower_msg_status}")
            last_wireless_log_time = time.time()
    except Exception as e:
        if time.time() - last_wireless_log_time > 5.0:
            print(f"获取无线手柄数据时出错: {e}")
            last_wireless_log_time = time.time()
    
    return None


# TCP连接管理函数
def setup_tcp_connection():
    """建立TCP连接并清空缓冲区"""
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_address = ("127.0.0.1", 65432)
    
    try:
        client_socket.connect(server_address)
        client_socket.settimeout(2.0)  # 设置超时
        print(Fore.GREEN +"tcp连接成功")
        
        # 清空接收缓冲区
        try:
            while True:
                data = client_socket.recv(1024)
                if not data:
                    break
                print(Fore.YELLOW + f"清空缓冲区数据: {data.decode('utf-8')}")
        except socket.timeout:
            print(Fore.GREEN + "缓冲区清空完成")
        except BlockingIOError:
            pass
            
        client_socket.settimeout(None)  # 移除超时设置
        
        # 发送初始start信号
        print(Fore.GREEN + "发送初始start信号")
        safe_socket_send(client_socket, "start")
        data = safe_socket_recv(client_socket)  # 接收相机拍照结果
        if data:
            print(Fore.GREEN + f"初始通信成功: {data}")
        
        return client_socket
        
    except Exception as e:
        print(Fore.RED + f"TCP连接失败: {e}")
        return None

def safe_socket_send(client_socket, message):
    """安全发送数据，避免粘包"""
    if client_socket:
        try:
            # 添加消息结束标记
            message_with_end = message + "\n"
            client_socket.send(message_with_end.encode())
            print(Fore.BLUE + f"发送消息: {message}")
            return True
        except Exception as e:
            print(Fore.RED + f"发送消息失败: {e}")
            return False
    return False

def safe_socket_recv(client_socket, buffer_size=1024):
    """安全接收数据，处理粘包"""
    if client_socket:
        try:
            data = client_socket.recv(buffer_size)
            if data:
                decoded_data = data.decode("utf-8").strip()
                print(Fore.GREEN + f"接收消息: {decoded_data}")
                return decoded_data
        except Exception as e:
            print(Fore.RED + f"接收消息失败: {e}")
    return None

# ******************************业务流程：取料和放料任务****************************
# --------------------------------------------------------------------------------
def execute_pick_and_place_task(amr, client_socket, pick_station, place_station):
    """执行取料和放料任务"""
    print(f"*********开始{pick_station}->{place_station}任务**********")
    
    # 1.amr移动到目标点，数据初始化
    print(f"移动到{pick_station}站点取料...")
    amr.amr_move(tag_id=pick_station, theta=270.0)    # amr移动到取料点
    atom.head_control(np.deg2rad([0,30]), duration=0.5)
    time.sleep(0.5)

    process_info = {
        "flag_right_vision_motionDone": False,
        "flag_left_vision_motionDone": False,
        "right_item_grabbed": False,  # 标记右手物品是否已抓取
        "left_item_grabbed": False,   # 标记左手物品是否已抓取
        "need_photo": True,           # 新增：是否需要拍照
    }
    
    # 左右手抓取目标点位
    temp_vision_tcpPos_left_identity = np.array([275.6, 111.4, 75.19, 0.2, -1.22, 0.06])
    temp_vision_tcppos_right_identity = np.array([500, -136.3, 102.9, 88.65, 87.22, -1.34])
    
    # 2.视觉抓取循环
    while(not flag_exit):
        if process_info["flag_right_vision_motionDone"] and process_info["flag_left_vision_motionDone"]:
            print(Fore.BLUE + f"两个物品已取走，开始前往{place_station}站点放料")
            break
            
        else:
            # 只有在需要拍照时才发送拍照请求
            if process_info["need_photo"]:
                # 使用安全的发送和接收函数
                if not safe_socket_send(client_socket, "resultOK"):
                    print(Fore.RED + "发送失败，TCP连接可能已断开")
                    return False
                    
                data = safe_socket_recv(client_socket)
                if not data:
                    print(Fore.RED + "接收失败，TCP连接可能已断开")
                    return False
                    
                tcp_stringResult = data.split(",")
                process_info["need_photo"] = False  # 拍照完成，等待抓取完成
                # RESULT:NONE 或格式不足时无有效坐标，只打印并继续请求拍照
                if len(tcp_stringResult) < 3 or "NONE" in (tcp_stringResult[0] or ""):
                    print(Fore.YELLOW + f"跳过无目标结果: {tcp_stringResult[0] if tcp_stringResult else data}")
                    process_info["need_photo"] = True
                    continue
                print(tcp_stringResult[0])
                print(tcp_stringResult[1])
                print(tcp_stringResult[2])
            else:
                # 不需要拍照时跳过
                time.sleep(0.1)
                continue
            
            # 视觉抓取逻辑 ：确保左右手各只抓取一次
            if ("RESULT:left" in tcp_stringResult[0]) and not process_info["left_item_grabbed"]:
                if not execute_left_arm_grasp(tcp_stringResult, temp_vision_tcpPos_left_identity, process_info, client_socket):
                    return False
                process_info["left_item_grabbed"] = True  # 标记左手物品已抓取
                process_info["need_photo"] = True  # 抓取完成，需要下一次拍照
                
            elif ("RESULT:right" in tcp_stringResult[0]) and not process_info["right_item_grabbed"]:
                if not execute_right_arm_grasp(tcp_stringResult, temp_vision_tcppos_right_identity, process_info, client_socket):
                    return False
                process_info["right_item_grabbed"] = True  # 标记右手物品已抓取
                process_info["need_photo"] = True  # 抓取完成，需要下一次拍照
                
            else:
                # 如果识别到的物品类型已经抓取过了，继续等待另一个物品
                print(Fore.YELLOW + f"跳过已抓取的物品类型: {tcp_stringResult[0]}")
                process_info["need_photo"] = True  # 需要重新拍照
                continue

    # =========================== 放料流程 ========================================
    # 3.移动到放料点
    # atom.head_control(np.deg2rad([0,0]), duration=0.5)
    # amr.amr_move(tag_id=1002, theta=180.0)    # amr移动到放置点
    amr.amr_move(tag_id=place_station, theta=90.0)    # amr移动到放置点
    print(f"*********底盘已到达{place_station}站点**********")
    # time.sleep(5)
    
    print("开始放置两个物品")
    # 4.前往放置点上方
    plan_info = {
        "targrt": np.deg2rad([-1.9523,-4.5075,-9.7142,-32.0973,17.4768,29.6969,0.3398]),
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_left = [plan_info]
    
    plan_info = {
        "targrt": np.deg2rad([-1.1481,6.3001,8.1385,-21.3906,-11.5998,12.1031,-5.3062]),
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_right = [plan_info]
    # 双手同时执行动作
    # atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=0.7)
    
    # 5.前往放置点
    plan_info = {
        "targrt": np.deg2rad([4.5585,5.1465,1.7533,14.3769,-6.7542,-26.9032,-14.1095]),
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_left = [plan_info]

    plan_info = {
        "targrt": np.deg2rad([-3.5040,-0.8056,3.5764,18.8710,2.6036,-23.0078,8.9557]),
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_right = [plan_info]
    
    atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=1)
    print(planning_traj_left)
    print(planning_traj_right)
    # time.sleep(10)
    # 6.打开手放置物品
    atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 500], arm_type=Arm_type_strucrt.left_arm)
    atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 500], arm_type=Arm_type_strucrt.right_arm)
    
    # 7.脱离放置点位置
    plan_info = {
        "targrt": np.deg2rad([32.5126,7.0667,3.9028,-33.3201,-4.3005,-7.1122,5.2445]),
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_left = [plan_info]
    
    plan_info = {
        "targrt": np.deg2rad([2.3511,2.8049,5.6354,-7.1553,-7.8451,-5.9931,-6.4252]),
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_right = [plan_info]
    
    # atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=1)
    
    # 8.回到初始位置（机械臂回到初始位置，但不移动AMR）
    plan_info = {
        "targrt": np.deg2rad([41.1462,3.3036,11.9829,-35.2149,-10.0145,-11.5680,1.5517]),
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_left = [plan_info]
    
    plan_info = {
        "targrt": np.deg2rad([34.7241,-17.7209,-7.9775,-34.7725,14.1454,-3.3537,-4.3219]),
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_right = [plan_info]
    
    atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=1)
    
    print(Fore.GREEN + f"{pick_station}->{place_station}任务完成！")
    return True

def execute_left_arm_grasp(tcp_stringResult, temp_vision_tcpPos_left_identity, process_info, client_socket):
    """执行左手抓取逻辑"""
    try:
        atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 500], arm_type=Arm_type_strucrt.left_arm)

        temp_vision_tcpPos_left_identity[0] = float(tcp_stringResult[1])*0.001
        temp_vision_tcpPos_left_identity[1] = float(tcp_stringResult[2])*0.001
        temp_vision_tcpPos_left_identity[2] = float(75)*0.001
        temp_vision_tcpPos_left_identity[3:6] = np.deg2rad([0.2, -1.22, 0.06])
        
        # 1.前往过渡点1
        tcp_command_pos_1 = temp_vision_tcpPos_left_identity.copy()
        tcp_command_pos_1[0] = tcp_command_pos_1[0] - 110*0.001
        tcp_command_pos_1[1] = tcp_command_pos_1[1] + 90*0.001
        atom.movL(pose=tcp_command_pos_1, sacle=0.1, arm_type=Arm_type_strucrt.left_arm)
        # 2.前往过渡点2
        tcp_command_pos_2 = temp_vision_tcpPos_left_identity.copy()
        tcp_command_pos_2[0] = tcp_command_pos_2[0] - 80*0.001
        tcp_command_pos_2[1] = tcp_command_pos_2[1] + 55*0.001
        atom.movL(pose=tcp_command_pos_2, sacle=0.1, arm_type=Arm_type_strucrt.left_arm)

        # 3.前往抓取点
        atom.movL(pose=temp_vision_tcpPos_left_identity, sacle=0.1, arm_type=Arm_type_strucrt.left_arm)

        # 4.抓取
        atom.hand_control(hand_angle_target=[500, 500, 500, 500, 500, 500], arm_type=Arm_type_strucrt.left_arm)
        
        # 5.抬升
        tcp_command_pos = temp_vision_tcpPos_left_identity.copy()
        tcp_command_pos[2] = tcp_command_pos[2] + 50*0.001
        atom.movL(pose=tcp_command_pos, sacle=0.1, arm_type=Arm_type_strucrt.left_arm)

        # # 6.返回过渡点
        # movL(pose=tcp_command_pos_2, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)
        # movL(pose=tcp_command_pos_1, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)
        
        # 7.离开视野点
        # tcpPos_left_identity = np.array([0.1569,0.2121,0.1672,14.8278,-14.6576,-17.3886])
        # tcpPos_left_identity = np.array([0.1569,0.25,0.1797,14.83,-5.3103,-8.08])
        # tcpPos_left_identity[3:6] = np.deg2rad([14.83,-5.3103,-8.08])
        tcpPos_left_identity = np.array([0.1434,0.2266,0.1129,0.0,-5.4286,5.4995])
        tcpPos_left_identity[3:6] = np.deg2rad([0.0,-5.4286,5.4995])
        atom.movL(pose=tcpPos_left_identity, sacle=0.1, arm_type=Arm_type_strucrt.left_arm)

        process_info["flag_left_vision_motionDone"] = True
        print(Fore.GREEN + "左手抓取完成")
        return True
    except Exception as e:
        print(Fore.RED + f"左手抓取失败: {e}")
        traceback.print_exc()
        return False

def execute_right_arm_grasp(tcp_stringResult, temp_vision_tcppos_right_identity, process_info, client_socket):
    """执行右手抓取逻辑"""
    try:
        atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 500], arm_type=Arm_type_strucrt.right_arm)

        temp_vision_tcppos_right_identity[0] = float(tcp_stringResult[1])*0.001
        temp_vision_tcppos_right_identity[1] = float(tcp_stringResult[2])*0.001
        temp_vision_tcppos_right_identity[2] = float(112)*0.001
        temp_vision_tcppos_right_identity[3:6] = np.deg2rad([88.65,87.22,-1.34])

        # 1.前往过渡点1
        tcp_command_pos_1 = temp_vision_tcppos_right_identity.copy()
        tcp_command_pos_1[0] = tcp_command_pos_1[0] - 165*0.001
        tcp_command_pos_1[1] = tcp_command_pos_1[1] - 50*0.001
        atom.movL(pose=tcp_command_pos_1, sacle=0.1, arm_type=Arm_type_strucrt.right_arm)
        # 2.前往过渡点2
        tcp_command_pos_2 = temp_vision_tcppos_right_identity.copy()
        tcp_command_pos_2[0] = tcp_command_pos_2[0] - 70*0.001
        tcp_command_pos_2[1] = tcp_command_pos_2[1] - 50*0.001
        # tcp_command_pos_2[0] = tcp_command_pos_2[0] - 40*0.001
        # tcp_command_pos_2[1] = tcp_command_pos_2[1] - 55*0.001
        atom.movL(pose=tcp_command_pos_2, sacle=0.1, arm_type=Arm_type_strucrt.right_arm)

        # 3.前往抓取点
        atom.movL(pose=temp_vision_tcppos_right_identity, sacle=0.1, arm_type=Arm_type_strucrt.right_arm)

        # 4.抓取
        atom.hand_control(hand_angle_target=[500, 500, 500, 500, 500, 500], arm_type=Arm_type_strucrt.right_arm)
        
        # 5.抬升
        tcp_command_pos = temp_vision_tcppos_right_identity.copy()
        tcp_command_pos[2] = tcp_command_pos[2] + 50*0.001
        atom.movL(pose=tcp_command_pos, sacle=0.1, arm_type=Arm_type_strucrt.right_arm)

        # # 6.返回过渡点
        # movL(pose=tcp_command_pos_2, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)
        # movL(pose=tcp_command_pos_1, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)

        # 7.离开视野点
        # tcpPos_right_identity = np.array([0.3461,-0.2138,0.2494,75.2013,76.0275,-15.2371])
        # tcpPos_right_identity = np.array([0.3461,-0.22,0.1867,25.87,85.58,-65.48])
        # tcpPos_right_identity[3:6] = np.deg2rad([25.87,85.58,-65.48])
        tcpPos_right_identity = np.array([0.357,-0.1662,0.1641,81.8631,84.5714,-5.5371])
        tcpPos_right_identity[3:6] = np.deg2rad([81.8631,84.5714,-5.5371])
        atom.movL(pose=tcpPos_right_identity, sacle=0.1, arm_type=Arm_type_strucrt.right_arm)

        process_info["flag_right_vision_motionDone"] = True
        print(Fore.GREEN + "右手抓取完成")
        return True
    except Exception as e:
        print(Fore.RED + f"右手抓取失败: {e}")
        return False

def execute_complete_mission(amr, client_socket):
    """执行双站点取放料任务"""
   # print(Fore.CYAN + "开始执行完整任务流程: 1003->1007 -> 1001->1005")
    
    # =================== 第一个任务: 1003->1007 ===================
   # print(Fore.YELLOW + "=== 开始1003->1007任务 ===")
    #if not execute_pick_and_place_task(amr, client_socket, pick_station=1003, place_station=1007):
       # return False
    
    # =================== 第二个任务: 1001->10015 ===================
    print(Fore.YELLOW + "=== 开始1001->10015任务 ===")
    if not execute_pick_and_place_task(amr, client_socket, pick_station=1002, place_station=1003):
        return False
    atom.head_control(np.deg2rad([0,10]), duration=1.0) #taitou
    print(Fore.GREEN + "完整任务流程执行完成！")
    return True


# =========================== 灯光巡S检业务流程 ========================================
def light_inspection_control(amr, a_robot, client_socket, vision_result_ok=True):
    """
    灯光巡检控制函数，纯函数实现，复用已初始化的对象和TCP连接
    
    Args:
        amr: AMR SDK实例（已初始化）
        a_robot: 机器人控制实例（已初始化）
        client_socket: TCP连接socket（已建立）
        vision_result_ok: 视觉识别结果，默认True
    
    Returns:
        bool: 执行是否成功
    """
    if vision_result_ok:
        print(Fore.CYAN + "视觉识别OK，执行抓取动作流程")
        
        # 初始点
        plan_info = {
            "targrt": [0, 0.17, 0, 1.48, 0, 0, 0],
            "vel": joint_vel,
            "acc": joint_acc,
            "CP": 0 * 0.01,
        }
        planning_traj_left = [plan_info]
        
        plan_info = {
            "targrt": [0, -0.17, 0, 1.48, 0, 0, 0],
            "vel": joint_vel,
            "acc": joint_acc,
            "CP": 0 * 0.01,
        }
        planning_traj_right = [plan_info]
        # 双手同时执行动作
        atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=0.7)
        
        # AMR移动到第一个目标点
        # amr.amr_move(tag_id=1005, theta=180)
        # time.sleep(1)
        # 26.6080,-12.5405,-25.7341,-41.3838,-39.6266,-12.5586,-12.2654
        # 11.1819,-13.4740,-18.1982,-32.3355,-45.2316,-8.4904,-8.9624

       # 右手过渡点
        atom.movJ(targetJoint=np.deg2rad([42.0485,-9.8769,-0.2217,-29.7411,-0.2338,0.4055,-0.2222]),
             sacle=1.5,
             arm_type=Arm_type_strucrt.right_arm,
             )
        # 右手过渡点
        
        atom.movJ(targetJoint=np.deg2rad([33.4548,-27.3078,-33.7409,-15.1797,-52.2037,-25.0947,-44.0455]),
             sacle=1.5,
             arm_type=Arm_type_strucrt.right_arm,
             )
        # 目标点
        atom.movJ(targetJoint=np.deg2rad([-19.8792,-15.6461,-12.9659,0.6537,1.1605,-36.2537,0.9732]),
             sacle=1.0,
             arm_type=Arm_type_strucrt.right_arm,
             )
        time.sleep(2)
        # 右手过渡点
        atom.movJ(targetJoint=np.deg2rad([33.4548,-27.3078,-33.7409,-15.1797,-52.2037,-25.0947,-44.0455]),
             sacle=1.5,
             arm_type=Arm_type_strucrt.right_arm,
             )

        # 右手过渡点
        atom.movJ(targetJoint=np.deg2rad([42.0485,-9.8769,-0.2217,-29.7411,-0.2338,0.4055,-0.2222]),
             sacle=1.5,
             arm_type=Arm_type_strucrt.right_arm,
             )
        # 右手过渡点
        atom.movJ(targetJoint=[0, -0.17, 0, 1.48, 0, 0, 0],
             sacle=1.0,
             arm_type=Arm_type_strucrt.right_arm,
             )
        
        # 移动AMR到最终位置
        # amr.amr_move(tag_id=1001, theta=180.0)
    else:
        print(Fore.YELLOW + "视觉识别未返回OK，直接前往目标站点")
        amr.amr_move(tag_id=10000004, theta=0.0)
    
    return True

# =========================== 商业分拣业务流程 ========================================
def shangye_control(amr, a_robot, client_socket):
    """
    商业分拣控制函数，纯函数实现，复用已初始化的对象和TCP连接
    
    Args:
        amr: AMR SDK实例（已初始化）
        a_robot: 机器人控制实例（已初始化）
        client_socket: TCP连接socket（已建立）
    
    Returns:
        bool: 执行是否成功
    """
    # 移动到初始位置
    #amr.amr_move(tag_id=1004, theta=0.0)
    #time.sleep(1)
    
    # 执行完整任务流程
    print(Fore.YELLOW + "开始执行完整双站点任务")
    tcp_connected = execute_complete_mission(amr, client_socket)
    
    if tcp_connected:
        print(Fore.CYAN + "所有任务完成，确认在总待机点")
        
        amr.amr_move(tag_id=1001, theta=180.0)
        plan_info = {
            "targrt": [0, 0.17, 0, 1.48, 0, 0, 0],
            "vel": joint_vel,
            "acc": joint_acc,
            "CP": 0 * 0.01,
        }
        planning_traj_left = [plan_info]
        
        plan_info = {
            "targrt": [0, -0.17, 0, 1.48, 0, 0, 0],
            "vel": joint_vel,
            "acc": joint_acc,
            "CP": 0 * 0.01,
        }
        planning_traj_right = [plan_info]
        # 双手同时执行动作
        atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=1)
    
    return True






# =========================== 主程序 ========================================
if __name__ == "__main__":
    
    signal.signal(signal.SIGINT, handle_sigint)
    
    # 机械臂模型（Atom 的 robot 参数必须是该实例，不能是 0）
    robot = robot_model()
    if simulate == False:
        real_robot = robot_control_handle.UpperControl()
    else:
        real_robot = None
    real_robot_q_left_dir = [1, 1, 1, 1, 1, 1, 1]
    real_robot_q_right_dir = [1, 1, 1, 1, 1, 1, 1]
    real_robot_q_left_offset = np.deg2rad([0, 0, 0, 0, 0, 0, 0])
    real_robot_q_right_offset = np.deg2rad([0, 0, 0, 0, 0, 0, 0])

    # 初始化 Atom 实例
    atom = Atom(
        robot=robot,
        real_robot=real_robot,
        tcp_vel=tcp_vel,
        tcp_acc=tcp_acc,
        joint_vel=joint_vel,
        joint_acc=joint_acc,
        cycle=CYCLE,
        simulate=simulate,
        isdrag=isdrag,
        isVisual=isVisual,
        tool_left=tool_left,
        tool_right=tool_right,
        real_robot_q_left_dir=real_robot_q_left_dir,
        real_robot_q_right_dir=real_robot_q_right_dir,
        real_robot_q_left_offset=real_robot_q_left_offset,
        real_robot_q_right_offset=real_robot_q_right_offset,
        viz=viz
    )

    # 同步当前实际角度并更新末端位置信息
    atom.sync_allJoint()
 

    
    # -------------------------------------------------------------
    # -------------------------------------------------------------
    # ***************** 业务控制流程 脚本控制部分 *******************
    # -------------------------全部标准单位 ------------------------
    # -------------------------------------------------------------
    # -------------------------------------------------------------

    
    from atom.robot_control_dds.amr.amr_sdk import AMR_SDK
    from atom.robot_upper_control import UpperControl
    amr = AMR_SDK()
    rpc = RpcClient()
    rpc.pc1_mic_server(True)
    rpc.pc2_switch_asr(True)
    rpc.pc2_switch_tts(True)
    a_robot = UpperControl()
    atom.torsor_control(np.deg2rad(0))
    # 程序启动时建立TCP连接
    print(Fore.GREEN + "程序启动，建立TCP连接...")
    client_socket = setup_tcp_connection()
    if not client_socket:
        print(Fore.RED + "无法建立TCP连接，程序退出")
        sys.exit(1)
    tcp_connected = True
    print(Fore.GREEN + "TCP连接建立成功，准备就绪")
    
    amr.amr_move(tag_id=10000004, theta=0.0)
    atom.head_control(np.deg2rad([0,5]), duration=1.0)

    target_1 = np.array([-0.54,1.36,-0.9])
    a_robot.dynamic_lift_approach(target_1)

    # target_1 = np.array([-0.39, 0.94, -0.53])
    # a_robot.dynamic_lift_approach(target_1)

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
    # atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000], arm_type=Arm_type_strucrt.left_arm)
    # atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000], arm_type=Arm_type_strucrt.right_arm)
    
    rpc.set_volume(50)
    rpc.play_audio("/home/dobotpc2/Documents/robot_dds-develop_new/robot_dds-develop/py_viewer/voice/init.wav")
    # rpc.pc2_play_tts("机器人初始化完成..........................................")

    while (not flag_exit) and tcp_connected:
        print("选择任务：")
        print("RT+X 灯光巡检")
        print("RT+Y 商业分拣")
        print("\r")
        js = get_button_state()
        # if (js is not None and js.button_R2_ == True and js.button_X_ == 1):
        is_played = True
        print("RT + X 按钮被按下")
        # rpc.pc2_play_tts("开始执行灯光巡检任务。。")
        rpc.play_audio("/home/dobotpc2/Documents/robot_dds-develop_new/robot_dds-develop/py_viewer/voice/strat_to_light.wav")

        move_state = amr.amr_move(tag_id=10000003, theta=0)    # amr移动到灯光检测点
        atom.head_control(np.deg2rad([25,15]), duration=0.5)    # 控制头抬高
        time.sleep(1)
        target_1 = np.array(np.deg2rad([-30.4, 79.2, -51.5]))
        time.sleep(1)
        a_robot.dynamic_lift_approach(target_1)
        atom.head_control(np.deg2rad([-40,-28]), duration=0.5)    # 控制头抬高
        time.sleep(1)
        # 触发相机拍照（使用已建立的TCP连接）
        safe_socket_send(client_socket, "start")
        vision_result_ok = False
        if safe_socket_send(client_socket, "light"):
            vision_response = safe_socket_recv(client_socket)
            if vision_response and "ok" in vision_response.lower():
                vision_result_ok = True
                print(Fore.YELLOW + f"视觉识别返回: {vision_response}")
            else:
                print(Fore.YELLOW + f"视觉识别返回: {vision_response}")
        else:
            print(Fore.RED + "发送视觉识别请求失败")
        # vision_result_ok = True     # 测试用，实际需要根据视觉识别结果判断
        if vision_result_ok:
            print(Fore.CYAN + "视觉识别OK，执行灯光操作流程")
            # rpc.pc2_play_tts("灯光处于打开状态,执行关闭动作。。")  
            rpc.play_audio("/home/dobotpc2/Documents/robot_dds-develop_new/robot_dds-develop/py_viewer/voice/light_is_open.wav")

            atom.head_control(np.deg2rad([-25,10]), duration=0.5)
            # 根据相机识别结果执行相应动作（复用已初始化的对象和TCP连接）
            light_inspection_control(amr=amr, a_robot=a_robot, client_socket=client_socket, 
                                        vision_result_ok=vision_result_ok)
            atom.head_control(np.deg2rad([0,5]), duration=0.5)  
            target_1 = np.array([-0.54,1.36,-0.9])
            a_robot.dynamic_lift_approach(target_1)                        
            amr.amr_move(tag_id=10000004, theta=0.0)
            # rpc.pc2_play_tts("灯光巡检任务结束。。")
            rpc.play_audio("/home/dobotpc2/Documents/robot_dds-develop_new/robot_dds-develop/py_viewer/voice/end.wav")
            # rpc.stop_audio()
        else:
            # rpc.pc2_play_tts("灯光处于关闭状态。。")
            rpc.play_audio("/home/dobotpc2/Documents/robot_dds-develop_new/robot_dds-develop/py_viewer/voice/light_is_close.wav")
            # rpc.stop_audio()                        
            atom.head_control(np.deg2rad([0,5]), duration=0.5)    # 控制头抬高
            target_1 = np.array([-0.54,1.36,-0.9])
            a_robot.dynamic_lift_approach(target_1)
            amr.amr_move(tag_id=10000004, theta=0.0)    # amr移动到起始点
            # rpc.pc2_play_tts("楼宇巡检任务结束。。")                                                     
            rpc.play_audio("/home/dobotpc2/Documents/robot_dds-develop_new/robot_dds-develop/py_viewer/voice/end.wav")
                
        # elif  (js is not None and js.button_R2_ == True and js.button_Y_ == 1):
        #     rpc.pc2_play_tts("开始执行商业分拣任务。。")

        #     atom.sync_allJoint()                     
        #     # 商业运动之前先全部回到初始位置
        #     businesstask_init()
        #     # 启动线程控制头部运动，注意不能和控制机械臂一起运行
        #     print("启动头部控制线程1...")
        #     # 执行商业分拣任务（复用已初始化的对象和TCP连接）
        #     shangye_control(amr=amr, a_robot=a_robot, client_socket=client_socket)
        #     rpc.pc2_play_tts("商业分拣任务完成。。")
        #     # atom.head_control(np.deg2rad([0,0]))
        #     atom.head_control(np.deg2rad([0,10]), duration=0.5)
        # else:
            #print("WARNING: ivw timeout", rt_json["result"])
            # pass
        time.sleep(0.5)

        
    
    # 清理资源
    if client_socket:
        client_socket.close()
        print(Fore.GREEN + "TCP连接已关闭")
        2.7815,-34.1323,-5.5746,0.4460,-61.9876,-1.9743,-59.9775
        # -21.8726,-27.5316,1.4430,-10.0311,-59.4890,2.3812,-24.1409
        
        
        # -17.3981,-27.0196,2.1945,-6.1838,-60.2984,0.7887,-32.2445