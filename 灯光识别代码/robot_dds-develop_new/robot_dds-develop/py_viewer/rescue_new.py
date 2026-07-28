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
import requests
import json

# 添加父目录到Python路径，以便导入robot_upper_control模块
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from atom.robot_model import Arm_IK as robot_model
import pinocchio as pin
import atom.robot_upper_control as robot_control_handle
from pinocchio.visualize import MeshcatVisualizer
from robot_control_dds.Joystick.JoystickState import JoystickButtonState
from robot_control_dds.voice_sdk.dobot_voice import RpcClient

# 从 atom_api 导入相关类和函数
from atom.atom_api import Atom, Arm_type_strucrt

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

tcp_vel = 0.7  #末端速度
tcp_acc = 20 #末端加速度

joint_vel = 1.4 #关节速度
joint_acc = 20 #关节加速度

robot = 0 #求解器模型
real_robot = 0 #真实机器人模型

flag_exit = False #ctrl+c信号

last_wireless_log_time = 0.0

num_robot_retry = 0 #机械臂连接重试次数

# 全局 atom 实例（将在主程序中初始化）
atom = None
viz = None  # 可视化器

# 工具变换
tool_left = SE3()  #左手工具位置（x, y, z，单位：米）
tool_right = SE3(0.21995386, 0.05015792, 0.03143192)  #右手工具位置（x, y, z，单位：米）
tool_right.A[:3, :3] = SO3.RPY(0, 90, -90, unit='deg').R #右手工具姿态（RPY欧拉角转旋转矩阵）

# 真实机器人参数
real_robot_q_left_dir = [1, 1, 1, 1, 1, 1, 1]
real_robot_q_right_dir = [1, 1, 1, 1, 1, 1, 1]
real_robot_q_left_offset = np.deg2rad([0, 0, 0, 0, 0, 0, 0])
real_robot_q_right_offset = np.deg2rad([0, 0, 0, 0, 0, 0, 0])


# ==================== 业务流程函数 ====================
# 运动之前先全部回到初始位置
def control_init():
    """将头部、双手与双臂移动回预设的起始姿态。"""
    atom.head_control(np.deg2rad([0,30]))
    
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
    
# 救援运动之前先全部回到初始位置
def rescuestask_init():
    """救援流程前的姿态复位：抬头、双手张开并将双臂移到准备位。"""
    atom.head_control(np.deg2rad([0,30]))
    
    atom.hand_control(
            hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000],
            arm_type=Arm_type_strucrt.left_arm,
        )

    atom.hand_control(
            hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000],
            arm_type=Arm_type_strucrt.right_arm,
        )


    plan_info = {
        "targrt": np.deg2rad([-5.6868,45.6578,3.1437,2.2861,31.7548,-2.1768,-9.7183]),
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_left = [plan_info]
    
    plan_info = {
        "targrt": np.deg2rad([-1.3226,-42.3350,-1.5460,-7.9504,-38.9858,-2.6274,10.0583]),
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_right = [plan_info]
    atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=0.7)
    # amr.amr_move(tag_id=1001, theta=0.0)    # amr移动到抓取点
    # torsor_control(np.deg2rad(0))           # 腰部控制

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
def setup_tcp_connection(ip,port):
    """建立TCP连接并清空缓冲区"""
    client_socket_vision = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket_vision.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_address = (ip, port)   #服务器地址和端口
    
    try:
        client_socket_vision.connect(server_address)
        client_socket_vision.settimeout(2.0)  # 设置超时
        print(Fore.GREEN +"tcp连接成功")
        
        # 清空接收缓冲区
        try:
            while True:
                data = client_socket_vision.recv(1024)
                if not data:
                    break
                print(Fore.YELLOW + f"清空缓冲区数据: {data.decode('utf-8')}")
        except socket.timeout:
            print(Fore.GREEN + "缓冲区清空完成")
        except BlockingIOError:
            pass
            
        client_socket_vision.settimeout(None)  # 移除超时设置
        
        # 发送初始start信号
        print(Fore.GREEN + "发送初始start信号")
        safe_socket_send(client_socket_vision, "start")
        data = safe_socket_recv(client_socket_vision)  # 接收相机拍照结果
        if data:
            print(Fore.GREEN + f"初始通信成功: {data}")
        
        return client_socket_vision
        
    except Exception as e:
        print(Fore.RED + f"TCP连接失败: {e}")
        return None

def safe_socket_send(client_socket_vision, message):
    """安全发送数据并自动追加换行，防止粘包。"""
    if client_socket_vision:
        try:
            # 添加消息结束标记
            message_with_end = message + "\n"
            client_socket_vision.send(message_with_end.encode())
            print(Fore.BLUE + f"发送消息: {message}")
            return True
        except Exception as e:
            print(Fore.RED + f"发送消息失败: {e}")
            return False
    return False

def safe_socket_recv(client_socket_vision, buffer_size=1024):
    """安全接收数据，处理粘包并去掉首尾空白字符。"""
    if client_socket_vision:
        try:
            data = client_socket_vision.recv(buffer_size)
            if data:
                decoded_data = data.decode("utf-8").strip()
                print(Fore.GREEN + f"接收消息: {decoded_data}")
                return decoded_data
        except Exception as e:
            print(Fore.RED + f"接收消息失败: {e}")
    return None

"""
DOBOT智能体2-LLM对话
"""
SUPPORT_LANGUAGE = {"en": "英文", "ch": "中文", "ja": "日语", "ko": "韩语"}

def difyLLM(api_key, inp_txt, lang):
    """调用本地 Dify 服务，将识别文本按指定语言发送并返回回答。"""
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json"
    }
    payload = {
        "inputs": {"system": f"{SUPPORT_LANGUAGE[lang]}"},
        "query": str(inp_txt),
        "response_mode": "blocking",
        "user": "user123"
    }
    response = requests.post(
        f'http://192.168.8.13/v1/chat-messages',
        headers=headers,
        data=json.dumps(payload),
        stream=False
    )
    if response.status_code==200:
        rt = json.loads(response.content.decode("utf-8"))["answer"].strip()
        return rt
    else:
        assert 0==1, f"error: {response.status_code}, {response.text}"

# ******************************业务流程：取料和放料任务****************************
# --------------------------------------------------------------------------------
def execute_pick_and_place_task(amr, client_socket_vision, pick_station, place_station):
    """执行取料和放料任务"""
    print(f"*********开始{pick_station}->{place_station}任务**********")
    
    # 1.amr移动到目标点，数据初始化
    print(f"移动到{pick_station}站点取料...")
    amr.amr_move(tag_id=pick_station, theta=180.0)    # amr移动到取料点
    time.sleep(0.5)

    process_info = {
        "flag_right_vision_motionDone": False,
        "flag_left_vision_motionDone": False,
        "right_item_grabbed": False,  # 标记右手物品是否已抓取
        "left_item_grabbed": False,   # 标记左手物品是否已抓取
        "need_photo": True,           # 新增：是否需要拍照
    }
    
    # 左右手抓取目标点位
    temp_vision_begPos_left_identity = np.array([269.8, 200.5, 62.98, 80.6316, -5.6, -1.056]) #左手急救包抓取点位
    temp_vision_begPos_right_identity = np.array([488.9, -113.1, 54.7, 5.23, 3.99, -81.25]) #右手急救包抓取点位
    temp_vision_stickPos_left_identity = np.array([370.5, 266.2, 0.588, 0.9015, -7.1948, -20.9785]) #左手灭火器抓取点位
    temp_vision_stickPos_right_identity = np.array([591.2, -172.5, 73.26, 86.76, 78.24, 16.185]) #右手灭火器抓取点位
    
    # 2.视觉抓取循环
    while(not flag_exit):
        if process_info["flag_right_vision_motionDone"] or process_info["flag_left_vision_motionDone"]:
            print(Fore.BLUE + f"两个物品已取走，开始前往{place_station}站点放料")
            break
            
        else:
            # 只有在需要拍照时才发送拍照请求
            if process_info["need_photo"]:
                # 使用安全的发送和接收函数
                if not safe_socket_send(client_socket_vision, "resultOK"):#发送拍照请求
                    print(Fore.RED + "发送拍照请求失败，TCP连接可能已断开")
                    return False
                    
                data = safe_socket_recv(client_socket_vision)#接收视觉识别结果
                if not data:
                    print(Fore.RED + "接收视觉识别结果失败，TCP连接可能已断开")
                    return False
                    
                tcp_stringResult = data.split(",")#解析视觉识别结果
                process_info["need_photo"] = False  # 拍照完成，等待抓取完成
            else:
                # 不需要拍照时跳过
                time.sleep(0.1)
                continue
            
            # 视觉抓取逻辑 ：确保左右手各只抓取一次
            if ("RESULT:left" in tcp_stringResult[0]) and not process_info["left_item_grabbed"]:
                if tcp_stringResult[1] == "0":
                    temp_vision_tcpPos_left_identity = temp_vision_begPos_left_identity
                    type_id = "beg"
                    type_id_left = "beg"
                elif tcp_stringResult[1] == "1":
                    temp_vision_tcpPos_left_identity = temp_vision_stickPos_left_identity
                    type_id = "stick"
                    type_id_left = "stick"
                else:
                    print(Fore.RED + "识别到的物品类型错误")
                    return False
                if not execute_left_arm_grasp(tcp_stringResult, temp_vision_tcpPos_left_identity, process_info, client_socket_vision, type_id):
                    return False
                process_info["left_item_grabbed"] = True  # 标记左手物品已抓取
                process_info["need_photo"] = True  # 抓取完成，需要下一次拍照
                
            elif ("RESULT:right" in tcp_stringResult[0]) and not process_info["right_item_grabbed"]:
                if tcp_stringResult[1] == "0":
                    temp_vision_tcpPos_right_identity = temp_vision_begPos_right_identity
                    type_id = "beg"
                    type_id_right = "beg"
                elif tcp_stringResult[1] == "1":
                    temp_vision_tcpPos_right_identity = temp_vision_stickPos_right_identity
                    type_id = "stick"
                    type_id_right = "stick"
                else:
                    print(Fore.RED + "识别到的物品类型错误")
                    return False
                if not execute_right_arm_grasp(tcp_stringResult, temp_vision_tcpPos_right_identity, process_info, client_socket_vision, type_id):
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
    amr.amr_move(tag_id=place_station, theta=270.0)    # amr移动到放置点
    print(f"*********底盘已到达{place_station}站点**********")
    # time.sleep(5)
    
    print("开始放置两个物品")

    if process_info["flag_right_vision_motionDone"] or process_info["flag_left_vision_motionDone"]:
        if process_info["flag_left_vision_motionDone"]:
            if  type_id_left == "beg":
                # 放置左手急救包上方点
                tcp_command_pos = temp_vision_tcpPos_left_identity.copy()
                tcp_command_pos[2] = tcp_command_pos[2] + 130*0.001
                atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)


                # 放置左手急救包目标点
                tcp_command_pos = temp_vision_tcpPos_left_identity.copy()
                atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)

                atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 800], arm_type=Arm_type_strucrt.left_arm)

                tcp_command_pos = temp_vision_tcpPos_left_identity.copy()
                tcp_command_pos[0] = tcp_command_pos[0] - 70*0.001
                tcp_command_pos[2] = tcp_command_pos[2] + 80*0.001
                atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)

                
                atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000], arm_type=Arm_type_strucrt.left_arm)
            elif type_id_left == "stick":
                tcp_command_pos = temp_vision_tcpPos_left_identity.copy()
                tcp_command_pos[2] = tcp_command_pos[2] + 100*0.001
                atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)

                tcp_command_pos = temp_vision_tcpPos_left_identity.copy()
                tcp_command_pos[2] = tcp_command_pos[2] + 5*0.001
                atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)

                atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 500], arm_type=Arm_type_strucrt.left_arm)

                tcp_command_pos_1 = temp_vision_tcpPos_left_identity.copy()
                tcp_command_pos_1[0] = tcp_command_pos_1[0] - 30*0.001
                tcp_command_pos_1[1] = tcp_command_pos_1[1] + 50*0.001
                atom.movL(pose=tcp_command_pos_1, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)
            
            # 移动到过渡点，避免碰撞
            plan_info = {
                "targrt": np.deg2rad([-5.6868,45.6578,3.1437,2.2861,31.7548,-2.1768,-9.7183]),
                "vel": joint_vel,
                "acc": joint_acc,
                "CP": 0 * 0.01,
            }
            planning_traj_left = [plan_info]
            atom.movJ_CP(planning_traj_left, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)
        if process_info["flag_right_vision_motionDone"]:
            if type_id_right == "beg":
                tcp_command_pos = temp_vision_tcpPos_right_identity.copy()
                tcp_command_pos[2] = tcp_command_pos[2] + 100*0.001
                atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)

                tcp_command_pos = temp_vision_tcpPos_right_identity.copy()
                # tcp_command_pos[2] = tcp_command_pos[2] + 50*0.001
                atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)

                atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 800], arm_type=Arm_type_strucrt.right_arm)

                tcp_command_pos = temp_vision_tcpPos_right_identity.copy()
                tcp_command_pos[0] = tcp_command_pos[0] - 70*0.001
                tcp_command_pos[2] = tcp_command_pos[2] + 100*0.001
                atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)

                atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000], arm_type=Arm_type_strucrt.right_arm)
            elif type_id_right == "stick":
                tcp_command_pos = temp_vision_tcpPos_right_identity.copy()
                tcp_command_pos[2] = tcp_command_pos[2] + 100*0.001
                atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)

                tcp_command_pos = temp_vision_tcpPos_right_identity.copy()
                tcp_command_pos[2] = tcp_command_pos[2] + 3*0.001
                atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)

                atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 500], arm_type=Arm_type_strucrt.right_arm)

                tcp_command_pos_1 = temp_vision_tcpPos_right_identity.copy()
                tcp_command_pos_1[0] = tcp_command_pos_1[0] - 30*0.001
                tcp_command_pos_1[1] = tcp_command_pos_1[1] - 50*0.001
                atom.movL(pose=tcp_command_pos_1, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)
            
            plan_info = {
                "targrt": np.deg2rad([-1.3226,-42.3350,-1.5460,-7.9504,-38.9858,-2.6274,10.0583]),
                "vel": joint_vel,
                "acc": joint_acc,
                "CP": 0 * 0.01,
            }
            planning_traj_right = [plan_info]
            atom.movJ_CP(planning_traj_right, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)
    else:
        print(Fore.RED + "识别到的物品类型错误")
        return False
    
    print(Fore.GREEN + f"{pick_station}->{place_station}任务完成！")
    return True

def execute_left_arm_grasp(tcp_stringResult, temp_vision_tcpPos_left_identity, process_info, client_socket_vision, type_id):
    """执行左手抓取逻辑"""
    try:
        atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 500], arm_type=Arm_type_strucrt.left_arm)

        if type_id == "beg":
            temp_vision_tcpPos_left_identity[0] = float(tcp_stringResult[2])*0.001 - 15*0.001
            temp_vision_tcpPos_left_identity[1] = float(tcp_stringResult[3])*0.001 - 30*0.001
            temp_vision_tcpPos_left_identity[2] = float(140)*0.001
            temp_vision_tcpPos_left_identity[3:6] = np.deg2rad([80.5684, -5.5765, -1.0807])
            tcp_command_pos_1 = temp_vision_tcpPos_left_identity.copy()
            tcp_command_pos_1[0] = tcp_command_pos_1[0] 
            tcp_command_pos_1[1] = tcp_command_pos_1[1] 
            tcp_command_pos_1[2] = tcp_command_pos_1[2] + 100*0.001
            atom.movL(pose=tcp_command_pos_1, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)
        elif type_id == "stick":
            temp_vision_tcpPos_left_identity[0] = float(tcp_stringResult[2])*0.001 + 13*0.001
            temp_vision_tcpPos_left_identity[1] = float(tcp_stringResult[3])*0.001
            temp_vision_tcpPos_left_identity[2] = float(74.81)*0.001
            temp_vision_tcpPos_left_identity[3:6] = np.deg2rad([1.3164, -6.8262, -20.8884])
            tcp_command_pos_1 = temp_vision_tcpPos_left_identity.copy()
            tcp_command_pos_1[0] = tcp_command_pos_1[0] - 30*0.001
            tcp_command_pos_1[1] = tcp_command_pos_1[1] + 50*0.001
            atom.movL(pose=tcp_command_pos_1, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)
            # tcp_command_pos_2 = temp_vision_tcpPos_left_identity.copy()
            # tcp_command_pos_2[0] = tcp_command_pos_2[0] - 36.3*0.001
            # tcp_command_pos_2[1] = tcp_command_pos_2[1] + 42.7*0.001
            # atom.movL(pose=tcp_command_pos_2, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)
        else:
            print(Fore.RED + "识别到的物品类型错误")
            return False
        atom.movL(pose=temp_vision_tcpPos_left_identity, sacle=0.5, arm_type=Arm_type_strucrt.left_arm)


        if type_id == "beg":
            atom.hand_control(hand_angle_target=[0, 0, 0, 0, 0, 500], arm_type=Arm_type_strucrt.left_arm)
            tcp_command_pos = temp_vision_tcpPos_left_identity.copy()
            tcp_command_pos[2] = tcp_command_pos[2] + 150*0.001
            atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)
        elif type_id == "stick":
            atom.hand_control(hand_angle_target=[600,400,400,400,500,500], arm_type=Arm_type_strucrt.left_arm)
            tcp_command_pos = temp_vision_tcpPos_left_identity.copy()
            tcp_command_pos[2] = tcp_command_pos[2] + 150*0.001
            atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)
        else:
            print(Fore.RED + "识别到的物品类型错误")
            return False

        plan_info = {
            "targrt": np.deg2rad([-5.6868,45.6578,3.1437,2.2861,31.7548,-2.1768,-9.7183]),
            "vel": joint_vel,
            "acc": joint_acc,
            "CP": 0 * 0.01,
        }
        planning_traj_left = [plan_info]
        
        plan_info = {
            "targrt": np.deg2rad([-1.3226, -42.3350, -1.5460, -7.9504, -38.9858, -2.6274, 10.0583]),
            "vel": joint_vel,
            "acc": joint_acc,
            "CP": 0 * 0.01,
        }
        planning_traj_right = [plan_info]
        atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=0.7) 

        process_info["flag_left_vision_motionDone"] = True
        print(Fore.GREEN + "左手抓取完成")
        return True
    except Exception as e:
        print(Fore.RED + f"左手抓取失败: {e}")
        return False

def execute_right_arm_grasp(tcp_stringResult, temp_vision_tcppos_right_identity, process_info, client_socket_vision, type_id):
    """执行右手抓取逻辑"""
    try:
        atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 500], arm_type=Arm_type_strucrt.right_arm)

        if type_id == "beg":
            temp_vision_tcppos_right_identity[0] = float(tcp_stringResult[2])*0.001 - 10*0.001
            temp_vision_tcppos_right_identity[1] = float(tcp_stringResult[3])*0.001 + 30*0.001
            temp_vision_tcppos_right_identity[2] = float(130)*0.001
            temp_vision_tcppos_right_identity[3:6] = np.deg2rad([5.2322, 4.0762, -81.2012])
        elif type_id == "stick":
            temp_vision_tcppos_right_identity[0] = float(tcp_stringResult[2])*0.001 + 30*0.001
            temp_vision_tcppos_right_identity[1] = float(tcp_stringResult[3])*0.001 + 11*0.001
            temp_vision_tcppos_right_identity[2] = float(140.7)*0.001
            temp_vision_tcppos_right_identity[3:6] = np.deg2rad([87.2398, 78.5458, 16.6408])
        else:
            print(Fore.RED + "识别到的物品类型错误")
            return False

        if type_id == "beg":
            tcp_command_pos_1 = temp_vision_tcppos_right_identity.copy()
            tcp_command_pos_1[0] = tcp_command_pos_1[0]
            tcp_command_pos_1[1] = tcp_command_pos_1[1]
            tcp_command_pos_1[2] = tcp_command_pos_1[2] + 100*0.001
            atom.movL(pose=tcp_command_pos_1, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)
        elif type_id == "stick":
            tcp_command_pos_1 = temp_vision_tcppos_right_identity.copy()
            tcp_command_pos_1[0] = tcp_command_pos_1[0] - 30*0.001
            tcp_command_pos_1[1] = tcp_command_pos_1[1] - 30*0.001
            atom.movL(pose=tcp_command_pos_1, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)
            # tcp_command_pos_2 = temp_vision_tcppos_right_identity.copy()
            # tcp_command_pos_2[0] = tcp_command_pos_2[0] - 76.3*0.001
            # tcp_command_pos_2[1] = tcp_command_pos_2[1] - 65.6*0.001
            # atom.movL(pose=tcp_command_pos_2, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)
        else:
            print(Fore.RED + "识别到的物品类型错误")
            return False
        atom.movL(pose=temp_vision_tcppos_right_identity, sacle=0.5, arm_type=Arm_type_strucrt.right_arm)

        if type_id == "beg":
            atom.hand_control(hand_angle_target=[0, 0, 0, 0, 0, 500], arm_type=Arm_type_strucrt.right_arm)
            tcp_command_pos = temp_vision_tcppos_right_identity.copy()
            tcp_command_pos[2] = tcp_command_pos[2] + 150*0.001
            atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)
        elif type_id == "stick":
            atom.hand_control(hand_angle_target=[600,400,400,400,500,500], arm_type=Arm_type_strucrt.right_arm)
            tcp_command_pos = temp_vision_tcppos_right_identity.copy()
            tcp_command_pos[2] = tcp_command_pos[2] + 150*0.001
            atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)
        else:
            print(Fore.RED + "识别到的物品类型错误")
            return False
        plan_info = {
            "targrt": np.deg2rad([-5.6868,45.6578,3.1437,2.2861,31.7548,-2.1768,-9.7183]),
            "vel": joint_vel,
            "acc": joint_acc,
            "CP": 0 * 0.01,
        }
        planning_traj_left = [plan_info]
        
        plan_info = {
            "targrt": np.deg2rad([-1.3226,-42.3350,-1.5460,-7.9504,-38.9858,-2.6274,10.0583]),
            "vel": joint_vel,
            "acc": joint_acc,
            "CP": 0 * 0.01,
        }
        planning_traj_right = [plan_info]
        atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=0.7)

        process_info["flag_right_vision_motionDone"] = True
        print(Fore.GREEN + "右手抓取完成")
        return True
    except Exception as e:
        print(Fore.RED + f"右手抓取失败: {e}")
        return False

def execute_complete_mission(amr, client_socket_vision):
    """执行双站点取放料任务"""
   # print(Fore.CYAN + "开始执行完整任务流程: 1003->1007 -> 1001->1005")
    
    # =================== 第一个任务: 1003->1007 ===================
   # print(Fore.YELLOW + "=== 开始1003->1007任务 ===")
    #if not execute_pick_and_place_task(amr, client_socket_vision, pick_station=1003, place_station=1007):
       # return False
    
    # =================== 第二个任务: 1001->1005 ===================
    print(Fore.YELLOW + "=== 开始1001->1005任务 ===")
    if not execute_pick_and_place_task(amr, client_socket_vision, pick_station=1200, place_station=10015):
        return False
    atom.head_control(np.deg2rad([0,-5])) #头部控制到初始位置
    print(Fore.GREEN + "完整任务流程执行完成！") #打印日志
    return True


# =========================== 救援行动业务流程 ========================================
def rescue_control(amr, a_robot, client_socket_vision):
    """
    商业分拣控制函数，纯函数实现，复用已初始化的对象和TCP连接
    
    Args:
        amr: AMR SDK实例（已初始化）
        a_robot: 机器人控制实例（已初始化）
        client_socket_vision: TCP连接socket（已建立）
    
    Returns:
        bool: 执行是否成功
    """
    # 移动到初始位置
    amr.amr_move(tag_id=1008, theta=180.0)
    time.sleep(1)
    
    # 执行完整任务流程
    print(Fore.YELLOW + "开始执行完整双站点任务")
    status= execute_complete_mission(amr, client_socket_vision)

    
    if status:
        print(Fore.CYAN + "所有任务完成，确认在总待机点")
        
        amr.amr_move(tag_id=1008, theta=180.0)
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
    
    signal.signal(signal.SIGINT, handle_sigint) #注册信号处理函数，捕获Ctrl+C中断信号   
    
    # 机械臂模型
    robot = robot_model() #机器人运动学求解器模型
    
    # 如果是实体机器,需要获取实际关节角度作为初始值
    if simulate == False:
        real_robot = robot_control_handle.UpperControl() #真实机器人控制实例
    else:
        real_robot = None #仿真模式下不需要真实机器人接口
        
    real_robot_q_left_dir = [1, 1, 1, 1, 1, 1, 1] #左手关节方向（1=正向，-1=反向）
    real_robot_q_right_dir = [1, 1, 1, 1, 1, 1, 1] #右手关节方向（1=正向，-1=反向）
    real_robot_q_left_offset = np.deg2rad([0, 0, 0, 0, 0, 0, 0]) #左手关节偏移量（单位：弧度）
    real_robot_q_right_offset = np.deg2rad([0, 0, 0, 0, 0, 0, 0]) #右手关节偏移量（单位：弧度）

    # 如果需要可视化
    if isVisual == True:
        # mesh启动
        viz = MeshcatVisualizer(
            robot.interface_model,
            robot.interface_geom_model,
            robot.interface_geom_model,
        )
        viz.initViewer(loadModel=True, zmq_url="tcp://192.168.8.123:6000")
        viz.loadViewerModel()
        viz.displayVisuals(True)
        viz.displayCollisions(False)

        # mesh要求显示对应坐标系
        viz.displayFrames(
            1,
            [
                robot.interface_model.getFrameId("right_wrist_yaw_joint"),
                robot.interface_model.getFrameId("left_wrist_yaw_joint"),
                robot.interface_model.getFrameId("torso_link"),
                robot.interface_model.getFrameId("tool_left"),
                robot.interface_model.getFrameId("tool_right"),
            ],
        )
        viz.display(pin.neutral(robot.interface_model))
    else:
        viz = None

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

    # if simulate == False:
    #     control_init()  # 先全部动作初始化下
    

    import requests
    import json

    SUPPORT_LANGUAGE = {"en": "英文", "ch": "中文", "ja": "日语", "ko": "韩语"}

    from robot_control_dds.amr.amr_sdk import AMR_SDK #AMR底盘控制实例
    from robot_upper_control import UpperControl #机器人上层控制实例（用于升降轴等）
    amr = AMR_SDK() #AMR底盘控制实例            
    rpc = RpcClient() #语音RPC客户端
    rpc.pc1_mic_server(True) #启用PC1麦克风服务器
    rpc.pc2_switch_asr(True) #启用PC2语音识别
    rpc.pc2_switch_tts(True) #启用PC2语音合成
    a_robot = UpperControl() #机器人上层控制实例（用于升降轴等）

    # 程序启动时建立TCP连接
    print(Fore.GREEN + "程序启动，建立视觉服务器TCP连接...") #打印日志
    client_socket_vision = setup_tcp_connection('127.0.0.1', 65432) #建立TCP连接
    if not client_socket_vision: #如果TCP连接失败
         print(Fore.RED + "无法建立相机TCP连接，程序退出") #打印日志
         sys.exit(1) #退出程序
    tcp_connected_vision = True #相机TCP连接成功   
    print(Fore.GREEN + "相机TCP连接建立成功，准备就绪") #打印日志

    print(Fore.GREEN + "建立TCP机械臂连接...") #打印日志
    tcp_connected_robot= True #机械臂TCP连接成功   
    
    # client_socket_robot = setup_tcp_connection('192.168.201.1', 6600) #建立TCP连接
    # if not client_socket_robot: #如果TCP连接失败
    #      print(Fore.RED + "无法建立机械臂TCP连接，程序退出") #打印日志
    #      sys.exit(1) #退出程序
    # tcp_connected_robot= True #机械臂TCP连接成功   
    # print(Fore.GREEN + "机械臂TCP连接建立成功，准备就绪") #机械臂TCP连接成功   
    
    amr.amr_move(tag_id=1008, theta=180.0) #底盘移动到待机点
    atom.head_control(np.deg2rad([0,-5])) #头部控制到初始位置

    target_1 = np.array([-0.79,1.71,-0.91]) #升降轴移动到初始位置
    a_robot.dynamic_lift_approach(target_1) #升降轴运动到初始位置

    target_1 = np.array([-0.690057,1.600973,-0.865479]) #升降轴移动到初始位置
    a_robot.dynamic_lift_approach(target_1) #升降轴运动到初始位置

    # 双手移动到初始位置
    plan_info = {
        "targrt": [0,0.17,0,1.48,0,0,0], #左手初始位置
        "vel": joint_vel, #关节速度
        "acc": joint_acc, #关节加速度
        "CP": 0 * 0.01,
    }
    planning_traj_left = [plan_info]
    plan_info = {
        "targrt": [0,-0.17,0,1.48,0,0,0], #右手初始位置
        "vel": joint_vel, #关节速度
        "acc": joint_acc, #关节加速度
        "CP": 0 * 0.01,
    }
    planning_traj_right = [plan_info]
    # 双手同时执行动作
    atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=1) #双手同时执行动作

    torso_angle_target = np.deg2rad(6)  # 目标腰部角度
    atom.torsor_control(torso_angle_target, duration=1.0)

    # 双手打开
    atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000], arm_type=Arm_type_strucrt.left_arm) #双手打开
    atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000], arm_type=Arm_type_strucrt.right_arm) #双手打开
    rpc.pc2_play_tts("机器人初始化完成") #播放语音提示
    #主循环
    while (not flag_exit) and tcp_connected_vision and tcp_connected_robot: #循环条件为程序未退出且TCP连接正常
        js = get_button_state() #获取手柄按钮状态  
        if (js is not None and js.button_R2_ == True and js.button_X_ == True):     #判断RT + X按钮是否被按下
            is_played = True   #设置标志位为True
            print("RT + X被按钮被按下") #打印日志
            amr.amr_move(tag_id=1008, theta=180.0)  # 移动到初始位置
            rpc.pc2_play_tts("情况紧急，我们需要运送能量模块和应急物资到受灾区域。指挥权交给你——是否授权开始救援行动？")  #播放语音提示
            # 内层循环：持续检测手柄按钮状态
            while (not flag_exit) and tcp_connected_vision and tcp_connected_robot:
                # rpc.play_audio("~/atom/robot_control_dds/voice_sdk/data/dingdong.wav") #播放音频 
                _, rt_json = rpc.pc2_play_asr("ch", 5)  #语音识别
                js = get_button_state()   #获取手柄按钮状态
                print("asr:",rt_json)
                
                # 获取语音识别结果
                recognized_text = rt_json["result"]
                if recognized_text == "Function timeout":
                    # 识别超时，继续等待
                    print(Fore.YELLOW + "语音识别超时，继续等待...")
                    continue
                else:
                    print(Fore.GREEN + f"识别到的语音内容: {recognized_text}")
                    api_key = "app-emaP1iMcggARsqEPfGGokDeO" 
                    lang = "ch"
                    dify_response = difyLLM(api_key, recognized_text, lang)

                    if dify_response == "否":
                        # 拒绝指令
                        print(Fore.RED + "检测到拒绝指令")
                        rpc.pc2_play_tts("任务已取消救援刻不容缓，请再次确认是否立即启动任务！")
                        continue
                    elif dify_response == "空":
                        # 空指令
                        print(Fore.RED + "检测到空指令")
                        rpc.pc2_play_tts("请您再重新下发正确的指令")
                        continue
                    elif dify_response == "是":
                        # 授权指令（除了"不"和超时之外的其他内容都视为授权）
                        print(Fore.CYAN + "检测到授权指令，开始执行救援任务")   
                        rpc.pc2_play_tts("紧急救援任务启动，需要收集物资并送往灾区。")  
                        # 紧急救援任务初始化
                        rescuestask_init()

                        # target_1 = np.array([-0.690057, 1.600973, -0.865479]) #升降轴运动到初始位置
                        # a_robot.dynamic_lift_approach(target_1) #升降轴运动到初始位置
                        # 执行商业分拣任务（复用已初始化的对象和TCP连接）
                        rescue_control(amr=amr, a_robot=a_robot, client_socket_vision=client_socket_vision) #执行紧急救援任务
                        rpc.pc2_play_tts("紧急救援任务完成，请准备装载。。。。。。。")#播放语音提示
                        atom.head_control(np.deg2rad([0,-5]))#头部控制到初始位置
                        amr.amr_move(tag_id=1008, theta=180.0)  # 移动到初始位置
                        while tcp_connected_robot:
                            if not safe_socket_send(client_socket_robot, "OK"):#给机械臂发送抓取完成信号
                                print(Fore.RED + "发送抓取完成信号失败，TCP连接可能已断开")
                                client_socket_robot.close() #关闭机械臂TCP连接
                                time.sleep(1) #等待1秒
                                while True:
                                    client_socket_robot = setup_tcp_connection('192.168.201.1', 6600) #建立TCP连接
                                    if client_socket_robot: #如果TCP连接成功
                                        tcp_connected_robot= True #机械臂TCP连接成功   
                                        print(Fore.GREEN + "机械臂TCP连接建立成功，准备就绪") #机械臂TCP连接成功  
                                        break
                                    else:
                                        print(Fore.RED + "建立机械臂TCP连接失败，继续重试") #打印日志
                                        num_robot_retry += 1
                                        if num_robot_retry > 5:
                                            print(Fore.RED + "建立机械臂TCP连接失败，重试次数超过5次，程序退出") #打印日志
                                            sys.exit(1) #退出程序
                            else:
                                print(Fore.GREEN + "发送抓取完成信号成功")
                                break
                        break

            
    # 清理资源
    if client_socket_vision or client_socket_robot: #如果TCP连接存在
        client_socket_vision.close() #关闭相机TCP连接
        client_socket_robot.close() #关闭机械臂TCP连接
        print(Fore.GREEN + "相机和机械臂TCP连接已关闭") #打印日志