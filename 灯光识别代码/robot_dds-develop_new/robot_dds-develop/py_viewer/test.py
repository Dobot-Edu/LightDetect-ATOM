
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

parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from atom.robot_model import Arm_IK as robot_model
import pinocchio as pin
import atom.robot_upper_control as robot_control_handle
from pinocchio.visualize import MeshcatVisualizer
from robot_control_dds.Joystick.JoystickState import JoystickButtonState
from robot_control_dds.voice_sdk.dobot_voice import RpcClient

from atom.atom_api import Atom, Arm_type_strucrt

joint_angles_left = np.zeros((7))
joint_angles_right = np.zeros((7))
joint_angles_handle_left = np.zeros((6))
joint_angles_handle_right = np.zeros((6))
head_angle = np.zeros((2))
torso_angle = 0
pose_left = SE3()
pose_right = SE3()

CYCLE = 0.01

simulate = False
isdrag = False
isVisual = False

tcp_vel = 0.2
tcp_acc = 10

joint_vel = 0.7
joint_acc = 10

robot = 0
real_robot = 0

flag_exit = False

last_wireless_log_time = 0.0

num_robot_retry = 0

atom = None
viz = None

tool_left = SE3()
tool_right = SE3(0.21995386, 0.05015792, 0.03143192)
tool_right.A[:3, :3] = SO3.RPY(0, 90, -90, unit='deg').R

real_robot_q_left_dir = [1, 1, 1, 1, 1, 1, 1]
real_robot_q_right_dir = [1, 1, 1, 1, 1, 1, 1]
real_robot_q_left_offset = np.deg2rad([0, 0, 0, 0, 0, 0, 0])
real_robot_q_right_offset = np.deg2rad([0, 0, 0, 0, 0, 0, 0])


def control_init():
    
    
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
        "targrt": np.deg2rad([27.0201,11.9929,-2.6176,-37.0925,3.7974,-9.2938,-5.7138]),
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_left = []
    planning_traj_left.append(plan_info)

    plan_info = {
        "targrt": np.deg2rad([32.2010,-8.4353,-13.1994,-40.4444,14.2924,-4.0446,4.5302]),
        "vel": joint_vel,
        "acc": joint_acc,
        "CP": 0 * 0.01,
    }
    planning_traj_right = []
    planning_traj_right.append(plan_info)
    
    atom.TwoArm_movJ_CP(planning_traj_left,planning_traj_right, sacle=0.3)
    
def rescuestask_init():
    
    
    atom.head_control(np.deg2rad([0,30]))
    
    atom.hand_control(
            hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000],
            arm_type=Arm_type_strucrt.left_arm,
        )

    atom.hand_control(
            hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000],
            arm_type=Arm_type_strucrt.right_arm,
        )

    # plan_info = {
    #     "targrt": np.deg2rad([32.5126,12.4606,3.9028,-33.3201,-4.3005,-7.1122,-9.9677]),
    #     "vel": joint_vel,
    #     "acc": joint_acc,
    #     "CP": 0 * 0.01,
    # }
    # planning_traj_left = []
    # planning_traj_left.append(plan_info)

    # plan_info = {
    #     "targrt": np.deg2rad([32.2010,-8.4353,-13.1994,-40.4444,14.2924,-4.0446,4.5302]),
    #     "vel": joint_vel,
    #     "acc": joint_acc,
    #     "CP": 0 * 0.01,
    # }
    # planning_traj_right = []
    # planning_traj_right.append(plan_info)
    
    # atom.TwoArm_movJ_CP(planning_traj_left,planning_traj_right, sacle=0.8)

def handle_sigint(signum, frame):
    global flag_exit
    print("\n捕获到 Ctrl+C,程序将优雅地退出。")
    flag_exit = True
    sys.exit(0)


def get_button_state():
    global last_wireless_log_time
    if simulate:
        return None

    if not hasattr(real_robot, "robot") or real_robot == 0:
        if time.time() - last_wireless_log_time > 5.0:
            print("无线手柄数据未获取到: real_robot 未初始化或 robot 属性不存在")
            last_wireless_log_time = time.time()
        return None

    try:
        wireless_remote = None

        upper_msg = getattr(real_robot.robot, "upper_msg", None)
        if upper_msg is not None:
            wireless_remote_attr = getattr(upper_msg, "wireless_remote", None)
            if wireless_remote_attr is not None:
                try:
                    wireless_remote = list(wireless_remote_attr)
                except (TypeError, ValueError):
                    pass

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

        if time.time() - last_wireless_log_time > 5.0:
            upper_msg_status = "存在" if upper_msg is not None else "None"
            lower_msg_status = "存在" if getattr(real_robot.robot, "lower_msg", None) is not None else "None"
            print(f"无线手柄数据未获取到: upper_msg={upper_msg_status}, lower_msg={lower_msg_status}")
            last_wireless_log_time = time.time()
    except Exception as e:
        if time.time() - last_wireless_log_time > 5.0:
            print(f"获取无线手柄数据时出错: {e}")
            last_wireless_log_time = time.time()
    
    return None


def setup_tcp_connection(ip,port):
    client_socket_vision = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket_vision.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_address = (ip, port)
    
    try:
        client_socket_vision.connect(server_address)
        client_socket_vision.settimeout(2.0)
        print(Fore.GREEN +"tcp连接成功")
        
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
            
        client_socket_vision.settimeout(None)
        
        print(Fore.GREEN + "发送初始start信号")
        safe_socket_send(client_socket_vision, "start")
        data = safe_socket_recv(client_socket_vision)
        if data:
            print(Fore.GREEN + f"初始通信成功: {data}")
        
        return client_socket_vision
        
    except Exception as e:
        print(Fore.RED + f"TCP连接失败: {e}")
        return None

def safe_socket_send(client_socket_vision, message):
    if client_socket_vision:
        try:
            message_with_end = message + "\n"
            client_socket_vision.send(message_with_end.encode())
            print(Fore.BLUE + f"发送消息: {message}")
            return True
        except Exception as e:
            print(Fore.RED + f"发送消息失败: {e}")
            return False
    return False

def safe_socket_recv(client_socket_vision, buffer_size=1024):
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

SUPPORT_LANGUAGE = {"en": "英文", "ch": "中文", "ja": "日语", "ko": "韩语"}

def difyLLM(api_key, inp_txt, lang):
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

def execute_pick_and_place_task(amr, client_socket_vision, pick_station, place_station):
    print(f"*********开始{pick_station}->{place_station}任务**********")
    

    process_info = {
        "flag_right_vision_motionDone": False,
        "flag_left_vision_motionDone": False,
        "right_item_grabbed": False,
        "left_item_grabbed": False,
        "need_photo": True,
    }
    
    temp_vision_begPos_left_identity = np.array([277.5, 187.6, 93.13, 80.5684, -5.5765, -1.0807])
    temp_vision_begPos_right_identity = np.array([510.2, -107.4, 106.1, 5.2322, 4.0762, -81.2012])
    temp_vision_stickPos_left_identity = np.array([396.9, 240.1, 94.81, 1.3164, -6.8262, -20.8884])
    temp_vision_stickPos_right_identity = np.array([575.3, -108.2, 140.7, 87.2398, 78.5458, 16.6408])
    
    while(not flag_exit):
        if process_info["flag_right_vision_motionDone"] or process_info["flag_left_vision_motionDone"]:
            print(Fore.BLUE + f"两个物品已取走，开始前往{place_station}站点放料")
            break
            
        else:
            if process_info["need_photo"]:
                if not safe_socket_send(client_socket_vision, "resultOK"):
                    print(Fore.RED + "发送拍照请求失败，TCP连接可能已断开")
                    return False
                    
                data = safe_socket_recv(client_socket_vision)
                if not data:
                    print(Fore.RED + "接收视觉识别结果失败，TCP连接可能已断开")
                    return False
                    
                tcp_stringResult = data.split(",")
                process_info["need_photo"] = False
            else:
                time.sleep(0.1)
                continue
            
            if ("RESULT:left" in tcp_stringResult[0]) and not process_info["left_item_grabbed"]:
                if tcp_stringResult[1] == "0":
                    temp_vision_tcpPos_left_identity = temp_vision_begPos_left_identity
                    type_id = "beg"
                elif tcp_stringResult[1] == "1":
                    temp_vision_tcpPos_left_identity = temp_vision_stickPos_left_identity
                    type_id = "stick"
                else:
                    print(Fore.RED + "识别到的物品类型错误")
                    return False
                if not execute_left_arm_grasp(tcp_stringResult, temp_vision_tcpPos_left_identity, process_info, client_socket_vision, type_id):
                    return False
                process_info["left_item_grabbed"] = True
                process_info["need_photo"] = True
                
            elif ("RESULT:right" in tcp_stringResult[0]) and not process_info["right_item_grabbed"]:
                if tcp_stringResult[1] == "0":
                    temp_vision_tcpPos_right_identity = temp_vision_begPos_right_identity
                    type_id = "beg"
                elif tcp_stringResult[1] == "1":
                    temp_vision_tcpPos_right_identity = temp_vision_stickPos_right_identity
                    type_id = "stick"
                else:
                    print(Fore.RED + "识别到的物品类型错误")
                    return False
                if not execute_right_arm_grasp(tcp_stringResult, temp_vision_tcpPos_right_identity, process_info, client_socket_vision, type_id):
                    return False
                process_info["right_item_grabbed"] = True
                process_info["need_photo"] = True
                
            else:
                print(Fore.YELLOW + f"跳过已抓取的物品类型: {tcp_stringResult[0]}")
                process_info["need_photo"] = True
                continue
    time.sleep(5)
    atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000], arm_type=Arm_type_strucrt.left_arm)
    atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000], arm_type=Arm_type_strucrt.right_arm)
    
    # plan_info = {
    #     "targrt": np.deg2rad([32.5126,12.4606,3.9028,-33.3201,-4.3005,-7.1122,-9.9677]),
    #     "vel": joint_vel,
    #     "acc": joint_acc,
    #     "CP": 0 * 0.01,
    # }
    # planning_traj_left = [plan_info]
    
    # plan_info = {
    #     "targrt": np.deg2rad([2.3511,2.8049,5.6354,-7.1553,-7.8451,-5.9931,-6.4252]),
    #     "vel": joint_vel,
    #     "acc": joint_acc,
    #     "CP": 0 * 0.01,
    # }
    # planning_traj_right = [plan_info]
    
    # atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=0.7)
    
    # plan_info = {
    #     "targrt": np.deg2rad([28.2599,11.9847,5.8641,-35.0102,1.5703,-3.0291,-5.4551]),
    #     "vel": joint_vel,
    #     "acc": joint_acc,
    #     "CP": 0 * 0.01,
    # }
    # planning_traj_left = [plan_info]
    
    # plan_info = {
    #     "targrt": np.deg2rad([34.7241,-17.7209,-7.9775,-34.7725,14.1454,-3.3537,-4.3219]),
    #     "vel": joint_vel,
    #     "acc": joint_acc,
    #     "CP": 0 * 0.01,
    # }
    # planning_traj_right = [plan_info]
    
    # atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=0.7)
    
    # print(Fore.GREEN + f"{pick_station}->{place_station}任务完成！")
    # return True

def execute_left_arm_grasp(tcp_stringResult, temp_vision_tcpPos_left_identity, process_info, client_socket_vision, type_id):
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
            tcp_command_pos[2] = tcp_command_pos[2] + 130*0.001
            atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)

            tcp_command_pos = temp_vision_tcpPos_left_identity.copy()
            # tcp_command_pos[2] = tcp_command_pos[2] + 50*0.001
            atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)

            atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 800], arm_type=Arm_type_strucrt.left_arm)

            tcp_command_pos = temp_vision_tcpPos_left_identity.copy()
            tcp_command_pos[0] = tcp_command_pos[0] - 70*0.001
            tcp_command_pos[2] = tcp_command_pos[2] + 80*0.001
            atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)

            
            atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000], arm_type=Arm_type_strucrt.left_arm)

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
        elif type_id == "stick":
            atom.hand_control(hand_angle_target=[600,400,400,400,500,500], arm_type=Arm_type_strucrt.left_arm)
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
        else:
            print(Fore.RED + "识别到的物品类型错误")
            return False
        
        



        process_info["flag_left_vision_motionDone"] = True
        print(Fore.GREEN + "左手抓取完成")
        return True
    except Exception as e:
        print(Fore.RED + f"左手抓取失败: {e}")
        return False

def execute_right_arm_grasp(tcp_stringResult, temp_vision_tcppos_right_identity, process_info, client_socket_vision, type_id):
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
            tcp_command_pos[2] = tcp_command_pos[2] + 100*0.001
            atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)

            tcp_command_pos = temp_vision_tcppos_right_identity.copy()
            # tcp_command_pos[2] = tcp_command_pos[2] + 50*0.001
            atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)

            atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 800], arm_type=Arm_type_strucrt.right_arm)

            tcp_command_pos = temp_vision_tcppos_right_identity.copy()
            tcp_command_pos[0] = tcp_command_pos[0] - 70*0.001
            tcp_command_pos[2] = tcp_command_pos[2] + 100*0.001
            atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)

            atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000], arm_type=Arm_type_strucrt.right_arm)


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
        elif type_id == "stick":
            atom.hand_control(hand_angle_target=[600,400,400,400,500,500], arm_type=Arm_type_strucrt.right_arm)
            tcp_command_pos = temp_vision_tcppos_right_identity.copy()
            tcp_command_pos[2] = tcp_command_pos[2] + 100*0.001
            atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)

            tcp_command_pos = temp_vision_tcppos_right_identity.copy()
            tcp_command_pos[2] = tcp_command_pos[2] + 3*0.001
            atom.movL(pose=tcp_command_pos, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)

            atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 500], arm_type=Arm_type_strucrt.right_arm)

            tcp_command_pos_1 = temp_vision_tcppos_right_identity.copy()
            tcp_command_pos_1[0] = tcp_command_pos_1[0] - 30*0.001
            tcp_command_pos_1[1] = tcp_command_pos_1[1] - 50*0.001
            atom.movL(pose=tcp_command_pos_1, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)
            

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
        else:
            print(Fore.RED + "识别到的物品类型错误")
            return False
        
        


        # tcpPos_right_identity = np.array([0.3461,-0.2138,0.2494,75.2013,76.0275,-15.2371])
        # tcpPos_right_identity[3:6] = np.deg2rad([75.2013,76.0275,-15.2371])
        # atom.movL(pose=tcpPos_right_identity, sacle=0.7, arm_type=Arm_type_strucrt.right_arm)

        process_info["flag_right_vision_motionDone"] = True
        print(Fore.GREEN + "右手抓取完成")
        return True
    except Exception as e:
        print(Fore.RED + f"右手抓取失败: {e}")
        return False

def execute_complete_mission(amr, client_socket_vision):
    print(Fore.YELLOW + "=== 开始1001->1005任务 ===")
    if not execute_pick_and_place_task(amr, client_socket_vision, pick_station=1001, place_station=1005):
        return False
    atom.head_control(np.deg2rad([0,30]))
    print(Fore.GREEN + "完整任务流程执行完成！")
    return True


def rescue_control(amr, a_robot, client_socket_vision):
    
    print(Fore.YELLOW + "开始执行完整双站点任务")
    status= execute_complete_mission(amr, client_socket_vision)

    status = True
    if status:
        print(Fore.CYAN + "所有任务完成，确认在总待机点")
        
        # amr.amr_move(tag_id=1008, theta=180.0)
        # plan_info = {
        #     "targrt": [0, 0.17, 0, 1.48, 0, 0, 0],
        #     "vel": joint_vel,
        #     "acc": joint_acc,
        #     "CP": 0 * 0.01,
        # }
        # planning_traj_left = [plan_info]
        
        # plan_info = {
        #     "targrt": [0, -0.17, 0, 1.48, 0, 0, 0],
        #     "vel": joint_vel,
        #     "acc": joint_acc,
        #     "CP": 0 * 0.01,
        # }
        # planning_traj_right = [plan_info]
        # atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=1)
    
    return True


if __name__ == "__main__":
    
    signal.signal(signal.SIGINT, handle_sigint)
    
    robot = robot_model()
    
    if simulate == False:
        real_robot = robot_control_handle.UpperControl()
    else:
        real_robot = None
        
    real_robot_q_left_dir = [1, 1, 1, 1, 1, 1, 1]
    real_robot_q_right_dir = [1, 1, 1, 1, 1, 1, 1]
    real_robot_q_left_offset = np.deg2rad([0, 0, 0, 0, 0, 0, 0])
    real_robot_q_right_offset = np.deg2rad([0, 0, 0, 0, 0, 0, 0])

    if isVisual == True:
        viz = MeshcatVisualizer(
            robot.interface_model,
            robot.interface_geom_model,
            robot.interface_geom_model,
        )
        viz.initViewer(loadModel=True, zmq_url="tcp://192.168.8.123:6000")
        viz.loadViewerModel()
        viz.displayVisuals(True)
        viz.displayCollisions(False)

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

    atom.sync_allJoint()
    
    

    

    import requests
    import json

    SUPPORT_LANGUAGE = {"en": "英文", "ch": "中文", "ja": "日语", "ko": "韩语"}

    from robot_control_dds.amr.amr_sdk import AMR_SDK
    from robot_upper_control import UpperControl
    amr = AMR_SDK()
    rpc = RpcClient()
    rpc.pc1_mic_server(True)
    rpc.pc2_switch_asr(True)
    rpc.pc2_switch_tts(True)
    a_robot = UpperControl()

    print(Fore.GREEN + "程序启动，建立视觉服务器TCP连接...")
    client_socket_vision = setup_tcp_connection('127.0.0.1', 65432)
    if not client_socket_vision:
         print(Fore.RED + "无法建立相机TCP连接，程序退出")
         sys.exit(1)
    tcp_connected_vision = True
    print(Fore.GREEN + "相机TCP连接建立成功，准备就绪")

    
    # atom.head_control(np.deg2rad([0,-5]))

    # target_1 = np.array([-0.79,1.71,-0.91])
    # a_robot.dynamic_lift_approach(target_1)
    # plan_info = {
    #     "targrt": [0,0.17,0,1.48,0,0,0],
    #     "vel": joint_vel,
    #     "acc": joint_acc,
    #     "CP": 0 * 0.01,
    # }
    # planning_traj_left = [plan_info]
    # plan_info = {
    #     "targrt": [0,-0.17,0,1.48,0,0,0],
    #     "vel": joint_vel,
    #     "acc": joint_acc,
    #     "CP": 0 * 0.01,
    # }
    # planning_traj_right = [plan_info]
    # atom.TwoArm_movJ_CP(planning_traj_left, planning_traj_right, sacle=1)
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

    atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000], arm_type=Arm_type_strucrt.left_arm)
    atom.hand_control(hand_angle_target=[1000, 1000, 1000, 1000, 1000, 1000], arm_type=Arm_type_strucrt.right_arm)
    rpc.set_volume(10)
    rpc.pc2_play_tts("机器人初始化完成")
    while (not flag_exit) and tcp_connected_vision:
        while (not flag_exit) and tcp_connected_vision:
            rescuestask_init()
            # target_1 = np.array([-0.7, 1.57, -0.87])
            # a_robot.dynamic_lift_approach(target_1)
            rescue_control(amr=amr, a_robot=a_robot, client_socket_vision=client_socket_vision)
            break

            
    if client_socket_vision:
        client_socket_vision.close()
        print(Fore.GREEN + "相机TCP连接已关闭")
