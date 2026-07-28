# pip install ruckig
# conda install  colorama
# conda install pinocchio -c conda-forge
# pip install roboticstoolbox-python
# pip install meshcat
#  export CYCLONEDDS_URI=/home/dobotpc2/Documents/robot_dds-develop/cyclonedds.xml


import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText
import random
import time
from threading import Thread
import numpy as np
import copy
from typing import Optional
from enum import Enum
from spatialmath import SE3, SO3
from pinocchio.visualize import MeshcatVisualizer   

from ruckig import InputParameter, OutputParameter, Result, Ruckig, ControlInterface

import robot_model 
from robot_model import Arm_IK as robot_model
import pinocchio as pin                             
import re                           

from colorama import Fore, Back, Style, init
init() 

import sys
import os
import json
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
print(BASE_DIR)
from robot_control_dds.control_sim import Control_sim
import robot_upper_control as robot_control_handle

# 点位数据保存文件路径
SAVED_POINTS_FILE = os.path.join(BASE_DIR, "saved_points.json")

joint_angles_left = np.zeros((7)) #全局左手关节角度
joint_angles_right = np.zeros((7))#全局右手关节角度
joint_angles_handle_left = np.zeros((6)) #全局灵巧手角度
joint_angles_handle_right = np.zeros((6)) #全局灵巧手角度
head_angle = np.zeros((2)) #全局头部角度
torso_angle = 0.0  # 全局腰部角度

pose_left = SE3() # flange系
pose_right = SE3()# flange系


tool_left=SE3() #left 工具
# tool_left=SE3(   0.18776726 ,  -0.0824215,  -0.038187     )#工具
# tool_left.A[:3,:3] =SO3.RPY(0,90,-90,unit='deg')

# tool_right=SE3() #right 工具
tool_right=SE3(0.21995386 ,0.05015792 ,0.03143192)#工具
tool_right.A[:3,:3] =SO3.RPY(0,90,-90,unit='deg')


CYCLE  = 0.001

simulate  =False #是否仿真
isdrag  =False #是否拖拽

serial_test_flag = False #是否开启老化测试
movaJ_test_flag = False #是否开启movJ运动
movaL_test_flag = False #是否开启movJ运动

# 灵巧手默认参数配置（可在程序启动前修改）
default_hand_values_left = ["1000", "1000", "1000", "1000", "1000", "1000"]  # 左手默认角度值
# default_hand_values_left = ["600", "600", "600", "600", "600", "500"]  # 备用配置

default_hand_values_right = ["1000", "1000", "1000", "1000", "1000", "1000"]  # 右手默认角度值
# default_hand_values_right = ["600", "600", "600", "600", "600", "500"]  # 备用配置

# 点位存储字典（P1, P2, P3等），可在其他文件中导入使用
# 格式: {"P1": {"joint_left": [...], "joint_right": [...], "pose_left": [...], "pose_right": [...]}, ...}
saved_points = {}

def normalize_saved_points_structure():
    """确保点位数据结构兼容新格式"""
    for point_name, entry in list(saved_points.items()):
        if not isinstance(entry, dict):
            continue
        if "type_left" not in entry:
            if entry.get("joint_left"):
                entry["type_left"] = "joint"
            elif entry.get("pose_left"):
                entry["type_left"] = "pose"
            else:
                entry["type_left"] = entry.get("type")
        if "type_right" not in entry:
            if entry.get("joint_right"):
                entry["type_right"] = "joint"
            elif entry.get("pose_right"):
                entry["type_right"] = "pose"
            else:
                entry["type_right"] = entry.get("type")

def load_saved_points():
    """从JSON文件加载保存的点位数据"""
    global saved_points
    try:
        if os.path.exists(SAVED_POINTS_FILE):
            with open(SAVED_POINTS_FILE, 'r', encoding='utf-8') as f:
                saved_points = json.load(f)
            print(f"成功加载点位数据，共 {len(saved_points)} 个点位")
            normalize_saved_points_structure()
            return True
        else:
            print("点位数据文件不存在，使用空字典")
            saved_points = {}
            return False
    except Exception as e:
        print(f"加载点位数据失败: {str(e)}")
        saved_points = {}
        return False

def save_saved_points():
    """将点位数据保存到JSON文件"""
    global saved_points
    try:
        with open(SAVED_POINTS_FILE, 'w', encoding='utf-8') as f:
            json.dump(saved_points, f, ensure_ascii=False, indent=2)
        return True
    except Exception as e:
        print(f"保存点位数据失败: {str(e)}")
        return False


# 人形臂192.168.8.234  root ,123456,enablemotor 1为电机上使能


class class_ruckig_joint_teach:

    class State(Enum):
        working = "working"
        end = "end"
        error = "error"
        standill = "standill"

    def __init__(self, vel, acc):
        self.vel = vel
        self.acc = acc
        self.status = class_ruckig_joint_teach.State.standill
        self.otg = Ruckig(7, CYCLE)  # DoFs, control cycle
        self.inp = InputParameter(7)
        self.out = OutputParameter(7)
        self.inp.control_interface = ControlInterface.Velocity
        self.inp.max_velocity = np.ones(7)*self.vel
        self.inp.max_acceleration = np.ones(7)*self.acc
        self.inp.max_jerk = np.ones(7)*self.acc*30
        self.titck = 25
        self.dir = -1
        self.id = -1

    def start(self, dir, id, scale):
        if self.status == class_ruckig_joint_teach.State.standill:
            self.status = class_ruckig_joint_teach.State.working
            
            if int(arm_type_var.get()) == 0:
                self.inp.current_position = joint_angles_left
            else:
                self.inp.current_position = joint_angles_right
            
            self.inp.current_velocity = np.zeros(7)
            self.inp.current_acceleration = np.zeros(7)

            target_velocity = np.zeros(7)
            target_velocity[id] = self.vel*dir*scale
            self.inp.target_velocity = target_velocity
            self.inp.target_acceleration = np.zeros(7)

            self.titck = 25

            self.dir = dir
            self.id = id

        elif self.status == class_ruckig_joint_teach.State.working:
            if dir != self.dir or id != self.id:
                self.end()
            else:
                self.titck = 25

    def end(self):
        if self.status != class_ruckig_joint_teach.State.standill:
            self.status = class_ruckig_joint_teach.State.end
            self.inp.target_velocity = np.zeros(7)

            self.inp.max_acceleration = np.array(self.inp.max_acceleration) * 2
            self.inp.max_jerk = np.array(self.inp.max_jerk)  * 4
            
        
    def update(self):

        global pose_left
        global pose_right
        global joint_angles_left
        global joint_angles_right
        
        res = self.otg.update(self.inp, self.out)

        if self.status == class_ruckig_joint_teach.State.working:
            self.titck = self.titck - 1

            if self.titck == 0:
                    self.end()
                    res = self.otg.update(self.inp, self.out)
                

        if self.status == class_ruckig_joint_teach.State.end and res == Result.Finished:
            self.status = class_ruckig_joint_teach.State.standill

        self.out.pass_to_input(self.inp)

        if int(arm_type_var.get()) == 0:
            joint_angles_left = np.array(self.out.new_position).copy()
        else:
            joint_angles_right = np.array(self.out.new_position).copy()
            
        # 更新实时数据
        pin.forwardKinematics(robot.reduced_robot.model, robot.data,  np.append( joint_angles_left,joint_angles_right)  )
        frame_id = robot.reduced_robot.model.getJointId('left_wrist_yaw_joint') 
        init_T  = robot.data.oMi[  frame_id]
        init_T  = SE3(  init_T .homogeneous)
        pose_left = init_T  
        
        frame_id = robot.reduced_robot.model.getJointId('right_wrist_yaw_joint') 
        init_T  = robot.data.oMi[  frame_id]
        init_T  = SE3(  init_T .homogeneous)
        pose_right = init_T  
        # ------------------------------------------


class class_ruckig_tcp_teach:

    class State(Enum):
        working = "working"
        end = "end"
        error = "error"
        standill = "standill"

    def __init__(self, vel, acc):
        self.vel = vel
        self.acc = acc
        self.status = class_ruckig_tcp_teach.State.standill

        self.otg = Ruckig(6, CYCLE)  # DoFs, control cycle
        self.inp = InputParameter(6)
        self.out = OutputParameter(6)
        self.inp.control_interface = ControlInterface.Velocity
        self.inp.max_velocity = np.ones(7)*self.vel
        self.inp.max_acceleration = np.ones(7)*self.acc
        self.inp.max_jerk = np.ones(7)*self.acc*30
        self.titck = 15
        self.dir = -1
        self.id = -1

        self.init_T =SE3()

    def start(self, dir, id, scale):
        if self.status == class_ruckig_tcp_teach.State.standill:
            self.status = class_ruckig_tcp_teach.State.working
            self.inp.current_position = np.zeros(6)
            self.inp.current_velocity = np.zeros(6)
            self.inp.current_acceleration = np.zeros(6)

            target_velocity = np.zeros(6)
            target_velocity[id] = self.vel*dir*scale

            self.inp.max_velocity = target_velocity
            self.inp.max_acceleration = np.ones(7)*self.acc
            self.inp.max_jerk = np.ones(7)*self.acc*30

            if id >= 3:
                target_velocity = target_velocity * 10
                self.inp.max_velocity = target_velocity
                self.inp.max_acceleration = np.array(self.inp.max_acceleration) * 20
                self.inp.max_jerk = np.array(self.inp.max_jerk) * 40
            else:
                target_velocity = target_velocity * 5
                self.inp.max_velocity = target_velocity
                self.inp.max_acceleration = np.array(self.inp.max_acceleration) * 10
                self.inp.max_jerk = np.array(self.inp.max_jerk) * 40
                
            self.inp.target_velocity = target_velocity
            self.inp.target_acceleration = np.zeros(7)

            self.titck = 15

            self.dir = dir
            self.id = id

            robot.init_data = np.append( joint_angles_left,joint_angles_right).copy()

            if int(arm_type_var.get()) == 0:
                self.init_T  = pose_left
            else:
                self.init_T  = pose_right

        elif self.status == class_ruckig_tcp_teach.State.working:

            if dir != self.dir or id != self.id:
                self.end()
                print(Fore.RED +f" dir = {dir}-{self.dir}")
                print(Fore.RED +f" id = {id}-{self.id}")
            else:
                self.titck = 15

    def end(self):
        if self.status != class_ruckig_tcp_teach.State.standill:
            self.status = class_ruckig_tcp_teach.State.end
            self.inp.target_velocity = np.zeros(6)
            self.inp.max_acceleration = np.array(self.inp.max_acceleration) * 20
            self.inp.max_jerk = np.array(self.inp.max_jerk)  * 100

    def update(self):

        res = self.otg.update(self.inp, self.out)

        if self.status == class_ruckig_tcp_teach.State.working:
            self.titck = self.titck - 1

            if self.titck == 0:
                self.end()
                res = self.otg.update(self.inp, self.out)

        if self.status == class_ruckig_tcp_teach.State.end and res == Result.Finished:
            self.status = class_ruckig_tcp_teach.State.standill

        self.out.pass_to_input(self.inp)

        global pose_left
        global pose_right
        global joint_angles_left
        global joint_angles_right
        global tool_left
        global tool_right

        # 工具系下点动

        # detal_T = SE3(self.out.new_position[0:3])
        # src_R = self.out.new_position[3:6]
        # src_R = SO3.AngVec(np.linalg.norm(src_R),  src_R)
        # detal_T.A[:3, :3] = src_R.R

        # if int(arm_type_var.get()) == 0:
        #     pose_left = self.init_T * tool_left * detal_T * tool_left.inv()
        # else:
        #     pose_right = self.init_T * tool_right * detal_T * tool_right.inv()
        # ---------------------------------

        # base系下点动
        if int(arm_type_var.get()) == 0:
            tool_T = tool_left
        else:
            tool_T = tool_right

        temp = (self.init_T * tool_T ).inv()
        xyz_dir = temp.A[:3,:3]@self.out.new_position[0:3]
        rpy_dir = temp.A[:3,:3]@self.out.new_position[3:6]

        detal_T = SE3(xyz_dir)
        src_R = SO3.AngVec(np.linalg.norm(rpy_dir),  rpy_dir)
        detal_T.A[:3, :3] = src_R.R

        if int(arm_type_var.get()) == 0:
            pose_left = self.init_T * tool_left * detal_T * tool_left.inv()  

        else:
            pose_right = self.init_T * tool_right * detal_T * tool_right.inv()  

        # ---------------------------------

        ik_res =  robot.ik_fun(pose_left.A.reshape(4, 4), pose_right.A.reshape(4, 4),np.append(joint_angles_left, joint_angles_right).copy()  )

        if type( ik_res)!=type( np.ndarray([])):
            ik_res = ik_res[0]

        if int(arm_type_var.get()) == 0:
            joint_angles_left = np.array(ik_res[0:7]).copy()
        else:
            joint_angles_right = np.array(ik_res[7:14]).copy()


class class_ruckig_tcp_Runto:

    class State(Enum):
        working = "working"
        end = "end"
        error = "error"
        standill = "standill"

    def __init__(self, vel, acc):
        self.vel = vel
        self.acc = acc
        self.status = class_ruckig_tcp_Runto.State.standill

        self.otg = Ruckig(1, CYCLE)  # DoFs, control cycle
        self.inp = InputParameter(1)
        self.out = OutputParameter(1)
        self.inp.control_interface = ControlInterface.Position

        self.inp.current_position = [0.0]
        self.inp.current_velocity = [0.0]
        self.inp.current_acceleration = [0.0]

        self.inp.target_position = [1.0]
        self.inp.target_velocity = [0.0]
        self.inp.target_acceleration = [0.0]

        self.inp.max_velocity = np.ones(1)*self.vel
        self.inp.max_acceleration = np.ones(1)*self.acc*0.5
        self.inp.max_jerk = np.ones(1)*self.acc * 20*0.5
        self.titck = 15

        self.init_T =SE3()
        
        
    def start(self, target_T, scale):
        global pose_left
        global pose_right
        
        if self.status == class_ruckig_tcp_Runto.State.standill:
            self.status = class_ruckig_tcp_Runto.State.working

            self.inp.current_position = [0.0]
            self.inp.current_velocity = [0.0]
            self.inp.current_acceleration = [0.0]

            self.inp.target_position = [1.0]
            self.inp.target_velocity = [0.0]
            self.inp.target_acceleration = [0.0]

            self.titck = 15

            if int(arm_type_var.get()) == 0:
                self.init_T  = pose_left.copy()
            else:
                self.init_T  = pose_right.copy()

            if int(arm_type_var.get()) == 0:
                self.target_T = target_T * tool_left.inv() #转flange系下
            else:
                self.target_T = target_T * tool_right.inv() #转flange系下

            self.inp.control_interface = ControlInterface.Position

            self.detal_T  =  copy.deepcopy(self.init_T.inv() *  self.target_T)
            self.detal_xyz = copy.deepcopy(self.detal_T.A[:3, 3] )
            [self.detal_R_angle , self.detal_R_vec]  =self.detal_T.angvec()

            T_max_length = np.max([np.linalg.norm(self.detal_xyz   ) ,self.detal_R_angle  *0.5] )
            if (T_max_length<1e-5):
                T_max_length  =1e-3

            self.inp.max_velocity = np.ones(1)*self.vel/T_max_length * scale
            self.inp.max_acceleration = np.ones(1)*self.acc/T_max_length * scale * 5
            self.inp.max_jerk = np.ones(1)*self.acc/T_max_length*35

            robot.init_data = np.append( joint_angles_left,joint_angles_right).copy()

        elif self.status == class_ruckig_tcp_Runto.State.working:
            self.titck = 15

    def end(self):
        if self.status != class_ruckig_tcp_Runto.State.standill:
            self.status = class_ruckig_tcp_Runto.State.end
            self.inp.control_interface = ControlInterface.Velocity

            self.inp.target_velocity = [0]
            self.inp.target_acceleration = [0.0]

            self.inp.max_acceleration = np.array(self.inp.max_acceleration) * 10
            self.inp.max_jerk = np.array(self.inp.max_jerk)  * 60

    def update(self):

        res = self.otg.update(self.inp, self.out)

        if self.status == class_ruckig_tcp_Runto.State.working:
            self.titck = self.titck - 1

            if self.titck == 0:
                self.end()
                res = self.otg.update(self.inp, self.out)

        if self.status == class_ruckig_tcp_Runto.State.end and res == Result.Finished:
            self.status = class_ruckig_tcp_Runto.State.standill

        self.out.pass_to_input(self.inp)

        global pose_left
        global pose_right
        global joint_angles_left
        global joint_angles_right
        global joint_angles_handle_left
        global joint_angles_handle_right
        
        #  生成轨迹
        
        detal_T = SE3(self.detal_xyz*self.out.new_position[0])
        src_R = SO3.AngVec(self.detal_R_angle * self.out.new_position[0],  self.detal_R_vec)
        detal_T.A[:3, :3] = src_R.R
        # ------------------------------------

        if int(arm_type_var.get()) == 0:
            pose_left =  self.init_T * detal_T #T_B_flange
        else:
            pose_right =  self.init_T * detal_T #T_B_flange
        
        # print(
        #     Fore.RED + f"self.out.new_position[0]  ={self.out.new_position[0]} , self.titck = {self.titck}")
        
        ik_res =  robot.ik_fun(pose_left.A.reshape(4, 4), pose_right.A.reshape(4, 4),np.append(joint_angles_left, joint_angles_right).copy()  )

        #!误差校验
        # pin.forwardKinematics(robot.reduced_robot.model, robot.data,  ik_res )
        # if int(arm_type_var.get()) == 0:
        #         frame_id = robot.reduced_robot.model.getJointId('left_wrist_yaw_joint')
        # else:
        #         frame_id = robot.reduced_robot.model.getJointId('right_wrist_yaw_joint')
        # test_T = robot.data.oMi[  frame_id]
        # test_T = SE3(  test_T.homogeneous)

        # print(Fore.YELLOW + f"test_T  ={test_T.A[0:3,3] }")
        #!-------------------------------

        if type( ik_res)!=type( np.ndarray([])):
            ik_res = ik_res[0]

        if int(arm_type_var.get()) == 0:
            joint_angles_left = np.array(ik_res[0:7])
        else:
            joint_angles_right = np.array(ik_res[7:14])


class class_ruckig_joint_Runto:

    class State(Enum):
        working = "working"
        end = "end"
        error = "error"
        standill = "standill"

    def __init__(self, vel, acc):
        self.vel = vel
        self.acc = acc
        self.status = class_ruckig_joint_Runto.State.standill
        
        
        self.otg = Ruckig(7, CYCLE)  # DoFs, control cycle
        self.inp = InputParameter(7)
        self.out = OutputParameter(7)
        self.inp.control_interface = ControlInterface.Position
        
        self.inp.current_position = np.zeros(7)
        self.inp.current_velocity = np.zeros(7)
        self.inp.current_acceleration = np.zeros(7)
    
        self.inp.target_position = np.zeros(7)
        self.inp.target_velocity = np.zeros(7)
        self.inp.target_acceleration = np.zeros(7)
        
        self.inp.max_velocity = np.ones(7)*self.vel
        self.inp.max_acceleration = np.ones(7)*self.acc
        self.inp.max_jerk = np.ones(7)*self.acc*40
        self.titck = 25
        
        
        

    def start(self,target_joint ,scale):
        if self.status == class_ruckig_joint_Runto.State.standill:
            self.status = class_ruckig_joint_Runto.State.working
            
            if int(arm_type_var.get()) == 0:
                self.inp.current_position = joint_angles_left
            else:
                self.inp.current_position = joint_angles_right
                
                
            self.inp.current_velocity =np.zeros(7)
            self.inp.current_acceleration = np.zeros(7)

            self.inp.target_position = target_joint
            self.inp.target_velocity = np.zeros(7)
            self.inp.target_acceleration = np.zeros(7)


            self.titck = 25
            
            self.inp.control_interface = ControlInterface.Position
  
            self.inp.max_velocity = np.ones(7)*self.vel * scale
            self.inp.max_acceleration = np.ones(7)*self.acc * scale * 5
            self.inp.max_jerk = np.ones(7)*self.acc*20


        elif self.status == class_ruckig_joint_Runto.State.working:
            self.titck = 25
            

    def end(self):
        if self.status != class_ruckig_joint_Runto.State.standill:
            self.status = class_ruckig_joint_Runto.State.end
            self.inp.control_interface = ControlInterface.Velocity
            
            self.inp.target_velocity =  np.zeros(7)
            self.inp.target_acceleration =  np.zeros(7)
            
            self.inp.max_acceleration = np.array(self.inp.max_acceleration) * 1.4
            self.inp.max_jerk = np.array(self.inp.max_jerk)  * 1.4
        
        

    def update(self):

        res = self.otg.update(self.inp, self.out)

        if self.status == class_ruckig_joint_Runto.State.working:
            self.titck = self.titck - 1

            if self.titck == 0:
                self.end()
                res = self.otg.update(self.inp, self.out)
                

        if self.status == class_ruckig_joint_Runto.State.end and res == Result.Finished:
            self.status = class_ruckig_joint_Runto.State.standill


        self.out.pass_to_input(self.inp)
        
        
        global pose_left
        global pose_right
        global joint_angles_left
        global joint_angles_right
        global joint_angles_handle_left
        global joint_angles_handle_right
        
        if int(arm_type_var.get()) == 0:
            joint_angles_left = np.array(self.out.new_position)
        else:
            joint_angles_right = np.array(self.out.new_position)

        # print(
            # Fore.RED + f"self.out.new_position  ={self.out.new_position} , self.titck = {self.titck}")
        

        # 更新实时数据
        pin.forwardKinematics(robot.reduced_robot.model, robot.data,  np.append( joint_angles_left,joint_angles_right)  )
        frame_id = robot.reduced_robot.model.getJointId('left_wrist_yaw_joint') 
        init_T  = robot.data.oMi[  frame_id]
        init_T  = SE3(  init_T .homogeneous)
        pose_left = init_T  
        
        frame_id = robot.reduced_robot.model.getJointId('right_wrist_yaw_joint') 
        init_T  = robot.data.oMi[  frame_id]
        init_T  = SE3(  init_T .homogeneous)
        pose_right = init_T  
        # ------------------------------------------


class class_ruckig_head_Runto:

        
    class State(Enum):
        working = "working"
        end = "end"
        error = "error"
        standill = "standill"

    def __init__(self, vel, acc):
        self.vel = vel
        self.acc = acc
        self.status = class_ruckig_head_Runto.State.standill
        
        
        self.otg = Ruckig(2, CYCLE)  # DoFs, control cycle
        self.inp = InputParameter(2)
        self.out = OutputParameter(2)
        self.inp.control_interface = ControlInterface.Position
        
        self.inp.current_position = np.zeros(2)
        self.inp.current_velocity = np.zeros(2)
        self.inp.current_acceleration = np.zeros(2)
    
        self.inp.target_position = np.zeros(2)
        self.inp.target_velocity = np.zeros(2)
        self.inp.target_acceleration = np.zeros(2)
        
        self.inp.max_velocity = np.ones(2)*self.vel
        self.inp.max_acceleration = np.ones(2)*self.acc
        self.inp.max_jerk = np.ones(2)*self.acc*40
        self.titck = 25
        

    def start(self,target_joint ,scale):
        global head_angle
        
        if self.status == class_ruckig_head_Runto.State.standill:
            self.status = class_ruckig_head_Runto.State.working
            
            self.inp.current_position = head_angle
            self.inp.current_velocity =np.zeros(2)
            self.inp.current_acceleration = np.zeros(2)

            self.inp.target_position = target_joint
            self.inp.target_velocity = np.zeros(2)
            self.inp.target_acceleration = np.zeros(2)


            self.titck = 25
            
            self.inp.control_interface = ControlInterface.Position
  
            self.inp.max_velocity = np.ones(2)*self.vel * scale
            self.inp.max_acceleration = np.ones(2)*self.acc * scale * 5
            self.inp.max_jerk = np.ones(2)*self.acc*20


        elif self.status == class_ruckig_head_Runto.State.working:
            self.titck = 25
            

    def end(self):
        if self.status != class_ruckig_head_Runto.State.standill:
            self.status = class_ruckig_head_Runto.State.end
            self.inp.control_interface = ControlInterface.Velocity
            
            self.inp.target_velocity =  np.zeros(2)
            self.inp.target_acceleration =  np.zeros(2)
            
            self.inp.max_acceleration = np.array(self.inp.max_acceleration) * 1.4
            self.inp.max_jerk = np.array(self.inp.max_jerk)  * 1.4
        
        

    def update(self):
        
        global head_angle

        res = self.otg.update(self.inp, self.out)

        if self.status == class_ruckig_head_Runto.State.working:
            self.titck = self.titck - 1

            if self.titck == 0:
                self.end()
                res = self.otg.update(self.inp, self.out)
                

        if self.status == class_ruckig_head_Runto.State.end and res == Result.Finished:
            self.status = class_ruckig_head_Runto.State.standill


        self.out.pass_to_input(self.inp)

        head_angle = np.array(self.out.new_position)



# CP运动还未完成
class class_ruckig_joint_cp:

    class State(Enum):
        working = "working"
        end = "end"
        error = "error"
        standill = "standill"

    def __init__(self, vel, acc):
        self.vel = vel
        self.acc = acc
        self.status = class_ruckig_joint_cp.State.standill
        
        
        self.otg = Ruckig(7, CYCLE)  # DoFs, control cycle

        self.titck = 25
        
        self.traj = []

        self.current_traj_index=0
        
    def start(self ,scale):
        if self.status == class_ruckig_joint_Runto.State.standill:
            self.status = class_ruckig_joint_Runto.State.working
            
            self.titck = 25
            
            
            plan_info = {
                "current": np.array([0, 0, 0, 0, 0, 0]),
                "targrt": np.array([100, 100, 100, 100, 100, 100]),
                "vel": 1,
                "acc": 10,
                "jerk": 100,
                "time_all": 0,
                "CP": 20,
                "time_cp": 0,
                "inp":InputParameter(7),
                "out" :OutputParameter(7),
                "count_cycle":0
            }
                        
            plan_info["inp"].current_position= plan_info["current"]
            plan_info["inp"].current_velocity =np.zeros(7)
            plan_info["inp"].current_acceleration = np.zeros(7)

            plan_info["inp"].target_position = plan_info["targrt"]
            plan_info["inp"].target_velocity = np.zeros(7)
            plan_info["inp"].target_acceleration = np.zeros(7)

            plan_info["inp"].control_interface = ControlInterface.Position
            plan_info["inp"].max_velocity = np.ones(7)*plan_info["vel"]* scale
            plan_info["inp"].max_acceleration = np.ones(7)*plan_info["acc"]
            plan_info["inp"].max_jerk = np.ones(7)*plan_info["jerk"]
            
            self.otg.update(plan_info["inp"], plan_info["out"])
            plan_info["time_all"] =   plan_info["out"].trajectory.duration
            plan_info["time_cp"]  = plan_info["time_all"]*(1-plan_info["CP"]*0.01)
            
            self.traj.append(plan_info)
            
            plan_info = {
                "current": np.array([100, 100, 100, 100, 100, 100]),
                "targrt": np.array([100, 150, 150, 100, 100, 100]),
                "vel": 1,
                "acc": 10,
                "jerk": 100,
                "time_all": 0,
                "CP": 20,
                "time_cp": 0,
                "inp":InputParameter(7),
                "out" :OutputParameter(7),
                "count_cycle":0
            }
                        
            plan_info["inp"].current_position= plan_info["current"]
            plan_info["inp"].current_velocity =np.zeros(7)
            plan_info["inp"].current_acceleration = np.zeros(7)

            plan_info["inp"].target_position = plan_info["targrt"]
            plan_info["inp"].target_velocity = np.zeros(7)
            plan_info["inp"].target_acceleration = np.zeros(7)

            plan_info["inp"].control_interface = ControlInterface.Position
            plan_info["inp"].max_velocity = np.ones(7)*plan_info["vel"]* scale
            plan_info["inp"].max_acceleration = np.ones(7)*plan_info["acc"]
            plan_info["inp"].max_jerk = np.ones(7)*plan_info["jerk"]
            
            self.otg.update(plan_info["inp"], plan_info["out"])
            plan_info["time_all"] =   plan_info["out"].trajectory.duration
            plan_info["time_cp"]  = plan_info["time_all"]*(1-plan_info["CP"]*0.01)
            
            self.traj.append(plan_info)

            plan_info = {
                "current": np.array([100, 150, 150, 100, 100, 100]),
                "targrt": np.array([100, 150, 150, 200, 200, 200]),
                "vel": 1,
                "acc": 10,
                "jerk": 100,
                "time_all": 0,
                "CP": 20,
                 "time_cp": 0,
                "inp":InputParameter(7),
                "out" :OutputParameter(7),
                "count_cycle":0
            }
                        
            plan_info["inp"].current_position= plan_info["current"]
            plan_info["inp"].current_velocity =np.zeros(7)
            plan_info["inp"].current_acceleration = np.zeros(7)

            plan_info["inp"].target_position = plan_info["targrt"]
            plan_info["inp"].target_velocity = np.zeros(7)
            plan_info["inp"].target_acceleration = np.zeros(7)

            plan_info["inp"].control_interface = ControlInterface.Position
            plan_info["inp"].max_velocity = np.ones(7)*plan_info["vel"]* scale
            plan_info["inp"].max_acceleration = np.ones(7)*plan_info["acc"]
            plan_info["inp"].max_jerk = np.ones(7)*plan_info["jerk"]
            
            self.otg.update(plan_info["inp"], plan_info["out"])
            plan_info["time_all"] =   plan_info["out"].trajectory.duration
            plan_info["time_cp"]  = plan_info["time_all"]*(1-plan_info["CP"]*0.01)
            
            self.traj.append(plan_info)
            
            
        elif self.status == class_ruckig_joint_Runto.State.working:
            self.titck = 25
            

    def end(self):
        if self.status != class_ruckig_joint_Runto.State.standill:
            self.status = class_ruckig_joint_Runto.State.end
            self.inp.control_interface = ControlInterface.Velocity
            
            self.inp.target_velocity =  np.zeros(7)
            self.inp.target_acceleration =  np.zeros(7)
            
            self.inp.max_acceleration = np.array(self.inp.max_acceleration) * 1.4
            self.inp.max_jerk = np.array(self.inp.max_jerk)  * 1.4
        

    def update(self):

        if  self.current_traj_index < self.traj.count():
            plan_unit = self.traj[self.current_traj_index]
            if(plan_unit["count_cycle"]< plan_unit["time_cp"]):
                self.otg.update(plan_unit["inp"], plan_unit["out"])
            elif( plan_unit["count_cycle"] < plan_unit["time_all"]):
                self.otg.update(plan_unit["inp"], plan_unit["out"])
                plan_unit = self.traj[self.current_traj_index]
                



        res = self.otg.update(self.inp, self.out)

        if self.status == class_ruckig_joint_Runto.State.working:
            self.titck = self.titck - 1

            if self.titck == 0:
                self.end()
                res = self.otg.update(self.inp, self.out)
                

        if self.status == class_ruckig_joint_Runto.State.end and res == Result.Finished:
            self.status = class_ruckig_joint_Runto.State.standill


        self.out.pass_to_input(self.inp)
        
        
        global pose_left
        global pose_right
        global joint_angles_left
        global joint_angles_right
        
        if int(arm_type_var.get()) == 0:
            joint_angles_left = np.array(self.out.new_position)
        else:
            joint_angles_right = np.array(self.out.new_position)

        # print(
            # Fore.RED + f"self.out.new_position  ={self.out.new_position} , self.titck = {self.titck}")
        

        # 更新实时数据
        pin.forwardKinematics(robot.reduced_robot.model, robot.data,  np.append( joint_angles_left,joint_angles_right)  )
        frame_id = robot.reduced_robot.model.getJointId('left_wrist_yaw_joint') 
        init_T  = robot.data.oMi[  frame_id]
        init_T  = SE3(  init_T .homogeneous)
        pose_left = init_T  
        
        frame_id = robot.reduced_robot.model.getJointId('right_wrist_yaw_joint') 
        init_T  = robot.data.oMi[  frame_id]
        init_T  = SE3(  init_T .homogeneous)
        pose_right = init_T  
        # ------------------------------------------


def is_command_running():
    return (serial_test_flag==True or movaJ_test_flag==True or movaL_test_flag==True )

# 模拟数据生成器
def generate_data(update_joint_labels, update_pose_label):
    global joint_angles_left
    global joint_angles_right
    global pose_left
    global pose_right

    global joint_angles_handle_left
    global joint_angles_handle_right

    global head_angle
    global torso_angle

    while True:

        # 规划更新
        if ruckig_joint_teach.status == class_ruckig_joint_teach.State.working or ruckig_joint_teach.status == class_ruckig_joint_teach.State.end:
            ruckig_joint_teach.update()
            # print(
            #     Fore.YELLOW + f"ruckig_joint_teach.status ={ruckig_joint_teach.status} , tick ={ ruckig_joint_teach.titck}")

        elif ruckig_tcp_teach.status == class_ruckig_tcp_teach.State.working or ruckig_tcp_teach.status == class_ruckig_tcp_teach.State.end:
            ruckig_tcp_teach.update()
            # print(
            #     Fore.YELLOW + f"ruckig_tcp_teach.status ={ruckig_tcp_teach.status} , tick ={ ruckig_tcp_teach.titck}")

        elif ruckig_tcp_runto.status == class_ruckig_tcp_Runto.State.working or ruckig_tcp_runto.status == class_ruckig_tcp_Runto.State.end:
            ruckig_tcp_runto.update()
            # print(
            #     Fore.YELLOW + f"ruckig_tcp_runto.status ={ruckig_tcp_runto.status} , tick ={ ruckig_tcp_runto.titck}")

        elif ruckig_joint_runto.status == class_ruckig_joint_Runto.State.working or ruckig_joint_runto.status == class_ruckig_joint_Runto.State.end:
            ruckig_joint_runto.update()
            # print(
            #     Fore.YELLOW + f"ruckig_joint_runto.status ={ruckig_joint_runto.status} , tick ={ ruckig_joint_runto.titck}")
        
        elif ruckig_head_runto.status == class_ruckig_head_Runto.State.working or ruckig_head_runto.status == class_ruckig_head_Runto.State.end:
            ruckig_head_runto.update()

        # 上位机信息更新,不需要显示头部角度
        update_joint_labels(joint_angles_left, joint_angles_right, joint_angles_handle_left,joint_angles_handle_right)
        update_pose_label(pose_left, pose_right)
        # mechs更新
        # viz.display(np.append(joint_angles_left, joint_angles_right))
        # 伺服更新
        servoJ(
            np.append(joint_angles_left, joint_angles_right).copy(),
            np.append(joint_angles_handle_left, joint_angles_handle_right).copy(),
            head_angle.copy(),
            torso_angle,
        )

        time.sleep(CYCLE)  # 模拟数据刷新频率


def servoJ(
    command_q_14dof: np.array,
    command_hand_12dof: np.array,
    command_head_2dof: np.array,
    command_torsor_1dof: Optional[float] = None,
    qd_limit=np.deg2rad(1000),
):
    global simulate
    global isdrag
    global torso_angle

    if command_torsor_1dof is None:
        command_torsor_1dof = torso_angle

    current_torsor_state = torso_angle

    if simulate==True:
        if not hasattr(servoJ, "last_q"):
            servoJ.last_q = command_q_14dof.copy()
            
        servo_q = command_q_14dof.copy()
        qd = (servo_q - servoJ.last_q)/CYCLE
        
        
        # for i in range(14):
        #     if abs(qd[i]) > qd_limit:
        #         print( Fore.RED + f"q_command_{i} = {  np.rad2deg( servo_q[i])  } q_last_{i} = {  np.rad2deg( servoJ.last_q[i])  } ,overspeed {np.rad2deg(qd[i])},limit = {np.rad2deg(qd_limit ) }")
        #         log_text.insert(  tk.END,  f"q_command_{i} = {  np.rad2deg( servo_q[i])  } q_last_{i} = {  np.rad2deg( servoJ.last_q[i])  } ,overspeed {np.rad2deg(qd[i])},limit = {np.rad2deg(qd_limit ) }\n")
        #         log_text.see(tk.END)

                # root.destroy()
                # sys.exit(-1)

        q_lower_limit = robot.reduced_robot.model.lowerPositionLimit+ np.ones(len(robot.reduced_robot.model.lowerPositionLimit))*np.deg2rad(10)
        q_upper_limit = robot.reduced_robot.model.upperPositionLimit- np.ones(len(robot.reduced_robot.model.lowerPositionLimit))*np.deg2rad(10)

        command_q_14dof_pre = command_q_14dof.copy()
        qd_pre = qd.copy()
        for i in range(14):
            command_q_14dof_pre[i] = command_q_14dof_pre[i] +  qd_pre[i]*0.008

        # for i in range(14):
        #     if (command_q_14dof_pre[i]) > q_upper_limit[i]:
        #         print(
        #             Fore.RED + f"q_{i} = {  np.rad2deg( command_q_14dof[i])  } outside,limit = {np.rad2deg(q_upper_limit[i] ) }")
        #         log_text.insert(  tk.END,    f"q_{i} = {  np.rad2deg( command_q_14dof[i])  } outside,limit = {np.rad2deg(q_upper_limit[i] ) }\n")
        #         log_text.see(tk.END)
        #         # root.destroy()
        #         # sys.exit(-1)

        # for i in range(14):
        #     if (command_q_14dof_pre[i]) <  q_lower_limit[i]:
        #         print(
        #             Fore.RED + f"q_{i} = {  np.rad2deg( command_q_14dof[i])  } outside,limit = {np.rad2deg(q_lower_limit[i] ) }")
        #         log_text.insert(  tk.END,    f"q_{i} = {  np.rad2deg( command_q_14dof[i])  } outside,limit = {np.rad2deg(q_lower_limit[i] ) }\n")
        #         log_text.see(tk.END)
        #         # root.destroy()
        #         # sys.exit(-1)

        servoJ.last_q = servo_q.copy()
        torso_angle = command_torsor_1dof

    else:
        
        if not isdrag : # 不是拖拽模式
        
            if not hasattr(servoJ, "last_q"):
                (
                    current_arm_state,
                    current_hand_state,
                    current_head_state,
                    current_torsor_state,
                ) = real_robot.get_joint_state(include_torso=True)
                servoJ.last_q = current_arm_state.copy()
                torso_angle = current_torsor_state


            current_arm_state, current_hand_state, current_head_state, current_torsor_state = real_robot.get_joint_state(include_torso=True)
            
            servo_q = current_arm_state.copy()
            servo_q[0:7] = command_q_14dof[0:7] #left_arm_joint
            servo_q[0:7] = np.multiply(servo_q[0:7] , real_robot_q_left_dir)+real_robot_q_left_offset

            servo_q[7:14] = command_q_14dof[7:14] #right_arm_joint
            servo_q[7:14] = np.multiply(servo_q[7:14] , real_robot_q_right_dir)+real_robot_q_right_offset

            qd = (servo_q - servoJ.last_q)/CYCLE

            for i in range(14):
                if abs(qd[i]) > qd_limit:
                    print( Fore.RED + f"q_command_{i} = {  np.rad2deg( servo_q[i])  } q_last_{i} = {  np.rad2deg( servoJ.last_q[i])  } ,overspeed {np.rad2deg(qd[i])},limit = {np.rad2deg(qd_limit ) }")
                    log_text.insert(  tk.END,  f"q_command_{i} = {  np.rad2deg( servo_q[i])  } q_last_{i} = {  np.rad2deg( servoJ.last_q[i])  } ,overspeed {np.rad2deg(qd[i])},limit = {np.rad2deg(qd_limit ) }\n")
                    log_text.see(tk.END)
                    root.destroy()
                    sys.exit(-1)

            q_lower_limit = robot.reduced_robot.model.lowerPositionLimit+ np.ones(len(robot.reduced_robot.model.lowerPositionLimit))*np.deg2rad(0)
            q_upper_limit = robot.reduced_robot.model.upperPositionLimit- np.ones(len(robot.reduced_robot.model.lowerPositionLimit))*np.deg2rad(0)
            q_upper_limit[5]=np.deg2rad(100)
            q_lower_limit[5]=np.deg2rad(-100)
            
            q_upper_limit[4]=np.deg2rad(180)
            q_lower_limit[4]=np.deg2rad(-180)
            
            
            command_q_14dof_pre = command_q_14dof.copy()
            qd_pre = qd.copy()
            qd_pre[0:7] = np.multiply(qd_pre[0:7] , real_robot_q_left_dir)
            qd_pre[7:14] = np.multiply(qd_pre[7:14] , real_robot_q_right_dir)
            for i in range(14):
                command_q_14dof_pre[i] = command_q_14dof_pre[i] 
            

            for i in range(14):
                if (command_q_14dof_pre[i]) > q_upper_limit[i]:
                    print(
                        Fore.RED + f"q_{i} = {  np.rad2deg( command_q_14dof[i])  } outside,upper limit = {np.rad2deg(q_upper_limit[i] ) }")
                    log_text.insert(  tk.END,    f"q_{i} = {  np.rad2deg( command_q_14dof[i])  } outside,limit = {np.rad2deg(q_upper_limit[i] ) }\n")
                    log_text.see(tk.END)
                    # root.destroy()
                    # sys.exit(-1)

            for i in range(14):
                if (command_q_14dof_pre[i]) <  q_lower_limit[i]:
                    print(
                        Fore.RED + f"q_{i} = {  np.rad2deg( command_q_14dof[i])  } outside,lower limit = {np.rad2deg(q_lower_limit[i] ) }")
                    log_text.insert(  tk.END,    f"q_{i} = {  np.rad2deg( command_q_14dof[i])  } outside,lower limit = {np.rad2deg(q_lower_limit[i] ) }\n")
                    log_text.see(tk.END)
                    # root.destroy()
                    # sys.exit(-1)

            servoJ.last_q = servo_q.copy()

            

            #限制最大关节速度
            for i in range(len(qd)):
                if abs(qd[i]) > np.deg2rad(60):
                    qd[i] = np.sign(qd[i]) *np.deg2rad(60)

            #灵巧手下发
            q_hand = np.array([1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000])
            q_hand[0:6] = command_hand_12dof[0:6]
            q_hand[6:12] = command_hand_12dof[6:12]
            q_hand = q_hand/1000*180
            q_hand = np.deg2rad(q_hand)
            
            #!指令下发
            real_robot.command_joint_state( servo_q[0:7],servo_q[7:14],q_hand[0:6],q_hand[6:12],(command_head_2dof), command_torsor_1dof )
            torso_angle = command_torsor_1dof
            
        else: #拖拽模式
            # 更新实时数据
            global pose_left
            global pose_right
            global joint_angles_left
            global joint_angles_right
            
            current_arm_state, current_hand_state, current_head_state, current_torsor_state = real_robot.get_joint_state(include_torso=True)
            
            joint_angles_left = np.multiply(current_arm_state[0:7]-real_robot_q_left_offset, real_robot_q_left_dir)
            joint_angles_right = np.multiply(current_arm_state[7:14]-real_robot_q_right_offset,real_robot_q_right_dir )
            torso_angle = current_torsor_state

            pin.forwardKinematics(robot.reduced_robot.model, robot.data,  np.append( joint_angles_left,joint_angles_right)  )
            frame_id = robot.reduced_robot.model.getJointId('left_wrist_yaw_joint')
            init_T  = robot.data.oMi[  frame_id]
            init_T  = SE3(  init_T .homogeneous)
            pose_left = init_T

            frame_id = robot.reduced_robot.model.getJointId('right_wrist_yaw_joint')
            init_T  = robot.data.oMi[  frame_id]
            init_T  = SE3(  init_T .homogeneous)
            pose_right = init_T


# 更新关节角度显示
def update_joint_labels(angles_left, angles_right,joint_angles_handle_left,joint_angles_handle_right):
    for i in range(angles_left.shape[0]):
        joint_labels_left[i].config(
            text=f"关节 {i + 1}: {np.rad2deg(angles_left[i]):.4f}°")

    for i in range(angles_right.shape[0]):
        joint_labels_right[i].config(
            text=f"关节 {i + 1}: {np.rad2deg(angles_right[i]):.4f}°")
        
    for i in range(joint_angles_handle_left.shape[0]):
        joint_labels_handle_left[i].config(
            text=f"关节 {i + 1}: {(joint_angles_handle_left[i])}°")

    for i in range(joint_angles_handle_right.shape[0]):
        joint_labels_handle_right[i].config(
            text=f"关节 {i + 1}: {(joint_angles_handle_right[i])}°")


# 更新末端位姿显示
def update_pose_label(pose_left: SE3(), pose_right: SE3()):
    
    pose_left_tool = pose_left*tool_left
    src_R_angle = pose_left_tool.rpy()
    temp1 = [f"{np.rad2deg(angle): .4f}" for angle in src_R_angle]
    temp2 = [f"{var: .4}" for var in pose_left_tool.A[0:3, 3]]
    pose_label_left[0].config(text=f"XYZ: { list(map(float, temp2)) }")
    pose_label_left[1].config(text=f"RxRyRz: {list(map(float, temp1))}")
    
    pose_right_tool = pose_right*tool_right
    src_R_angle = pose_right_tool.rpy()
    temp1 = [f"{np.rad2deg(angle): .4f}" for angle in src_R_angle]
    temp2 = [f"{var: .4}" for var in  pose_right_tool.A[0:3,3]    ]
    pose_label_right[0].config(text=f"XYZ: { list(map(float, temp2)) }")
    pose_label_right[1].config(text=f"RxRyRz: {list(map(float, temp1))}")

# 按住按钮触发关节点动,实际执行动作
def send_joint_jog(joint_idx, direction):
    # 获取滑动条速度值
    speed = float(speed_var.get())
    log_text.insert(
        tk.END, f"关节 {joint_idx + 1} 点动: {direction}，速度: {speed:.2f}\n")
    log_text.see(tk.END)

    if ruckig_head_runto.status == ruckig_head_runto.State.standill and ruckig_tcp_teach.status == ruckig_tcp_teach.State.standill and ruckig_tcp_runto.status == ruckig_tcp_runto.State.standill and ruckig_joint_runto.status == ruckig_joint_runto.State.standill:
        if direction == '+':
            ruckig_joint_teach.start(1, joint_idx, speed)
        else:
            ruckig_joint_teach.start(-1, joint_idx, speed)
    
    time.sleep(CYCLE*2)


# 按住按钮触发TCP点动,实际执行动作
def send_pose_jog(axis, direction):
    speed = float(speed_var.get())
    log_text.insert(tk.END, f"末端 {axis} 点动: {direction},速度: {speed:.2f}\n")
    log_text.see(tk.END)
    
    axis = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"].index(axis)
    
    if ruckig_head_runto.status == ruckig_head_runto.State.standill  and ruckig_joint_teach.status == ruckig_joint_teach.State.standill and ruckig_tcp_runto.status == ruckig_tcp_runto.State.standill and ruckig_joint_runto.status == ruckig_joint_runto.State.standill:
        if direction == '+':
            ruckig_tcp_teach.start(1, axis, speed)
        else:
            ruckig_tcp_teach.start(-1, axis, speed)
    
    time.sleep(CYCLE*2)


# TCP   Runto点动,实际动作
def send_pose_command_repeatedly():
    speed = float(speed_var.get())

    pose = [pose_entries[i].get() for i in range(6)]


    pose = list(map(float, pose))

    target_T = SE3(np.array(pose[0:3]) * 0.001)
    Rot = SO3.RPY(np.deg2rad(pose[3:6] ) )
    target_T.A[:3, :3] = Rot.R

    if ruckig_head_runto.status == ruckig_head_runto.State.standill  and ruckig_tcp_teach.status == ruckig_tcp_teach.State.standill and ruckig_joint_teach.status == ruckig_joint_teach.State.standill and ruckig_joint_runto.status == ruckig_joint_runto.State.standill:
        ruckig_tcp_runto.start(target_T, speed)
        log_text.insert(tk.END, f"发送位姿指令: {pose}, speed = {speed}\n")
        log_text.see(tk.END)
    else:
        log_text.insert(tk.END, f"其他运动在进行,拒绝tcp runto\n")
        log_text.see(tk.END)
    time.sleep(CYCLE*2)


# joint   Runto点动,实际动作
def send_joint_command_repeatedly():
    
    speed = float(speed_var.get())
    
    angles = [joint_entries[i].get() for i in range(7)]

    
    angles = np.deg2rad(list(map(float, angles)))
    
    if ruckig_head_runto.status == ruckig_head_runto.State.standill  and ruckig_tcp_teach.status == ruckig_tcp_teach.State.standill and ruckig_joint_teach.status == ruckig_joint_teach.State.standill and ruckig_tcp_runto.status == ruckig_tcp_runto.State.standill:
        ruckig_joint_runto.start(angles, speed)
        log_text.insert(tk.END, f"发送关节指令: {angles}, speed = {speed}\n")
        log_text.see(tk.END)
    else:
        log_text.insert(tk.END, f"其他运动在进行,拒绝关节runto\n")
        log_text.see(tk.END)

    time.sleep(CYCLE*2)


# head   Runto点动,实际动作
def send_head_command_repeatedly():
    
    speed = float(speed_var.get())
    
    angles = [head_entries[i].get() for i in range(2)]
    angles =np.deg2rad( (list(map(float, angles))) )
    
    if  ruckig_tcp_teach.status == ruckig_tcp_teach.State.standill and ruckig_joint_teach.status == ruckig_joint_teach.State.standill and ruckig_tcp_runto.status == ruckig_tcp_runto.State.standill and ruckig_joint_runto.status == ruckig_joint_runto.State.standill:
        ruckig_head_runto.start(angles, speed)
        log_text.insert(tk.END, f"发送头部指令: {angles}, speed = {speed}\n")
        log_text.see(tk.END)
    else:
        log_text.insert(tk.END, f"其他运动在进行,拒绝头部runto\n")
        log_text.see(tk.END)

    time.sleep(CYCLE*2)


# 按住关节点动按钮
def joint_jog_button(joint_idx, direction):
    def action():
        send_joint_jog(joint_idx, direction)
    return action


# 关节点动按住时持续触发
def joint_jog_press_and_hold(action, interval=10):
    def repeat():
        if pressing[action]:
            action()
            root.after(interval, repeat)
    pressing[action] = True
    repeat()
# 关节点动松开时的回调
def joint_jog_stop_repeating(action):
    pressing[action] = False
    print(Fore.GREEN + "停止关节点动")
    ruckig_joint_teach.end()


# 按住tcp点动按钮
def pose_jog_button(axis, direction):
    def action():
        send_pose_jog(axis, direction)
    return action

# tcp点动按住时持续触发
def pos_jog_press_and_hold(action, interval=10):
    def repeat():
        if pressing[action]:
            action()
            root.after(interval, repeat)
    pressing[action] = True
    repeat()

# tcp点动松开时的回调
def pos_jog_stop_repeating(action):
    pressing[action] = False
    print(Fore.GREEN + "停止tcp点动")
    ruckig_tcp_teach.end()


# 关节指令按住时持续触发
def joint_press_and_hold(action, interval=10):
    if is_command_running():
        return
    
    """按住按钮时持续调用 action 函数"""
    def repeat():
        if pressing_joint["send_joint"]:
            action()
            root.after(interval, repeat)
    pressing_joint["send_joint"] = True
    repeat()

def joint_stop_repeating(action):
    if is_command_running():
        return
    
    """松开按钮时停止调用 action 函数"""
    pressing_joint["send_joint"] = False
    print(Fore.GREEN + "停止关节runto")
    ruckig_joint_runto.end()


# TCP指令按住时持续触发
def tcp_press_and_hold(action, interval=10):
    if is_command_running():
        return
    
    """按住按钮时持续调用 action 函数"""
    def repeat():
        if pressing_pose["send_pose"]:
            action()
            root.after(interval, repeat)
    pressing_pose["send_pose"] = True
    repeat()

def tcp_stop_repeating(action):
    if is_command_running():
        return
    
    """松开按钮时停止调用 action 函数"""
    pressing_pose["send_pose"] = False
    print(Fore.GREEN + "停止 tcp runto")
    ruckig_tcp_runto.end()

    

# 头部指令按住时持续触发
def head_press_and_hold(action, interval=10):
    if is_command_running():
        return
    
    """按住按钮时持续调用 action 函数"""
    def repeat():
        if pressing_head["send_joint"]:
            action()
            root.after(interval, repeat)
    pressing_head["send_joint"] = True
    repeat()

def head_stop_repeating(action):
    if is_command_running():
        return
    
    """松开按钮时停止调用 action 函数"""
    pressing_head["send_joint"] = False
    print(Fore.GREEN + "停止头部runto")
    ruckig_head_runto.end()


# 滑块回调函数
def arm_type_update_state(val):

    if (
        ruckig_head_runto.status == ruckig_head_runto.State.standill
        and ruckig_joint_teach.status == ruckig_joint_teach.State.standill
        and ruckig_tcp_teach.status == ruckig_tcp_teach.State.standill
        and ruckig_tcp_runto.status == ruckig_tcp_runto.State.standill
        and ruckig_joint_runto.status == ruckig_joint_runto.State.standill
    ):
        arm_type_var.set(int(val))
        if int(val) == 1:
            status_label.config(text="状态: 右臂", foreground="green")
        else:
            status_label.config(text="状态: 左臂", foreground="red")

# tool工具设置
def send_tool_command():
    global tool_left
    global tool_right
    
    data = [tool_entries[i].get() for i in range(6)]
    log_text.insert(tk.END, f"TOOL refresh: {data}\n")
    log_text.see(tk.END)
    
    data = list(map(float, data))
    
    if int(arm_type_var.get()) == 0:
        tool_left = SE3(data[0:3])
        tool_left.A[:3,:3] = SO3.RPY(data[3],data[4],data[5],unit="deg")

    else:
        tool_right = SE3(data[0:3])
        tool_right.A[:3,:3] = SO3.RPY(data[3],data[4],data[5],unit="deg")
        
    time.sleep(CYCLE*2)


# 手设置
def send_hand_command_left():
    global joint_angles_handle_left
    
    data = [hand_entries_left[i].get() for i in range(6)]
    log_text.insert(tk.END, f"hand command: {data}\n")
    log_text.see(tk.END)
    print(Fore.GREEN + f"hand command: {data}\n")
    
    data = np.array(list(map(int, data)))

    joint_angles_handle_left = data.copy()
    
    time.sleep(CYCLE*2)

def send_hand_command_right():
    global joint_angles_handle_right
    
    data = [hand_entries_right[i].get() for i in range(6)]
    log_text.insert(tk.END, f"hand command: {data}\n")
    log_text.see(tk.END)
    print(Fore.GREEN + f"hand command: {data}\n")
    
    data = np.array(list(map(int, data)))

    joint_angles_handle_right = data.copy()
    
    time.sleep(CYCLE*2)



def expeort_command():
    
    entries_status={
        "joint_vars":[joint_vars[i].get() for i in range(7)],
        "pose_vars":[pose_vars[i].get() for i in range(6)],
        "tool_vars":[tool_vars[i].get() for i in range(6)],
        "hand_vars_left":[hand_vars_left[i].get() for i in range(6)],
        "hand_vars_right":[hand_vars_right[i].get() for i in range(6)],
          }
    with open(BASE_DIR+r'/rorobot_data_export.py', 'w', encoding='utf-8') as f:
        f.write(f"entries_status = {repr(entries_status)}\n")
    
    print("expeort_command entries_status",entries_status)

def import_command():

    import_data = {'array':np.array}
    with open(BASE_DIR+r'/rorobot_data_export.py', 'r', encoding='utf-8') as f:
        exec(f.read(), import_data)
    import_data = import_data['entries_status']
    
    temp = import_data['joint_vars'].copy()
    for i in range(7):
        joint_vars[i].set(temp[i])
    
    temp = import_data['pose_vars'].copy()
    for i in range(6):
        pose_vars[i].set(temp[i])
    
    temp = import_data['tool_vars'].copy()
    for i in range(6):
        tool_vars[i].set(temp[i])
    
    temp = import_data['hand_vars_left'].copy()
    for i in range(6):
        hand_vars_left[i].set(temp[i])
    
    temp = import_data['hand_vars_right'].copy()
    for i in range(6):
        hand_vars_right[i].set(temp[i])
        
    print("import_command import_data",import_data)

    

print_count = 0

# 标定打印
def print_tool():
    global tool_left
    global tool_right
    global pose_left
    global pose_right
    global print_count
    print_count +=1

    pose_left_tool = pose_left*tool_left
    src_R_angle = pose_left_tool.rpy()
    temp1 = [f"{np.rad2deg(angle): .4f}" for angle in src_R_angle]
    temp2 = [f"{var: .7}" for var in pose_left_tool.A[0:3, 3] * 1000]
    if int(arm_type_var.get()) == 0:
        print(f"left_XYZ: { list(map(float, temp2)) } mm" + f" RxRyRz: {list(map(float, temp1))} deg"+f",print_count={print_count}")

    pose_right_tool = pose_right*tool_right
    src_R_angle = pose_right_tool.rpy()
    temp1 = [f"{np.rad2deg(angle): .4f}" for angle in src_R_angle]
    temp2 = [f"{var: .7}" for var in pose_right_tool.A[0:3, 3] * 1000]
    if int(arm_type_var.get()) == 1:
        print(f"right_XYZ: { list(map(float, temp2)) } mm" + f" RxRyRz: {list(map(float, temp1))} deg"+f",print_count={print_count}")

    time.sleep(CYCLE*2)

def serial_test():
    global serial_test_flag
    serial_test_flag = not serial_test_flag


def serial_test_thread():
    global serial_test_flag
    thread_count = 0
    
    while True:
        thread_count+=1
        
        if(thread_count >10):
            break
        
        time.sleep(1)
        
        if(serial_test_flag):
        
            count = 0
            while count<250*10 and serial_test_flag:
                speed = float(speed_var.get())

                angles = ["0", "0", "0", "45", "0", "0", "0"]
                log_text.insert(tk.END, f"发送关节指令: {angles}, speed = {speed}\n")
                log_text.see(tk.END)

                angles = np.deg2rad(list(map(float, angles)))

                if ruckig_head_runto.status == ruckig_head_runto.State.standill and ruckig_tcp_teach.status == ruckig_tcp_teach.State.standill and ruckig_joint_teach.status == ruckig_joint_teach.State.standill and ruckig_tcp_runto.status == ruckig_tcp_runto.State.standill:
                    ruckig_joint_runto.start(angles, speed)

                time.sleep(CYCLE*4)
                count += 1
                print("count = ", count)
                
            
            
            while ruckig_joint_runto.status!=ruckig_joint_runto.State.standill and serial_test_flag:
                time.sleep(CYCLE*4)
                print("ruckig_joint_runto = ",ruckig_joint_runto.status)
                
            
            count = 0
            while count<250*10 and serial_test_flag:
                speed = float(speed_var.get())
                
                angles = ["0","0","0","-45","0","0","0"]
                log_text.insert(tk.END, f"发送关节指令: {angles}, speed = {speed}\n")
                log_text.see(tk.END)

                angles = np.deg2rad(list(map(float, angles)))

                if ruckig_head_runto.status == ruckig_head_runto.State.standill and ruckig_tcp_teach.status == ruckig_tcp_teach.State.standill and ruckig_joint_teach.status == ruckig_joint_teach.State.standill and ruckig_tcp_runto.status == ruckig_tcp_runto.State.standill:
                    ruckig_joint_runto.start(angles, speed)

                time.sleep(CYCLE*4)
                count +=1
                print("count = ",count)
                
                
            while ruckig_joint_runto.status!=ruckig_joint_runto.State.standill and serial_test_flag:
                time.sleep(CYCLE*4)
                print("ruckig_joint_runto = ",ruckig_joint_runto.status)


def moveJ_test():
    global movaJ_test_flag
    movaJ_test_flag = not movaJ_test_flag

def moveJ_thread():
    global movaJ_test_flag
    while True:
        time.sleep(0.1)
        if(movaJ_test_flag):
            send_joint_command_repeatedly()
            time.sleep(0.004)

def moveL_test():
    global movaL_test_flag
    movaL_test_flag = not movaL_test_flag

def torsor_control(torsor_angle_target):
    
    # sync_allJoint() #同步一次实际角度
    
    global joint_angles_left
    global joint_angles_right
    global joint_angles_handle_left
    global joint_angles_handle_right
    global head_angle
    global torso_angle

    torsor_angle_current = torso_angle

    torsor_angle_offset = torsor_angle_target - torsor_angle_current

    setp_total = int(6.0 / CYCLE)  # 3S内完成的步数

    for setp in range(setp_total+1):
        torso_angle = torsor_angle_current + torsor_angle_offset * setp / setp_total
        # 伺服下发
        servoJ(
            np.append(joint_angles_left, joint_angles_right).copy(),
            np.append(joint_angles_handle_left, joint_angles_handle_right).copy(),
            head_angle.copy(),
            torso_angle
        )
        time.sleep(CYCLE)


def moveL_thread():
    global movaL_test_flag
    while True:
        time.sleep(0.1)
        if(movaL_test_flag):
            send_pose_command_repeatedly()
            time.sleep(0.004)

# 定义一个信号处理函数
import signal
def handle_sigint(signum, frame):
    global serial_test_flag
    print("\n捕获到 Ctrl+C，程序将优雅地退出。")
    serial_test_flag =False
    root.destroy()
    sys.exit(0)


class CoordinateDisplayWindow:
    def __init__(self, parent):
        self.window = tk.Toplevel(parent)
        self.window.title("关节坐标显示")
        self.window.geometry("450x350")  # 优化窗口尺寸
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 创建主框架 - 使用网格布局
        main_frame = ttk.Frame(self.window)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 上部分：坐标显示区域
        coord_frame = ttk.Frame(main_frame)
        coord_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # 左臂坐标框架
        left_frame = ttk.LabelFrame(coord_frame, text="左臂关节坐标")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # 左臂坐标文本框 - 减小高度
        self.left_text = ScrolledText(left_frame, width=25, height=6)
        self.left_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 左臂复制按钮
        self.copy_left_btn = ttk.Button(left_frame, text="复制左臂坐标", 
                                       command=self.copy_left_coordinates)
        self.copy_left_btn.pack(pady=5)
        
        # 右臂坐标框架
        right_frame = ttk.LabelFrame(coord_frame, text="右臂关节坐标")
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        # 右臂坐标文本框 - 减小高度
        self.right_text = ScrolledText(right_frame, width=25, height=6)
        self.right_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 右臂复制按钮
        self.copy_right_btn = ttk.Button(right_frame, text="复制右臂坐标", 
                                        command=self.copy_right_coordinates)
        self.copy_right_btn.pack(pady=5)
        
        # 下部分：控制按钮区域 - 使用Frame包装
        control_frame = ttk.LabelFrame(main_frame, text="控制")
        control_frame.pack(fill=tk.X, pady=(10, 0))
        
        # 控制按钮容器
        button_container = ttk.Frame(control_frame)
        button_container.pack(fill=tk.X, padx=10, pady=10)
        
        # 第一行：主要功能按钮
        top_row = ttk.Frame(button_container)
        top_row.pack(fill=tk.X, pady=(0, 5))
        
        # 获取坐标按钮 - 放在显眼位置
        self.get_coord_btn = ttk.Button(top_row, text="获取当前坐标", 
                                       command=self.get_current_coordinates,
                                       width=15)
        self.get_coord_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        # 自动更新复选框
        self.auto_update_var = tk.BooleanVar()
        self.auto_update_cb = ttk.Checkbutton(top_row, text="自动更新", 
                                             variable=self.auto_update_var,
                                             command=self.toggle_auto_update)
        self.auto_update_cb.pack(side=tk.LEFT, padx=(0, 10))
        
        # 清空按钮
        self.clear_btn = ttk.Button(top_row, text="清空", 
                                   command=self.clear_text)
        self.clear_btn.pack(side=tk.LEFT)
        
        # 第二行：状态显示
        bottom_row = ttk.Frame(button_container)
        bottom_row.pack(fill=tk.X)
        
        # 状态标签
        self.status_label = ttk.Label(bottom_row, text="点击'获取当前坐标'按钮显示关节角度", 
                                     foreground="blue")
        self.status_label.pack(side=tk.LEFT)
        
        self.auto_update = False
        self.update_interval = 500  # 毫秒
        
    def get_current_coordinates(self):
        """获取当前左右臂的关节坐标 - 点击按钮时调用"""
        global joint_angles_left, joint_angles_right
        
        # 获取左臂坐标 - 转换为度并格式化为逗号分隔的字符串
        left_coords = [f"{np.rad2deg(angle):.4f}" for angle in joint_angles_left]
        left_coords_str = ",".join(left_coords)
        
        # 获取右臂坐标 - 转换为度并格式化为逗号分隔的字符串
        right_coords = [f"{np.rad2deg(angle):.4f}" for angle in joint_angles_right]
        right_coords_str = ",".join(right_coords)
        
        # 更新文本框
        self.left_text.delete(1.0, tk.END)
        self.left_text.insert(tk.END, left_coords_str)
        
        self.right_text.delete(1.0, tk.END)
        self.right_text.insert(tk.END, right_coords_str)
        
        # 更新状态标签
        self.status_label.config(text="坐标已更新 ✓", foreground="green")
        
        # 在日志中显示获取成功信息
        log_text.insert(tk.END, f"已获取当前关节坐标\n")
        log_text.see(tk.END)
        
    def copy_left_coordinates(self):
        """复制左臂坐标到剪贴板"""
        left_text = self.left_text.get(1.0, tk.END).strip()
        if left_text:
            self.window.clipboard_clear()
            self.window.clipboard_append(left_text)
            self.copy_left_btn.config(text="已复制!")
            self.window.after(1000, lambda: self.copy_left_btn.config(text="复制左臂坐标"))
            log_text.insert(tk.END, f"已复制左臂坐标到剪贴板\n")
            log_text.see(tk.END)
    
    def copy_right_coordinates(self):
        """复制右臂坐标到剪贴板"""
        right_text = self.right_text.get(1.0, tk.END).strip()
        if right_text:
            self.window.clipboard_clear()
            self.window.clipboard_append(right_text)
            self.copy_right_btn.config(text="已复制!")
            self.window.after(1000, lambda: self.copy_right_btn.config(text="复制右臂坐标"))
            log_text.insert(tk.END, f"已复制右臂坐标到剪贴板\n")
            log_text.see(tk.END)
    
    def toggle_auto_update(self):
        """切换自动更新状态"""
        self.auto_update = self.auto_update_var.get()
        if self.auto_update:
            log_text.insert(tk.END, "开启关节坐标自动更新\n")
            log_text.see(tk.END)
            self.status_label.config(text="自动更新中...", foreground="orange")
            self.auto_update_coordinates()
        else:
            log_text.insert(tk.END, "关闭关节坐标自动更新\n")
            log_text.see(tk.END)
            self.status_label.config(text="自动更新已关闭", foreground="blue")
    
    def auto_update_coordinates(self):
        """自动更新坐标"""
        if self.auto_update and self.window.winfo_exists():
            global joint_angles_left, joint_angles_right
        
            # 获取左臂坐标 - 转换为度并格式化为逗号分隔的字符串
            left_coords = [f"{np.rad2deg(angle):.4f}" for angle in joint_angles_left]
            left_coords_str = ",".join(left_coords)
            
            # 获取右臂坐标 - 转换为度并格式化为逗号分隔的字符串
            right_coords = [f"{np.rad2deg(angle):.4f}" for angle in joint_angles_right]
            right_coords_str = ",".join(right_coords)
            
            # 更新文本框
            self.left_text.delete(1.0, tk.END)
            self.left_text.insert(tk.END, left_coords_str)
            
            self.right_text.delete(1.0, tk.END)
            self.right_text.insert(tk.END, right_coords_str)
            
            self.window.after(self.update_interval, self.auto_update_coordinates)
    
    def clear_text(self):
        """清空文本框"""
        self.left_text.delete(1.0, tk.END)
        self.right_text.delete(1.0, tk.END)
        self.status_label.config(text="已清空坐标显示", foreground="red")
        log_text.insert(tk.END, "已清空坐标显示文本框\n")
        log_text.see(tk.END)
    
    def on_closing(self):
        """窗口关闭时的处理"""
        self.auto_update = False
        self.window.destroy()
        log_text.insert(tk.END, "坐标显示窗口已关闭\n")
        log_text.see(tk.END)


# 打开坐标显示窗口的函数
def open_coordinate_display():
    """打开坐标显示窗口"""
    global coord_window
    try:
        if hasattr(coord_window, 'window') and coord_window.window.winfo_exists():
            coord_window.window.lift()
        else:
            coord_window = CoordinateDisplayWindow(root)
    except:
        coord_window = CoordinateDisplayWindow(root)


class PointSaveWindow:
    """点位保存窗口"""
    def __init__(self, parent):
        self.window = tk.Toplevel(parent)
        self.window.title("点位保存管理")
        self.window.geometry("600x500")
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 主框架
        main_frame = ttk.Frame(self.window)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 上部分：点位列表显示
        list_frame = ttk.Frame(main_frame)
        list_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # 左手点位列表
        left_frame = ttk.LabelFrame(list_frame, text="左手点位")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        left_columns = ("点位名", "类型")
        self.left_point_tree = ttk.Treeview(left_frame, columns=left_columns, show="headings", height=10)
        for col in left_columns:
            self.left_point_tree.heading(col, text=col)
            self.left_point_tree.column(col, width=100 if col == "点位名" else 80)
        self.left_point_tree.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        left_scrollbar = ttk.Scrollbar(left_frame, orient=tk.VERTICAL, command=self.left_point_tree.yview)
        left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.left_point_tree.configure(yscrollcommand=left_scrollbar.set)
        self.left_point_tree.bind("<<TreeviewSelect>>", lambda e: self._on_tree_select("left"))
        
        # 右手点位列表
        right_frame = ttk.LabelFrame(list_frame, text="右手点位")
        right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))
        right_columns = ("点位名", "类型")
        self.right_point_tree = ttk.Treeview(right_frame, columns=right_columns, show="headings", height=10)
        for col in right_columns:
            self.right_point_tree.heading(col, text=col)
            self.right_point_tree.column(col, width=100 if col == "点位名" else 80)
        self.right_point_tree.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        right_scrollbar = ttk.Scrollbar(right_frame, orient=tk.VERTICAL, command=self.right_point_tree.yview)
        right_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.right_point_tree.configure(yscrollcommand=right_scrollbar.set)
        self.right_point_tree.bind("<<TreeviewSelect>>", lambda e: self._on_tree_select("right"))
        
        # 下部分：控制按钮
        control_frame = ttk.LabelFrame(main_frame, text="操作")
        control_frame.pack(fill=tk.X)
        
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # 点位名称输入
        name_frame = ttk.Frame(button_frame)
        name_frame.pack(fill=tk.X, pady=(0, 5))
        ttk.Label(name_frame, text="点位名称:").pack(side=tk.LEFT, padx=(0, 5))
        self.point_name_var = tk.StringVar(value="P1")
        name_entry = ttk.Entry(name_frame, textvariable=self.point_name_var, width=10)
        name_entry.pack(side=tk.LEFT, padx=(0, 10))
        
        # 坐标类型选择
        ttk.Label(name_frame, text="坐标类型:").pack(side=tk.LEFT, padx=(0, 5))
        self.coord_type_var = tk.StringVar(value="joint")
        joint_radio = ttk.Radiobutton(name_frame, text="关节角度", variable=self.coord_type_var, value="joint")
        joint_radio.pack(side=tk.LEFT, padx=(0, 5))
        pose_radio = ttk.Radiobutton(name_frame, text="笛卡尔坐标", variable=self.coord_type_var, value="pose")
        pose_radio.pack(side=tk.LEFT)
        
        # 按钮行
        btn_row = ttk.Frame(button_frame)
        btn_row.pack(fill=tk.X)
        
        # 保存点位按钮
        save_left_btn = ttk.Button(btn_row, text="保存左手当前点位",
                                   command=lambda: self.save_current_point("left"))
        save_left_btn.pack(side=tk.LEFT, padx=(0, 5))
        save_right_btn = ttk.Button(btn_row, text="保存右手当前点位",
                                    command=lambda: self.save_current_point("right"))
        save_right_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        # 删除点位按钮
        delete_btn = ttk.Button(btn_row, text="删除选中点位", command=self.delete_selected_point)
        delete_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        # 清空所有点位按钮
        clear_btn = ttk.Button(btn_row, text="清空所有点位", command=self.clear_all_points)
        clear_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        # 刷新列表按钮
        refresh_btn = ttk.Button(btn_row, text="刷新列表", command=self.refresh_list)
        refresh_btn.pack(side=tk.LEFT)
        
        # 运行点位设置
        run_frame = ttk.LabelFrame(button_frame, text="运行点位")
        run_frame.pack(fill=tk.X, padx=5, pady=(10, 0))
        
        run_btn = ttk.Button(run_frame, text="Run To 选中点位")
        run_btn.pack(anchor="w", padx=5, pady=(0, 5))
        run_btn.bind("<ButtonPress-1>", self.on_run_button_press)
        run_btn.bind("<ButtonRelease-1>", self.on_run_button_release)
        ttk.Label(run_frame, text="提示: 速度沿用主界面滑块设置").pack(anchor="w", padx=5, pady=(0, 5))
        
        # 状态标签
        self.status_label = ttk.Label(button_frame, text="准备就绪", foreground="blue")
        self.status_label.pack(pady=(5, 0))
        self.is_run_button_pressed = False
        self.current_run_type = None
        
        # 初始化时加载点位数据并刷新列表
        load_saved_points()
        self._normalize_all_points()
        self.refresh_list()
    
    def save_current_point(self, arm):
        """保存当前点位"""
        global saved_points, joint_angles_left, joint_angles_right, pose_left, pose_right
        
        point_name = self.point_name_var.get().strip()
        if not point_name:
            self.status_label.config(text="错误: 请输入点位名称", foreground="red")
            return
        
        coord_type = self.coord_type_var.get()
        arm_desc = "左手" if arm == "left" else "右手"
        
        try:
            point_entry = self._ensure_point_schema(point_name)
            if coord_type == "joint":
                data = joint_angles_left if arm == "left" else joint_angles_right
                point_entry[f"type_{arm}"] = "joint"
                point_entry[f"joint_{arm}"] = data.copy().tolist()
                point_entry[f"pose_{arm}"] = None
            else:
                pose = pose_left if arm == "left" else pose_right
                pos = pose.t.tolist()
                rpy = SO3(pose.R).rpy().tolist()
                pose_list = pos + rpy
                point_entry[f"type_{arm}"] = "pose"
                point_entry[f"joint_{arm}"] = None
                point_entry[f"pose_{arm}"] = pose_list
            
            if save_saved_points():
                self.status_label.config(text=f"{arm_desc}成功保存点位: {point_name} ({coord_type})", foreground="green")
                log_text.insert(tk.END, f"{arm_desc}已保存点位 {point_name} ({coord_type}) 到文件\n")
            else:
                self.status_label.config(text=f"{arm_desc}点位已保存但文件写入失败: {point_name}", foreground="orange")
                log_text.insert(tk.END, f"{arm_desc}点位 {point_name} 已保存到内存，但文件写入失败\n")
            log_text.see(tk.END)
            self.refresh_list()
            
        except Exception as e:
            self.status_label.config(text=f"{arm_desc}保存失败: {str(e)}", foreground="red")
            log_text.insert(tk.END, f"{arm_desc}保存点位失败: {str(e)}\n")
            log_text.see(tk.END)
    
    def delete_selected_point(self):
        """删除选中的点位"""
        global saved_points
        
        selections = self._collect_selected_points()
        if not selections:
            self.status_label.config(text="错误: 请先选择要删除的点位", foreground="red")
            return
        
        removed = 0
        for arm, point_name in selections:
            if point_name not in saved_points:
                continue
            entry = self._ensure_point_schema(point_name)
            entry[f"type_{arm}"] = None
            entry[f"joint_{arm}"] = None
            entry[f"pose_{arm}"] = None
            if not self._has_any_data(entry):
                del saved_points[point_name]
            removed += 1
        
        if removed == 0:
            self.status_label.config(text="没有可删除的点位数据", foreground="blue")
            return
        
        if save_saved_points():
            self.status_label.config(text=f"已删除 {removed} 项点位数据", foreground="green")
            log_text.insert(tk.END, f"已删除 {removed} 项点位数据并更新文件\n")
        else:
            self.status_label.config(text="点位已删除但文件更新失败", foreground="orange")
            log_text.insert(tk.END, "点位已从内存删除，但文件更新失败\n")
        log_text.see(tk.END)
        
        self.refresh_list()
    
    def clear_all_points(self):
        """清空所有点位"""
        global saved_points
        
        if len(saved_points) == 0:
            self.status_label.config(text="没有可清空的点位", foreground="blue")
            return
        
        saved_points.clear()
        # 保存到文件
        if save_saved_points():
            self.status_label.config(text="已清空所有点位", foreground="green")
            log_text.insert(tk.END, "已清空所有点位并更新文件\n")
        else:
            self.status_label.config(text="点位已清空但文件更新失败", foreground="orange")
            log_text.insert(tk.END, "点位已从内存清空，但文件更新失败\n")
        log_text.see(tk.END)
        self.refresh_list()
    
    def refresh_list(self):
        """刷新点位列表"""
        global saved_points
        
        for tree in (self.left_point_tree, self.right_point_tree):
            for item in tree.get_children():
                tree.delete(item)
        
        for point_name, point_data in saved_points.items():
            self._ensure_point_schema(point_name)
            left_type = self._resolve_arm_type(point_data, "left")
            right_type = self._resolve_arm_type(point_data, "right")
            
            if left_type:
                display = self._format_arm_display(point_data, "left", left_type)
                self.left_point_tree.insert("", tk.END, values=(point_name, display))
            if right_type:
                display = self._format_arm_display(point_data, "right", right_type)
                self.right_point_tree.insert("", tk.END, values=(point_name, display))
    
    def run_selected_point(self):
        """运行选中的点位"""
        global saved_points
        
        selections = self._collect_selected_points()
        if not selections:
            self.status_label.config(text="错误: 请先选择要运行的点位", foreground="red")
            return
        if len(selections) > 1:
            self.status_label.config(text="错误: 请选择单个点位运行", foreground="red")
            return
        
        if not self._is_motion_idle():
            self.status_label.config(text="错误: 当前有运动任务在执行", foreground="red")
            log_text.insert(tk.END, "运行点位失败: 运动任务未完成\n")
            log_text.see(tk.END)
            return
        
        arm, point_name = selections[0]
        point_data = saved_points.get(point_name)
        if not point_data:
            self.status_label.config(text=f"错误: 点位 {point_name} 不存在", foreground="red")
            return
        
        speed = self._get_speed_value()
        point_type = self._resolve_arm_type(point_data, arm)
        if not point_type:
            self.status_label.config(text=f"错误: 点位 {point_name} 未保存{ '左手' if arm == 'left' else '右手' }数据", foreground="red")
            return
        
        try:
            if point_type == "joint":
                success = self._run_joint_point(point_name, point_data, arm, speed)
            elif point_type == "pose":
                success = self._run_pose_point(point_name, point_data, arm, speed)
            else:
                self.status_label.config(text=f"错误: 点位类型未知: {point_type}", foreground="red")
                return
            
            if success:
                self.current_run_type = point_type
                arm_desc = "左臂" if arm == "left" else "右臂"
                self.status_label.config(text=f"{arm_desc} 正在运行点位 {point_name}", foreground="green")
        except Exception as e:
            self.status_label.config(text=f"运行失败: {str(e)}", foreground="red")
            log_text.insert(tk.END, f"运行点位失败: {str(e)}\n")
            log_text.see(tk.END)
    
    def on_run_button_press(self, event=None):
        """按下运行按钮时触发"""
        self.is_run_button_pressed = True
        self._run_button_loop()
    
    def on_run_button_release(self, event=None):
        """松开运行按钮时停止当前RunTo"""
        self.is_run_button_pressed = False
        self._stop_running_point()
    
    def _run_button_loop(self):
        if not self.is_run_button_pressed:
            return
        
        # 检查当前运行是否已结束，若结束则允许再次触发
        if self.current_run_type == "joint" and ruckig_joint_runto.status == ruckig_joint_runto.State.standill:
            self.current_run_type = None
        elif self.current_run_type == "pose" and ruckig_tcp_runto.status == ruckig_tcp_runto.State.standill:
            self.current_run_type = None
        
        if self.current_run_type is None:
            self.run_selected_point()
        root.after(30, self._run_button_loop)
    
    def _stop_running_point(self):
        if self.current_run_type == "joint":
            ruckig_joint_runto.end()
            log_text.insert(tk.END, "已停止关节点位运行\n")
        elif self.current_run_type == "pose":
            ruckig_tcp_runto.end()
            log_text.insert(tk.END, "已停止位姿点位运行\n")
        else:
            return
        
        self.current_run_type = None
        self.status_label.config(text="RunTo已停止", foreground="blue")
        log_text.see(tk.END)
    
    def _run_joint_point(self, point_name, point_data, arm, speed):
        global ruckig_joint_runto
        
        joint_key = "joint_left" if arm == "left" else "joint_right"
        joint_values = point_data.get(joint_key)
        if not joint_values:
            self.status_label.config(text=f"错误: 点位 {point_name} 未保存{ '左' if arm == 'left' else '右' }臂关节数据", foreground="red")
            return False
        
        if len(joint_values) != 7:
            self.status_label.config(text=f"错误: 点位 {point_name} 数据长度不正确", foreground="red")
            return False
        
        target_joint = np.array(joint_values, dtype=float)
        self._set_active_arm(arm)
        ruckig_joint_runto.start(target_joint, speed)
        log_text.insert(tk.END, f"运行关节点位 {point_name} ({arm})，速度 {speed:.2f}\n")
        log_text.see(tk.END)
        return True
    
    def _run_pose_point(self, point_name, point_data, arm, speed):
        global ruckig_tcp_runto
        
        pose_key = "pose_left" if arm == "left" else "pose_right"
        pose_values = point_data.get(pose_key)
        if not pose_values:
            self.status_label.config(text=f"错误: 点位 {point_name} 未保存{ '左' if arm == 'left' else '右' }臂位姿数据", foreground="red")
            return False
        
        if len(pose_values) != 6:
            self.status_label.config(text=f"错误: 点位 {point_name} 位姿数据长度不正确", foreground="red")
            return False
        
        position = np.array(pose_values[0:3], dtype=float)
        rpy = np.array(pose_values[3:6], dtype=float)
        target_T = SE3(position)
        target_T.A[:3, :3] = SO3.RPY(*rpy).R
        
        self._set_active_arm(arm)
        ruckig_tcp_runto.start(target_T, speed)
        log_text.insert(tk.END, f"运行位姿点位 {point_name} ({arm})，速度 {speed:.2f}\n")
        log_text.see(tk.END)
        return True
    
    def _is_motion_idle(self):
        try:
            return (
                ruckig_head_runto.status == ruckig_head_runto.State.standill
                and ruckig_joint_teach.status == ruckig_joint_teach.State.standill
                and ruckig_tcp_teach.status == ruckig_tcp_teach.State.standill
                and ruckig_tcp_runto.status == ruckig_tcp_runto.State.standill
                and ruckig_joint_runto.status == ruckig_joint_runto.State.standill
            )
        except NameError:
            return False
    
    def _get_speed_value(self):
        try:
            return max(0.01, float(speed_var.get()))
        except Exception:
            return 0.2
    
    def _set_active_arm(self, arm):
        arm_val = 0 if arm == "left" else 1
        try:
            arm_type_update_state(str(arm_val))
        except Exception:
            try:
                arm_type_var.set(arm_val)
            except Exception:
                pass
    
    def _ensure_point_schema(self, point_name):
        entry = saved_points.setdefault(point_name, {})
        entry.setdefault("joint_left", entry.get("joint_left"))
        entry.setdefault("joint_right", entry.get("joint_right"))
        entry.setdefault("pose_left", entry.get("pose_left"))
        entry.setdefault("pose_right", entry.get("pose_right"))
        
        if "type_left" not in entry or entry["type_left"] is None:
            if entry.get("joint_left"):
                entry["type_left"] = "joint"
            elif entry.get("pose_left"):
                entry["type_left"] = "pose"
            else:
                entry["type_left"] = entry.get("type")
        if "type_right" not in entry or entry["type_right"] is None:
            if entry.get("joint_right"):
                entry["type_right"] = "joint"
            elif entry.get("pose_right"):
                entry["type_right"] = "pose"
            else:
                entry["type_right"] = entry.get("type")
        return entry
    
    def _resolve_arm_type(self, point_data, arm):
        type_key = f"type_{arm}"
        joint_key = f"joint_{arm}"
        pose_key = f"pose_{arm}"
        arm_type = point_data.get(type_key)
        if arm_type:
            return arm_type
        if point_data.get(joint_key):
            return "joint"
        if point_data.get(pose_key):
            return "pose"
        return None
    
    def _format_arm_display(self, point_data, arm, arm_type):
        if arm_type == "joint":
            joint_data = point_data.get(f"joint_{arm}") or []
            return f"关节({len(joint_data)}项)"
        elif arm_type == "pose":
            return "位姿"
        return "未知"
    
    def _collect_selected_points(self):
        selections = []
        for arm, tree in (("left", self.left_point_tree), ("right", self.right_point_tree)):
            for item in tree.selection():
                values = tree.item(item)['values']
                if values:
                    selections.append((arm, values[0]))
        return selections
    
    def _has_any_data(self, entry):
        left_has = entry.get("joint_left") or entry.get("pose_left")
        right_has = entry.get("joint_right") or entry.get("pose_right")
        return bool(left_has or right_has)
    
    def _normalize_all_points(self):
        for point_name in list(saved_points.keys()):
            self._ensure_point_schema(point_name)

    def _on_tree_select(self, arm):
        """确保左右列表互斥选择，并同步点位名称"""
        if arm == "left":
            tree = self.left_point_tree
            other = self.right_point_tree
        else:
            tree = self.right_point_tree
            other = self.left_point_tree
        other.selection_remove(other.selection())
        
        selection = tree.selection()
        if selection:
            point_name = tree.item(selection[0])['values'][0]
            self.point_name_var.set(point_name)
    
    def on_closing(self):
        """窗口关闭时的处理"""
        self.window.destroy()
        log_text.insert(tk.END, "点位保存窗口已关闭\n")
        log_text.see(tk.END)


# 打开点位保存窗口的函数
point_save_window = None
def open_point_save_window():
    """打开点位保存窗口"""
    global point_save_window
    try:
        if point_save_window is not None and hasattr(point_save_window, 'window') and point_save_window.window.winfo_exists():
            point_save_window.window.lift()
        else:
            point_save_window = PointSaveWindow(root)
    except:
        point_save_window = PointSaveWindow(root)


if __name__ == "__main__":

    signal.signal(signal.SIGINT, handle_sigint)

    # 关节示教功能
    ruckig_joint_teach = class_ruckig_joint_teach(5, 30) #1rad/s
    # TCP示教功能
    ruckig_tcp_teach = class_ruckig_tcp_teach(1, 25) #1m/
    #   TCP Runto 功能
    ruckig_tcp_runto = class_ruckig_tcp_Runto(4, 20) #速度1m/s
    #   关节 Runto 功能
    ruckig_joint_runto = class_ruckig_joint_Runto(5, 40)  # 1rad/s
    #  头runto
    ruckig_head_runto = class_ruckig_head_Runto(2.5, 20)  # 1rad/s
    

    # 机械臂模型
    robot = robot_model()

    # 如果是实体机器，需要获取实际关节角度作为初始值
    if simulate==False:
        real_robot = robot_control_handle.UpperControl()
        current_arm_state, current_hand_state, current_head_state, torso_state = real_robot.get_joint_state(include_torso=True)
  
        real_robot_q_left_dir = [1,1,1,1,1,1,1]
        real_robot_q_right_dir = [1,1,1,1,1,1,1]
        real_robot_q_left_offset = np.deg2rad([ 0,0,0,0,0,0,0])
        real_robot_q_right_offset = np.deg2rad([ 0,0,0,0,0,0,0])
        
        joint_angles_left= current_arm_state[0:7].copy()
        joint_angles_right= current_arm_state[7:14].copy()
        joint_angles_left = np.multiply(joint_angles_left-real_robot_q_left_offset, real_robot_q_left_dir)
        joint_angles_right = np.multiply(joint_angles_right-real_robot_q_right_offset,real_robot_q_right_dir )
        
        joint_angles_handle_left = np.array(current_hand_state[0:6])/np.deg2rad(180)*1000
        joint_angles_handle_right = np.array(current_hand_state[6:12])/np.deg2rad(180)*1000
        
        head_angle = current_head_state.copy()
        torso_angle = torso_state
        
        
       

    # 更新实时数据
    pin.forwardKinematics(robot.reduced_robot.model, robot.data,  np.append( joint_angles_left,joint_angles_right)  )
    frame_id = robot.reduced_robot.model.getJointId('left_wrist_yaw_joint') 
    init_T  = robot.data.oMi[  frame_id]
    init_T  = SE3(  init_T .homogeneous)
    pose_left = init_T  

    frame_id = robot.reduced_robot.model.getJointId('right_wrist_yaw_joint') 
    init_T  = robot.data.oMi[  frame_id]
    init_T  = SE3(  init_T .homogeneous)
    pose_right = init_T  
    # ------------------------------------------

    # 启动前示例：将腰部旋转到 5°
    # torsor_control(np.deg2rad(0))

    # 主界面
    root = tk.Tk()
    root.title("7轴MIT操作上位机")
    root.geometry("950x1000")
    
    # 程序启动时自动加载已保存的点位数据
    load_saved_points()

    # 数据实时显示区域
    
    ##左臂数据实时显示
    data_frame = ttk.LabelFrame(root, text="数据实时显示")
    data_frame.pack(fill="x", padx=10, pady=2)

    data_frame_left = ttk.LabelFrame(data_frame, text="左臂数据实时显示")
    data_frame_left.pack(side="left", fill="x", padx=5, pady=5)

    joint_labels_left = [ttk.Label(
        data_frame_left, text=f"关节 {i + 1}: --°", font=("Arial", 10)) for i in range(7)]
    for lbl in joint_labels_left:
        lbl.pack(anchor="w", padx=10)

    pose_label_left = [ttk.Label(
        data_frame_left, text="XYZ: --, --, -- ", font=("Arial", 12)),
        ttk.Label(
        data_frame_left, text="RPY: --, --, --", font=("Arial", 12))
    ]
    for ito in pose_label_left:
        ito.pack(anchor="w", padx=10, pady=5)



    ##右手数据实时显示
    data_frame_handle_left = ttk.LabelFrame(data_frame, text="左手数据实时显示")
    data_frame_handle_left.pack(side="left", fill="x", padx=5, pady=5)

    joint_labels_handle_left = [ttk.Label(
        data_frame_handle_left, text=f"关节 {i + 1}: --°", font=("Arial", 10)) for i in range(6)]
    for lbl in joint_labels_handle_left:
        lbl.pack(anchor="w", padx=10)


    data_frame_handle_right = ttk.LabelFrame(data_frame, text="右手数据实时显示")
    data_frame_handle_right.pack(side="left", fill="x", padx=5, pady=5)

    joint_labels_handle_right = [ttk.Label(
        data_frame_handle_right, text=f"关节 {i + 1}: --°", font=("Arial", 10)) for i in range(6)]
    for lbl in joint_labels_handle_right:
        lbl.pack(anchor="w", padx=10)
        

    ##右臂数据实时显示
    data_frame_right = ttk.LabelFrame(data_frame, text="右臂数据实时显示")
    data_frame_right.pack(side="right", fill="x", padx=5, pady=6)

    joint_labels_right = [ttk.Label(
        data_frame_right, text=f"关节 {i + 1}: --°", font=("Arial", 10)) for i in range(7)]
    for lbl in joint_labels_right:
        lbl.pack(anchor="w", padx=10)

    pose_label_right = [ttk.Label(
    data_frame_right, text="XYZ: --, --, -- ", font=("Arial", 12)),
    ttk.Label(
    data_frame_right, text="RPY: --, --, --", font=("Arial", 12))
    ]
    for ito in pose_label_right:
        ito.pack(anchor="w", padx=10, pady=5)
    # ---------------------------------------------



    # 控制部分和日志区域容器
    control_and_log_frame = ttk.Frame(root)
    control_and_log_frame.pack(fill="both", expand=True, padx=10, pady=10)

    # 控制部分
    control_frame = ttk.LabelFrame(control_and_log_frame, text="控制部分")
    control_frame.pack(side="left", fill="both", expand=True, padx=5, pady=5)

    # 点动控制
    jog_frame = ttk.LabelFrame(control_frame, text="点动控制")
    jog_frame.pack(fill="x", padx=5, pady=5)

    # 存储按住状态
    pressing = {}

    # 关节点动
    for i in range(7):
        joint_frame = ttk.Frame(jog_frame)
        joint_frame.pack(anchor="w", pady=2)
        ttk.Label(joint_frame, text=f"关节 {i + 1}:").pack(side="left")
        for direction in ["+", "-"]:
            button = ttk.Button(joint_frame, text=direction)
            action = joint_jog_button(i, direction)
            button.bind("<ButtonPress>", lambda e,
                        action=action: joint_jog_press_and_hold(action))
            button.bind("<ButtonRelease>", lambda e,
                        action=action: joint_jog_stop_repeating(action))
            button.pack(side="left", padx=2)

    # TCP点动
    pose_jog_frame = ttk.Frame(jog_frame)
    pose_jog_frame.pack(anchor="w", pady=5)
    for i, axis in enumerate(["X", "Y", "Z", "Roll", "Pitch", "Yaw"]):
        axis_frame = ttk.Frame(pose_jog_frame)
        axis_frame.pack(anchor="w", pady=2)
        ttk.Label(axis_frame, text=f"{axis}:").pack(side="left")
        for direction in ["↑", "↓"]:
            button = ttk.Button(axis_frame, text=direction)
            action = pose_jog_button(axis, "+" if direction == "↑" else "-")
            button.bind("<ButtonPress>", lambda e,
                        action=action: pos_jog_press_and_hold(action))
            button.bind("<ButtonRelease>", lambda e,
                        action=action: pos_jog_stop_repeating(action))
            button.pack(side="left", padx=2)

    # 滑动条 - 速度调节
    speed_frame = ttk.LabelFrame(control_frame, text="速度调节")
    speed_frame.pack(fill="x", padx=5, pady=5)

    speed_var = tk.DoubleVar(value=0.1)  # 默认速度为 0.1°
    speed_scale = ttk.Scale(speed_frame, from_=0.01,
                            to=1.0, orient="horizontal", variable=speed_var)
    speed_label = ttk.Label(speed_frame, text="速度 (%):")
    speed_label.pack(side="left", padx=5)
    speed_scale.pack(fill="x", padx=10)

    # 滑块控件：切换左右臂
    switch_slider = tk.Scale(control_frame, from_=0, to=1, orient="horizontal",
                             length=200, tickinterval=1, command=arm_type_update_state)
    switch_slider.pack(fill="x", padx=5, pady=5)
    default_velue = 1
    switch_slider.set(default_velue)
    arm_type_var = tk.DoubleVar(value=default_velue)  

    # 切换左右臂,需要对应切换状态显示标签
    status_label = ttk.Label(control_frame, text="状态: 右臂", font=("Arial", 12),foreground="green")
    status_label.pack(pady=10)
    arm_type_update_state(default_velue)


    # 老化测试按钮
    test_button = ttk.Button(
        control_frame, text="老化测试", command=serial_test)
    test_button.pack(anchor="w", pady=5)

    # 直线运动测试按钮
    test_button = ttk.Button(
        control_frame, text="直线运动", command=moveL_test)
    test_button.pack(anchor="w", pady=5)
    
    # 关节运动测试按钮
    test_button = ttk.Button(
        control_frame, text="关节运动", command=moveJ_test)
    test_button.pack(anchor="w", pady=5)
    
    # 新增：坐标显示按钮
    coord_button = ttk.Button(control_frame, text="获取并复制坐标", command=open_coordinate_display)
    coord_button.pack(anchor="w", pady=5)
    
    # 新增：点位保存按钮
    point_save_button = ttk.Button(control_frame, text="点位保存管理", command=open_point_save_window)
    point_save_button.pack(anchor="w", pady=5)

    # 指令下发
    command_frame = ttk.LabelFrame(control_and_log_frame, text="指令下发")
    command_frame.pack(side="left", fill="both", expand=True, padx=5, pady=5)

    # 定义验证函数，确保输入的是有效的浮点数
    def validate_float_input(input_value):
        if input_value == "" or re.match(r"^-?\d*\.?\d*$", input_value):  # 允许空值（防止删除后清空）
            return True
        return False

    validate_cmd = root.register(validate_float_input)

    # 关节角度指令输入
    default_joint_values = ["0", "0", "0", "0", "0", "0", "0"] 
    joint_vars = [tk.StringVar(value=default_joint_values[i]) for i in range(7)]
    joint_entries=[]
    ttk.Label(command_frame, text="关节角度:[°]").pack(anchor="w")
    for i in range(7):
        entry = ttk.Entry(command_frame, width=8, textvariable=joint_vars[i],
                        validate="key", validatecommand=(validate_cmd, "%P"))
        entry.pack(anchor="w", padx=5, pady=2)
        joint_entries.append(entry)

    # 关节角度指令绑定回调
    pressing_joint = {"send_joint": False}
    send_joint_button = ttk.Button(command_frame, text="发送关节指令")
    send_joint_button.pack(anchor="w", pady=5)
    send_joint_button.bind("<ButtonPress>", lambda e: joint_press_and_hold(send_joint_command_repeatedly))
    send_joint_button.bind("<ButtonRelease>", lambda e: joint_stop_repeating(send_joint_command_repeatedly))

    # tcp位姿指令输入
    default_pose_values = ["466", "29", "0", "-153", "-17", "-84"] 
    pose_vars = [tk.StringVar(value=default_pose_values[i]) for i in range(6)]
    pose_entries = []
    ttk.Label(command_frame, text="XYZRPY 位姿 [mm/°]:").pack(anchor="w")
    for i in range(6):
        entry = ttk.Entry(command_frame, width=8, textvariable=pose_vars[i],
                        validate="key", validatecommand=(validate_cmd, "%P"))
        entry.pack(anchor="w", padx=5, pady=2)
        pose_entries.append(entry)

    # tcp位姿指令绑定回调
    pressing_pose = {"send_pose": False}
    send_pose_button = ttk.Button(command_frame, text="发送位姿指令")
    send_pose_button.pack(anchor="w", pady=5)
    send_pose_button.bind("<ButtonPress>", lambda e: tcp_press_and_hold(send_pose_command_repeatedly))
    send_pose_button.bind("<ButtonRelease>", lambda e: tcp_stop_repeating(send_pose_command_repeatedly))

    # 工具设置
    default_tool_values = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0"] 
    tool_vars = [tk.StringVar(value=default_tool_values[i]) for i in range(6)]
    tool_entries = []
    ttk.Label(command_frame, text="TOOL XYZRPY 位姿 [m/°]:").pack(anchor="w")
    for i in range(6):
        entry = ttk.Entry(command_frame, width=8, textvariable=tool_vars[i],
                        validate="key", validatecommand=(validate_cmd, "%P"))
        entry.pack(anchor="w", padx=5, pady=2)
        tool_entries.append(entry)

    # 工具按钮回调
    tool_button = ttk.Button(
        command_frame, text="tool", command=send_tool_command)
    tool_button.pack(anchor="w", pady=5)

    # 标定点打印
    caillr_button = ttk.Button(
        command_frame, text="标定打印", command=print_tool)
    caillr_button.pack(anchor="w", pady=5)
    
    
    #hand角度设置
    hand_frame = ttk.LabelFrame(control_and_log_frame, text="hand")
    hand_frame.pack(side="left", fill="both", expand=True, padx=5, pady=5)
    
    # 左手输入（使用文件开头定义的默认值）
    hand_vars_left = [tk.StringVar(value=default_hand_values_left[i]) for i in range(6)]
    hand_entries_left=[]
    ttk.Label(hand_frame, text="左手角度:[count]").pack(anchor="w")
    for i in range(6):
        entry = ttk.Entry(hand_frame, width=8, textvariable=hand_vars_left[i],
                        validate="key", validatecommand=(validate_cmd, "%P"))
        entry.pack(anchor="w", padx=5, pady=2)
        hand_entries_left.append(entry)

    # 左手数值回调
    hand_button_left = ttk.Button(
        hand_frame, text="hand command left", command=send_hand_command_left)
    hand_button_left.pack(anchor="w", pady=5)
    
    joint_angles_handle_left =np.array(list(map(int, default_hand_values_left)))
    
    # 右手输入（使用文件开头定义的默认值）
    hand_vars_right = [tk.StringVar(value=default_hand_values_right[i]) for i in range(6)]
    hand_entries_right=[]
    ttk.Label(hand_frame, text="右手角度:[count]").pack(anchor="w")
    for i in range(6):
        entry = ttk.Entry(hand_frame, width=8, textvariable=hand_vars_right[i],
                        validate="key", validatecommand=(validate_cmd, "%P"))
        entry.pack(anchor="w", padx=5, pady=2)
        hand_entries_right.append(entry)

    # 右手数值回调
    hand_button_right = ttk.Button(
        hand_frame, text="hand command right", command=send_hand_command_right)
    hand_button_right.pack(anchor="w", pady=5)
    joint_angles_handle_right =np.array(list(map(int, default_hand_values_right)))
    
    
    # 头部输入
    head_values = ["0", "30"] 
    head_vars = [tk.StringVar(value=head_values[i]) for i in range(2)]
    head_entries=[]
    ttk.Label(hand_frame, text="头部角度-左右-上下:[°]").pack(anchor="w")
    for i in range(2):
        entry = ttk.Entry(hand_frame, width=8, textvariable=head_vars[i],
                        validate="key", validatecommand=(validate_cmd, "%P"))
        entry.pack(anchor="w", padx=5, pady=2)
        head_entries.append(entry)

    # 头部指令绑定回调
    pressing_head = {"send_joint": False}
    head_button = ttk.Button(hand_frame, text="command")
    head_button.pack(anchor="w", pady=5)
    head_button.bind("<ButtonPress>", lambda e: head_press_and_hold(send_head_command_repeatedly))
    head_button.bind("<ButtonRelease>", lambda e: head_stop_repeating(send_head_command_repeatedly))
    
    #导入导出
    export_frame = ttk.LabelFrame(control_and_log_frame, text="hand")
    export_frame.pack(side="left", fill="both", expand=True, padx=5, pady=5)
    
    # 导出回调
    export_button = ttk.Button(
        export_frame, text="export", command=expeort_command)
    export_button.pack(anchor="w", pady=5)
    
    # 导如回调
    import_button = ttk.Button(
        export_frame, text="import", command=import_command)
    import_button.pack(anchor="w", pady=5)
    
    # 日志显示区域
    log_frame = ttk.LabelFrame(control_and_log_frame, text="日志")
    log_frame.pack(side="right", fill="both", expand=True, padx=5, pady=5)

    log_text = ScrolledText(log_frame, wrap="word")
    log_text.pack(fill="both", expand=True, padx=5, pady=5)



    #  mesh可视化
    # 干涉模式添加工具坐标
    robot.interface_model.addFrame(
        pin.Frame(
            "tool_left",
            robot.interface_model.getJointId("left_wrist_yaw_joint"),
            pin.SE3(
                tool_left.A[:3, :3], np.array(tool_left.A[0:3, 3]).T
            ),  # 这个参数不对应？
            pin.FrameType.OP_FRAME,
        )
    )
    robot.interface_model.addFrame(
        pin.Frame(
            "tool_right",
            robot.interface_model.getJointId("right_wrist_yaw_joint"),
            pin.SE3(
                tool_right.A[:3, :3], np.array(tool_right.A[0:3, 3]).T
            ),  # 这个参数不对应？
            pin.FrameType.OP_FRAME,
        )
    )
    # 干涉模式重新生成
    robot.interface_data = robot.interface_model.createData()
    robot.interface_geom_data = pin.GeometryData(robot.interface_geom_model)
    
    # mesh启动
    # viz = MeshcatVisualizer(robot.interface_model, robot.interface_geom_model, robot.interface_geom_model)
    # viz.initViewer(loadModel=True,zmq_url="tcp://127.0.0.1:6000")
    # viz.loadViewerModel()
    # viz.displayVisuals(True )
    # viz.displayCollisions( False )

    # # mesh要求显示对应坐标系
    # viz.displayFrames(
    #     1, [robot.interface_model.getFrameId("right_wrist_yaw_joint"),robot.interface_model.getFrameId("left_wrist_yaw_joint"),robot.interface_model.getFrameId("torso_link"),robot.interface_model.getFrameId("tool_left"),robot.interface_model.getFrameId("tool_right")]
    # )
    # viz.display(pin.neutral(robot.interface_model))

    # --------------------------------------
    
    # 启动数据生成线程
    data_thread = Thread(target=generate_data, args=(
        update_joint_labels, update_pose_label), daemon=True)
    data_thread.start()
    
    #老化任务开启/movL/movJ
    motion_moveJ_tread=  Thread(target=moveJ_thread)
    motion_moveJ_tread.start()
    motion_moveL_tread=  Thread(target=moveL_thread)
    motion_moveL_tread.start()
    # --------------------------------------
    

    print("mainloop")
    root.mainloop()
