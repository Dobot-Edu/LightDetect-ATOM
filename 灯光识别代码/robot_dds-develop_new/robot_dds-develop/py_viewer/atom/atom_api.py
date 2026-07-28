# -*- coding: utf-8 -*-
"""
ATOM机器人运动控制API模块
封装了机械臂运动控制相关的类和方法，可在其他程序中调用
"""

from enum import Enum
import math
from spatialmath import SE3, SO3
import numpy as np
import time
import copy
from colorama import Fore
from ruckig import InputParameter, OutputParameter, Result, Ruckig, ControlInterface
import pinocchio as pin
import sys


class Arm_type_strucrt(Enum):
    """机械臂类型枚举"""
    left_arm = "left_arm"
    right_arm = "right_arm"


def _to_scalar(x):
    """将可能为 numpy 数组的值安全转为 Python float，避免 'only 0-dimensional arrays can be converted to Python scalars'。"""
    return float(np.asarray(x).ravel()[0])


def _angvec2r(angle: float, v1: float, v2: float, v3: float):
    """从 angle-axis 计算 3x3 旋转矩阵，仅用 Python math 标量，避免 spatialmath.angvec2r 内 math.sin(θ) 对 numpy 数组报错。"""
    # Rodrigues: R = I + sin(θ)*K + (1-cos(θ))*K^2，K 为轴 v 的反对称矩阵
    c, s = math.cos(angle), math.sin(angle)
    K = np.array([[0, -v3, v2], [v3, 0, -v1], [-v2, v1, 0]], dtype=np.float64)
    R = np.eye(3) + s * K + (1.0 - c) * (K @ K)
    return R


def _rotation_matrix_to_angle_axis(R):
    """从 3x3 旋转矩阵计算 angle-axis，返回 (angle, axis) 均为 Python 标量/list，避免 spatialmath.angvec 的 scalar 转换错误。"""
    R = np.asarray(R)
    trace = float(R[0, 0]) + float(R[1, 1]) + float(R[2, 2])
    angle = float(np.arccos(np.clip((trace - 1) / 2.0, -1.0, 1.0)))
    if angle < 1e-10:
        return 0.0, [0.0, 0.0, 1.0]
    ax = float(R[2, 1]) - float(R[1, 2])
    ay = float(R[0, 2]) - float(R[2, 0])
    az = float(R[1, 0]) - float(R[0, 1])
    s = float(2.0 * np.sin(angle))
    return angle, [float(ax / s), float(ay / s), float(az / s)]


class class_ruckig_tcp_Runto:
    """TCP末端位置运动控制类"""

    class State(Enum):
        working = "working"
        end = "end"
        error = "error"
        standill = "standill"

    def __init__(self, vel, acc, cycle, controller):
        self.vel = vel
        self.acc = acc
        self.cycle = cycle
        self.controller = controller  # 引用RobotMotionController
        self.status = class_ruckig_tcp_Runto.State.standill

        self.otg = Ruckig(1, cycle)  # DoFs, control cycle
        self.inp = InputParameter(1)
        self.out = OutputParameter(1)
        self.inp.control_interface = ControlInterface.Position

        self.inp.current_position = [0.0]
        self.inp.current_velocity = [0.0]
        self.inp.current_acceleration = [0.0]

        self.inp.target_position = [1.0]
        self.inp.target_velocity = [0.0]
        self.inp.target_acceleration = [0.0]

        self.inp.max_velocity = np.ones(1) * self.vel
        self.inp.max_acceleration = np.ones(1) * self.acc * 0.5
        self.inp.max_jerk = np.ones(1) * self.acc * 20 * 0.5
        self.titck = 15

        self.init_T = SE3()
        self.otg_res = 0
        self.arm_type = 0

    def start(self, target_T, scale, arm_type: Arm_type_strucrt):
        if self.status == class_ruckig_tcp_Runto.State.standill:
            self.status = class_ruckig_tcp_Runto.State.working

            self.inp.current_position = [0.0]
            self.inp.current_velocity = [0.0]
            self.inp.current_acceleration = [0.0]

            self.inp.target_position = [1.0]
            self.inp.target_velocity = [0.0]
            self.inp.target_acceleration = [0.0]

            self.titck = 15
            self.arm_type = arm_type

            if self.arm_type == Arm_type_strucrt.left_arm:
                self.init_T = self.controller.pose_left.copy()
            else:
                self.init_T = self.controller.pose_right.copy()

            if self.arm_type == Arm_type_strucrt.left_arm:
                self.target_T = target_T * self.controller.tool_left.inv()  # 转flange系下
            else:
                self.target_T = target_T * self.controller.tool_right.inv()  # 转flange系下

            self.inp.control_interface = ControlInterface.Position

            self.detal_T = copy.deepcopy(self.init_T.inv() * self.target_T)
            A = np.array([[float(self.detal_T.A[i, j]) for j in range(4)] for i in range(4)])
            self.detal_T = SE3(A)
            # 展平为一维 (3,)，避免 A[:3,3] 为 (3,1) 时 detal_xyz[i] 成数组导致 float() 报错
            self.detal_xyz = np.asarray(self.detal_T.A[:3, 3]).flatten()
            self.detal_R_angle, vec_list = _rotation_matrix_to_angle_axis(self.detal_T.A[:3, :3])
            self.detal_R_vec = np.array(vec_list, dtype=float)

            T_max_length = float(np.max(
                [np.linalg.norm(self.detal_xyz), self.detal_R_angle * 0.5]
            ))
            if T_max_length < 1e-5:
                T_max_length = 1e-3

            self.inp.max_velocity = [float(self.vel / T_max_length * scale)]
            self.inp.max_acceleration = [float(self.acc / T_max_length * scale * 5)]
            self.inp.max_jerk = [float(self.acc / T_max_length * 35)]

        elif self.status == class_ruckig_tcp_Runto.State.working:
            # 在working状态下，如果目标或scale发生变化，需要重新计算速度参数
            # 检查目标是否发生变化
            if self.arm_type == Arm_type_strucrt.left_arm:
                new_target_T = target_T * self.controller.tool_left.inv()
            else:
                new_target_T = target_T * self.controller.tool_right.inv()
            
            # 如果目标发生变化，重新计算速度参数
            if not hasattr(self, 'target_T') or not np.allclose(self.target_T.A, new_target_T.A, atol=1e-6):
                self.target_T = new_target_T
                self.detal_T = copy.deepcopy(self.init_T.inv() * self.target_T)
                A = np.array([[float(self.detal_T.A[i, j]) for j in range(4)] for i in range(4)])
                self.detal_T = SE3(A)
                self.detal_xyz = np.asarray(self.detal_T.A[:3, 3]).flatten()
                self.detal_R_angle, vec_list = _rotation_matrix_to_angle_axis(self.detal_T.A[:3, :3])
                self.detal_R_vec = np.array(vec_list, dtype=float)

                T_max_length = float(np.max(
                    [np.linalg.norm(self.detal_xyz), self.detal_R_angle * 0.5]
                ))
                if T_max_length < 1e-5:
                    T_max_length = 1e-3
                
                # 重新计算速度参数，应用新的scale
                self.inp.max_velocity = [float(self.vel / T_max_length * scale)]
                self.inp.max_acceleration = [float(self.acc / T_max_length * scale * 5)]
                self.inp.max_jerk = [float(self.acc / T_max_length * 35)]
            else:
                # 即使目标没变，如果scale变化，也需要更新速度参数
                # 使用当前的T_max_length重新计算
                if hasattr(self, 'detal_xyz') and hasattr(self, 'detal_R_angle'):
                    T_max_length = float(np.max(
                        [np.linalg.norm(self.detal_xyz), self.detal_R_angle * 0.5]
                    ))
                    if T_max_length < 1e-5:
                        T_max_length = 1e-3
                    self.inp.max_velocity = [float(self.vel / T_max_length * scale)]
                    self.inp.max_acceleration = [float(self.acc / T_max_length * scale * 5)]
            
            self.titck = 15

    def end(self):
        if self.status != class_ruckig_tcp_Runto.State.standill:
            self.status = class_ruckig_tcp_Runto.State.end
            self.inp.control_interface = ControlInterface.Velocity

            self.inp.target_velocity = [0]
            self.inp.target_acceleration = [0.0]

            self.inp.max_acceleration = np.array(self.inp.max_acceleration) * 10
            self.inp.max_jerk = np.array(self.inp.max_jerk) * 60

    def update(self):
        self.otg_res = self.otg.update(self.inp, self.out)

        if self.status == class_ruckig_tcp_Runto.State.working:
            self.titck = self.titck - 1

            if self.titck == 0:
                self.end()
                res = self.otg.update(self.inp, self.out)

        if (
            self.status == class_ruckig_tcp_Runto.State.end
            and self.otg_res == Result.Finished
        ):
            self.status = class_ruckig_tcp_Runto.State.standill

        self.out.pass_to_input(self.inp)

        # 生成轨迹（统一用 _to_scalar 转为 Python 标量，避免 spatialmath/NumPy 报 scalar 转换错误）
        t = _to_scalar(self.out.new_position[0])
        x1, x2, x3 = _to_scalar(self.detal_xyz[0]) * t, _to_scalar(self.detal_xyz[1]) * t, _to_scalar(self.detal_xyz[2]) * t
        detal_T = SE3([x1, x2, x3])
        angle = _to_scalar(self.detal_R_angle * t)
        v1, v2, v3 = _to_scalar(self.detal_R_vec[0]), _to_scalar(self.detal_R_vec[1]), _to_scalar(self.detal_R_vec[2])
        # 用自实现 _angvec2r 替代 SO3.AngVec，避免 spatialmath 内 math.sin(θ) 对 numpy 标量报错
        detal_T.A[:3, :3] = _angvec2r(angle, v1, v2, v3)

        if self.arm_type == Arm_type_strucrt.left_arm:
            self.controller.pose_left = self.init_T * detal_T  # T_B_flange
        else:
            self.controller.pose_right = self.init_T * detal_T  # T_B_flange

        # 逐元素转为 Python float，避免 SE3.A 中 numpy 标量导致下游 "only 0-dimensional arrays" 报错
        pose_left_arr = np.array(
            [[_to_scalar(self.controller.pose_left.A[i, j]) for j in range(4)] for i in range(4)],
            dtype=np.float64,
        )
        pose_right_arr = np.array(
            [[_to_scalar(self.controller.pose_right.A[i, j]) for j in range(4)] for i in range(4)],
            dtype=np.float64,
        )
        ik_res = self.controller.robot.ik_fun(
            pose_left_arr,
            pose_right_arr,
            np.append(self.controller.joint_angles_left, self.controller.joint_angles_right).copy(),
        )

        if type(ik_res) != type(np.ndarray([])):
            ik_res = ik_res[0]

        if self.arm_type == Arm_type_strucrt.left_arm:
            self.controller.joint_angles_left = np.array(ik_res[0:7])
        else:
            self.controller.joint_angles_right = np.array(ik_res[7:14])

        # 伺服下发（torso_angle 转为标量，避免 only 0-dim arrays 报错）
        self.controller.servoJ(
            np.append(self.controller.joint_angles_left, self.controller.joint_angles_right).copy(),
            np.append(self.controller.joint_angles_handle_left, self.controller.joint_angles_handle_right).copy(),
            self.controller.head_angle.copy(),
            _to_scalar(self.controller.torso_angle),
        )


class class_ruckig_joint_Runto:
    """关节空间运动控制类"""

    class State(Enum):
        working = "working"
        end = "end"
        error = "error"
        standill = "standill"

    def __init__(self, vel, acc, cycle, controller):
        self.vel = vel
        self.acc = acc
        self.cycle = cycle
        self.controller = controller
        self.status = class_ruckig_joint_Runto.State.standill

        self.otg = Ruckig(7, cycle)  # DoFs, control cycle
        self.inp = InputParameter(7)
        self.out = OutputParameter(7)
        self.inp.control_interface = ControlInterface.Position

        self.inp.current_position = np.zeros(7)
        self.inp.current_velocity = np.zeros(7)
        self.inp.current_acceleration = np.zeros(7)

        self.inp.target_position = np.zeros(7)
        self.inp.target_velocity = np.zeros(7)
        self.inp.target_acceleration = np.zeros(7)

        self.inp.max_velocity = np.ones(7) * self.vel
        self.inp.max_acceleration = np.ones(7) * self.acc
        self.inp.max_jerk = np.ones(7) * self.acc * 40
        self.titck = 25
        self.arm_type = 0
        self.otg_res = 0

    def start(self, target_joint, scale, arm_type: Arm_type_strucrt):
        if self.status == class_ruckig_joint_Runto.State.standill:
            self.status = class_ruckig_joint_Runto.State.working

            self.arm_type = arm_type

            if self.arm_type == Arm_type_strucrt.left_arm:
                self.inp.current_position = self.controller.joint_angles_left
            else:
                self.inp.current_position = self.controller.joint_angles_right

            self.inp.current_velocity = np.zeros(7)
            self.inp.current_acceleration = np.zeros(7)

            self.inp.target_position = target_joint
            self.inp.target_velocity = np.zeros(7)
            self.inp.target_acceleration = np.zeros(7)

            self.titck = 25
            self.inp.control_interface = ControlInterface.Position

            self.inp.max_velocity = np.ones(7) * self.vel * scale
            self.inp.max_acceleration = np.ones(7) * self.acc * scale * 5
            self.inp.max_jerk = np.ones(7) * self.acc * 20

        elif self.status == class_ruckig_joint_Runto.State.working:
            # 在working状态下，如果目标或scale发生变化，需要重新计算速度参数
            # 检查目标是否发生变化
            if not np.allclose(self.inp.target_position, target_joint, atol=1e-6):
                # 目标发生变化，更新目标并重新计算速度参数
                self.inp.target_position = target_joint
                self.inp.max_velocity = np.ones(7) * self.vel * scale
                self.inp.max_acceleration = np.ones(7) * self.acc * scale * 5
            else:
                # 即使目标没变，如果scale变化，也需要更新速度参数
                self.inp.max_velocity = np.ones(7) * self.vel * scale
                self.inp.max_acceleration = np.ones(7) * self.acc * scale * 5
            
            self.titck = 25

    def end(self):
        if self.status != class_ruckig_joint_Runto.State.standill:
            self.status = class_ruckig_joint_Runto.State.end
            self.inp.control_interface = ControlInterface.Velocity

            self.inp.target_velocity = np.zeros(7)
            self.inp.target_acceleration = np.zeros(7)

            self.inp.max_acceleration = np.array(self.inp.max_acceleration) * 1.4
            self.inp.max_jerk = np.array(self.inp.max_jerk) * 1.4

    def update(self):
        self.otg_res = self.otg.update(self.inp, self.out)

        if self.status == class_ruckig_joint_Runto.State.working:
            self.titck = self.titck - 1

            if self.titck == 0:
                self.end()
                res = self.otg.update(self.inp, self.out)

        if (
            self.status == class_ruckig_joint_Runto.State.end
            and self.otg_res == Result.Finished
        ):
            self.status = class_ruckig_joint_Runto.State.standill

        self.out.pass_to_input(self.inp)

        if self.arm_type == Arm_type_strucrt.left_arm:
            self.controller.joint_angles_left = np.array(self.out.new_position)
        else:
            self.controller.joint_angles_right = np.array(self.out.new_position)

        # 更新实时数据
        pin.forwardKinematics(
            self.controller.robot.reduced_robot.model,
            self.controller.robot.data,
            np.append(self.controller.joint_angles_left, self.controller.joint_angles_right),
        )
        frame_id = self.controller.robot.reduced_robot.model.getJointId("left_wrist_yaw_joint")
        init_T = self.controller.robot.data.oMi[frame_id]
        init_T = SE3(init_T.homogeneous)
        self.controller.pose_left = init_T.copy()

        frame_id = self.controller.robot.reduced_robot.model.getJointId("right_wrist_yaw_joint")
        init_T = self.controller.robot.data.oMi[frame_id]
        init_T = SE3(init_T.homogeneous)
        self.controller.pose_right = init_T.copy()

        # 伺服下发
        self.controller.servoJ(
            np.append(self.controller.joint_angles_left, self.controller.joint_angles_right).copy(),
            np.append(self.controller.joint_angles_handle_left, self.controller.joint_angles_handle_right).copy(),
            self.controller.head_angle.copy(),
            self.controller.torso_angle
        )


class class_ruckig_joint_cp:
    """关节空间连续路径运动控制类"""

    class State(Enum):
        working = "working"
        end = "end"
        error = "error"
        standill = "standill"

    def __init__(self, cycle, controller):
        self.cycle = cycle
        self.controller = controller
        self.status = class_ruckig_joint_cp.State.standill

        self.otg = Ruckig(7, cycle)  # DoFs, control cycle
        self.titck = 25
        self.traj = []
        self.current_traj_index = 0
        self.arm_type = 0
        self.otg_res = 0

    def start(self, planning_info, sacle, arm_type: Arm_type_strucrt):
        if self.status == class_ruckig_joint_cp.State.standill:
            self.status = class_ruckig_joint_cp.State.working

            self.traj = []
            self.titck = 25
            self.current_traj_index = 0
            self.arm_type = arm_type

            current_joint = 0
            if self.arm_type == Arm_type_strucrt.left_arm:
                current_joint = self.controller.joint_angles_left
            else:
                current_joint = self.controller.joint_angles_right

            if (len(planning_info) == 0):
                print(Fore.RED + "class_ruckig_joint_cp:点位数量为0!")

            elif (len(planning_info) == 1):
                plan_info = {
                    "current": current_joint,
                    "targrt": planning_info[0]["targrt"],
                    "vel": planning_info[0]["vel"] * sacle,
                    "acc": planning_info[0]["acc"],
                    "jerk": planning_info[0]["acc"] * 20,
                    "CP": 0,
                    "time_all": 0,
                    "time_cp": 0,
                    "inp": InputParameter(7),
                    "out": OutputParameter(7),
                    "count_cycle": 0,
                }

                plan_info["inp"].current_position = plan_info["current"]
                plan_info["inp"].current_velocity = np.zeros(7)
                plan_info["inp"].current_acceleration = np.zeros(7)

                plan_info["inp"].target_position = plan_info["targrt"]
                plan_info["inp"].target_velocity = np.zeros(7)
                plan_info["inp"].target_acceleration = np.zeros(7)

                plan_info["inp"].control_interface = ControlInterface.Position
                plan_info["inp"].max_velocity = np.ones(7) * plan_info["vel"]
                plan_info["inp"].max_acceleration = np.ones(7) * plan_info["acc"]
                plan_info["inp"].max_jerk = np.ones(7) * plan_info["jerk"]
                self.otg.update(plan_info["inp"], plan_info["out"])
                plan_info["time_all"] = plan_info["out"].trajectory.duration
                plan_info["time_cp"] = plan_info["time_all"]

                self.traj.append(plan_info)

            else:
                for index in range(1, len(planning_info)):

                    if (index == 1):
                        plan_info_1_current = current_joint
                    else:
                        plan_info_1_current = planning_info[index-2]["targrt"]

                    plan_info_1 = {
                        "current": plan_info_1_current,
                        "targrt": planning_info[index-1]["targrt"],
                        "vel": planning_info[index-1]["vel"] * sacle,
                        "acc": planning_info[index-1]["acc"],
                        "jerk": planning_info[index-1]["acc"] * 20,
                        "CP": min(abs(planning_info[index - 1]["CP"]), 50 * 0.01),
                        "time_all": 0,
                        "time_cp": 0,
                        "inp": InputParameter(7),
                        "out": OutputParameter(7),
                        "count_cycle": 0,
                    }

                    plan_info_1["inp"].current_position = plan_info_1["current"]
                    plan_info_1["inp"].current_velocity = np.zeros(7)
                    plan_info_1["inp"].current_acceleration = np.zeros(7)

                    plan_info_1["inp"].target_position = plan_info_1["targrt"]
                    plan_info_1["inp"].target_velocity = np.zeros(7)
                    plan_info_1["inp"].target_acceleration = np.zeros(7)

                    plan_info_1["inp"].control_interface = ControlInterface.Position
                    plan_info_1["inp"].max_velocity = np.ones(7) * plan_info_1["vel"]
                    plan_info_1["inp"].max_acceleration = np.ones(7) * plan_info_1["acc"]
                    plan_info_1["inp"].max_jerk = np.ones(7) * plan_info_1["jerk"]
                    self.otg.update(plan_info_1["inp"], plan_info_1["out"])
                    plan_info_1["time_all"] = plan_info_1["out"].trajectory.duration
                    plan_info_1["time_cp"] = plan_info_1["time_all"]

                    plan_info_2 = {
                        "current": planning_info[index-1]["targrt"],
                        "targrt": planning_info[index]["targrt"],
                        "vel": planning_info[index]["vel"] * sacle,
                        "acc": planning_info[index]["acc"],
                        "jerk": planning_info[index]["acc"] * 20,
                        "CP": min(abs(planning_info[index]["CP"]), 50 * 0.01),
                        "time_all": 0,
                        "time_cp": 0,
                        "inp": InputParameter(7),
                        "out": OutputParameter(7),
                        "count_cycle": 0,
                    }

                    plan_info_2["inp"].current_position = plan_info_2["current"]
                    plan_info_2["inp"].current_velocity = np.zeros(7)
                    plan_info_2["inp"].current_acceleration = np.zeros(7)

                    plan_info_2["inp"].target_position = plan_info_2["targrt"]
                    plan_info_2["inp"].target_velocity = np.zeros(7)
                    plan_info_2["inp"].target_acceleration = np.zeros(7)

                    plan_info_2["inp"].control_interface = ControlInterface.Position
                    plan_info_2["inp"].max_velocity = np.ones(7) * plan_info_2["vel"]
                    plan_info_2["inp"].max_acceleration = np.ones(7) * plan_info_2["acc"]
                    plan_info_2["inp"].max_jerk = np.ones(7) * plan_info_2["jerk"]
                    self.otg.update(plan_info_2["inp"], plan_info_2["out"])
                    plan_info_2["time_all"] = plan_info_2["out"].trajectory.duration
                    plan_info_2["time_cp"] = plan_info_2["time_all"]

                    plan_info_1["time_cp"] = (
                        plan_info_1["time_all"]
                        - min(plan_info_1["time_all"], plan_info_2["time_all"])
                        * plan_info_1["CP"])

                    self.traj.append(plan_info_1)

                self.traj.append(plan_info_2)

        elif self.status == class_ruckig_joint_cp.State.working:
            self.titck = 25

    def end(self):
        if self.status != class_ruckig_joint_cp.State.standill:
            self.status = class_ruckig_joint_cp.State.end

        if self.current_traj_index < len(self.traj):

            if (self.traj[self.current_traj_index]["count_cycle"] < self.traj[self.current_traj_index]["time_cp"]):
                self.traj[self.current_traj_index]["inp"].control_interface = ControlInterface.Velocity
                self.traj[self.current_traj_index]["inp"].target_velocity = np.zeros(7)
                self.traj[self.current_traj_index]["inp"].target_acceleration = (np.zeros(7))
                self.traj[self.current_traj_index]["inp"].max_acceleration = (np.array(self.traj[self.current_traj_index]["inp"].max_acceleration) * 1.4)
                self.traj[self.current_traj_index]["inp"].max_jerk = (np.array(self.traj[self.current_traj_index]["inp"].max_jerk) * 1.4)

            elif (self.traj[self.current_traj_index]["count_cycle"] < self.traj[self.current_traj_index]["time_all"]):
                self.traj[self.current_traj_index]["inp"].control_interface = ControlInterface.Velocity
                self.traj[self.current_traj_index]["inp"].target_velocity = np.zeros(7)
                self.traj[self.current_traj_index]["inp"].target_acceleration = (np.zeros(7))
                self.traj[self.current_traj_index]["inp"].max_acceleration = (np.array(self.traj[self.current_traj_index]["inp"].max_acceleration) * 1.4)
                self.traj[self.current_traj_index]["inp"].max_jerk = (np.array(self.traj[self.current_traj_index]["inp"].max_jerk) * 1.4)
                self.traj[self.current_traj_index + 1]["inp"].control_interface = ControlInterface.Velocity
                self.traj[self.current_traj_index + 1]["inp"].target_velocity = (np.zeros(7))
                self.traj[self.current_traj_index + 1]["inp"].target_acceleration = (np.zeros(7))
                self.traj[self.current_traj_index + 1]["inp"].max_acceleration = (np.array(self.traj[self.current_traj_index + 1]["inp"].max_acceleration) * 1.4)
                self.traj[self.current_traj_index + 1]["inp"].max_jerk = (np.array(self.traj[self.current_traj_index + 1]["inp"].max_jerk) * 1.4)

    def update(self, needServoCommand=True):
        command_joint = np.zeros(7)
        if self.arm_type == Arm_type_strucrt.left_arm:
            command_joint = self.controller.joint_angles_left.copy()
        else:
            command_joint = self.controller.joint_angles_right.copy()

        self.otg_res = Result.Working

        while True:
            if self.current_traj_index < len(self.traj):
                plan_unit = self.traj[self.current_traj_index]

                if plan_unit["count_cycle"] <= plan_unit["time_cp"]:
                    self.otg_res = self.otg.update(plan_unit["inp"], plan_unit["out"])
                    command_joint = np.array(plan_unit["out"].new_position)
                    plan_unit["out"].pass_to_input(self.traj[self.current_traj_index]["inp"])
                    self.traj[self.current_traj_index]["count_cycle"] = (plan_unit["count_cycle"] + self.cycle)
                    break

                elif plan_unit["count_cycle"] <= plan_unit["time_all"]:
                    res1 = self.otg.update(plan_unit["inp"], plan_unit["out"])
                    plan_unit_next = self.traj[self.current_traj_index + 1]
                    res2 = self.otg.update(plan_unit_next["inp"], plan_unit_next["out"])

                    command_joint = (np.array(plan_unit["out"].new_position) + np.array(plan_unit_next["out"].new_position) - plan_unit_next["current"])

                    if res1 == Result.Finished and res2 == Result.Finished:
                        self.otg_res = Result.Finished
                    else:
                        self.otg_res = Result.Working

                    plan_unit["out"].pass_to_input(self.traj[self.current_traj_index]["inp"])
                    plan_unit_next["out"].pass_to_input(self.traj[self.current_traj_index + 1]["inp"])
                    self.traj[self.current_traj_index]["count_cycle"] = (plan_unit["count_cycle"] + self.cycle)
                    self.traj[self.current_traj_index + 1]["count_cycle"] = (plan_unit_next["count_cycle"] + self.cycle)
                    break

                else:
                    self.current_traj_index = self.current_traj_index + 1
            else:
                self.otg_res = Result.Finished
                break

        # 轨迹全部执行完判断
        if (self.otg_res == Result.Finished):
            if self.current_traj_index >= (len(self.traj) - 1):
                self.otg_res = Result.Finished
            else:
                self.otg_res = Result.Working

        if self.status == class_ruckig_joint_cp.State.working:
            self.titck = self.titck - 1

            if self.titck == 0:
                self.end()

        if self.status == class_ruckig_joint_cp.State.end and self.otg_res == Result.Finished:
            self.status = class_ruckig_joint_cp.State.standill

        if self.arm_type == Arm_type_strucrt.left_arm:
            self.controller.joint_angles_left = command_joint
        else:
            self.controller.joint_angles_right = command_joint

        if needServoCommand == True:
            # 更新实时数据
            pin.forwardKinematics(
                self.controller.robot.reduced_robot.model,
                self.controller.robot.data,
                np.append(self.controller.joint_angles_left, self.controller.joint_angles_right),
            )
            frame_id = self.controller.robot.reduced_robot.model.getJointId("left_wrist_yaw_joint")
            init_T = self.controller.robot.data.oMi[frame_id]
            init_T = SE3(init_T.homogeneous)
            self.controller.pose_left = init_T

            frame_id = self.controller.robot.reduced_robot.model.getJointId("right_wrist_yaw_joint")
            init_T = self.controller.robot.data.oMi[frame_id]
            init_T = SE3(init_T.homogeneous)
            self.controller.pose_right = init_T

        # 伺服下发
        self.controller.servoJ(
            np.append(self.controller.joint_angles_left, self.controller.joint_angles_right).copy(),
            np.append(self.controller.joint_angles_handle_left, self.controller.joint_angles_handle_right).copy(),
            self.controller.head_angle.copy(),
            self.controller.torso_angle
        )


class RobotMotionController:
    """
    机器人运动控制器
    封装了所有运动控制相关的功能，可在其他程序中调用
    """

    def __init__(self, robot, real_robot=None, 
                 tcp_vel=0.7, tcp_acc=20, 
                 joint_vel=1.4, joint_acc=20,
                 cycle=0.01,
                 simulate=False, isdrag=False, isVisual=False,
                 tool_left=None, tool_right=None,
                 real_robot_q_left_dir=None, real_robot_q_right_dir=None,
                 real_robot_q_left_offset=None, real_robot_q_right_offset=None,
                 viz=None):
        """
        初始化机器人运动控制器
        
        Args:
            robot: 机器人模型实例 (Arm_IK)
            real_robot: 真实机器人控制实例 (可选)
            tcp_vel: TCP末端速度 (默认0.7)
            tcp_acc: TCP末端加速度 (默认20)
            joint_vel: 关节速度 (默认1.4)
            joint_acc: 关节加速度 (默认20)
            cycle: 控制周期 (默认0.01)
            simulate: 是否仿真模式 (默认False)
            isdrag: 是否拖拽模式 (默认False)
            isVisual: 是否可视化 (默认False)
            tool_left: 左手工具变换 (SE3, 可选)
            tool_right: 右手工具变换 (SE3, 可选)
            real_robot_q_left_dir: 左手关节方向 (可选)
            real_robot_q_right_dir: 右手关节方向 (可选)
            real_robot_q_left_offset: 左手关节偏移 (可选)
            real_robot_q_right_offset: 右手关节偏移 (可选)
            viz: 可视化器实例 (可选)
        """
        # 机器人模型
        self.robot = robot
        self.real_robot = real_robot

        # 控制参数
        self.tcp_vel = tcp_vel
        self.tcp_acc = tcp_acc
        self.joint_vel = joint_vel
        self.joint_acc = joint_acc
        self.CYCLE = cycle

        # 模式标志
        self.simulate = simulate
        self.isdrag = isdrag
        self.isVisual = isVisual
        self.viz = viz

        # 工具变换
        if tool_left is None:
            self.tool_left = SE3()
        else:
            self.tool_left = tool_left

        if tool_right is None:
            self.tool_right = SE3(0.21995386, 0.05015792, 0.03143192)
            self.tool_right.A[:3, :3] = SO3.RPY(0, 90, -90, unit='deg').R
        else:
            self.tool_right = tool_right

        # 真实机器人参数
        if real_robot_q_left_dir is None:
            self.real_robot_q_left_dir = np.array([1, 1, 1, 1, 1, 1, 1])
        else:
            self.real_robot_q_left_dir = np.array(real_robot_q_left_dir)

        if real_robot_q_right_dir is None:
            self.real_robot_q_right_dir = np.array([1, 1, 1, 1, 1, 1, 1])
        else:
            self.real_robot_q_right_dir = np.array(real_robot_q_right_dir)

        if real_robot_q_left_offset is None:
            self.real_robot_q_left_offset = np.deg2rad([0, 0, 0, 0, 0, 0, 0])
        else:
            self.real_robot_q_left_offset = np.array(real_robot_q_left_offset)

        if real_robot_q_right_offset is None:
            self.real_robot_q_right_offset = np.deg2rad([0, 0, 0, 0, 0, 0, 0])
        else:
            self.real_robot_q_right_offset = np.array(real_robot_q_right_offset)

        # 状态变量
        self.joint_angles_left = np.zeros(7)
        self.joint_angles_right = np.zeros(7)
        self.joint_angles_handle_left = np.zeros(6)
        self.joint_angles_handle_right = np.zeros(6)
        self.head_angle = np.zeros(2)
        self.torso_angle = 0.0
        self.pose_left = SE3()
        self.pose_right = SE3()

        # servoJ的last_q缓存
        self._servoJ_last_q = None

    def movL(self, pose, sacle, arm_type: Arm_type_strucrt):
        """TCP末端直线运动"""
        # 转为 Python 原生 float，避免 numpy 标量导致 spatialmath 报错 "only 0-dimensional arrays can be converted to Python scalars"
        pose = [float(x) for x in np.asarray(pose).flatten()]
        if len(pose) != 6:
            print(Fore.RED + f"点位数量不为6! {pose}")
            return False

        target_T = SE3([pose[0], pose[1], pose[2]])
        Rot = SO3.RPY(pose[3], pose[4], pose[5])
        target_T.A[:3, :3] = Rot.R

        print(Fore.BLUE + f"开始runto tcp {pose},arm type {arm_type}")

        ruckig_tcp_runto = class_ruckig_tcp_Runto(self.tcp_vel, self.tcp_acc, self.CYCLE, self)

        count = 0
        while 1:
            ruckig_tcp_runto.start(target_T, sacle, arm_type)
            ruckig_tcp_runto.update()
            time.sleep(self.CYCLE)

            count = count + 1
            if count == 1000:
                print(Fore.BLUE + "正在执行runto tcp运动")
                count = 0

            if ruckig_tcp_runto.otg_res == Result.Finished:
                print(Fore.GREEN + "runto tcp运动执行完成")
                break
        return True

    def movJ(self, targetJoint, sacle, arm_type: Arm_type_strucrt):
        """关节空间运动"""
        if len(targetJoint) != 7:
            print(Fore.RED + f"点位数量不为7! {np.rad2deg(targetJoint)}")
            return False

        print(Fore.BLUE + f"开始runto joint {np.rad2deg(targetJoint)},arm type {arm_type}")

        count = 0
        ruckig_joint_runto = class_ruckig_joint_Runto(self.joint_vel, self.joint_acc, self.CYCLE, self)
        while 1:
            ruckig_joint_runto.start(targetJoint, sacle, arm_type)
            ruckig_joint_runto.update()
            time.sleep(self.CYCLE)

            count = count + 1
            if count == 1000:
                print(Fore.BLUE + "正在执行runto joint运动")
                count = 0

            if ruckig_joint_runto.otg_res == Result.Finished:
                print(Fore.GREEN + "runto joint运动执行完成")
                break
        return True

    def movJ_CP(self, planning_info, sacle, arm_type: Arm_type_strucrt):
        """关节空间连续路径运动"""
        if len(planning_info) == 0:
            print(Fore.RED + f"点位数量0! {planning_info}")
            return False

        print(Fore.BLUE + f"开始movJ_CP ,arm type {arm_type}")

        count = 0
        joint_cp = class_ruckig_joint_cp(self.CYCLE, self)
        while 1:
            joint_cp.start(planning_info, sacle, arm_type)
            joint_cp.update()
            time.sleep(self.CYCLE)

            count = count + 1
            if count == 1000:
                print(Fore.BLUE + "正在执行movJ_CP运动")
                count = 0

            if joint_cp.otg_res == Result.Finished:
                print(Fore.GREEN + "movJ_CP运动执行完成")
                break
        return True

    def TwoArm_movJ_CP(self, planning_info_left, planning_info_right, sacle):
        """双臂协调连续路径运动"""
        if len(planning_info_left) == 0:
            print(Fore.RED + f"左手点位数量0! {planning_info_left}")
            return False

        if len(planning_info_right) == 0:
            print(Fore.RED + f"右手点位数量0! {planning_info_right}")
            return False

        print(Fore.BLUE + f"TwoArm_movJ_CP")

        count = 0
        joint_cp_left = class_ruckig_joint_cp(self.CYCLE, self)
        joint_cp_right = class_ruckig_joint_cp(self.CYCLE, self)
        while 1:
            joint_cp_left.start(planning_info_left, sacle, Arm_type_strucrt.left_arm)
            joint_cp_left.update(needServoCommand=False)
            joint_cp_right.start(planning_info_right, sacle, Arm_type_strucrt.right_arm)
            joint_cp_right.update(needServoCommand=False)

            # 更新实时数据
            pin.forwardKinematics(
                self.robot.reduced_robot.model,
                self.robot.data,
                np.append(self.joint_angles_left, self.joint_angles_right),
            )
            frame_id = self.robot.reduced_robot.model.getJointId("left_wrist_yaw_joint")
            init_T = self.robot.data.oMi[frame_id]
            init_T = SE3(init_T.homogeneous)
            self.pose_left = init_T

            frame_id = self.robot.reduced_robot.model.getJointId("right_wrist_yaw_joint")
            init_T = self.robot.data.oMi[frame_id]
            init_T = SE3(init_T.homogeneous)
            self.pose_right = init_T

            # 伺服下发
            self.servoJ(
                np.append(self.joint_angles_left, self.joint_angles_right).copy(),
                np.append(self.joint_angles_handle_left, self.joint_angles_handle_right).copy(),
                self.head_angle.copy(),
                self.torso_angle
            )

            time.sleep(self.CYCLE)

            count = count + 1
            if count == 1000:
                print(Fore.BLUE + "正在执行TwoArm_movJ_CP运动")
                count = 0

            if joint_cp_left.otg_res == Result.Finished and joint_cp_right.otg_res == Result.Finished:
                print(Fore.GREEN + "TwoArm_movJ_CP运动执行完成")
                break
        return True

    def hand_control(self, hand_angle_target, arm_type: Arm_type_strucrt):
        """灵巧手控制"""
        if arm_type == Arm_type_strucrt.left_arm:
            hand_angle_current = self.joint_angles_handle_left
        else:
            hand_angle_current = self.joint_angles_handle_right

        hand_angle_offset = hand_angle_target - hand_angle_current

        setp_total = int(0.5 / self.CYCLE)

        for setp in range(setp_total):
            hand_angle_command = hand_angle_current + hand_angle_offset * setp / setp_total

            if arm_type == Arm_type_strucrt.left_arm:
                self.joint_angles_handle_left = hand_angle_command
            else:
                self.joint_angles_handle_right = hand_angle_command

            self.servoJ(
                np.append(self.joint_angles_left, self.joint_angles_right).copy(),
                np.append(self.joint_angles_handle_left, self.joint_angles_handle_right).copy(),
                self.head_angle.copy(),
                self.torso_angle
            )
            time.sleep(self.CYCLE)

    def head_control(self, head_angle_target, duration: float = 2.0):
        """头部控制

        Args:
            head_angle_target: 目标头部角度
            duration: 期望完成时间（秒），默认 2s，可根据需要加速/减速
        """
        head_angle_current = self.head_angle

        head_angle_offset = head_angle_target - head_angle_current

        duration = max(float(duration), self.CYCLE)
        setp_total = max(1, int(duration / self.CYCLE))

        for setp in range(setp_total + 1):
            ratio = setp / setp_total
            self.head_angle = head_angle_current + head_angle_offset * ratio
            self.servoJ(
                np.append(self.joint_angles_left, self.joint_angles_right).copy(),
                np.append(self.joint_angles_handle_left, self.joint_angles_handle_right).copy(),
                self.head_angle.copy(),
                self.torso_angle
            )
            time.sleep(self.CYCLE)

    def torsor_control(self, torsor_angle_target, duration: float = 6.0):
        """腰部控制

        Args:
            torsor_angle_target: 目标腰部角度（弧度）
            duration: 期望完成时间（秒），默认 6s，可根据需要加速/减速
        """
        torsor_angle_current = self.torso_angle
        torsor_angle_offset = torsor_angle_target - torsor_angle_current

        duration = max(float(duration), self.CYCLE)
        setp_total = max(1, int(duration / self.CYCLE))

        for setp in range(setp_total + 1):
            ratio = setp / setp_total
            self.torso_angle = torsor_angle_current + torsor_angle_offset * ratio
            self.servoJ(
                np.append(self.joint_angles_left, self.joint_angles_right).copy(),
                np.append(self.joint_angles_handle_left, self.joint_angles_handle_right).copy(),
                self.head_angle.copy(),
                self.torso_angle
            )
            time.sleep(self.CYCLE)

    def sync_allJoint(self):
        """同步实际关节角度"""
        if self.simulate == False and self.real_robot is not None:
            (
                current_arm_state,
                current_hand_state,
                current_head_state,
                current_torsor_state,
            ) = self.real_robot.get_joint_state(include_torso=True)

            self.joint_angles_left = current_arm_state[0:7].copy()
            self.joint_angles_right = current_arm_state[7:14].copy()
            self.joint_angles_left = np.multiply(
                self.joint_angles_left - self.real_robot_q_left_offset, 
                self.real_robot_q_left_dir
            )
            
            self.joint_angles_right = np.multiply(
                self.joint_angles_right - self.real_robot_q_right_offset, 
                self.real_robot_q_right_dir
            )

            self.joint_angles_handle_left = np.array(current_hand_state[0:6]) / np.deg2rad(180) * 1000
            self.joint_angles_handle_right = np.array(current_hand_state[6:12]) / np.deg2rad(180) * 1000

            self.head_angle = current_head_state.copy()
            self.torso_angle = current_torsor_state

        # 更新实时数据
        pin.forwardKinematics(
            self.robot.reduced_robot.model,
            self.robot.data,
            np.append(self.joint_angles_left, self.joint_angles_right),
        )
        frame_id = self.robot.reduced_robot.model.getJointId("left_wrist_yaw_joint")
        init_T = self.robot.data.oMi[frame_id]
        init_T = SE3(init_T.homogeneous)
        self.pose_left = init_T

        frame_id = self.robot.reduced_robot.model.getJointId("right_wrist_yaw_joint")
        init_T = self.robot.data.oMi[frame_id]
        init_T = SE3(init_T.homogeneous)
        self.pose_right = init_T

    def servoJ(self, command_q_14dof: np.array, command_hand_12dof: np.array, 
               command_head_2dof: np.array, command_torsor_1dof: float, 
               qd_limit=np.deg2rad(150000)):
        """伺服下发关节角度"""
        current_torsor_state = self.torso_angle
        if self.simulate == True:
            if self._servoJ_last_q is None:
                self._servoJ_last_q = command_q_14dof.copy()

            servo_q = command_q_14dof.copy()
            qd = (servo_q - self._servoJ_last_q) / self.CYCLE

            q_lower_limit = self.robot.reduced_robot.model.lowerPositionLimit + np.ones(
                len(self.robot.reduced_robot.model.lowerPositionLimit)
            ) * np.deg2rad(10)
            q_upper_limit = self.robot.reduced_robot.model.upperPositionLimit - np.ones(
                len(self.robot.reduced_robot.model.lowerPositionLimit)
            ) * np.deg2rad(10)

            command_q_14dof_pre = command_q_14dof.copy()
            qd_pre = qd.copy()
            for i in range(14):
                command_q_14dof_pre[i] = command_q_14dof_pre[i] + qd_pre[i] * 0.008

            self._servoJ_last_q = servo_q.copy()

            if self.isVisual == True and self.viz is not None:
                self.viz.display(servo_q)

        else:
            if not self.isdrag:  # 不是拖拽模式
                if self._servoJ_last_q is None:
                    if self.real_robot is not None:
                        (
                            current_arm_state,
                            current_hand_state,
                            current_head_state,
                            current_torsor_state,
                        ) = self.real_robot.get_joint_state(include_torso=True)
                        self._servoJ_last_q = current_arm_state.copy()
                        self.torso_angle = current_torsor_state

                if self.real_robot is not None:
                    (
                        current_arm_state,
                        current_hand_state,
                        current_head_state,
                        current_torsor_state,
                    ) = self.real_robot.get_joint_state(include_torso=True)

                    servo_q = current_arm_state.copy()
                    servo_q[0:7] = command_q_14dof[0:7]  # left_arm_joint
                    servo_q[0:7] = (
                        np.multiply(servo_q[0:7], self.real_robot_q_left_dir)
                        + self.real_robot_q_left_offset
                    )

                    servo_q[7:14] = command_q_14dof[7:14]  # right_arm_joint
                    servo_q[7:14] = (
                        np.multiply(servo_q[7:14], self.real_robot_q_right_dir)
                        + self.real_robot_q_right_offset
                    )

                    qd = (servo_q - self._servoJ_last_q) / self.CYCLE

                    for i in range(14):
                        if abs(qd[i]) > qd_limit:
                            print(
                                Fore.RED
                                + f"q_command_{i} = {np.rad2deg(servo_q[i])} q_last_{i} = {np.rad2deg(self._servoJ_last_q[i])}, overspeed {np.rad2deg(qd[i])}, limit = {np.rad2deg(qd_limit)}"
                            )
                            sys.exit(-1)

                    q_lower_limit = self.robot.reduced_robot.model.lowerPositionLimit + np.ones(
                        len(self.robot.reduced_robot.model.lowerPositionLimit)
                    ) * np.deg2rad(0)
                    q_upper_limit = self.robot.reduced_robot.model.upperPositionLimit - np.ones(
                        len(self.robot.reduced_robot.model.lowerPositionLimit)
                    ) * np.deg2rad(0)
                    q_upper_limit[5] = np.deg2rad(100)
                    q_lower_limit[5] = np.deg2rad(-100)

                    q_upper_limit[4] = np.deg2rad(180)
                    q_lower_limit[4] = np.deg2rad(-180)

                    command_q_14dof_pre = command_q_14dof.copy()
                    qd_pre = qd.copy()
                    qd_pre[0:7] = np.multiply(qd_pre[0:7], self.real_robot_q_left_dir)
                    qd_pre[7:14] = np.multiply(qd_pre[7:14], self.real_robot_q_right_dir)
                    for i in range(14):
                        command_q_14dof_pre[i] = command_q_14dof_pre[i]

                    self._servoJ_last_q = servo_q.copy()
                    self.torso_angle = current_torsor_state

                    # 限制最大关节速度
                    for i in range(len(qd)):
                        if abs(qd[i]) > np.deg2rad(120):
                            qd[i] = np.sign(qd[i]) * np.deg2rad(120)

                    # 灵巧手下发
                    q_hand = np.array(
                        [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
                    )
                    q_hand[0:6] = command_hand_12dof[0:6]
                    q_hand[6:12] = command_hand_12dof[6:12]
                    q_hand = q_hand / 1000 * 180
                    q_hand = np.deg2rad(q_hand)

                    # 指令下发
                    self.real_robot.command_joint_state(
                        servo_q[0:7],
                        servo_q[7:14],
                        q_hand[0:6],
                        q_hand[6:12],
                        command_head_2dof,
                        command_torsor_1dof,
                    )

            else:  # 拖拽模式
                if self.real_robot is not None:
                    (
                        current_arm_state,
                        current_hand_state,
                        current_head_state,
                        current_torsor_state,
                    ) = self.real_robot.get_joint_state(include_torso=True)

                    self.joint_angles_left = np.multiply(
                        current_arm_state[0:7] - self.real_robot_q_left_offset, 
                        self.real_robot_q_left_dir
                    )
                    self.joint_angles_right = np.multiply(
                        current_arm_state[7:14] - self.real_robot_q_right_offset,
                        self.real_robot_q_right_dir,
                    )
                    self.head_angle = current_head_state.copy()
                    self.torso_angle = current_torsor_state

                    pin.forwardKinematics(
                        self.robot.reduced_robot.model,
                        self.robot.data,
                        np.append(self.joint_angles_left, self.joint_angles_right),
                    )
                    frame_id = self.robot.reduced_robot.model.getJointId("left_wrist_yaw_joint")
                    init_T = self.robot.data.oMi[frame_id]
                    init_T = SE3(init_T.homogeneous)
                    self.pose_left = init_T

                    frame_id = self.robot.reduced_robot.model.getJointId("right_wrist_yaw_joint")
                    init_T = self.robot.data.oMi[frame_id]
                    init_T = SE3(init_T.homogeneous)
                    self.pose_right = init_T


# 为兼容性提供别名，支持 atom.movL() 的调用方式
Atom = RobotMotionController

