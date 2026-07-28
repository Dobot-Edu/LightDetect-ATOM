#!/usr/bin/env python3
# 遥操作数据写入文件

try:
    import casadi
    from pinocchio import casadi as cpin
    _has_pinocchio_casadi = True
except ImportError:
    casadi = None
    cpin = None
    _has_pinocchio_casadi = False

import meshcat.geometry as mg
import numpy as np
import pinocchio as pin
import time
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import MeshcatVisualizer
import os
import sys
from spatialmath.base import trplot
from spatialmath import *
import numpy as np
from spatialmath import SE3, SO3, UnitQuaternion
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import numpy as np
import sys


current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

# 统一管理URDF与mesh路径，指向 py_viewer/urdf/atom 目录
URDF_PACKAGE_ROOT = os.path.expanduser(os.path.join(parent_dir, "urdf", "atom"))
URDF_MODEL_PATH = os.path.join(URDF_PACKAGE_ROOT, "urdf", "atom.urdf")
SRDF_MODEL_PATH = os.path.join(URDF_PACKAGE_ROOT, "urdf", "dobot.srdf")
MESH_DIR = URDF_PACKAGE_ROOT


class Arm_IK:
    def __init__(self):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        self.robot = pin.RobotWrapper.BuildFromURDF(
            URDF_MODEL_PATH,
            MESH_DIR,
        )

        self.mixed_jointsToLockIDs = []
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )

        #  生成干涉检测专用的模型
        urdf_model_path = URDF_MODEL_PATH
        mesh_dir = MESH_DIR
        srdf_model_path = SRDF_MODEL_PATH

        self.interface_model = pin.buildModelFromUrdf(
            urdf_model_path,
        )

        # Load collision geometries
        self.interface_geom_model = pin.buildGeomFromUrdf(
            self.interface_model, urdf_model_path, pin.GeometryType.COLLISION, mesh_dir
        )

        self.interface_geom_model.addAllCollisionPairs()
        pin.removeCollisionPairs(
            self.interface_model, self.interface_geom_model, srdf_model_path
        )

        self.interface_data = self.interface_model.createData()
        self.interface_geom_data = pin.GeometryData(self.interface_geom_model)

        # --------------------------------------

        # self.reduced_robot.model.addFrame(
        #     pin.Frame('L_ee',
        #               self.reduced_robot.model.getJointId('left_wrist_yaw_joint'),
        #               pin.SE3(SO3.RPY(0,0,90,unit="deg").R,
        #                       np.array([0.052, 0, 0]).T),  # 这个参数不对应？
        #               pin.FrameType.OP_FRAME)
        # )

        # self.reduced_robot.model.addFrame(
        #     pin.Frame('R_ee',
        #               self.reduced_robot.model.getJointId('right_wrist_yaw_joint'),
        #               pin.SE3((SO3.RPY(180,0,0,unit="deg")*SO3.RPY(0,0,90,unit="deg")).R ,
        #                       np.array([0.052, 0, 0]).T),
        #               pin.FrameType.OP_FRAME)
        # )

        self.reduced_robot.model.addFrame(
            pin.Frame(
                "L_ee",
                self.reduced_robot.model.getJointId("left_wrist_yaw_joint"),
                pin.SE3(SO3().R, np.array([0.0, 0, 0]).T),  # 这个参数不对应？
                pin.FrameType.OP_FRAME,
            )
        )

        self.reduced_robot.model.addFrame(
            pin.Frame(
                "R_ee",
                self.reduced_robot.model.getJointId("right_wrist_yaw_joint"),
                pin.SE3((SO3()).R, np.array([0.0, 0, 0]).T),
                pin.FrameType.OP_FRAME,
            )
        )

        self.init_data = np.zeros(self.reduced_robot.model.nq)

        self.data = self.reduced_robot.model.createData()

        self.L_hand_id = self.reduced_robot.model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_robot.model.getFrameId("R_ee")

        self._use_casadi_ik = _has_pinocchio_casadi
        if _has_pinocchio_casadi:
            # Creating Casadi models and data for symbolic computing
            self.cmodel = cpin.Model(self.reduced_robot.model)
            self.cdata = self.cmodel.createData()

            # Creating symbolic variables
            self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
            self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
            self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
            cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

            self.translational_error = casadi.Function(
                "translational_error",
                [self.cq, self.cTf_l, self.cTf_r],
                [
                    casadi.vertcat(
                        self.cdata.oMf[self.L_hand_id].translation - self.cTf_l[:3, 3],
                        self.cdata.oMf[self.R_hand_id].translation - self.cTf_r[:3, 3],
                    )
                ],
            )
            self.rotational_error = casadi.Function(
                "rotational_error",
                [self.cq, self.cTf_l, self.cTf_r],
                [
                    casadi.vertcat(
                        cpin.log3(
                            self.cdata.oMf[self.L_hand_id].rotation @ self.cTf_l[:3, :3].T
                        ),
                        cpin.log3(
                            self.cdata.oMf[self.R_hand_id].rotation @ self.cTf_r[:3, :3].T
                        ),
                    )
                ],
            )

            # Defining the optimization problem
            self.opti = casadi.Opti()
            self.var_q = self.opti.variable(self.reduced_robot.model.nq)
            self.var_q_init = self.opti.parameter(self.reduced_robot.model.nq)
            self.param_tf_l = self.opti.parameter(4, 4)
            self.param_tf_r = self.opti.parameter(4, 4)
            self.translational_cost = casadi.sumsqr(
                self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r)
            )
            self.rotation_cost = casadi.sumsqr(
                self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r)
            )
            self.regularization_cost = casadi.sumsqr(self.var_q)
            self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_init)  # for smooth

            self.opti.minimize(
                50 * self.translational_cost
                + self.rotation_cost * 20
                + 0.02 * self.regularization_cost
                + 5 * self.smooth_cost
            )
            opts = {
                "ipopt": {"print_level": 0, "max_iter": 400, "tol": 1e-4},
                "print_time": False,
                "calc_lam_p": False,
            }
            self.opti.solver("ipopt", opts)
        else:
            from scipy.optimize import minimize
            self._scipy_minimize = minimize

        self.reduced_robot.model.upperPositionLimit[0] = np.deg2rad(170)
        self.reduced_robot.model.lowerPositionLimit[0] = (
            -self.reduced_robot.model.upperPositionLimit[0]
        )

        self.reduced_robot.model.upperPositionLimit[1] = np.deg2rad(180)
        self.reduced_robot.model.lowerPositionLimit[1] = -np.deg2rad(11)

        self.reduced_robot.model.upperPositionLimit[3] = np.deg2rad(150)
        self.reduced_robot.model.lowerPositionLimit[3] = -np.deg2rad(25)

        self.reduced_robot.model.upperPositionLimit[5] = np.deg2rad(80)
        self.reduced_robot.model.lowerPositionLimit[5] = -np.deg2rad(80)

        print("lower_limit=", np.degrees(self.reduced_robot.model.lowerPositionLimit))
        print("upper_limit=", np.degrees(self.reduced_robot.model.upperPositionLimit))
        if not _has_pinocchio_casadi:
            print("pinocchio.casadi 不可用，IK 将使用 scipy.optimize 回退。")

    def _ik_cost_scipy(self, q, param_tf_l, param_tf_r, q_init):
        """Scipy 回退：与 CasADi 版本一致的代价函数（数值计算）"""
        pin.framesForwardKinematics(
            self.reduced_robot.model, self.data, np.asarray(q).ravel()
        )
        t_l = self.data.oMf[self.L_hand_id].translation
        t_r = self.data.oMf[self.R_hand_id].translation
        R_l = self.data.oMf[self.L_hand_id].rotation
        R_r = self.data.oMf[self.R_hand_id].rotation
        transl_err = np.hstack([
            t_l - param_tf_l[:3, 3],
            t_r - param_tf_r[:3, 3],
        ])
        rot_err_l = pin.log3(R_l @ param_tf_l[:3, :3].T)
        rot_err_r = pin.log3(R_r @ param_tf_r[:3, :3].T)
        rot_err = np.hstack([rot_err_l.ravel(), rot_err_r.ravel()])
        q = np.asarray(q).ravel()
        return (
            50 * np.sum(transl_err ** 2)
            + 20 * np.sum(rot_err ** 2)
            + 0.02 * np.sum(q ** 2)
            + 5 * np.sum((q - q_init) ** 2)
        )

    def ik_fun(self, left_pose, right_pose, motorstate=None, motorV=None):
        left_pose_copy = left_pose.copy()
        right_pose_copy = right_pose.copy()
        current_motor_q_copy = motorstate.copy()

        self.init_data = current_motor_q_copy

        if self._use_casadi_ik:
            self.opti.set_initial(self.var_q, current_motor_q_copy)
            self.opti.set_value(self.param_tf_l, left_pose_copy)
            self.opti.set_value(self.param_tf_r, right_pose_copy)
            self.opti.set_value(self.var_q_init, current_motor_q_copy)

            try:
                sol = self.opti.solve()
                if sol.stats()["return_status"] == "Solve_Succeeded":
                    sol_q = sol.value(self.var_q)
                else:
                    sol_q = self.opti.debug.value(self.var_q)
                self.init_data = sol_q
                return sol_q.copy()
            except Exception as e:
                print(f"求解失败:{e}")
                print(f"left_pose:{left_pose}")
                print(f"right_pose:{right_pose}")
                sol_q = self.opti.debug.value(self.var_q)
                self.init_data = sol_q
                return self.init_data.copy()

        # Scipy 回退
        try:
            bounds = list(zip(
                self.reduced_robot.model.lowerPositionLimit.tolist(),
                self.reduced_robot.model.upperPositionLimit.tolist(),
            ))
            res = self._scipy_minimize(
                self._ik_cost_scipy,
                current_motor_q_copy,
                args=(left_pose_copy, right_pose_copy, current_motor_q_copy),
                method="L-BFGS-B",
                bounds=bounds,
                options={"maxiter": 400},
            )
            if res.success:
                sol_q = np.asarray(res.x).ravel()
            else:
                sol_q = current_motor_q_copy
            self.init_data = sol_q
            return sol_q.copy()
        except Exception as e:
            print(f"求解失败:{e}")
            print(f"left_pose:{left_pose}")
            print(f"right_pose:{right_pose}")
            self.init_data = current_motor_q_copy
            return current_motor_q_copy.copy()
