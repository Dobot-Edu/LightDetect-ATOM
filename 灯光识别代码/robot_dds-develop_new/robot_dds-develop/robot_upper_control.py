import sys
import os
import time
from typing import Optional

parent_DIR = os.path.dirname(os.path.abspath(__file__))
# ROBOT_CONTROL_DIR = BASE_DIR+'/arm_control_dds/unitree_go'
# print(BASE_DIR)
print(parent_DIR)
sys.path.append(parent_DIR)

from robot_control_dds.control_sim import Control_sim
# from robot.robot_control_dds_P1.control_sim import control_sim

import numpy as np


class UpperControl():
    def __init__(self):
        self.robot = Control_sim()
        self.robot.start()
        time.sleep(2)
        self._torso_state = 0.0

    def has_error(self)-> bool:
        """
        检查机器人上肢是否存在报警
        参数:
            无
        返回:
            bool: True表示存在报警，False表示正常
        """
        main_nodes_state = self.robot.main_state_msg

        # 定义需要检查的所有关节组及其名称（用于调试）
        joint_groups = [
            # (main_nodes_state.left_leg, "左腿"),
            # (main_nodes_state.right_leg, "右腿"),
            # ([main_nodes_state.waist], "腰部"),
            (main_nodes_state.left_arm, "左臂"),
            (main_nodes_state.right_arm, "右臂"),
            (main_nodes_state.head, "头部")
        ]

        # 检查所有关节状态
        for joint_group, group_name in joint_groups:
            for i, axis in enumerate(joint_group):
                if (axis.pos_err_code != 0 or axis.vel_err_code != 0 or axis.torque_err_code != 0):
                    print(f"关节位置/速度/扭矩超限制: {group_name} 第{i + 1}个关节")
                    return 1
                if (axis.servo_state != 3 or axis.error_code != 0 or axis.node_state != 5):
                    print(f"未使能或报警: {group_name} 第{i + 1}个关节")
                    # print(f"报警位置: {group_name} ")
                    return 2

        # 检查ECAT2CAN从站状态
        for i, slave in enumerate(main_nodes_state.ecat2can):
            if slave.slave_state != 1 or slave.error_code != 0:
                print(f"ECAT2CAN从站{i + 1}报警")
                return 2

        return 0

    def clearUpperData(self):
        for i in range(17):
            self.robot.cmd_upper_msg.motor_cmd[i].q = 0
            self.robot.cmd_upper_msg.motor_cmd[i].dq = 0
            self.robot.cmd_upper_msg.motor_cmd[i].tau = 0
            self.robot.cmd_upper_msg.motor_cmd[i].kp = 0
            self.robot.cmd_upper_msg.motor_cmd[i].kd = 5
        self.robot.low_cmd_writer.write(self.robot.cmd_upper_msg)


    def parse_robot_type(self,char_array):
        valid_types = ["ATOM MAX", "ATOM Standard", "ATOM D", "ATOM Trainer", "ATOM W", "unknown"]
        clean_str = ''.join(char_array).rstrip('\x00').strip()
        return clean_str if clean_str in valid_types else "unknown"

    def get_robot_type(self):
        robot_type = self.robot.upper_msg.robot_type
        robot_type = self.parse_robot_type(robot_type)
        return robot_type

    def get_fsm_state(self):
        fsm_id = self.robot.fsm_msg.id
        return fsm_id

    def set_fsm_state(self):
        self.robot.send_fsm_cmd()

    def setUpperControl(self,state: bool):
        for index in range(10):
            res = self.robot.RPC.CallSwitchUpperLimbControl(state)
            if res ==1:
                print("setUpperControl:success")
                return res
        print("setUpperControl:fail")
        return res

    def get_joint_state(self, include_torso: bool = False):
        """Get the current arm state of the  robot.
        Returns:
            T: The current arm state of the robot.
        """
        arm_state = []
        for i in range(1, 15):
            arm_val = self.robot.upper_msg.motor_state[i].q
            arm_state.append(arm_val)

        head_state = []
        for i in range(15, 17):
            arm_val = self.robot.upper_msg.motor_state[i].q
            head_state.append(arm_val)

        hand_state = []
        for i in range(6, 12):
            left_hand_val = self.robot.hand_msg.hands[i].q
            hand_state.append(left_hand_val)

        for i in range(6):
            right_hand_val = self.robot.hand_msg.hands[i].q
            hand_state.append(right_hand_val)
        torso_state = self.robot.upper_msg.motor_state[0].q
        self._torso_state = torso_state

        if include_torso:
            return (
                np.array(arm_state),
                np.array(hand_state),
                np.array(head_state),
                torso_state,
            )
        return np.array(arm_state), np.array(hand_state), np.array(head_state)

    # def get_XYZrxryrz_state(self) -> np.ndarray:
    #     """Get the current X Y Z rx ry rz arm state of the robot.
    #     Returns:
    #         T: The current X Y Z rx ry rz arm state of the robot.
    #     """
    #     return

    def command_joint_state(self, left_joint_state: np.ndarray, right_joint_state: np.ndarray,
                            left_hand_state: np.ndarray, right_hand_state: np.ndarray,
                            head_state: np.ndarray, torso_state: Optional[float] = None) -> None:
        """Command the arms to a given joint state through the ServeJ.

        Args:
            joint_state (np.ndarray): The state to command the arms to.
        """
        torso_val = self._torso_state if torso_state is None else float(torso_state)
        self._torso_state = torso_val

        q_ref = np.zeros(17)
        q_ref[0] = torso_val  # torso joint
        q_ref[1:8] = left_joint_state  # left arm,unit:rad
        q_ref[8:15] = right_joint_state  # right arm,unit:rad
        q_ref[15:17] = head_state  # head
        # print(f"[UpperControl] command_joint_state torso={np.rad2deg(torso_val):.2f}deg")

        q_hand = np.array(np.concatenate([right_hand_state, left_hand_state]))  # hand,unit:rad
        # print("q_hand:",q_hand)
        self.robot.send_cmd(q_ref, q_hand)
        return 1

    def command_joint_vel_state(self, left_joint_state: np.ndarray, right_joint_state: np.ndarray,
                                left_hand_state: np.ndarray, right_hand_state: np.ndarray, head_state: np.ndarray,
                                arm_vel, send_finger, torso_state: Optional[float] = None) -> None:
        """Command the arms to a given joint state through the ServeJ.

        Args:
            joint_state (np.ndarray): The state to command the arms to.
        """
        torso_val = self._torso_state if torso_state is None else float(torso_state)
        self._torso_state = torso_val

        q_ref = np.zeros(17)
        q_ref[0] = torso_val
        q_ref[1:8] = left_joint_state  # left arm,unit:rad
        q_ref[8:15] = right_joint_state  # right arm,unit:rad
        q_ref[15:17] = head_state  # head
        q_ref_vel = np.zeros(17)
        q_ref_vel[1:15] = arm_vel

        q_hand = np.array(np.concatenate([right_hand_state, left_hand_state]))  # hand,unit:rad
        # print("q_hand:",q_hand)
        self.robot.send_vel_cmd(q_ref, q_hand, q_ref_vel, send_finger)
        return 1

    def dynamic_approach(self, current_arm_state, current_hand_state, current_head_state,target_arm, tager_finger, target_head):
        target_joints = np.concatenate([target_arm,tager_finger,target_head])

        # current_arm_state2, current_hand_state2, current_head_state2 = self.get_joint_state()
        current_state = np.concatenate([current_arm_state, current_hand_state, current_head_state])

        max_delta = (np.abs(current_state - target_joints)).max()
        # 手臂缓慢到初始化位置
        steps = min(int(max_delta / 0.001), 500)
        # steps = min(int(max_delta / 0.005), 200)
        for jnt in np.linspace(current_state, target_joints, steps):
            self.command_joint_state(jnt[0:7],jnt[7:14],jnt[14:20],jnt[20:26],jnt[26:28])
            time.sleep(0.005)

    def check_pose_protection(self, current_arm, current_finger, current_head, target_q, target_finger, target_head):
        flag = True
        sol_q = target_q.copy()
        # 1）Joint position protection
        #
        # 2)End position protection
        #     flag = False
        # 3)If the joint changes too much, prompt and follow slowly to prevent too fast.
        if np.any(np.abs(sol_q - current_arm) > 0.017*15):  # 大于10度
            print("检测到动作变化超过15度")
            print("Not flow...")
            #dynamic_approach(robot, sol_q, target_finger, target_head)
            flag = False

        elif np.any(np.abs(sol_q - current_arm) > 0.017*4):  # 大于4度
            print("检测到动作变化超过4度")
            print("Slow approach...")
            self.dynamic_approach(current_arm, current_finger, current_head,sol_q, target_finger, target_head)
        return flag

    def check_sync_pose(self,target_q):
        current_arm_state, current_hand_state, current_head_state = self.get_joint_state()
        flag = True
        sol_q = target_q.copy()
        if np.any(np.abs(sol_q - current_arm_state) > 0.017 * 90):  # 大于15度
            # print("人机手臂姿态差异过大（超过90度）！请调整人的姿态接近机器人再重新进行同步")
            flag = False
        return flag

    def get_lift_state(self):
        """
        获取升降轴当前状态
        Returns:
            np.ndarray: 包含3个升降轴关节当前位置的数组
        """
        lift_state = []
        # 读取左腿前3个关节作为升降轴关节
        for i in range(3):
            lift_val = self.robot.lower_msg.motor_state[i].q
            lift_state.append(lift_val)
        return np.array(lift_state)

    def command_lift_state(self, lift_state: np.ndarray, kp: float = 800, kd: float = 40) -> None:
        """
        控制升降轴到指定位置
        
        Args:
            lift_state (np.ndarray): 目标位置数组，长度为3
            kp (float): 位置控制比例增益
            kd (float): 位置控制微分增益
        """
        # 确保输入数组长度为3
        if len(lift_state) != 3:
            raise ValueError("升降轴状态数组长度必须为3")
        
        # 设置升降轴关节控制指令
        for i in range(3):
            self.robot.cmd_lower_msg.motor_cmd[i].q = lift_state[i]
            self.robot.cmd_lower_msg.motor_cmd[i].dq = 0.
            self.robot.cmd_lower_msg.motor_cmd[i].tau = 0.
            self.robot.cmd_lower_msg.motor_cmd[i].kp = kp
            self.robot.cmd_lower_msg.motor_cmd[i].kd = kd

        # 发送指令到 rt/lower/cmd 话题
        self.robot.lower_cmd_writer.write(self.robot.cmd_lower_msg)

    def command_lift_vel_state(self, lift_state: np.ndarray, lift_vel: np.ndarray, 
                            kp: float = 800, kd: float = 40) -> None:
        """
        控制升降轴到指定位置和速度
        
        Args:
            lift_state (np.ndarray): 目标位置数组，长度为3
            lift_vel (np.ndarray): 目标速度数组，长度为3
            kp (float): 位置控制比例增益
            kd (float): 位置控制微分增益
        """
        # 确保输入数组长度为3
        if len(lift_state) != 3 or len(lift_vel) != 3:
            raise ValueError("升降轴状态和速度数组长度必须为3")
        
        # 设置升降轴关节控制指令
        for i in range(3):
            self.robot.cmd_lower_msg.motor_cmd[i].q = lift_state[i]
            self.robot.cmd_lower_msg.motor_cmd[i].dq = lift_vel[i]
            self.robot.cmd_lower_msg.motor_cmd[i].tau = 0.
            self.robot.cmd_lower_msg.motor_cmd[i].kp = kp
            self.robot.cmd_lower_msg.motor_cmd[i].kd = kd

        # 发送指令到 rt/lower/cmd 话题
        self.robot.lower_cmd_writer.write(self.robot.cmd_lower_msg)

    def dynamic_lift_approach(self, target_lift):
        """
        缓慢移动升降轴到目标位置
        
        Args:
            target_lift (np.ndarray): 目标位置数组，长度为3
        """
        current_lift_state = self.get_lift_state()
        print("current_lift_state:", current_lift_state)
        
        # 注释掉：这段代码试图修改只读的当前状态，且语法有误
        # for i in range(len(current_lift_state)):
        #     current_lift_state[i] += 0.01
        # current_lift_state = current_lift_state + 0.01

        max_delta = (np.abs(current_lift_state - target_lift)).max()
        # 升降轴缓慢移动到目标位置
        steps = min(int(max_delta / 0.001), 1000)
        
        for lift_pos in np.linspace(current_lift_state, target_lift, steps):
            self.command_lift_state(lift_pos)
            time.sleep(0.005)

class P1_UpperControl():
    def __init__(self):
        self.robot = control_sim()
        self.robot.start()
        time.sleep(2)

    def get_joint_state(self):
        """Get the current arm state of the  robot.
        Returns:
            T: The current arm state of the robot.
        """
        arm_state = []
        for i in range(13, 27):
            arm_val = self.robot.msg.motor_state[i].q
            arm_state.append(arm_val)

        head_state = []
        for i in range(27, 29):
            arm_val = self.robot.msg.motor_state[i].q
            head_state.append(arm_val)

        hand_state = []
        for i in range(6, 12):
            left_hand_val = self.robot.hand_msg.hands[i].q
            hand_state.append(left_hand_val)

        for i in range(6):
            right_hand_val = self.robot.hand_msg.hands[i].q
            hand_state.append(right_hand_val)
        # hand_state = np.zeros(12)

        return np.array(arm_state), np.array(hand_state), np.array(head_state)

    # def get_XYZrxryrz_state(self) -> np.ndarray:
    #     """Get the current X Y Z rx ry rz arm state of the robot.
    #     Returns:
    #         T: The current X Y Z rx ry rz arm state of the robot.
    #     """
    #     return

    def command_joint_state(self, left_joint_state: np.ndarray, right_joint_state: np.ndarray,
                            left_hand_state: np.ndarray, right_hand_state: np.ndarray, head_state: np.ndarray) -> None:
        """Command the arms to a given joint state through the ServeJ.

        Args:
            joint_state (np.ndarray): The state to command the arms to.
        """
        q_ref = np.zeros(30)
        q_ref[13:20] = left_joint_state  # left arm,unit:rad
        q_ref[20:27] = right_joint_state  # right arm,unit:rad
        q_ref[27:29] = head_state  # head

        q_hand = np.array(np.concatenate([right_hand_state, left_hand_state]))  # hand,unit:rad
        # print("q_hand:",q_hand)
        self.robot.send_cmd(q_ref, q_hand)
        return 1

    def command_joint_vel_state(self, left_joint_state: np.ndarray, right_joint_state: np.ndarray,
                                left_hand_state: np.ndarray, right_hand_state: np.ndarray, head_state: np.ndarray,
                                arm_vel, send_finger_flag) -> None:
        """Command the arms to a given joint state through the ServeJ.

        Args:
            joint_state (np.ndarray): The state to command the arms to.
        """
        q_ref = np.zeros(30)
        q_ref[13:20] = left_joint_state  # left arm,unit:rad
        q_ref[20:27] = right_joint_state  # right arm,unit:rad
        q_ref[27:29] = head_state  # head
        q_ref_vel = np.zeros(30)
        q_ref_vel[13:27] = arm_vel

        q_hand = np.array(np.concatenate([right_hand_state, left_hand_state]))  # hand,unit:rad
        # print("q_hand:",q_hand)
        self.robot.send_vel_cmd(q_ref, q_hand, q_ref_vel, send_finger_flag)
        return 1

    def dynamic_approach(self, target_arm, tager_finger, target_head):
        target_joints = np.concatenate([target_arm, tager_finger, target_head])

        current_arm_state, current_hand_state, current_head_state = self.get_joint_state()
        current_state = np.concatenate([current_arm_state, current_hand_state, current_head_state])

        max_delta = (np.abs(current_state - target_joints)).max()
        # 手臂缓慢到初始化位置
        steps = min(int(max_delta / 0.001), 500)
        # steps = min(int(max_delta / 0.005), 200)
        for jnt in np.linspace(current_state, target_joints, steps):
            self.command_joint_state(jnt[0:7], jnt[7:14], jnt[14:20], jnt[20:26], jnt[26:28])
            time.sleep(0.005)

    def check_pose_protection(self, target_q, current_q, target_finger, target_head):
        flag = True
        sol_q = target_q.copy()
        # 1）Joint position protection
        #
        # 2)End position protection
        #     flag = False
        # 3)If the joint changes too much, prompt and follow slowly to prevent too fast danger or robot speeding alarm.
        if np.any(np.abs(sol_q - current_q) > 0.017 * 8):  # 大于15度
            print("检测到动作变化超过8度")
            print("Not flow...")
            # dynamic_approach(robot, sol_q, target_finger, target_head)
            flag = False

        if np.any(np.abs(sol_q - current_q) > 0.017 * 4):  # 大于10度
            print("检测到动作变化超过4度")
            print("Slow approach...")
            self.dynamic_approach(sol_q, target_finger, target_head)
        return flag



def main():
    robot = UpperControl()
    if robot.has_error()!=0:
        print("机器人存在报警，或双臂和头部存在未上使能的关节！请处理后再启动")
        return
    robot_type = robot.get_robot_type()
    print("robot_type",robot_type)
    if robot_type =="ATOM MAX" or robot_type =="ATOM Trainer" :
        # ---------------状态机判断---------------------------------
        fsm_state = robot.get_fsm_state()
        print("get_fsm_state:", fsm_state)
        if int(fsm_state) not in [2, 100, 101, 102, 300, 301]:
            print("[Error]:fsm state is not in state: [2, 100, 101, 102, 300, 301]")
            return
        # ----------------------------------------------------------

        # ---------设置接管上肢控制，状态2时无需设置----------------------
        if int(fsm_state) in [100, 101, 102, 300, 301]:  # 状态2调试模式无需设置接管
            res = robot.setUpperControl(True)
            if res != 1:
                # set fail
                return
        # ----------------------------------------------------------
    elif robot_type =="ATOM D":
        pass
    else:
        print("This robot_type is not support!")
        # return
    time.sleep(2)

    print("控制升降轴到位置 [-0.61,1.60,-0.98]")
    current_l = robot.get_lift_state()
    print("current_l:",current_l)

    
    # target_lift1 = np.array([current_l[0], current_l[1] ,current_l[2]+0.17])
    # robot.dynamic_lift_approach(target_lift1)

    # target_lift1 = np.array([-0.61,1.60,-0.98])
    # robot.dynamic_lift_approach(target_lift1)
    # time.sleep(2)  # 等待2秒观察效果
    
    # 控制升降轴到另一位置
    print("控制升降轴移动")
    target_lift2 = np.array([-0.72,1.36,-0.64])
    robot.dynamic_lift_approach(target_lift2)
    # time.sleep(2)

        # 控制升降轴到另一位置
    print("控制升降轴移动")
    target_lift2 = np.array([-0.89,1.62,-0.70])
    robot.dynamic_lift_approach(target_lift2)
    time.sleep(2)

if __name__ == '__main__':
    main()
    # current_arm_state, current_hand_state, current_head_state = robot.get_joint_state()

    # left_arm = np.radians([0, 0, 0, 0, 0, 0, 0])
    # right_arm = np.radians([0, 0, 0, 0, 0, 0, 0])
    # left_finger = np.radians([176, 176, 176, 176, 53, 165])
    # right_finger = np.radians([176, 176, 176, 176, 53, 165])
    # head = np.radians([0, 0])

    # target_arm1 = np.concatenate([left_arm, right_arm])
    # tager_finger1 = np.concatenate([left_finger, right_finger])
    # target_head1 = head.copy()
    # robot.dynamic_approach(current_arm_state, current_hand_state, current_head_state,target_arm1, tager_finger1, target_head1)
    # time.sleep(1)

    # current_arm_state, current_hand_state, current_head_state = robot.get_joint_state()
    # print("current_arm_state:", np.rad2deg(current_arm_state))
    # print("current_hand_state:", np.rad2deg(current_hand_state))
    # print("current_head_state:", np.rad2deg(current_head_state))
    # time.sleep(2)

    # left_arm = np.radians([0, 0, 0, 0, 0, 0, 0])
    # right_arm = np.radians([0, 0, 0, 0, 0, 0, 0])
    # left_finger = np.radians([19, 19, 19, 19, 53, 90])
    # right_finger = np.radians([19, 19, 19, 19, 53, 90])
    # head = np.radians([0, 0])

    # target_arm2 = np.concatenate([left_arm, right_arm])
    # tager_finger2 = np.concatenate([left_finger, right_finger])
    # target_head2 = head.copy()
    # # robot.dynamic_approach(current_arm_state, current_hand_state, current_head_state, target_arm2, tager_finger2, target_head2)
    # robot.dynamic_approach(target_arm1, tager_finger1, target_head1, target_arm2, tager_finger2, target_head2)

    # time.sleep(1)
    # current_arm_state, current_hand_state, current_head_state = robot.get_joint_state()
    # print("current_arm_state:", np.rad2deg(current_arm_state))
    # print("current_hand_state:", np.rad2deg(current_hand_state))
    # print("current_head_state:", np.rad2deg(current_head_state))
    # time.sleep(2)

    # if robot_type == "ATOM MAX" or robot_type == "ATOM Trainer":
    #     if int(fsm_state) in [100, 101, 102, 300, 301]:  # 设置接管,状态2调试模式无需设置接管
    #         robot.setUpperControl(False)
