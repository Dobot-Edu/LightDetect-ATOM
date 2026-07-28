# -*- coding: utf-8 -*-
"""
点位保存模块
从 robot_control.py 导入保存的点位，并提供便捷的变量访问接口
支持从JSON文件直接加载点位数据（即使robot_control.py未运行）

注意: 优先从robot_control.py导入，如果失败则从JSON文件加载
"""

import numpy as np
import sys
import os
import json

# 点位数据文件路径
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SAVED_POINTS_FILE = os.path.join(BASE_DIR, "saved_points.json")

# 导入 robot_control 模块中的 saved_points
# 使用延迟导入避免循环导入问题
_saved_points = None

def _get_saved_points():
    """获取保存的点位字典（延迟导入）"""
    global _saved_points
    if _saved_points is None:
        try:
            # 尝试导入 saved_points
            import importlib
            if __name__ == "__main__" or not __package__:
                # 作为脚本运行或不在包中
                sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
                from robot_control import saved_points as sp
            else:
                # 作为模块导入
                from .robot_control import saved_points as sp
            _saved_points = sp
            # 如果成功导入，但数据为空，尝试调用 robot_control 中的加载函数或直接从文件加载
            if not _saved_points:
                try:
                    if __name__ == "__main__" or not __package__:
                        from robot_control import load_saved_points as _load_saved_points
                    else:
                        from .robot_control import load_saved_points as _load_saved_points
                    if _load_saved_points():
                        _saved_points = sp
                except Exception:
                    pass
            if not _saved_points and os.path.exists(SAVED_POINTS_FILE):
                try:
                    with open(SAVED_POINTS_FILE, 'r', encoding='utf-8') as f:
                        loaded_points = json.load(f)
                    # 如果 robot_control 已成功导入，则更新其全局 saved_points
                    if isinstance(_saved_points, dict) and _saved_points is sp:
                        _saved_points.clear()
                        _saved_points.update(loaded_points)
                    else:
                        _saved_points = loaded_points
                    print(f"从文件加载点位数据，共 {len(loaded_points)} 个点位")
                except Exception as e:
                    print(f"警告: 尝试从文件加载点位数据失败: {str(e)}")
        except (ImportError, AttributeError):
            # 如果导入失败，尝试从JSON文件加载
            try:
                if os.path.exists(SAVED_POINTS_FILE):
                    with open(SAVED_POINTS_FILE, 'r', encoding='utf-8') as f:
                        _saved_points = json.load(f)
                    print(f"从文件加载点位数据，共 {len(_saved_points)} 个点位")
                    _normalize_all_points(_saved_points)
                else:
                    _saved_points = {}
                    print("警告: 无法导入 saved_points 且文件不存在，使用空字典。")
            except Exception as e:
                _saved_points = {}
                print(f"警告: 无法导入 saved_points 且文件加载失败: {str(e)}，使用空字典。")
        if isinstance(_saved_points, dict):
            _normalize_all_points(_saved_points)
    return _saved_points


def get_point_joint(point_name, arm='left'):
    """
    获取点位的关节角度
    
    Args:
        point_name: 点位名称，如 'P1', 'P2' 等
        arm: 'left' 或 'right'
    
    Returns:
        np.ndarray: 关节角度数组（弧度），如果点位不存在或类型不匹配返回 None
    """
    saved_points = _get_saved_points()
    if point_name not in saved_points:
        print(f"警告: 点位 {point_name} 不存在")
        return None
    
    point_data = saved_points[point_name]
    _normalize_point_data(point_data)
    
    arm_type = _get_arm_type(point_data, arm)
    if arm_type != "joint":
        print(f"警告: 点位 {point_name} ({arm}) 不是关节角度类型")
        return None
    
    joint_key = "joint_left" if arm == 'left' else "joint_right"
    joint_data = point_data.get(joint_key)
    
    if joint_data is None:
        return None
    
    return np.array(joint_data)


def get_point_pose(point_name, arm='left'):
    """
    获取点位的笛卡尔坐标
    
    Args:
        point_name: 点位名称，如 'P1', 'P2' 等
        arm: 'left' 或 'right'
    
    Returns:
        list: [x, y, z, roll, pitch, yaw]，如果点位不存在或类型不匹配返回 None
    """
    saved_points = _get_saved_points()
    if point_name not in saved_points:
        print(f"警告: 点位 {point_name} 不存在")
        return None
    
    point_data = saved_points[point_name]
    _normalize_point_data(point_data)
    
    arm_type = _get_arm_type(point_data, arm)
    if arm_type != "pose":
        print(f"警告: 点位 {point_name} ({arm}) 不是笛卡尔坐标类型")
        return None
    
    pose_key = "pose_left" if arm == 'left' else "pose_right"
    pose_data = point_data.get(pose_key)
    
    return pose_data


# 点位变量字典（用于动态访问）
_point_variables = {}

def get_point_variable(point_name, arm='left', coord_type='joint'):
    """
    获取点位变量（通用接口）
    
    Args:
        point_name: 点位名称，如 'P1', 'P2' 等
        arm: 'left' 或 'right'
        coord_type: 'joint' 或 'pose'
    
    Returns:
        点位数据（关节角度或笛卡尔坐标）
    """
    if coord_type == 'joint':
        return get_point_joint(point_name, arm)
    else:
        return get_point_pose(point_name, arm)


# 刷新点位变量的函数（当点位更新后调用）
def refresh_point_variables():
    """刷新点位变量（重新加载点位数据）"""
    global _saved_points
    _saved_points = None  # 强制重新加载


def _normalize_all_points(points):
    for entry in points.values():
        if isinstance(entry, dict):
            _normalize_point_data(entry)


def _normalize_point_data(point_data):
    if "type_left" not in point_data:
        if point_data.get("joint_left"):
            point_data["type_left"] = "joint"
        elif point_data.get("pose_left"):
            point_data["type_left"] = "pose"
        else:
            point_data["type_left"] = point_data.get("type")
    if "type_right" not in point_data:
        if point_data.get("joint_right"):
            point_data["type_right"] = "joint"
        elif point_data.get("pose_right"):
            point_data["type_right"] = "pose"
        else:
            point_data["type_right"] = point_data.get("type")


def _get_arm_type(point_data, arm):
    key = "type_left" if arm == 'left' else "type_right"
    arm_type = point_data.get(key)
    if arm_type:
        return arm_type
    if arm == 'left':
        if point_data.get("joint_left"):
            return "joint"
        if point_data.get("pose_left"):
            return "pose"
    else:
        if point_data.get("joint_right"):
            return "joint"
        if point_data.get("pose_right"):
            return "pose"
    return None


# 导出所有点位名称
def list_points():
    """列出所有已保存的点位名称"""
    saved_points = _get_saved_points()
    return list(saved_points.keys())


# 使用示例：
# from atom.saved_points import get_point_joint, get_point_pose
# 
# # 方式1: 使用函数获取关节角度
# P1 = get_point_joint('P1', 'left')
# atom.movJ(P1, sacle=0.5, arm_type=Arm_type_strucrt.left_arm)
# 
# # 方式2: 使用函数获取笛卡尔坐标
# P1_pose = get_point_pose('P1', 'left')
# atom.movL(P1_pose, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)
# 
# # 方式3: 直接使用点位名称（需要先获取）
# P1 = get_point_joint('P1', 'left')
# P2 = get_point_joint('P2', 'left')
# P3 = get_point_joint('P3', 'left')
# atom.movJ(P1, sacle=0.5, arm_type=Arm_type_strucrt.left_arm)
# atom.movJ(P2, sacle=0.5, arm_type=Arm_type_strucrt.left_arm)
# atom.movJ(P3, sacle=0.5, arm_type=Arm_type_strucrt.left_arm)

