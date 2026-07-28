# 机器人运动控制器模块使用说明

## 概述

`atom_api.py` 是基于robot_control_dds代码的DDS接口封装成的独立模块，可以在其他程序中方便地调用。主要控制上肢手臂,头部,腰部。

## 主要功能

1. **TCP末端直线运动** (`movL`) - 控制机械臂末端沿直线运动到目标位置
2. **关节空间运动** (`movJ`) - 控制机械臂关节运动到目标角度
3. **关节空间连续路径运动** (`movJ_CP`) - 执行多段连续路径运动
4. **双臂协调运动** (`TwoArm_movJ_CP`) - 控制左右双臂同时执行协调运动
5. **灵巧手控制** (`hand_control`) - 控制灵巧手手指
6. **头部控制** (`head_control`) - 控制机器人头部
7. **腰部控制** (`torsor_control`) - 控制机器人腰部
8. **关节同步** (`sync_allJoint`) - 同步实际机器人关节角度
9. **升降轴控制** (`UpperControl`) - 控制机器人升降轴（需通过 `real_robot` 实例调用）
10. **底盘ARM控制** (`AMR_SDK`) - 控制底盘移动到指定标签点
11. **点位保存与加载** (`saved_points`) - 保存和加载点位数据，支持持久化存储

## 使用方法


### 1. 基本导入

```python
from atom.robot_model import Arm_IK
from atom.atom_api import Atom, Arm_type_strucrt
# 或者使用简化导入
from atom import Arm_IK, Atom, Arm_type_strucrt
import numpy as np
```

### 2. 初始化控制器

#### 仿真模式

```python
# 初始化机器人模型
robot = Arm_IK()

# 创建运动控制器
atom = Atom(
    robot=robot,
    real_robot=None,  # 仿真模式不需要真实机器人
    simulate=True,    # 启用仿真模式
    tcp_vel=0.7,      # TCP末端速度
    tcp_acc=20,       # TCP末端加速度
    joint_vel=1.4,    # 关节速度
    joint_acc=20,     # 关节加速度
    cycle=0.01        # 控制周期
)
```

#### 真实机器人模式

```python
from robot_upper_control import UpperControl

# 初始化机器人模型
robot = Arm_IK()

# 初始化真实机器人
real_robot = UpperControl()

# 创建运动控制器
atom = Atom(
    robot=robot,
    real_robot=real_robot,  # 传入真实机器人实例
    simulate=False,         # 真实机器人模式
    tcp_vel=0.7,
    tcp_acc=20,
    joint_vel=1.4,
    joint_acc=20,
    cycle=0.01,
    # 真实机器人参数
    real_robot_q_left_dir=[1, 1, 1, 1, 1, 1, 1],
    real_robot_q_right_dir=[1, 1, 1, 1, 1, 1, 1],
    real_robot_q_left_offset=np.deg2rad([0, 0, 0, 0, 0, 0, 0]),
    real_robot_q_right_offset=np.deg2rad([0, 0, 0, 0, 0, 0, 0])
)

# 同步实际关节角度
atom.sync_allJoint()
```

### 3. 执行运动

#### TCP末端直线运动

```python
# 方式1: 直接指定位姿
# pose: [x, y, z, roll, pitch, yaw] (单位: 米, 弧度)
target_pose = [0.3, 0.2, 0.15, 0, 0, 0]
atom.movL(
    pose=target_pose,
    sacle=0.7,  # 速度缩放因子
    arm_type=Arm_type_strucrt.left_arm  # 或 right_arm
)

# 方式2: 使用保存的点位（推荐）
from atom.saved_points import get_point_pose

P1_pose = get_point_pose('P1', 'left')  # 获取P1点位的左臂笛卡尔坐标
if P1_pose is not None:
    atom.movL(P1_pose, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)
```

#### 关节空间运动

```python
# 方式1: 直接指定关节角度
target_joint = np.deg2rad([30, 20, -10, -40, 5, -10, -5])
atom.movJ(
    targetJoint=target_joint,
    sacle=0.5,
    arm_type=Arm_type_strucrt.left_arm
)

# 方式2: 使用保存的点位（推荐）
from atom.saved_points import get_point_joint, list_points

# 查看所有已保存的点位
saved_point_names = list_points()
print(f"已保存的点位: {saved_point_names}")

# 使用保存的点位
P1 = get_point_joint('P1', 'left')  # 获取P1点位的左臂关节角度
if P1 is not None:
    atom.movJ(P1, sacle=0.5, arm_type=Arm_type_strucrt.left_arm)
```

#### 连续路径运动

```python
planning_info = [
    {
        "targrt": np.deg2rad([30, 20, -10, -40, 5, -10, -5]),
        "vel": 1.4,    # 速度
        "acc": 20,     # 加速度
        "CP": 0.3,     # 连续路径系数
    },
    {
        "targrt": np.deg2rad([40, 25, -15, -45, 8, -12, -6]),
        "vel": 1.4,
        "acc": 20,
        "CP": 0.3,
    }
]
atom.movJ_CP(
    planning_info=planning_info,
    sacle=0.7,
    arm_type=Arm_type_strucrt.left_arm
)
```

#### 双臂协调运动

```python
planning_info_left = [
    {
        "targrt": np.deg2rad([30, 20, -10, -40, 5, -10, -5]),
        "vel": 1.4,
        "acc": 20,
        "CP": 0.3,
    }
]
planning_info_right = [
    {
        "targrt": np.deg2rad([-30, -20, 10, 40, -5, 10, 5]),
        "vel": 1.4,
        "acc": 20,
        "CP": 0.3,
    }
]
atom.TwoArm_movJ_CP(
    planning_info_left=planning_info_left,
    planning_info_right=planning_info_right,
    sacle=0.7
)
```

#### 灵巧手控制
#### 灵巧手控制

```python
# hand_angle_target: 6个手指角度值 (范围: 0-1000)
hand_angle_target = np.array([1000, 1000, 1000, 1000, 1000, 1000])
atom.hand_control(
    hand_angle_target=hand_angle_target,
    arm_type=Arm_type_strucrt.left_arm
)
```

#### 头部控制

```python
# head_angle_target: [pitch, yaw] (单位: 弧度)
# duration: 期望完成时间 (秒)，默认 2s，可根据需要加速/减速
head_angle_target = np.deg2rad([0, 30])

# 使用默认2.0秒
atom.head_control(head_angle_target)

# 快速移动，1秒完成
atom.head_control(head_angle_target, duration=1.0)

# 慢速移动，4秒完成
atom.head_control(head_angle_target, duration=4.0)
```

#### 腰部控制

```python
# torsor_angle_target: 腰部角度 (单位: 弧度)
# duration: 期望完成时间 (秒)，默认 6s
torso_angle_target = np.deg2rad(5)
atom.torsor_control(torso_angle_target, duration=4.0)
```

#### 升降轴控制

升降轴控制需要通过 `UpperControl` 实例直接调用，不属于 `Atom` 类的方法。

```python
from robot_upper_control import UpperControl

# 初始化真实机器人（包含升降轴控制）
real_robot = UpperControl()

# 获取当前升降轴状态
current_lift_state = real_robot.get_lift_state()
print(f"当前升降轴位置: {current_lift_state}")

# 方法1: 直接控制升降轴到指定位置（快速）
target_lift = np.array([-0.79, 1.71, -0.91])  # 3个关节的目标位置（弧度）
real_robot.command_lift_state(target_lift, kp=800, kd=40)

# 方法2: 缓慢移动到目标位置（推荐，更安全）
target_lift = np.array([-0.88, 2.24, -0.8])
real_robot.dynamic_lift_approach(target_lift)
```

**注意事项**：
- 升降轴有3个关节，目标位置数组长度必须为3
- `command_lift_state()` 直接发送目标位置，速度较快
- `dynamic_lift_approach()` 会缓慢插值移动到目标位置，更安全
- 升降轴位置单位为弧度 (rad)

#### 底盘ARM控制

底盘控制需要通过 `AMR_SDK` 实例调用，不属于 `Atom` 类的方法，用于控制底盘移动到指定标签点。

```python
from robot_control_dds.amr.amr_sdk import AMR_SDK

# 初始化AMR SDK
amr = AMR_SDK()

# 等待SDK初始化完成
import time
time.sleep(2)

# 控制底盘移动到指定标签点
# tag_id: 目标标签ID（整数）
# theta: 目标角度（度）
result = amr.amr_move(tag_id=1008, theta=180.0)

# 检查移动结果
if result is True:
    print("底盘移动成功完成")
elif result == "A":
    print("底盘移动被按钮A中断")
else:
    print("底盘移动失败或超时")

# 示例：移动到取料点
amr.amr_move(tag_id=1001, theta=0.0)

# 示例：移动到放置点
amr.amr_move(tag_id=1002, theta=0.0)

# 获取当前底盘状态
state = amr.get_amr_state()
if state:
    print(f"底盘位置: x={state['position']['x']:.2f}, y={state['position']['y']:.2f}, theta={state['position']['theta']:.2f}")
    print(f"导航状态: {state['navigation_status']}")
    print(f"电池电量: {state['basic_status']['battery_level']}%")
```

**注意事项**：
- `amr_move()` 会等待任务开始和完成，是阻塞调用
- 返回值：`True`=成功，`False`=失败，`"A"`=被按钮A中断
- 移动过程中可以通过按钮A中断任务
- 建议在移动前先检查底盘状态，确保设备正常

## 参数说明

### Atom 初始化参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `robot` | Arm_IK | 必需 | 机器人模型实例 |
| `real_robot` | UpperControl | None | 真实机器人控制实例（真实模式需要） |
| `tcp_vel` | float | 0.7 | TCP末端速度 (m/s) |
| `tcp_acc` | float | 20 | TCP末端加速度 (m/s²) |
| `joint_vel` | float | 1.4 | 关节速度 (rad/s) |
| `joint_acc` | float | 20 | 关节加速度 (rad/s²) |
| `cycle` | float | 0.01 | 控制周期 (s) |
| `simulate` | bool | False | 是否仿真模式 |
| `isdrag` | bool | False | 是否拖拽模式 |
| `isVisual` | bool | False | 是否可视化 |
| `tool_left` | SE3 | None | 左手工具变换 |
| `tool_right` | SE3 | None | 右手工具变换 |
| `real_robot_q_left_dir` | list | [1,1,1,1,1,1,1] | 左手关节方向 |
| `real_robot_q_right_dir` | list | [1,1,1,1,1,1,1] | 右手关节方向 |
| `real_robot_q_left_offset` | np.array | 全0 | 左手关节偏移 |
| `real_robot_q_right_offset` | np.array | 全0 | 右手关节偏移 |
| `viz` | MeshcatVisualizer | None | 可视化器实例 |

## 状态变量访问

控制器内部的状态变量可以通过实例属性访问：

```python
# 关节角度
atom.joint_angles_left    # 左手7个关节角度
atom.joint_angles_right   # 右手7个关节角度

# 灵巧手角度
atom.joint_angles_handle_left   # 左手6个手指角度
atom.joint_angles_handle_right  # 右手6个手指角度

# 头部和腰部
atom.head_angle   # 头部角度 [pitch, yaw]
atom.torso_angle  # 腰部角度

# 末端位姿
atom.pose_left   # 左手末端位姿 (SE3)
atom.pose_right  # 右手末端位姿 (SE3)
```

### 4. 点位保存与加载

点位数据可以保存到文件，程序重启后仍可使用。

#### 保存点位

在 `robot_control.py` 中：
1. 打开"点位保存管理"窗口
2. 输入点位名称（如 P1, P2, P3）
3. 选择坐标类型（关节角度或笛卡尔坐标）
4. 点击"保存当前点位"

点位数据会自动保存到 `saved_points.json` 文件。

#### 加载和使用点位

```python
from atom.saved_points import get_point_joint, get_point_pose, list_points

# 查看所有已保存的点位
saved_point_names = list_points()
print(f"已保存的点位: {saved_point_names}")

# 获取关节角度点位
P1 = get_point_joint('P1', 'left')   # 左臂关节角度
P2 = get_point_joint('P2', 'right')  # 右臂关节角度

# 获取笛卡尔坐标点位
P1_pose = get_point_pose('P1', 'left')   # 左臂位姿 [x, y, z, roll, pitch, yaw]
P2_pose = get_point_pose('P2', 'right')  # 右臂位姿

# 使用点位执行运动
if P1 is not None:
    atom.movJ(P1, sacle=0.5, arm_type=Arm_type_strucrt.left_arm)

if P1_pose is not None:
    atom.movL(P1_pose, sacle=0.7, arm_type=Arm_type_strucrt.left_arm)
```

#### 点位数据文件

- **文件位置**: `robot_dds-develop/py_viewer/atom/saved_points.json`
- **数据格式**: JSON格式，包含点位名称、类型（joint/pose）、左右臂数据
- **持久化**: 程序关闭后数据不会丢失，重启后自动加载

**注意事项**：
- 点位数据会自动保存到文件，无需手动操作
- 程序启动时会自动加载已保存的点位
- 即使 `robot_control.py` 未运行，也可以通过 `saved_points.py` 模块访问点位数据
- 如果点位不存在或类型不匹配，函数会返回 `None` 并打印警告信息

## 完整示例

参考 `atom_api_example.py` 文件中的示例代码。

### 综合示例：结合升降轴和底盘控制

```python
from atom.robot_model import Arm_IK
from atom.atom_api import Atom, Arm_type_strucrt
from robot_upper_control import UpperControl
from robot_control_dds.amr.amr_sdk import AMR_SDK
import numpy as np
import time

# 1. 初始化机器人模型
robot = Arm_IK()

# 2. 初始化真实机器人和底盘
real_robot = UpperControl()
amr = AMR_SDK()
time.sleep(2)  # 等待SDK初始化

# 3. 创建运动控制器
atom = Atom(
    robot=robot,
    real_robot=real_robot,
    simulate=False,
    tcp_vel=0.7,
    tcp_acc=20,
    joint_vel=1.4,
    joint_acc=20,
    cycle=0.01
)

# 4. 同步关节角度
atom.sync_allJoint()

# 5. 控制升降轴到初始位置
target_lift = np.array([-0.79, 1.71, -0.91])
real_robot.dynamic_lift_approach(target_lift)

# 6. 控制底盘移动到取料点
amr.amr_move(tag_id=1001, theta=0.0)
print("底盘已到达取料点")

# 7. 执行抓取动作（示例）
# ... 执行手臂运动、抓取等操作 ...

# 8. 控制底盘移动到放置点
amr.amr_move(tag_id=1002, theta=0.0)
print("底盘已到达放置点")

# 9. 控制升降轴下降
target_lift_down = np.array([-0.88, 2.24, -0.8])
real_robot.dynamic_lift_approach(target_lift_down)

# 10. 执行放置动作
# ... 执行放置操作 ...

# 11. 控制升降轴上升
target_lift_up = np.array([-0.79, 1.71, -0.91])
real_robot.dynamic_lift_approach(target_lift_up)
```

## 注意事项

1. **初始化顺序**: 必须先初始化 `Arm_IK` 模型，再创建 `Atom` 实例
2. **真实机器人模式**: 使用真实机器人时，需要先调用 `sync_allJoint()` 同步实际关节角度
3. **单位**: 
   - 位置单位：米 (m)
   - 角度单位：弧度 (rad)
   - 灵巧手角度：0-1000 的数值
4. **线程安全**: 当前实现不是线程安全的，多线程使用时需要加锁
5. **错误处理**: 运动函数会打印错误信息，但不会抛出异常，需要检查返回值

## 与原代码的对应关系

| 原代码 (grap_control.py) | 封装后 (atom_api.py) |
|--------------------------|-------------------------------------|
| `movL()` | `atom.movL()` |
| `movJ()` | `atom.movJ()` |
| `movJ_CP()` | `atom.movJ_CP()` |
| `TwoArm_movJ_CP()` | `atom.TwoArm_movJ_CP()` |
| `hand_control()` | `atom.hand_control()` |
| `head_control()` | `atom.head_control()` |
| `torsor_control()` | `atom.torsor_control()` |
| `sync_allJoint()` | `atom.sync_allJoint()` |
| `servoJ()` | `atom.servoJ()` (内部方法) |
| 全局变量 | `atom.xxx` (实例属性) |

## 依赖库

- numpy
- spatialmath (SE3, SO3)
- ruckig
- pinocchio
- colorama
- robot_model (项目内部模块)
- robot_upper_control (项目内部模块，真实机器人模式需要)
- robot_control_dds.amr.amr_sdk (项目内部模块，底盘控制需要)

---

## 修订历史

| 版本 | 日期 | 修改人 | 修改内容 |
|------|------|--------|----------|
| 1.0.0 | 2025-11-27 | Y | 初始版本，创建 atom_api.py 模块，封装基本运动控制功能 |
| 1.0.1 | 2025-11-28 | Y | 添加升降轴控制和底盘ARM控制的使用示例说明 |
| 1.0.2 | 2025-11-28 | Y | 添加点位保存与加载功能，支持JSON文件持久化存储 |

