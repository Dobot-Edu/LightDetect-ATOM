import sys
import os
import time
import threading
from typing import Optional, Dict, Any, Union
from dataclasses import dataclass

# 添加父目录到路径以导入 DDS 消息
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader
from cyclonedds.pub import DataWriter
from cyclonedds.util import duration
from dobot_atom.msg import dds_


# 直接复用 DDS 枚举，避免与实际枚举类型不一致
NavigationStatus = dds_.NavigationStatus_
DeviceStatus = dds_.DeviceStatus_
AMRCommandType = dds_.AMRCommandType


@dataclass
class LastCommand:
    """记录最后发送的命令，用于重发"""
    tag_id: int
    theta: float
    command_id: int
    timestamp: float


class AMR_SDK:
    """AMR 底盘控制 Python SDK"""
    
    def __init__(self):
        """初始化 AMR SDK"""
        # DDS 参与者和话题
        self.participant = DomainParticipant()
        
        # 创建话题
        self.state_topic = Topic(self.participant, "rt/amr/state", dds_.AMRState_)
        self.cmd_topic = Topic(self.participant, "rt/amr/cmd", dds_.AMRCommand_)
        # 创建手柄按钮状态相关的话题
        self.upper_state_topic = Topic(self.participant, "rt/upper/state", dds_.UpperState_)
        self.lower_state_topic = Topic(self.participant, "rt/lower/state", dds_.LowerState_)
        
        # 创建读写器
        self.state_reader = DataReader(self.participant, self.state_topic)
        self.cmd_writer = DataWriter(self.participant, self.cmd_topic)
        # 创建手柄按钮状态相关的读取器
        self.upper_state_reader = DataReader(self.participant, self.upper_state_topic)
        self.lower_state_reader = DataReader(self.participant, self.lower_state_topic)
        
        # 状态缓存
        self.amr_state: Optional[dds_.AMRState_] = None
        self.state_lock = threading.Lock()
        # 手柄按钮状态缓存
        self.upper_state: Optional[dds_.UpperState_] = None
        self.lower_state: Optional[dds_.LowerState_] = None
        self.joystick_state_lock = threading.Lock()
        
        # 命令 ID 计数器
        self.command_id_counter = 1
        
        # 最后发送的命令（用于重发）
        self.last_command: Optional[LastCommand] = None
        
        # 缓存 DDSJoystickReader 相关导入（延迟导入，避免循环导入）
        self._get_button_state_func = None
        self._DDSJoystickReader = None
        self._Control_sim = None
        # 按钮检查节流：减少按钮状态检查频率，避免频繁打印日志
        self._last_button_check_time = 0.0
        self._button_check_interval = 2.0  # 每2秒检查一次按钮状态
        # 按钮X状态检查的日志节流
        self._last_button_x_log_time = 0.0
        self._button_x_log_interval = 5.0  # 每5秒打印一次按钮X检查日志
        
        # 后台监听线程
        self.read_state_thread = threading.Thread(target=self._read_amr_state)
        self.read_state_thread.setDaemon(True)
        self.read_state_thread.start()
        
        # 后台监听手柄按钮状态线程
        self.read_upper_state_thread = threading.Thread(target=self._read_upper_state)
        self.read_upper_state_thread.setDaemon(True)
        self.read_upper_state_thread.start()
        
        self.read_lower_state_thread = threading.Thread(target=self._read_lower_state)
        self.read_lower_state_thread.setDaemon(True)
        self.read_lower_state_thread.start()
        
        print("AMR SDK 初始化完成")
    
    def _read_amr_state(self):
        """后台线程持续监听 AMR 状态"""
        for msg in self.state_reader.read_iter(timeout=duration(minutes=0.01)):
            with self.state_lock:
                self.amr_state = msg
                time.sleep(0.001)
    
    def _read_upper_state(self):
        """后台线程持续监听 upper state（包含手柄按钮状态）"""
        for msg in self.upper_state_reader.read_iter(timeout=duration(minutes=0.01)):
            with self.joystick_state_lock:
                self.upper_state = msg
                time.sleep(0.001)
    
    def _read_lower_state(self):
        """后台线程持续监听 lower state（包含手柄按钮状态）"""
        for msg in self.lower_state_reader.read_iter(timeout=duration(minutes=0.01)):
            with self.joystick_state_lock:
                self.lower_state = msg
                time.sleep(0.001)
    
    def _now_us(self) -> int:
        """获取当前时间戳（微秒）"""
        return int(time.time() * 1000000)
    
    
    def move_to_tag(self, tag_id: int, theta: float) -> int:
        """
        发送移动到标签点命令
        
        Args:
            tag_id: 目标标签ID
            theta: 目标角度（度）
            
        Returns:
            command_id: 命令ID
        """
        # 创建命令
        cmd = dds_.AMRCommand_(
            command_type=AMRCommandType.MOVE_TO_TAG,
            target_id=tag_id,
            linear_vel=0.0,
            angular_vel=0.0,
            command_id=self.command_id_counter,
            timestamp=self._now_us(),
            theta=theta
        )
        
        # 发送命令
        self.cmd_writer.write(cmd)
        
        # 记录最后发送的命令
        self.last_command = LastCommand(
            tag_id=tag_id,
            theta=theta,
            command_id=self.command_id_counter,
            timestamp=time.time()
        )
        
        # 递增命令ID
        self.command_id_counter += 1
        
        print(f"[AMR_SDK] 发送 MOVE_TO_TAG 命令: tag_id={tag_id}, theta={theta}°, command_id={cmd.command_id}")
        return cmd.command_id
    
    def cancel_task(self) -> int:
        """
        取消当前正在执行的任务
        
        Returns:
            command_id: 命令ID
        """
        # 创建取消任务命令
        cmd = dds_.AMRCommand_(
            command_type=AMRCommandType.CANCEL_TASK,
            target_id=0,
            linear_vel=0.0,
            angular_vel=0.0,
            command_id=self.command_id_counter,
            timestamp=self._now_us(),
            theta=0.0
        )
        
        # 发送命令
        self.cmd_writer.write(cmd)
        
        # 递增命令ID
        self.command_id_counter += 1
        
        print(f"[AMR_SDK] 发送 CANCEL_TASK 命令: command_id={cmd.command_id}")
        return cmd.command_id
    
    def wait_task_start(self, timeout: float = 10.0) -> bool:
        """
        等待任务开始执行
        
        Args:
            timeout: 超时时间（秒）
            
        Returns:
            bool: True=任务已开始，False=超时失败
        """
        if self.last_command is None:
            print("[AMR_SDK] 错误：没有待执行的任务")
            return False
        
        start_time = time.time()
        last_resend_time = start_time
        resend_interval = 3.0  # 3秒重发间隔
        
        print(f"[AMR_SDK] 等待任务开始，超时时间: {timeout}秒")
        
        while time.time() - start_time < timeout:
            with self.state_lock:
                if self.amr_state is None:
                    time.sleep(0.1)
                    continue
                
                nav_status = self.amr_state.navigation_status
                
                if nav_status == NavigationStatus.RUNNING:
                    print("[AMR_SDK] 任务已开始执行")
                    return True
                
                # 检查是否需要重发命令
                current_time = time.time()
                if (current_time - last_resend_time >= resend_interval and 
                    nav_status != NavigationStatus.RUNNING):
                    
                    print(f"[AMR_SDK] 任务未开始，重新发送命令 (已等待 {current_time - start_time:.1f}s)")
                    self.move_to_tag(self.last_command.tag_id, self.last_command.theta)
                    last_resend_time = current_time
            
            time.sleep(1.0)  # 每秒检查一次
        
        print(f"[AMR_SDK] 等待任务开始超时 ({timeout}秒)")
        return False
    
    def wait_task_finish(self, timeout: float = 999.0) -> Union[bool, str]:
        """
        等待任务完成，同时检查手柄按钮X状态
        
        Args:
            timeout: 超时时间（秒）
            
        Returns:
            Union[bool, str]: 
                - True: 任务完成
                - False: 任务失败/超时
                - "X": 按钮X被按下（中断任务）
        """
        start_time = time.time()
        
        print(f"[AMR_SDK] 等待任务完成，超时时间: {timeout}秒")
        
        while time.time() - start_time < timeout:
            with self.state_lock:
                if self.amr_state is None:
                    time.sleep(0.1)
                    continue
                
                nav_status = self.amr_state.navigation_status
                
                # 检查导航状态（优先检查）
                if nav_status == NavigationStatus.COMPLETED:
                    print("[AMR_SDK] 任务已完成")
                    return True
                elif nav_status == NavigationStatus.FAILED:
                    print("[AMR_SDK] 任务执行失败")
                    return False
                elif nav_status == NavigationStatus.CANCELED:
                    print("[AMR_SDK] 任务被取消")
                    return False
                
                # 节流按钮状态检查，减少日志打印频率
                current_time = time.time()
                if current_time - self._last_button_check_time >= self._button_check_interval:
                    self._last_button_check_time = current_time
                    
                    # 使用 DDSJoystickReader 从 DDS 消息中读取按钮X状态
                    try:
                        from robot_control_dds.Joystick.JoystickState import DDSJoystickReader
                        
                        # 优先从 upper_state 读取，如果没有则从 lower_state 读取
                        with self.joystick_state_lock:
                            if self.upper_state is not None:
                                if DDSJoystickReader.get_button_A_state(self.upper_state):
                                    self.cancel_task()
                                    print("[AMR_SDK] 按钮X被按下，中断任务")
                                    return "A"
                            elif self.lower_state is not None:
                                if DDSJoystickReader.get_button_A_state(self.lower_state):
                                    self.cancel_task()
                                    print("[AMR_SDK] 按钮X被按下，中断任务")
                                    return "A"
                    except (ImportError, Exception) as e:
                        # 静默处理错误，避免频繁打印
                        pass
                           
            time.sleep(1.0)  # 每秒检查一次
        
        print(f"[AMR_SDK] 等待任务完成超时 ({timeout}秒)")
        return False
    
    def get_amr_state(self) -> Optional[Dict[str, Any]]:
        """
        获取当前 AMR 状态
        
        Returns:
            dict: AMR 状态字典，如果无状态则返回 None
        """
        with self.state_lock:
            if self.amr_state is None:
                return None
            
            state = self.amr_state
            return {
                'device_status': self.get_device_status_name(state.device_status),
                'navigation_status': self.get_navigation_status_name(state.navigation_status),
                'position': {
                    'x': state.position[0],
                    'y': state.position[1],
                    'theta': state.position[2]
                },
                'basic_status': {
                    'battery_level': state.basic_status.battery_level,
                    'battery_voltage': state.basic_status.battery_voltage,
                    'battery_current': state.basic_status.battery_current,
                    'heartbeat': state.basic_status.heartbeat
                },
                'amr_event': {
                    'emergency_stop_pressed': state.amr_event.emergency_stop_pressed,
                    'enable_pressed': state.amr_event.enable_pressed,
                    'path_blocked': state.amr_event.path_blocked,
                    'low_battery': state.amr_event.low_battery,
                    'obstacle_detected': state.amr_event.obstacle_detected
                },
                'task_id': state.task_id,
                'work_mode': state.work_mode,
                'error_codes': list(state.error_code)
            }
    
    def get_navigation_status_name(self, status: int) -> str:
        """获取导航状态名称"""
        if status is None:
            return "UNKNOWN"
        if hasattr(status, "name"):
            return status.name
        status_map = {
            NavigationStatus.UNKNOWN: "UNKNOWN",
            NavigationStatus.QUEUING: "QUEUING",
            NavigationStatus.RUNNING: "RUNNING",
            NavigationStatus.COMPLETED: "COMPLETED",
            NavigationStatus.FAILED: "FAILED",
            NavigationStatus.PAUSED: "PAUSED",
            NavigationStatus.CANCELED: "CANCELED",
            NavigationStatus.WAITING_CONFIRM: "WAITING_CONFIRM",
            NavigationStatus.IDLE: "IDLE",
            NavigationStatus.STOPPED: "STOPPED"
        }
        return status_map.get(status, "UNKNOWN")
    
    def get_device_status_name(self, status: int) -> str:
        """获取设备状态名称"""
        if status is None:
            return "UNKNOWN"
        if hasattr(status, "name"):
            return status.name
        status_map = {
            DeviceStatus.DEVUNKNOWN: "DEVUNKNOWN",
            DeviceStatus.DEVIDLE: "DEVIDLE",
            DeviceStatus.TASKING: "TASKING",
            DeviceStatus.ERROR: "ERROR",
            DeviceStatus.OFFLINE: "OFFLINE",
            DeviceStatus.INIT: "INIT",
            DeviceStatus.CHARGING: "CHARGING",
            DeviceStatus.UPGRADE: "UPGRADE"
        }
        return status_map.get(status, "UNKNOWN")
    
    def amr_move(self, tag_id: int, theta: float) -> Union[bool, str]:
        """
        执行移动到指定标签点的完整任务
        
        Args:
            tag_id: 目标标签ID
            theta: 目标角度（度）
            
        Returns:
            Union[bool, str]: 
                - True: 任务成功完成
                - False: 任务失败或超时
                - "X": 按钮X被按下（中断任务）
        """
        # 发送移动命令
        command_id = self.move_to_tag(tag_id=tag_id, theta=theta)
        print(f"发送命令 ID: {command_id}")
        
        # 等待任务开始
        if self.wait_task_start(timeout=10.0):
            print("任务已开始")
            
            # 等待任务完成（同时检查按钮X状态）
            result = self.wait_task_finish(timeout=300.0)
            if result is True:
                print("任务已完成")
                return True
            elif result == "A":
                print("任务被按钮A中断")
                return "A"
            else:
                print("任务完成超时或失败")
                return False
        else:
            print("任务开始超时")
            return False




# 使用示例
if __name__ == "__main__":
    # 创建 SDK 实例
    amr = AMR_SDK()
    
    # 等待状态更新
    time.sleep(2)
    
    # 获取当前状态
    for i in range(2):
        state = amr.get_amr_state()
        if state:
            print(f"当前状态: {state['navigation_status']}")
            print(f"位置: ({state['position']['x']:.2f}, {state['position']['y']:.2f}, {state['position']['theta']:.2f})")
            print(f"电池电量: {state['basic_status']['battery_level']:.1f}%")
        else:
            print("未获取到 AMR 状态")
        time.sleep(1.0)
    # 使用封装后的函数
    success = amr.amr_move(tag_id=1001, theta=0.0)
    time.sleep(10.0)
    success = amr.amr_move(tag_id=1002, theta=0.0)
    if success:
        print("移动任务执行成功")
    else:
        print("移动任务执行失败")