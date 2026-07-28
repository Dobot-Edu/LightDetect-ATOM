class JoystickButtonState:
    def __init__(self, wireless_remote: list):
        # 假设 wireless_remote 为 40字节uint8列表
        # 第3,4字节合成16位，仿C++ union
        if len(wireless_remote) < 4:
            self.value = 0
        else:
            self.value = int(wireless_remote[2]) | (int(wireless_remote[3]) << 8)

        # 解包
        self.button_R1_    = bool((self.value >> 0) & 1)
        self.button_L1_    = bool((self.value >> 1) & 1)
        self.button_START_ = bool((self.value >> 2) & 1)
        self.button_SELECT_= bool((self.value >> 3) & 1)
        self.button_R2_    = bool((self.value >> 4) & 1)
        self.button_L2_    = bool((self.value >> 5) & 1)
        self.button_F1_    = bool((self.value >> 6) & 1)
        self.button_F2_    = bool((self.value >> 7) & 1)
        self.button_A_     = bool((self.value >> 8) & 1)
        self.button_B_     = bool((self.value >> 9) & 1)
        self.button_X_     = bool((self.value >> 10) & 1)
        self.button_Y_     = bool((self.value >> 11) & 1)
        self.button_UP_    = bool((self.value >> 12) & 1)
        self.button_RIGHT_ = bool((self.value >> 13) & 1)
        self.button_DOWN_  = bool((self.value >> 14) & 1)
        self.button_LEFT_  = bool((self.value >> 15) & 1)


class DDSJoystickReader:
    """
    从 DDS 通讯中读取手柄按钮状态的类
    
    支持从以下方式读取：
    1. 从 UpperState_ 或 LowerState_ DDS 消息中读取
    2. 从 Control_sim 实例中读取 upper_msg 或 lower_msg
    3. 从包含 robot 属性的对象中读取（如 UpperControl 实例）
    
    使用示例:
        # 方式1: 从 DDS 消息直接读取
        from dobot_atom.msg import dds_
        upper_state = ...  # UpperState_ 消息
        reader = DDSJoystickReader()
        button_state = reader.from_dds_message(upper_state)
        
        # 方式2: 从 Control_sim 实例读取
        from robot_control_dds.control_sim import Control_sim
        control = Control_sim()
        button_state = reader.from_control_sim(control)
        
        # 方式3: 从 UpperControl 实例读取
        from robot_upper_control import UpperControl
        upper_control = UpperControl()
        button_state = reader.from_upper_control(upper_control)
        
        # 检查按钮X是否被按下
        if button_state and button_state.button_X_:
            print("按钮X被按下")
    """
    
    @staticmethod
    def _extract_wireless_remote(dds_message):
        """
        从 DDS 消息中提取 wireless_remote 数据
        
        Args:
            dds_message: UpperState_ 或 LowerState_ DDS 消息对象
            
        Returns:
            list: wireless_remote 列表，如果无法提取则返回 None
        """
        if dds_message is None:
            return None
        
        try:
            wireless_remote_attr = getattr(dds_message, "wireless_remote", None)
            if wireless_remote_attr is not None:
                # 将 DDS array 类型转换为 Python list
                try:
                    wireless_remote = list(wireless_remote_attr)
                    if len(wireless_remote) > 0:
                        return wireless_remote
                except (TypeError, ValueError):
                    pass
        except AttributeError:
            pass
        
        return None
    
    @staticmethod
    def from_dds_message(dds_message, prefer_upper=True):
        """
        从 DDS 消息中读取手柄按钮状态
        
        Args:
            dds_message: UpperState_ 或 LowerState_ DDS 消息对象
            prefer_upper: 如果传入的是包含 upper_msg 和 lower_msg 的对象，是否优先使用 upper_msg
            
        Returns:
            JoystickButtonState: 手柄按钮状态对象，如果无法读取则返回 None
        """
        # 如果传入的是包含 upper_msg 和 lower_msg 的对象（如 Control_sim）
        if hasattr(dds_message, "upper_msg") and hasattr(dds_message, "lower_msg"):
            if prefer_upper:
                # 优先尝试 upper_msg
                wireless_remote = DDSJoystickReader._extract_wireless_remote(dds_message.upper_msg)
                if wireless_remote:
                    return JoystickButtonState(wireless_remote)
                # 如果 upper_msg 没有数据，尝试 lower_msg
                wireless_remote = DDSJoystickReader._extract_wireless_remote(dds_message.lower_msg)
                if wireless_remote:
                    return JoystickButtonState(wireless_remote)
            else:
                # 优先尝试 lower_msg
                wireless_remote = DDSJoystickReader._extract_wireless_remote(dds_message.lower_msg)
                if wireless_remote:
                    return JoystickButtonState(wireless_remote)
                # 如果 lower_msg 没有数据，尝试 upper_msg
                wireless_remote = DDSJoystickReader._extract_wireless_remote(dds_message.upper_msg)
                if wireless_remote:
                    return JoystickButtonState(wireless_remote)
        else:
            # 直接是 DDS 消息对象
            wireless_remote = DDSJoystickReader._extract_wireless_remote(dds_message)
            if wireless_remote:
                return JoystickButtonState(wireless_remote)
        
        return None
    
    @staticmethod
    def from_control_sim(control_sim, prefer_upper=True):
        """
        从 Control_sim 实例中读取手柄按钮状态
        
        Args:
            control_sim: Control_sim 实例
            prefer_upper: 是否优先使用 upper_msg
            
        Returns:
            JoystickButtonState: 手柄按钮状态对象，如果无法读取则返回 None
        """
        if control_sim is None:
            return None
        
        return DDSJoystickReader.from_dds_message(control_sim, prefer_upper=prefer_upper)
    
    @staticmethod
    def from_upper_control(upper_control, prefer_upper=True):
        """
        从 UpperControl 实例中读取手柄按钮状态
        
        Args:
            upper_control: UpperControl 实例（包含 robot 属性，robot 是 Control_sim 实例）
            prefer_upper: 是否优先使用 upper_msg
            
        Returns:
            JoystickButtonState: 手柄按钮状态对象，如果无法读取则返回 None
        """
        if upper_control is None:
            return None
        
        if not hasattr(upper_control, "robot"):
            return None
        
        return DDSJoystickReader.from_control_sim(upper_control.robot, prefer_upper=prefer_upper)
    
    @staticmethod
    def get_button_x_state(source, prefer_upper=True):
        """
        便捷方法：获取按钮X的状态
        
        Args:
            source: 可以是 DDS 消息、Control_sim 实例或 UpperControl 实例
            prefer_upper: 是否优先使用 upper_msg
            
        Returns:
            bool: True=按钮X被按下，False=按钮X未按下或无法获取状态
        """
        button_state = DDSJoystickReader.from_dds_message(source, prefer_upper=prefer_upper)
        if button_state is not None and hasattr(button_state, 'button_X_'):
            return button_state.button_X_ == 1
        return False
    
    @staticmethod
    def get_button_A_state(source, prefer_upper=True):
        """
        便捷方法：获取按钮X的状态
        
        Args:
            source: 可以是 DDS 消息、Control_sim 实例或 UpperControl 实例
            prefer_upper: 是否优先使用 upper_msg
            
        Returns:
            bool: True=按钮X被按下，False=按钮X未按下或无法获取状态
        """
        button_state = DDSJoystickReader.from_dds_message(source, prefer_upper=prefer_upper)
        if button_state is not None and hasattr(button_state, 'button_X_'):
            return button_state.button_A_ == 1
        return False
