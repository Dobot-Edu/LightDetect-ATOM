import time
import sys
import os

# 添加路径以便导入 dobot_voice 模块
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'robot_control_dds', 'voice_sdk'))
from dobot_voice import RpcClient

# 导入手柄相关模块
sys.path.append(os.path.join(os.path.dirname(__file__), 'atom'))
from atom.robot_control_dds.Joystick.JoystickState import JoystickButtonState
from atom.robot_upper_control import UpperControl


# ========== 手柄按钮状态获取函数 ==========
def get_button_state(real_robot):
    """
    获取无线手柄按钮状态
    
    参数:
        real_robot: UpperControl 实例
    
    返回:
        JoystickButtonState: 手柄按钮状态对象，如果无法获取则返回 None
    """
    if real_robot is None:
        return None
    
    # 检查 real_robot 是否已初始化
    if not hasattr(real_robot, "robot") or real_robot.robot == 0:
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
    except Exception as e:
        # 静默处理错误，避免频繁打印
        pass
    
    return None


# ========== 语义判断辅助函数 ==========
def check_authorization_semantic(text):
    """
    判断用户指令的语义（确定/否定/不确定）
    
    参数:
        text: 用户语音识别的文本
    
    返回:
        "confirm": 确定/授权
        "deny": 否定/拒绝
        "unknown": 不确定/无法识别
    """
    if not text:
        return "unknown"
    
    text = text.strip()
    
    # 确定/授权的关键词
    confirm_keywords = [
        "是", "是的", "可以", "好的", "行", "没问题", "确定", "确认",
        "请开始", "开始", "开始执行", "开始救援", "开始救援任务",
        "授权", "授权开始", "授权执行", "授权开始执行",
        "开始吧", "行动", "执行", "立即", "马上", "立刻",
        "同意", "批准", "准许", "允许"
    ]
    
    # 否定/拒绝的关键词
    deny_keywords = [
        "不", "不是", "不可以", "不行", "不要", "不用",
        "不执行", "不授权", "不开始", "不行动",
        "拒绝", "取消", "停止", "暂停", "等等"
    ]
    
    # 检查确定语义
    for keyword in confirm_keywords:
        if keyword in text:
            return "confirm"
    
    # 检查否定语义
    for keyword in deny_keywords:
        if keyword in text:
            return "deny"
    
    return "unknown"


# ========== 救援任务语音交互场景 ==========
def rescue_mission_scenario():
    """
    救援任务语音交互场景
    
    流程：
    1. 播放背景介绍
    2. 等待观众语音指令
    3. 识别指令语义（确定/否定）
    4. 根据语义执行相应动作
    """
    print("救援任务语音交互场景")
    
    # 1. 初始化RPC客户端
    client = RpcClient()
    print("✓ RPC客户端初始化完成")
    
    # 1.5. 初始化手柄控制对象（用于获取手柄按钮状态）
    try:
        real_robot = UpperControl()
        print("✓ 手柄控制对象初始化完成")
    except Exception as e:
        print(f"⚠ 手柄控制对象初始化失败: {e}，将仅使用语音识别")
        real_robot = None
    
    # 2. 开启必要的服务
    print("\n2. 开启语音服务...")
    # 开启TTS服务（语音合成）
    code, result = client.pc1_mic_server(True)  
    code, result = client.pc2_switch_tts(True)  
    code, result = client.pc2_switch_asr(True)
    
    # 设置麦克风音量（0-100，设置为30以降低音量）
    volume_level = 10
    code, result = client.set_volume(volume_level)
    if code == 0:
        print(f"✓ 麦克风音量已设置为: {volume_level}%")
    time.sleep(1)  # 等待服务初始化完成
    
    try:
        # 3. 播放背景介绍
        background_text = "情况紧急，我们需要运送能量模块和应急物资到受灾区域。指挥权交给你——是否授权开始救援行动？"  
        status, _ = client.pc2_play_tts(background_text)
        print("\n【背景介绍播放完毕，现场互动环节启动，等待观众下发指令】\n")
        
        # 4. 并行等待观众语音指令或手柄按钮（最多尝试3次）
        max_attempts = 3
        for attempt in range(max_attempts):
            print("正在并行聆听观众指令和等待手柄按钮...")
            
            # 共享变量：用于线程间传递检测结果
            result_dict = {'button': None, 'semantic': None, 'voice_result': None, 'voice_status': None}
            stop_event = threading.Event()
            
            # 启动手柄检测线程（并行检测）
            joystick_thread = None
            if real_robot is not None:
                joystick_thread = threading.Thread(
                    target=monitor_joystick_button,
                    args=(real_robot, result_dict, stop_event),
                    daemon=True
                )
                joystick_thread.start()
                print("✓ 手柄检测线程已启动（并行模式）")
            
            # 主线程进行语音识别（可能阻塞）
            # 注意：如果手柄检测线程已经检测到按钮，stop_event会被设置
            print("开始语音识别...")
            try:
                status, rt_json = client.pc2_play_asr(language="ch", timeout=10)
                result_dict['voice_status'] = status
                result_dict['voice_result'] = rt_json
                print(f"语音识别返回: {rt_json}")
            except Exception as e:
                print(f"语音识别异常: {e}")
                result_dict['voice_status'] = -1
                result_dict['voice_result'] = None
            
            # 停止手柄检测线程
            stop_event.set()
            if joystick_thread is not None:
                joystick_thread.join(timeout=0.5)
            
            # 如果手柄检测线程已经检测到按钮，立即处理（提前响应）
            if result_dict.get('button') is not None:
                print(f"✓ [提前响应] 检测到手柄按钮: {result_dict.get('button')}")
            
            # ========== 并行检测结果处理（或逻辑：任意一方触发即执行） ==========
            
            # 检查手柄按钮结果（优先级：如果手柄已触发，优先处理）
            button_pressed = result_dict.get('button')
            button_semantic = result_dict.get('semantic')
            
            # 检查语音识别结果
            voice_status = result_dict.get('voice_status')
            voice_result = result_dict.get('voice_result')
            user_command = ""
            voice_semantic = "unknown"
            
            if voice_status == 0 and voice_result:
                user_command = voice_result.get("result", "")
                if user_command and user_command != "Function timeout":
                    print(f"✓ 识别到语音指令: {user_command}")
                    voice_semantic = check_authorization_semantic(user_command)
                    print(f"语音语义判断结果: {voice_semantic}")
            
            # ========== 或逻辑判断：任意一方确认即执行 ==========
            
            # 情况1: 手柄A按钮 或 语音确认 -> 执行救援任务
            if button_pressed == 'A' or voice_semantic == 'confirm':
                trigger_source = []
                if button_pressed == 'A':
                    trigger_source.append("手柄A按钮")
                if voice_semantic == 'confirm':
                    trigger_source.append("语音确认")
                
                source_str = "和".join(trigger_source)
                print(f"\n✓ 观众授权确认（{source_str}），启动救援任务")
                start_text = "紧急救援任务已启动，正在收集物资并送往灾区。"             
                status, _ = client.pc2_play_tts(start_text)
                if status == 0:
                    print("✓ 救援任务已启动")
                else:
                    print("启动台词播放失败")
                
                # 这里可以添加实际的救援任务执行逻辑
                # 例如：控制机器人移动、抓取物品等
                print("\n【救援任务执行中...】")
                return  # 任务启动成功，退出函数
            
            # 情况2: 手柄B按钮 或 语音拒绝 -> 拒绝并重新确认
            elif button_pressed == 'B' or voice_semantic == 'deny':
                trigger_source = []
                if button_pressed == 'B':
                    trigger_source.append("手柄B按钮")
                if voice_semantic == 'deny':
                    trigger_source.append("语音拒绝")
                
                source_str = "和".join(trigger_source)
                print(f"\n✗ 观众拒绝授权（{source_str}）")
                if attempt < max_attempts - 1:
                    client.pc2_play_tts("已收到拒绝指令，请重新确认是否授权开始救援行动")
                continue
            
            # 情况3: 语音识别超时或失败，且手柄未按下
            elif voice_status != 0 or (voice_result and voice_result.get("result") == "Function timeout"):
                print("语音识别超时或失败，未检测到有效指令")
                if attempt < max_attempts - 1:
                    client.pc2_play_tts("抱歉，我没有听到您的指令，请再次确认是否立即启动任务，或使用手柄按钮确认")
                continue
            
            # 情况4: 语音识别成功但语义不确定，且手柄未按下
            else:
                print("\n? 无法确定语义，请求重新确认")
                if attempt < max_attempts - 1:
                    client.pc2_play_tts("抱歉，无法识别您的指令，请使用语音或手柄按钮确认")
                continue
        
        # 如果所有尝试都失败
        print("多次尝试后仍无法获取有效指令，任务启动失败")
        client.pc2_play_tts("多次尝试后仍无法获取有效指令，任务启动失败")
        
    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理：关闭服务
        print("\n正在关闭服务...")
        client.pc2_switch_tts(False)
        client.pc2_switch_asr(False)
        print("服务已关闭，程序退出")


if __name__ == "__main__":
    # 运行救援任务语音交互场景
    rescue_mission_scenario()

