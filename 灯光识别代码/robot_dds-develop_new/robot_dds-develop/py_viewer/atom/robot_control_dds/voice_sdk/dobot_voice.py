import json
import socket
import os
import time
try:
    from .udp_wav import UdpAudioClient
    from .utils import log_record
except ImportError:
    # 如果相对导入失败（直接运行脚本时），使用绝对导入
    from udp_wav import UdpAudioClient
    from utils import log_record



class RpcClient:
    def __init__(self):
        self.ip = "192.168.8.234"
        self.port = 51235
        self.socket = None
        self.file_path = os.path.dirname(__file__)
        # wav udp
        self.wav_send = UdpAudioClient()

    def connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(25)
        try:
            self.socket.connect((self.ip, self.port))
            return 1
        except socket.error as e:
            print(f"Connect failed: {e}")
            return 0

    def close(self):
        self.socket.close()

    def call_method(self, method, params):
        request_send = {
            "jsonrpc": "2.0",
            "method": method,
            "params": params,
            "id": 1
        }
        # print(request_send)
        request_str = json.dumps(request_send)

        self.socket.send(str.encode(request_str, 'utf-8'))
        try:
            response_data = self.socket.recv(4096)
            if not response_data:
                return -1, "no response"
        except socket.timeout:
            return -1, "timeout"

        response_data = json.loads(response_data.decode())
        return 0, response_data

    # 获取麦克风状态
    def get_dev_state(self):
        self.connect()
        return self.call_method("GetDevState", {})

    # 设置音量大小
    def set_volume(self, vol_value):
        self.connect()
        return self.call_method("SetVolume", {"volume": vol_value})

    # 获取音量大小
    def get_volume(self):
        self.connect()
        return self.call_method("GetVolume", {})

    # 播放本地音频
    def play_audio(self, wav_path):
        self.connect()
        self.call_method("PlayAudio", {})
        self.wav_send.send_wav(wav_path)
        while 1:
            time.sleep(0.02)
            _, rt_json = self.get_dev_state()
            if rt_json["result"]==4:
                time.sleep(0.02)
                break
        return 1

    # 停止播放音频
    def stop_audio(self):
        self.connect()
        return self.call_method("StopAudio", {})

    # 暂停播放
    def pause_audio(self):
        self.connect()
        return self.call_method("PauseAudio", {})

    # 继续播放
    def continue_audio(self):
        self.connect()
        return self.call_method("ContinueAudio", {})

    # 开启关闭pc2 语音克隆服务
    def pc2_switch_tts(self, flag):
        self.connect()
        return self.call_method("Pc2SwitchTTS", { "switch": flag })

    # 开启状态下，调用语音克隆服务
    def pc2_play_tts(self, text):
        self.connect()
        return self.call_method("Pc2PlayTTS", {"text": text})

    # 开启关闭pc2 语音识别服务
    def pc2_switch_asr(self, flag):
        self.connect()
        return self.call_method("Pc2SwitchASR", {"switch": flag})

    # 开启状态下，调用语音识别服务
    def pc2_play_asr(self, language, timeout):
        self.connect()
        return self.call_method("Pc2PlayASR", {"language": language, "timeout": timeout})

    # 调用pc2 dify ai平台服务
    def pc2_play_dify(self, ques):
        self.connect()
        return self.call_method("Pc2PlayDify", { "ques": ques})

    # 开启麦克风组播服务
    def pc1_mic_server(self, flag):
        self.connect()
        return self.call_method("Pc1MicServer", {"switch": flag})

    # pc1唤醒
    def pc1_mic_ivw(self, timeout):
        self.connect()
        return self.call_method("Pc1MicIVW", {"timeout": timeout})

    # pc1命令词识别
    def pc1_mic_esr(self, timeout):
        self.connect()
        return self.call_method("Pc1MicESR", {"timeout": timeout})

    # pc1语音合成
    def pc1_mic_tts(self, text):
        self.connect()
        return self.call_method("Pc1MicTTS", {"wavPath": "1.wav", "text": text})



# /dobot/logServer/bin/port 65521
if __name__ == "__main__":
    # rpc
    client = RpcClient()
    client.set_volume(90)
    # print(client.stop_audio())
    print(client.play_audio("demo.wav"))
    # print(client.stop_audio())
    # client.play_audio("/home/dobotpc2/Documents/robot_dds-develop_20251208/robot_dds-develop/py_viewer/atom/robot_control_dds/voice_sdk/yinpin/灯光处于打开状态,执行关闭动作.wav")
    # client.play_audio("/home/dobotpc2/Documents/robot_dds-develop_20251208/robot_dds-develop/py_viewer/atom/robot_control_dds/voice_sdk/yinpin/楼宇巡检任务结束.wav")
    # print(client.pc2_play_dify("你是谁？"))
    # print(client.pc2_switch_asr(True))
    # print(client.pc2_play_asr("ch", 10))
    # print(client.pc1_mic_ivw(3))
    client.pc1_mic_tts("你好，我是越疆科技开发的人形机器人Atom")
    # log_record(__file__, "111")
    # client.play_audio("data/merged1.wav")
    # log_record(__file__, "222")
    # print(client.pc2_switch_asr(True))
    # print(client.pc2_switch_tts(True))
    # print(client.pc1_mic_server(True))
    # print("tcp connect: okkkkkkkkkkk!!!")
    # _, rt_json = client.pc2_play_asr("cn",1)
    # # _, rt_json = client.pc1_mic_ivw(51)
    # print(rt_json["result"])
    # if rt_json["result"]=="Function timeout":
    #     print(111)
    # while 1:
    #     print(client.pc1_mic_ivw(15))
    #     print(client.pc2_play_tts("您好，我在，请问有什么可以帮到您？"))