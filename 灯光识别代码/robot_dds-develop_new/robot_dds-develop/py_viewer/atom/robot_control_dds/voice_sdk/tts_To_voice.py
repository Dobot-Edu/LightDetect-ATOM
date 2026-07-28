import requests
import wave

class LocalTTS():
    """
    1. run in pc2
    2. mic plugged in
    """
    def __init__(self):
        print("local tts service!")

    def switch(self, status):
        data = {"switch": status}
        response = requests.post('http://127.0.0.1:5078/tts-service-switch/',
                                 headers={'Content-Type': 'application/json'},
                                 json=data)
        print(response.content)
        return response.content
    
    def tts(self, str):
        data = {
            "text": str,
            "clone_text": "希望你以后能够做的比我还好。",
            "speed": 1,
            "sample_rate": 16000,
            "whose_voice": "man"}
        tts_audio = b""
        with requests.post('http://127.0.0.1:5079/tts-service/', json=data, stream=True) as response:
            if response.status_code == 200:
                for i, chunk in enumerate(response.iter_content(24000)):
                    print(i, len(chunk))
                    tts_audio += chunk
            else:
                print(f"请求失败，状态码: {response.text}")

        with wave.open("demo.wav", "wb") as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)  # int16 = 2 bytes
            wav_file.setframerate(24000)
            wav_file.writeframes(tts_audio)

if __name__ == "__main__":
    # asr = LocalASR()
    # asr.switch(True)
    # print("asr result: ", asr.asr())
    

    tts = LocalTTS()
    tts.switch(True)
    tts.tts("机器人初始化完成")

    # 1.机器人初始化完成
    # 2.开始执行商业分拣任务
    # 3.商业分拣任务完成。
    # 4.开始执行楼宇巡检任务
    # 5.灯光处于打开状态,执行关闭动作
    # 6.灯光处于关闭状态
    # 7.楼宇巡检任务结束