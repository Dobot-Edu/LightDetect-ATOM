import requests
import json
import os
import socket
import struct
from threading import Event, Lock, Thread
import  time

from sympy import false


class ASR:
    def __init__(self):
        self.asr_http_service = f"http://192.168.8.13:5077/asr-service/"
        self.sock_mul=None

    def ping(self):
        self.sock_mul = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock_mul.bind(('', 51237))
        mreq = struct.pack('4sl', socket.inet_aton('224.0.0.1'), socket.INADDR_ANY)
        self.sock_mul.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    def forward(self, button_state):
        got_angle = False
        got_mic = False
        headers = {}
        print("button_state: ", button_state , got_mic, got_angle)

        while (not got_angle) or (not got_mic):
            try:
                rt_data, addr = self.sock_mul.recvfrom(6400)
                if len(rt_data) > 10 and not got_mic:
                    got_angle = True
                    files = [
                        (
                            "audio",
                            (
                                os.path.basename("asr_example_zh.wav"),
                                rt_data,
                                "application/octet-stream",
                            ),
                        )
                    ]
                    data = {
                        "lang": "zn",
                        "hot_words": "越疆科技",
                        "button_state": int(button_state)
                    }
                    print("http asr: ", data)
                    rt_ = requests.post(self.asr_http_service, headers=headers, files=files, data=data)
                else:
                    got_mic = True
                    print("angle: ", rt_data.decode("utf-8"))
            except Exception as e:
                return -1, e
        if button_state==0:
            self.sock_mul.close()
        return json.loads(rt_.text)["code"], json.loads(rt_.text)["msg"]


if __name__ == "__main__":
    aaa = ASR()
    aaa.ping()
    while 1:
        aaa.forward(1)