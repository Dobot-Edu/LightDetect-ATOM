import socket
import struct
import torch
import numpy as np
import torchaudio
import os


def receive_multicast(group, port):
    # 创建 UDP 套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    # 绑定到组播地址和端口
    sock.bind(('', port))

    # 加入组播组
    mreq = struct.pack('4sl', socket.inet_aton(group), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    print('接收组播消息中...')
    tts_audio = b""
    i = 0
    while 1:
        i+=1
        data, addr = sock.recvfrom(6400)
        if len(data)>50:
            tts_audio += data
        print(len(data))
        if i>80:
            break
    tts_speech = torch.from_numpy(np.array(np.frombuffer(tts_audio, dtype=np.int16))).unsqueeze(dim=0)
    torchaudio.save("demo.wav", tts_speech, 16000)

if __name__ == "__main__":
    #  sudo route add -net 0.0.0.0 netmask 0.0.0.0 gw 192.168.243.253 dev enp111s0
    # os.system("echo 123 | sudo -S route add -net 0.0.0.0 netmask 0.0.0.0 dev enp5s0")
    receive_multicast('224.0.0.1', 5000)
