import socket
import wave
import time


class UdpAudioClient:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = ("192.168.8.234", 51236)

    def send_start_load(self):
        try:
            self.sock.sendto(b'startloaddata', self.server_address)
            return True
        except:
            return False

    def send_audio_data(self, data):
        try:
            self.sock.sendto(data, self.server_address)
            return True
        except:
            return False

    def send_stop_load(self):
        try:
            self.sock.sendto(b'stoploaddata', self.server_address)
            return True
        except:
            return False

    def close(self):
        self.sock.close()


    def read_wave_file(self, file_path):
        try:
            with wave.open(file_path, 'rb') as wav_file:
                sample_rate = wav_file.getframerate()
                num_channels = wav_file.getnchannels()
                frames = wav_file.readframes(wav_file.getnframes())
                return (True, sample_rate, num_channels, frames)
        except:
            return (False, -1, 0, b'')


    def send_wav(self, wav_path):
        UDP_SERVER_IP = "192.168.8.234"
        UDP_SERVER_PORT = 51236
        CHUNK_SIZE = 1024

        filestate, sample_rate, num_channels, pcm_data = self.read_wave_file(wav_path)
        print("sample_rate: ", sample_rate)

        if filestate and num_channels == 1:

            if not self.send_start_load():
                print("[SendStartLoad] Failed to resume audio.")
                return 0

            offset = 0
            total_size = len(pcm_data)
            print("total_size: ", total_size)

            while offset < total_size:
                # print("sent...")
                chunk = pcm_data[offset:offset + CHUNK_SIZE]
                if not self.send_audio_data(chunk):
                    print("[SendAudioData] Failed to resume audio.")
                    break
                offset += len(chunk)
                time.sleep(0.001)  # 可选延迟

            if not self.send_stop_load():
                print("[SendStopLoad] Failed to resume audio.")

            # self.close()
            return 1
        return 0


if __name__ == "__main__":
    aaa = UdpAudioClient()
    aaa.send_wav("data/merged1.wav")
