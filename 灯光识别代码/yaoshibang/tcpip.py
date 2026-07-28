import socket
import time

# 服务端配置
TCP_HOST = '127.0.0.1'  # 改为服务端实际IP
TCP_PORT = 65432
INITIAL_MESSAGE = 'start'  # 可改为 'startr'

def send_start_signal():
    client_socket = None
    try:
        # 1. 创建 socket 并连接
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((TCP_HOST, TCP_PORT))
        print(f"✅ 已连接到服务端 {TCP_HOST}:{TCP_PORT}")

        # 2. 延时 1 秒
        print("⏳ 延迟 1 秒后发送初始信号...")
        time.sleep(1)

        # 3. 发送初始 start 信号
        client_socket.sendall(INITIAL_MESSAGE.encode('utf-8'))
        print(f" 已发送初始信号: {INITIAL_MESSAGE}")
        # time.sleep(20)
        # 4. 进入自动循环
        while True:
            try:
                # 等待接收服务端消息
                response = client_socket.recv(4096).decode('utf-8').strip()
                if not response:
                    print("⚠️ 服务端关闭连接或返回空消息")
                    break
                print(f" 服务端响应: {response}")
                if(("right" in response) or ("left" in response)):

                    # 延时 10 秒
                    print("⏳ 延迟 10 秒后回复 resultOK...")
                    time.sleep(10)

                    # 回复 resultOK
                    client_socket.sendall("resultOK".encode('utf-8'))
                    print(" 已发送: resultOK")
                elif "readyOK" in response:
                    # 延时 10 秒
                    print("⏳ 延迟 10 秒后回复 resultOK...")
                    time.sleep(10)

                    # 回复 resultOK
                    client_socket.sendall("resultOKresultOK".encode('utf-8'))
                    print(" 已发送: resultOK")
                elif "NG" in response:
                    print("not")
                    client_socket.sendall("resultOK".encode('utf-8'))
            except socket.timeout:
                print("⚠️ 接收超时，重试中...")
                continue
            except ConnectionResetError:
                print("❌ 服务端强制关闭了连接")
                break
            except Exception as e:
                print(f"❌ 接收/发送失败: {e}")
                break

    except ConnectionRefusedError:
        print("❌ 连接失败：请确保服务端已启动！")
    except Exception as e:
        print(f"❌ 连接异常: {e}")
    finally:
        if client_socket:
            client_socket.close()
            print(" 连接已关闭")

if __name__ == "__main__":
    send_start_signal()