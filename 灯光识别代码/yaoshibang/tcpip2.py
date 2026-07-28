import socket


def tcp_client():
    # 服务器配置
    TCP_HOST = '127.0.0.1'  # 服务端IP地址
    TCP_PORT = 65432  # 服务端端口号
    BUFFER_SIZE = 1024  # 缓冲区大小

    try:
        # 创建TCP套接字
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            # 连接服务器
            print(f"正在连接服务器 {TCP_HOST}:{TCP_PORT}...")
            client_socket.connect((TCP_HOST, TCP_PORT))
            print("连接服务器成功！")

            # 发送start信号
            start_signal = "start"
            client_socket.sendall(start_signal.encode('utf-8'))
            print(f"已发送: {start_signal}")

            # 等待服务器返回readyOK信号
            print("等待服务器响应...")
            response = client_socket.recv(BUFFER_SIZE).decode('utf-8').strip()

            if response == "readyOK":
                print(f"收到服务器响应: {response}")
                print("现在可以输入自定义信号发送（输入'quit'退出）")

                # 循环输入并发送自定义信号（带响应等待）
                while True:
                    custom_signal = input("\n请输入要发送的信号: ")

                    # 退出条件
                    if custom_signal.lower() == 'quit':
                        print("正在退出...")
                        break

                    # 发送自定义信号
                    client_socket.sendall(custom_signal.encode('utf-8'))
                    print(f"已发送自定义信号: {custom_signal}")

                    # 等待服务器对当前信号的响应
                    print("等待服务器响应...")
                    server_response = client_socket.recv(BUFFER_SIZE).decode('utf-8').strip()
                    print(f"收到服务器响应: {server_response}")

            else:
                print(f"收到意外响应: {response}，程序将退出")

    except ConnectionRefusedError:
        print("连接被拒绝，请检查服务器是否启动或配置是否正确")
    except ConnectionResetError:
        print("服务器主动断开了连接")
    except Exception as e:
        print(f"发生错误: {str(e)}")


if __name__ == "__main__":
    tcp_client()