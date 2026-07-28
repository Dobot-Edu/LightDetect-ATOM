import cv2
import time
from ultralytics import YOLO
import random
import pyrealsense2 as rs
import numpy as np
import socket
import threading


# -----------------------------
# 工具函数：读取坐标文件并计算仿射矩阵
# -----------------------------
def read_2d_points_from_file(file_path):
    points = []
    try:
        with open(file_path, 'r') as file:
            for i, line in enumerate(file):
                try:
                    x, y = map(float, line.strip().split())
                    points.append([x, y])
                except ValueError:
                    print(f"文件 '{file_path}' 第 {i + 1} 行格式错误，跳过。")
        return np.array(points, dtype=np.float32)
    except FileNotFoundError:
        print(f"文件未找到: {file_path}")
        return None


def estimate_affine_2d(src_points, dst_points):
    if src_points is None or dst_points is None or len(src_points) < 3 or len(src_points) != len(dst_points):
        print("仿射变换估计失败：点数不足或文件为空")
        return None
    matrix, _ = cv2.estimateAffine2D(src_points, dst_points)
    return matrix


# -----------------------------
# 预加载仿射矩阵
# -----------------------------
print("正在加载左右手仿射变换矩阵...")

src_left = read_2d_points_from_file('camPosLeft.txt')  # yellow → 左手
dst_left = read_2d_points_from_file('robPosLeft.txt')
matrix_left = estimate_affine_2d(src_left, dst_left)

src_right = read_2d_points_from_file('camPosRigth.txt')  # green → 右手
dst_right = read_2d_points_from_file('robPosRight.txt')
matrix_right = estimate_affine_2d(src_right, dst_right)

if matrix_left is None:
    print("❌ 错误: 左手仿射矩阵加载失败")
    exit(1)
if matrix_right is None:
    print("❌ 错误: 右手仿射矩阵加载失败")
    exit(1)

print("✅ 仿射矩阵加载成功")

# -----------------------------
# 模型加载
# -----------------------------
# model = YOLO('/home/dobotpc2/Documents/visionDetect/runs/train/exp4/weights/best_gy2.pt')
model = YOLO('/home/dobotpc2/Downloads/yaoshibang/runs/best_yaoshibang.pt')

num_classes = len(model.names)
colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for _ in range(num_classes)]

try:
    green_id = list(model.names.values()).index('Vita')
    yellow_id = list(model.names.values()).index('Aliens')
    print(green_id,"===11111green")
    print(yellow_id, "===11111white")
except ValueError:
    print("模型中未找到 'green' 或 'yellow' 类别")
    exit(1)

TARGET_SET = {green_id, yellow_id}
print(TARGET_SET, "===11111green")


# -----------------------------
# RealSense 相机设置
# -----------------------------
SECOND_CAMERA_SERIAL = "352222304906"
pipeline = rs.pipeline()
config = rs.config()

ctx = rs.context()
devices = ctx.query_devices()
if not any(dev.get_info(rs.camera_info.serial_number) == SECOND_CAMERA_SERIAL for dev in devices):
    print(f"未找到相机: {SECOND_CAMERA_SERIAL}")
    exit(1)

config.enable_device(SECOND_CAMERA_SERIAL)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# -----------------------------
# TCP 服务端设置
# -----------------------------
TCP_HOST = '127.0.0.1'
TCP_PORT = 65432

client_socket = None
client_lock = threading.Lock()

# 状态定义
STATE_WAITING_CMD = 'WAITING_CMD'
STATE_DETECTING = 'DETECTING'
STATE_WAITING_RESULT_OK = 'WAITING_RESULT_OK'

current_state = STATE_WAITING_CMD

# 延迟与缓存变量
delay_start_time = None
delay_duration = 5
last_valid_yellow_result = None
last_valid_green_result = None
image_counter = 0


def reset_detection_state():
    """重置所有检测状态，用于新连接或重启"""
    global delay_start_time, last_valid_yellow_result, last_valid_green_result
    delay_start_time = None
    last_valid_yellow_result = None
    last_valid_green_result = None


def handle_new_client_connection():
    """客户端连接时初始化状态"""
    global current_state
    with client_lock:

        current_state = STATE_WAITING_CMD
        reset_detection_state()
    print("🔄 已重置状态，等待 'start' 指令...")


def tcp_server():
    global client_socket
    global current_state

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((TCP_HOST, TCP_PORT))
    server_socket.listen(1)
    print(f"TCP服务端已启动，监听 {TCP_HOST}:{TCP_PORT}...")

    while True:
        conn, addr = server_socket.accept()
        print(f"客户端 {addr} 已连接")

        # 重置状态，确保新连接从头开始
        handle_new_client_connection()

        with client_lock:
            client_socket = conn

        try:
            # client_socket.sendall(b"ACK:READY\n")
            print(b"ACK:READY\n")

            while True:
                data = client_socket.recv(1024).decode('utf-8').strip()
                if not data:
                    break  # 客户端断开

                print(f"收到指令: {data}")

                with client_lock:
                    state = current_state

                if state == STATE_WAITING_CMD:
                    if data.lower() == 'start':
                        with client_lock:
                            current_state = STATE_DETECTING
                        # client_socket.sendall(b"ACK:DETECTION_STARTED\n")
                        print("ACK:DETECTION_STARTED")
                        print("🟢 收到 'start'，进入检测状态")
                    # 忽略 resultOK

                elif state == STATE_WAITING_RESULT_OK:
                    if data.lower() == 'resultok':
                        with client_lock:
                            current_state = STATE_DETECTING
                        # client_socket.sendall(b"ACK:DETECTION_RESTARTED\n")
                        print("🟢 收到 'resultOK'，重新开始检测")
                    # 忽略 start

        except (ConnectionResetError, ConnectionAbortedError, OSError):
            pass
        finally:
            with client_lock:
                client_socket = None
            print("❌ 客户端已断开，等待新连接...")
            # 断开后自动重置，等待下一次连接


# 启动 TCP 服务线程
tcp_thread = threading.Thread(target=tcp_server, daemon=True)
tcp_thread.start()

# -----------------------------
# 主循环
# -----------------------------
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        display_image = color_image.copy()

        # 获取当前状态
        with client_lock:
            state = current_state
            current_delay_start = delay_start_time
            current_duration = delay_duration

        # 状态机处理
        if state == STATE_DETECTING:
            results = model.predict(source=color_image, save=False, verbose=False)
            result = results[0]
            boxes = result.boxes

            detected = {}
            for box in boxes:
                cls_id = int(box.cls.item())
                # print(cls_id,"====cls_id")
                conf = box.conf.item()
                # 计算边界框的左上角坐标
                x_center, y_center, width, height = box.xywh.squeeze().tolist()
                # x_min = int(x_center - width / 2)
                # y_min = int(y_center - height / 2)
                x_min = x_center
                y_min = y_center
                # print("xy:",x_min,y_min,"==========")
                if conf > 0.7 and cls_id in TARGET_SET and 170 < x_min < 510 and y_min > 250:
                # if conf > 0.7 and cls_id in TARGET_SET:
                    detected[cls_id] = (box, conf)
                    # print("////////",detected[cls_id],"--------")

            green_detected = green_id in detected
            yellow_detected = yellow_id in detected

            # 情况1：尚未开始延迟
            if current_delay_start is None:
                if (green_detected and yellow_detected) or yellow_detected:
                    print("✅ 同时检测到 green 和 yellow，启动 10 秒延迟")
                    with client_lock:
                        delay_start_time = time.time()
                        delay_duration = 3.8
                        last_valid_yellow_result = None
                        last_valid_green_result = None
                    # 保存图像
                    # output_filename = f"trigger_result_{image_counter}.jpg"
                    # cv2.imwrite(output_filename, color_image)
                    # print(f"📷 已保存图像: {output_filename}")
                    # image_counter += 1

                elif green_detected and not yellow_detected:
                    print("🟢 检测到 green（无 yellow），启动 5 秒延迟")
                    with client_lock:
                        delay_start_time = time.time()
                        delay_duration = 2
                        last_valid_yellow_result = None
                        last_valid_green_result = None
                    # output_filename = f"trigger_result_{image_counter}.jpg"
                    # cv2.imwrite(output_filename, color_image)
                    # image_counter += 1

            # 情况2：正在延迟中
            elif current_delay_start is not None:
                elapsed = time.time() - current_delay_start
                remaining = max(0, current_duration - elapsed)
                cv2.putText(display_image, f"DELAY: {remaining:.1f}s", (50, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)

                # 更新最后有效位置
                if current_duration == 3.8 and ((green_detected and yellow_detected) or yellow_detected) :
                    box, score = detected[yellow_id]
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    center_x = int((xyxy[0] + xyxy[2]) / 2)
                    center_y = int((xyxy[1] + xyxy[3]) / 2)
                    # print("center:",center_x,center_y)
                    pixel_coords = np.array([center_x, center_y, 1], dtype=np.float32)
                    if center_x>340:
                        right_or_left="right"
                        pixel_coords = np.array([center_x, center_y, 1], dtype=np.float32)
                        x_robot, y_robot = matrix_right @ pixel_coords
                        x_robot -= 0
                        y_robot -= 50
                    else:
                        right_or_left = "left"
                        x_robot, y_robot = matrix_left @ pixel_coords
                        x_robot += 0
                        y_robot += 32
                    # result_str = f"yellow,{score:.2f},{x_robot:.2f},{y_robot:.2f}"
                    result_str = f"{right_or_left},{x_robot:.2f},{y_robot:.2f}"
                    with client_lock:
                        last_valid_yellow_result = result_str
                    # print(f"🔄 更新最后有效 yellow 位置: {result_str}")

                elif current_duration == 2 and green_detected:
                    box, score = detected[green_id]
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    center_x = int((xyxy[0] + xyxy[2]) / 2)
                    center_y = int((xyxy[1] + xyxy[3]) / 2)
                    pixel_coords = np.array([center_x, center_y, 1], dtype=np.float32)
                    x_robot, y_robot = matrix_right @ pixel_coords
                    x_robot -= 34
                    y_robot -= 30
                    # result_str = f"green,{score:.2f},{x_robot:.2f},{y_robot:.2f}"
                    result_str = f"green,{x_robot:.2f},{y_robot:.2f}"
                    with client_lock:
                        last_valid_green_result = result_str
                    # print(f"🔄 更新最后有效 green 位置: {result_str}")

                # 延迟结束
                if elapsed >= current_duration:
                    result_to_send = None
                    msg_sent = False

                    if current_duration == 3.8:
                        with client_lock:
                            result_to_send = last_valid_yellow_result
                        if result_to_send and client_socket:
                            msg = f"RESULT:{result_to_send}\n"
                            try:
                                client_socket.sendall(msg.encode('utf-8'))
                                print(f"📤 发送 yellow 位置: {msg.strip()}")
                                msg_sent = True
                            except:
                                print("❌ 发送失败")

                    elif current_duration == 2:
                        with client_lock:
                            result_to_send = last_valid_green_result
                        if result_to_send and client_socket:
                            msg = f"RESULT:{result_to_send}\n"
                            try:
                                client_socket.sendall(msg.encode('utf-8'))
                                print(f"📤 发送 green 位置: {msg.strip()}")
                                msg_sent = True
                            except:
                                print("❌ 发送失败")

                    if not result_to_send:
                        print("⚠️ 警告：延迟期间未检测到有效目标")
                        msg = f"RESULT:NG"
                        client_socket.sendall(msg.encode('utf-8'))

                    # 进入等待 resultOK 状态
                    with client_lock:
                        current_state = STATE_WAITING_RESULT_OK
                        delay_start_time = None
                        last_valid_yellow_result = None
                        last_valid_green_result = None

        elif state == STATE_WAITING_RESULT_OK:
            cv2.putText(display_image, "WAITING resultOK...", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        else:  # STATE_WAITING_CMD
            cv2.putText(display_image, "WAITING start...", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 100, 100), 2)

        # 绘制检测框
        if state == STATE_DETECTING:
            for box in result.boxes:
                if box.conf.item() > 0.7:
                    cls_id = int(box.cls.item())
                    name = result.names[cls_id]
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    color = colors[cls_id]
                    cv2.rectangle(display_image, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), color, 2)
                    cv2.putText(display_image, f"{name}:{box.conf.item():.2f}", (xyxy[0], xyxy[1] - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow('YOLOv8 Detection', display_image)
        cv2.waitKey(5)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    if client_socket:
        client_socket.close()