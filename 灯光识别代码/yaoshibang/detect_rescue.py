import cv2
import time
from ultralytics import YOLO
import random
import pyrealsense2 as rs
import numpy as np
import socket
import threading


# -----------------------------
# 灯亮检测函数
# -----------------------------
def detect_lights(image, brightness_threshold=200, min_area=100):
    """检测图像中的亮灯区域"""
    # 转换为灰度图
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 高斯模糊减少噪声
    blurred = cv2.GaussianBlur(gray, (11, 11), 0)

    # 亮度阈值分割
    _, bright_regions = cv2.threshold(blurred, brightness_threshold, 255, cv2.THRESH_BINARY)

    # 形态学操作
    kernel = np.ones((5, 5), np.uint8)
    bright_regions = cv2.dilate(bright_regions, kernel, iterations=2)
    bright_regions = cv2.erode(bright_regions, kernel, iterations=1)

    # 检测轮廓
    contours, _ = cv2.findContours(bright_regions.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 筛选并标记亮灯区域
    result = image.copy()
    light_count = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(result, f"Light {light_count + 1}", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        light_count += 1

    print(f"检测到 {light_count} 个亮灯区域")
    return result, light_count


# -----------------------------
# 仿射矩阵相关函数
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
model = YOLO('./runs/best_rescue.pt')
# model = YOLO('/home/dobotpc2/Downloads/yaoshibang/runs/best_yaoshibang.pt')
num_classes = len(model.names)
colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for _ in range(num_classes)]

try:
    green_id = list(model.names.values()).index('bag')
    yellow_id = list(model.names.values()).index('stick')
    print(f"绿色目标ID: {green_id}, 黄色目标ID: {yellow_id}")
except ValueError:
    print("模型中未找到 'bag' 或 'stick' 类别")
    exit(1)

TARGET_SET = {green_id, yellow_id}

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
STATE_WAITING_START = 'WAITING_START'  # 初始状态，仅等待首次start
STATE_READY = 'READY'  # 就绪状态，可接收light和resultOK
STATE_DETECTING_ONCE = 'DETECTING_ONCE'
STATE_LIGHT_DETECTING = 'LIGHT_DETECTING'
current_state = STATE_WAITING_START


def tcp_server():
    global client_socket
    global current_state
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((TCP_HOST, TCP_PORT))
    server_socket.listen(1)
    print(f"TCP服务端启动，监听 {TCP_HOST}:{TCP_PORT}...")

    while True:
        conn, addr = server_socket.accept()
        print(f"客户端 {addr} 连接")
        with client_lock:
            client_socket = conn
            current_state = STATE_WAITING_START  # 新连接重置状态
        try:
            while True:
                data = client_socket.recv(1024).decode('utf-8').strip()
                if not data:
                    break  # 客户端断开

                print(f"收到指令: {data}")
                with client_lock:
                    state = current_state

                # 仅在等待状态处理start指令（首次有效）
                if state == STATE_WAITING_START and data.lower() == 'start':
                    with client_lock:
                        current_state = STATE_READY
                    try:
                        client_socket.sendall(b"readyOK\n")
                        print("📤 发送: readyOK（已准备好）")
                    except Exception as e:
                        print(f"发送readyOK失败: {e}")
                    continue

                # 就绪状态处理light指令
                if state == STATE_READY and data.lower() == 'light':
                    with client_lock:
                        current_state = STATE_LIGHT_DETECTING
                    print("🟢 进入灯检测状态")
                    continue

                # 就绪状态处理resultOK指令
                if state == STATE_READY and (data.lower() == 'resultok' or data.lower() == 'resultokresultok'):
                    with client_lock:
                        current_state = STATE_DETECTING_ONCE
                    print("🟢 进入目标检测状态")
                    continue

                # 忽略不符合状态的指令
                print(f"忽略指令 '{data}'（当前状态: {state}）")

        except (ConnectionResetError, OSError) as e:
            print(f"连接错误: {e}")
        finally:
            with client_lock:
                client_socket = None
                current_state = STATE_WAITING_START
            print("客户端断开，重置状态")


# 启动TCP服务线程
tcp_thread = threading.Thread(target=tcp_server, daemon=True)
tcp_thread.start()

# -----------------------------
# 主循环
# -----------------------------
pipeline.start(config)
try:
    while True:
        # 获取相机帧
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        color_image = np.asanyarray(color_frame.get_data())
        display_image = color_image.copy()

        # 获取当前状态
        with client_lock:
            state = current_state

        # 目标检测处理（resultOK触发）
        if state == STATE_DETECTING_ONCE:
            results = model.predict(source=color_image, save=False, verbose=False)
            result = results[0]
            boxes = result.boxes
            detected = []
            for box in boxes:
                cls_id = int(box.cls.item())
                conf = box.conf.item()
                x_center, y_center, width, height = box.xywh.squeeze().tolist()
                # 过滤低置信度和不在目标区域的目标
                if conf > 0.6 and cls_id in TARGET_SET and 200 < x_center < 480 and 170 < y_center < 340:
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    center_x = int((xyxy[0] + xyxy[2]) / 2)
                    center_y = int((xyxy[1] + xyxy[3]) / 2)
                    detected.append({
                        'cls_id': cls_id,
                        'conf': conf,
                        'center': (center_x, center_y),
                        'box': box
                    })

            # 处理检测结果
            if detected:
                first = detected[0]
                center_x, center_y = first['center']
                cls_id = first['cls_id']
                print(cls_id,"77777777777777777777777777777")
                # 计算机器人坐标
                pixel_coords = np.array([center_x, center_y, 1], dtype=np.float32)
                if center_x > 340:
                    right_or_left = "right"
                    x_robot, y_robot = matrix_right @ pixel_coords
                    if cls_id==0:
                        #0是bag
                        x_robot += 83.54
                        y_robot -= 84.85
                    elif cls_id==1:
                        # 1是stick
                        x_robot += 47.65
                        y_robot += (-54)
                else:
                    right_or_left = "left"
                    x_robot, y_robot = matrix_left @ pixel_coords
                    if cls_id==0:
                        # 0是bag
                        x_robot += 51.43
                        y_robot += 32.34
                    elif cls_id==1:
                        # 1是stick
                        x_robot += 56.86
                        y_robot += 79.95
                result_str = f"{right_or_left},{cls_id},{x_robot:.2f},{y_robot:.2f}"
                msg = f"RESULT:{result_str}\n"
                try:
                    if client_socket:
                        client_socket.sendall(msg.encode('utf-8'))
                        print(f"📤 发送结果: {msg.strip()}")
                except Exception as e:
                    print(f"发送失败: {e}")

                # 绘制检测框
                box = first['box']
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                cv2.rectangle(display_image, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 0, 255), 3)
                cv2.putText(display_image, f"SENT: {result_str}", (50, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                try:
                    if client_socket:
                        client_socket.sendall(b"RESULT:NONE\n")
                        print("📤 未检测到目标，发送 RESULT:NONE")
                except Exception as e:
                    print(f"发送失败: {e}")

            # 回到就绪状态
            with client_lock:
                current_state = STATE_READY
            print("🔄 目标检测完成，回到就绪状态")

        # 灯检测处理（light触发）
        elif state == STATE_LIGHT_DETECTING:
            # 调用灯检测函数（使用当前相机帧）
            # image_path = "E://work//humanRobot//VisionGrap//testImage//3.png"  # 替换为你的图片路径
            # color_image1 = cv2.imread(image_path)
            # if color_image1 is None:
            #     raise ValueError(f"无法读取图片: {image_path}")

            light_img, light_count = detect_lights(
                image=color_image,
                brightness_threshold=200,
                min_area=700
            )
            display_image = light_img

            # 发送检测结果
            try:
                if client_socket:
                    if light_count == 1 :
                        client_socket.sendall(b"OK\n")
                        print("📤 发送: OK（检测到1个亮灯）")
                    else:
                        client_socket.sendall(b"NG\n")
                        print(f"📤 发送: NOT_OK（检测到{light_count}个亮灯）")
            except Exception as e:
                print(f"发送灯检测结果失败: {e}")

            # 回到就绪状态
            with client_lock:
                current_state = STATE_READY
            print("🔄 灯检测完成，回到就绪状态")

        # 绘制状态提示
        if state == STATE_WAITING_START:
            cv2.putText(display_image, "WAITING start...", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 100, 100), 2)
        elif state == STATE_READY:
            cv2.putText(display_image, "READY - waiting light/resultOK", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        elif state == STATE_DETECTING_ONCE:
            cv2.putText(display_image, "DETECTING...", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        elif state == STATE_LIGHT_DETECTING:
            cv2.putText(display_image, "正在执行灯检测...", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

        # 显示所有检测框（仅在目标检测状态）
        if state == STATE_DETECTING_ONCE and 'result' in locals():
            for box in result.boxes:
                if box.conf.item() > 0.7:
                    cls_id = int(box.cls.item())
                    if cls_id in TARGET_SET:
                        name = result.names[cls_id]
                        xyxy = box.xyxy[0].cpu().numpy().astype(int)
                        color = colors[cls_id]
                        cv2.rectangle(display_image, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), color, 2)
                        cv2.putText(display_image, f"{name}:{box.conf.item():.2f}", (xyxy[0], xyxy[1] - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow('Detection System', display_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    if client_socket:
        client_socket.close()
    print("程序已退出")