import cv2
import time
from ultralytics import YOLO
import random
import pyrealsense2 as rs
import numpy as np
import socket
import threading
import os
from datetime import datetime
import json


# ==================== 调试配置参数 ====================
DEBUG_MODE = 0  # 调试模式: 0=不保存调试图片, 1=保存调试图片
LIGHT_SAVE_DIR = "light"  # 灯检测图片保存目录
TARGET_SAVE_DIR = "target"  # 目标检测图片保存目录
FILTERED_SAVE_DIR = "filtered"  # 过滤后区域保存目录

# ==================== 鼠标框选功能开关 ====================
ENABLE_MOUSE_SELECTION = 1  # 鼠标框选功能: 0=禁用, 1=启用
# ===================================================

# ==================== 灯检测区域配置 ====================
# 格式: [x_min, y_min, x_max, y_max] - 检测区域的矩形范围
LIGHT_DETECTION_ZONE = [0, 0, 150, 100]  # 灯检测区域: x从0到150, y从0到100

# 是否启用区域检测过滤
ENABLE_ZONE_FILTER = True  # True=只在指定区域检测, False=全图检测

# 区域配置文件路径
ZONE_CONFIG_FILE = "detection_zones.json"
# ===================================================


# -----------------------------
# 保存/加载检测区域配置
# -----------------------------
def save_zone_config():
    """保存检测区域配置到文件"""
    config = {
        'light_zone': LIGHT_DETECTION_ZONE,
        'enable_filter': ENABLE_ZONE_FILTER
    }
    with open(ZONE_CONFIG_FILE, 'w') as f:
        json.dump(config, f, indent=4)
    print(f"✅ 检测区域配置已保存到: {ZONE_CONFIG_FILE}")


def load_zone_config():
    """从文件加载检测区域配置"""
    global LIGHT_DETECTION_ZONE, ENABLE_ZONE_FILTER
    try:
        with open(ZONE_CONFIG_FILE, 'r') as f:
            config = json.load(f)
            LIGHT_DETECTION_ZONE = config.get('light_zone', LIGHT_DETECTION_ZONE)
            ENABLE_ZONE_FILTER = config.get('enable_filter', ENABLE_ZONE_FILTER)
        print(f"✅ 已加载检测区域配置: {ZONE_CONFIG_FILE}")
        print(f"   灯检测区域: {LIGHT_DETECTION_ZONE}")
        return True
    except FileNotFoundError:
        print(f"⚠️ 配置文件不存在，使用默认配置")
        save_zone_config()
        return False
    except Exception as e:
        print(f"❌ 加载配置文件失败: {e}")
        return False


# 加载配置
load_zone_config()


# -----------------------------
# 检查点是否在检测区域内
# -----------------------------
def is_point_in_zone(point_x, point_y, zone):
    """
    检查点是否在检测区域内
    zone: [x_min, y_min, x_max, y_max]
    """
    if not ENABLE_ZONE_FILTER:
        return True
    x_min, y_min, x_max, y_max = zone
    return x_min <= point_x <= x_max and y_min <= point_y <= y_max


# -----------------------------
# 创建目录
# -----------------------------
if DEBUG_MODE:
    if not os.path.exists(LIGHT_SAVE_DIR):
        os.makedirs(LIGHT_SAVE_DIR)
        print(f"创建目录: {LIGHT_SAVE_DIR}")
    
    if not os.path.exists(TARGET_SAVE_DIR):
        os.makedirs(TARGET_SAVE_DIR)
        print(f"创建目录: {TARGET_SAVE_DIR}")
    
    if not os.path.exists(FILTERED_SAVE_DIR):
        os.makedirs(FILTERED_SAVE_DIR)
        print(f"创建目录: {FILTERED_SAVE_DIR}")
else:
    print("调试模式已关闭，不会保存调试图片")

# 打印鼠标框选功能状态
if ENABLE_MOUSE_SELECTION:
    print("鼠标框选功能: 启用 (可按 'l' 键框选灯检测区域)")
else:
    print("鼠标框选功能: 禁用")


# -----------------------------
# 生成时间戳文件名
# -----------------------------
def get_timestamp_filename(prefix="detect"):
    """生成带时间戳的文件名"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    return f"{prefix}_{timestamp}.jpg"


# -----------------------------
# 保存调试图像
# -----------------------------
def save_debug_image(image, save_dir, prefix, suffix=""):
    """保存调试图像"""
    if not DEBUG_MODE:
        return None
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    if suffix:
        filename = f"{prefix}_{timestamp}_{suffix}.jpg"
    else:
        filename = f"{prefix}_{timestamp}.jpg"
    
    filepath = os.path.join(save_dir, filename)
    cv2.imwrite(filepath, image)
    print(f"📸 调试图像已保存: {filepath}")
    return filepath


# -----------------------------
# 灯亮检测函数（支持自定义检测区域）
# -----------------------------
def detect_lights(image, brightness_threshold=200, min_area=100, max_area=5000):
    """检测图像中的亮灯区域"""
    # 转换为灰度图
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    print(f"\n[Light Detect] 开始检测")
    print(f"  亮度阈值: {brightness_threshold}")
    print(f"  面积范围: [{min_area}, {max_area}]")
    if ENABLE_ZONE_FILTER:
        print(f"  检测模式: 区域过滤模式")
        print(f"  检测区域: {LIGHT_DETECTION_ZONE}")
        x_min, y_min, x_max, y_max = LIGHT_DETECTION_ZONE
        print(f"  有效区域: x∈[{x_min},{x_max}], y∈[{y_min},{y_max}]")
    else:
        print(f"  检测模式: 全图检测模式")

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
    light_info_list = []
    passed_regions = []
    
    # 为调试模式创建黑底图像
    if DEBUG_MODE:
        filtered_result = np.zeros_like(image)  # 全黑背景
    
    for contour in contours:
        area = cv2.contourArea(contour)
        
        # 面积过滤
        if area < min_area or area > max_area:
            continue

        # 获取边界框
        x, y, w, h = cv2.boundingRect(contour)
        
        # 计算中心位置和亮度
        center_x = x + (w // 2)
        center_y = y + (h // 2)
        
        roi_gray = gray[y:y+h, x:x+w]
        mean_brightness = cv2.mean(roi_gray)[0]
        max_brightness = np.max(roi_gray)
        
        print(f"  -> [候选灯光 {light_count + 1}]")
        print(f"     | 中心坐标: ({center_x}, {center_y})")
        print(f"     | 区域大小: {w}x{h}, 面积: {area}")
        print(f"     | 亮度强度: 平均={mean_brightness:.2f}, 最大={max_brightness}")
        
        # 区域过滤 - 使用可配置的检测区域
        if ENABLE_ZONE_FILTER:
            is_valid_region = is_point_in_zone(center_x, center_y, LIGHT_DETECTION_ZONE)
            if not is_valid_region:
                print(f"       | ❌ 未通过检测 - 位置不在指定检测区域内")
                continue
            print(f"       | ✅ 通过检测 - 位于有效区域内")
        else:
            print(f"       | ✅ 通过检测 - 全图检测模式")
        
        # 保存通过区域的ROI
        if DEBUG_MODE:
            region_info = {
                'id': light_count + 1,
                'x': x,
                'y': y,
                'w': w,
                'h': h,
                'center_x': center_x,
                'center_y': center_y,
                'area': area,
                'brightness': mean_brightness
            }
            passed_regions.append(region_info)
            
            # 保存裁剪的ROI区域
            roi_color = image[y:y+h, x:x+w]
            roi_filename = f"roi_light_{light_count + 1}_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]}.png"
            roi_path = os.path.join(LIGHT_SAVE_DIR, roi_filename)
            cv2.imwrite(roi_path, roi_color)
            print(f"     | ROI已导出: {roi_path}")
            
            # 将通过的区域复制到filtered_result中
            filtered_result[y:y+h, x:x+w] = image[y:y+h, x:x+w]

        # 绘制检测框和标签
        cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 3)
        cv2.putText(result, f"Light {light_count + 1}", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.circle(result, (center_x, center_y), 5, (0, 0, 255), -1)
        
        # 添加面积和亮度信息
        cv2.putText(result, f"Area:{area:.0f}", (x, y + h + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        cv2.putText(result, f"Bright:{mean_brightness:.0f}", (x, y + h + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        light_info_list.append({
            'id': light_count + 1,
            'center': (center_x, center_y),
            'area': area,
            'brightness': mean_brightness
        })
        light_count += 1

    # 在图像顶部添加汇总信息
    cv2.putText(result, f"Lights Detected: {light_count}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(result, f"Threshold: {brightness_threshold}", (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(result, f"Area Range: {min_area}-{max_area}", (10, 75),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    if ENABLE_ZONE_FILTER:
        zone_text = f"Light Detection Zone: x∈[{LIGHT_DETECTION_ZONE[0]},{LIGHT_DETECTION_ZONE[2]}], y∈[{LIGHT_DETECTION_ZONE[1]},{LIGHT_DETECTION_ZONE[3]}]"
        cv2.putText(result, zone_text, (10, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    else:
        cv2.putText(result, "Mode: Full Image Detection", (10, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    
    # 保存完整检测结果
    if DEBUG_MODE:
        save_debug_image(result, LIGHT_SAVE_DIR, "light_detect", f"{light_count}lights")
        
        # 保存只包含通过检测区域的图像（屏蔽区域显示为黑色）
        if passed_regions and ENABLE_ZONE_FILTER:
            filtered_filename = f"light_filtered_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]}.jpg"
            filtered_filepath = os.path.join(FILTERED_SAVE_DIR, filtered_filename)
            cv2.imwrite(filtered_filepath, filtered_result)
            print(f"📸 过滤后区域图像已保存: {filtered_filepath}")

    print(f"[Light Detect] 结束检测 - 共标记 {light_count} 个有效亮灯区域\n")
    return result, light_count, light_info_list


# -----------------------------
# 目标检测保存函数
# -----------------------------
def save_target_detection(image, result_str, center_x, center_y, right_or_left):
    """保存目标检测结果图像（仅在调试模式下）"""
    if not DEBUG_MODE:
        return
    
    # 在图像上添加信息
    save_img = image.copy()
    cv2.putText(save_img, f"Target: {right_or_left}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(save_img, f"Robot: ({result_str})", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(save_img, f"Pixel: ({center_x}, {center_y})", (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    save_debug_image(save_img, TARGET_SAVE_DIR, "target_detect", f"{right_or_left}")


# -----------------------------
# 鼠标回调函数 - 用于在GUI中手动选择检测区域
# -----------------------------
selection_start = None
selection_end = None
selection_mode = None  # 'light'


def mouse_callback(event, x, y, flags, param):
    """鼠标回调函数，用于选择检测区域"""
    global selection_start, selection_end, selection_mode
    
    # 如果鼠标框选功能被禁用，直接返回
    if not ENABLE_MOUSE_SELECTION:
        return
    
    if event == cv2.EVENT_LBUTTONDOWN:
        selection_start = (x, y)
        selection_end = None
    elif event == cv2.EVENT_LBUTTONUP:
        selection_end = (x, y)
        if selection_start and selection_end:
            x1 = min(selection_start[0], selection_end[0])
            y1 = min(selection_start[1], selection_end[1])
            x2 = max(selection_start[0], selection_end[0])
            y2 = max(selection_start[1], selection_end[1])
            
            if selection_mode == 'light':
                global LIGHT_DETECTION_ZONE
                LIGHT_DETECTION_ZONE = [x1, y1, x2, y2]
                print(f"\n✅ 灯检测区域已更新: {LIGHT_DETECTION_ZONE}")
                save_zone_config()
                selection_mode = None
            
            selection_start = None
            selection_end = None
    elif event == cv2.EVENT_MOUSEMOVE and selection_start:
        selection_end = (x, y)


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
src_left = read_2d_points_from_file('camPosLeft.txt')
dst_left = read_2d_points_from_file('robPosLeft.txt')
matrix_left = estimate_affine_2d(src_left, dst_left)

src_right = read_2d_points_from_file('camPosRigth.txt')
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
model = YOLO('./runs/best_yaoshibang.pt')
num_classes = len(model.names)
colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for _ in range(num_classes)]

try:
    green_id = list(model.names.values()).index('Vita')
    yellow_id = list(model.names.values()).index('Aliens')
    print(f"绿色目标ID: {green_id}, 黄色目标ID: {yellow_id}")
except ValueError:
    print("模型中未找到 'Vita' 或 'Aliens' 类别")
    exit(1)

TARGET_SET = {green_id, yellow_id}

# -----------------------------
# RealSense 相机设置
# -----------------------------
SECOND_CAMERA_SERIAL = "244122306709"
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
STATE_WAITING_START = 'WAITING_START'
STATE_READY = 'READY'
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
            current_state = STATE_WAITING_START
        try:
            while True:
                data = client_socket.recv(1024).decode('utf-8').strip()
                if not data:
                    break

                print(f"收到指令: {data}")
                with client_lock:
                    state = current_state

                if state == STATE_WAITING_START and data.lower() == 'start':
                    with client_lock:
                        current_state = STATE_READY
                    try:
                        client_socket.sendall(b"readyOK\n")
                        print("📤 发送: readyOK（已准备好）")
                    except Exception as e:
                        print(f"发送readyOK失败: {e}")
                    continue

                if state == STATE_READY and data.lower() == 'light':
                    with client_lock:
                        current_state = STATE_LIGHT_DETECTING
                    print("🟢 进入灯检测状态")
                    continue

                if state == STATE_READY and (data.lower() == 'resultok' or data.lower() == 'resultokresultok'):
                    with client_lock:
                        current_state = STATE_DETECTING_ONCE
                    print("🟢 进入目标检测状态")
                    continue

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

# 设置鼠标回调
cv2.namedWindow('Detection System')
cv2.setMouseCallback('Detection System', mouse_callback)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        color_image = np.asanyarray(color_frame.get_data())
        display_image = color_image.copy()

        with client_lock:
            state = current_state

        # 目标检测处理
        if state == STATE_DETECTING_ONCE:
            results = model.predict(source=color_image, save=False, verbose=False)
            result = results[0]
            boxes = result.boxes
            detected = []
            
            for box in boxes:
                cls_id = int(box.cls.item())
                conf = box.conf.item()
                x_center, y_center, width, height = box.xywh.squeeze().tolist()
                
                # 目标检测不做区域过滤，全图检测
                if conf > 0.8 and cls_id in TARGET_SET:
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    center_x = int((xyxy[0] + xyxy[2]) / 2)
                    center_y = int((xyxy[1] + xyxy[3]) / 2)
                    detected.append({
                        'cls_id': cls_id,
                        'conf': conf,
                        'center': (center_x, center_y),
                        'box': box
                    })

            if detected:
                print(detected)
                first = detected[0]
                center_x, center_y = first['center']
                cls_id = first['cls_id']

                pixel_coords = np.array([center_x, center_y, 1], dtype=np.float32)
                if center_x > 340:
                    right_or_left = "right"
                    x_robot, y_robot = matrix_right @ pixel_coords
                    x_robot -= (-15)
                    y_robot -= 35
                else:
                    right_or_left = "left"
                    x_robot, y_robot = matrix_left @ pixel_coords
                    x_robot += (-5)
                    y_robot += 5

                result_str = f"{right_or_left},{x_robot:.2f},{y_robot:.2f}"
                msg = f"RESULT:{result_str}\n"
                
                try:
                    if client_socket:
                        client_socket.sendall(msg.encode('utf-8'))
                        print(f"📤 发送结果: {msg.strip()}")
                except Exception as e:
                    print(f"发送失败: {e}")

                # 保存目标检测调试图像
                save_target_detection(color_image, result_str, center_x, center_y, right_or_left)

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

            with client_lock:
                current_state = STATE_READY
            print("🔄 目标检测完成，回到就绪状态")

        # 灯检测处理
        elif state == STATE_LIGHT_DETECTING:
            light_img, light_count, light_info = detect_lights(
                image=color_image,
                brightness_threshold=250,
                min_area=500,
                max_area=5000
            )
            display_image = light_img
        
            try:
                if client_socket:
                    if light_count == 1:
                        client_socket.sendall(b"OK\n")
                        print(f"📤 发送: OK（检测到{light_count}个亮灯）")
                    else:
                        client_socket.sendall(b"NG\n")
                        print(f"📤 发送: NG（检测到{light_count}个亮灯）")
            except Exception as e:
                print(f"发送灯检测结果失败: {e}")

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

        # 绘制灯检测区域框（青色）
        if ENABLE_ZONE_FILTER:
            lx1, ly1, lx2, ly2 = LIGHT_DETECTION_ZONE
            cv2.rectangle(display_image, (lx1, ly1), (lx2, ly2), (255, 255, 0), 2)
            cv2.putText(display_image, "Light Detection Zone", (lx1, ly1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # 绘制鼠标选择区域（如果正在选择）
        if selection_start and selection_end:
            cv2.rectangle(display_image, selection_start, selection_end, (0, 0, 255), 2)
        
        # 添加调试模式状态显示
        if DEBUG_MODE:
            cv2.putText(display_image, f"DEBUG MODE: ON", (10, display_image.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # 添加操作提示（仅在鼠标框选功能启用时显示）
        if ENABLE_MOUSE_SELECTION:
            cv2.putText(display_image, "Press 'l' to config Light zone, 'r' to reset", 
                        (10, display_image.shape[0] - 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        else:
            cv2.putText(display_image, "Mouse selection DISABLED", 
                        (10, display_image.shape[0] - 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # 显示检测框
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
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('l') and ENABLE_MOUSE_SELECTION:
            # 配置灯检测区域
            selection_mode = 'light'
            print("\n🔧 进入灯检测区域配置模式，在图像上用鼠标拖动选择区域...")
            print("   提示：按住鼠标左键拖动选择矩形区域")
        elif key == ord('r') and ENABLE_MOUSE_SELECTION:
            # 重置区域
            LIGHT_DETECTION_ZONE = [0, 0, 150, 100]
            save_zone_config()
            print("\n🔄 灯检测区域已重置为默认值")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    if client_socket:
        client_socket.close()
    print("程序已退出")
