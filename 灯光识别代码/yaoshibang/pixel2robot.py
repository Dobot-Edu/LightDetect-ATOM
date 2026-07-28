import pyrealsense2 as rs
import numpy as np
import cv2
import os
import pupil_apriltags as apriltag

# 全局变量
detect_apriltag = False  # 是否检测AprilTag的标志
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
                    print(f"文件 '{file_path}' 第 {i+1} 行格式错误，跳过。")
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

def mouse_callback(event, x, y, flags, param):
    """鼠标回调函数，在左键点击时触发AprilTag检测。"""
    global detect_apriltag
    if event == cv2.EVENT_LBUTTONDOWN:
        detect_apriltag = True
        print("收到点击，准备检测AprilTag...")

# -----------------------------
# 预加载仿射矩阵
# -----------------------------
print("正在加载左右手仿射变换矩阵...")

# src_right = read_2d_points_from_file('camPosLeft.txt')
# dst_right = read_2d_points_from_file('robPosLeft.txt')
# matrix_right = estimate_affine_2d(src_right, dst_right)

src_right = read_2d_points_from_file('camPosRigth.txt')
dst_right = read_2d_points_from_file('robPosRight.txt')
matrix_right = estimate_affine_2d(src_right, dst_right)

# if matrix_left is None:
#     print("❌ 错误: 左手仿射矩阵加载失败")
#     exit(1)
if matrix_right is None:
    print("❌ 错误: 右手仿射矩阵加载失败")
    exit(1)

print("✅ 仿射矩阵加载成功")

# 初始化AprilTag检测器
try:
    detector = apriltag.Detector(families='tag36h11')
except ImportError:
    print("错误：pupil_apriltags 库未安装。")
    print("请运行: pip install pupil-apriltags")
    exit()

SECOND_CAMERA_SERIAL = "352222303497"
# 配置RealSense相机
pipeline = rs.pipeline()
config = rs.config()


# 创建配置对象
config = rs.config()
ctx = rs.context()
devices = ctx.query_devices()
if not any(dev.get_info(rs.camera_info.serial_number) == SECOND_CAMERA_SERIAL for dev in devices):
    print(f"未找到相机: {SECOND_CAMERA_SERIAL}")
    exit(1)
# 自动查找并启用设备
try:
    # 获取设备产品线以设置支持的分辨率
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

    # # 检查是否有RGB相机
    # found_rgb = any(s.get_info(rs.camera_info.name) == 'RGB Camera' for s in device.sensors)
    # if not found_rgb:
    #     print("本示例需要带颜色传感器的深度相机")
    #     exit(0)

    config.enable_device(SECOND_CAMERA_SERIAL)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 启动相机流
    profile = pipeline.start(config)
    print("相机已成功启动")

except Exception as e:
    print(f"启动相机失败: {e}")
    exit()

# 创建窗口并设置鼠标回调
cv2.namedWindow('RealSense AprilTag Detector', cv2.WINDOW_AUTOSIZE)
cv2.setMouseCallback('RealSense AprilTag Detector', mouse_callback)

print("程序已启动，请在窗口中点击鼠标左键以检测AprilTag。")

try:
    while True:
        # 等待颜色帧
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # 将图像转换为numpy数组
        color_image = np.asanyarray(color_frame.get_data())
        display_image = color_image.copy()

        # 如果触发了检测
        if detect_apriltag:
            print("正在检测AprilTag...")
            # 将图像转为灰度图
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # 检测AprilTag
            tags = detector.detect(gray_image)

            if tags:
                print(f"检测到 {len(tags)} 个AprilTag。")
                for tag in tags:
                    # 提取中心坐标
                    center_x, center_y = int(tag.center[0]), int(tag.center[1])
                    print("center:",center_x,";",center_y)
                    pixel_coords = np.array([center_x, center_y, 1], dtype=np.float32)
                    x_robot, y_robot = matrix_right @ pixel_coords
                    print(f"AprilTag中心坐标已保存: ({x_robot}, {y_robot})")

                    # 在图像上绘制边界和中心点
                    for i in range(4):
                        pt1 = tuple(tag.corners[i - 1, :].astype(int))
                        pt2 = tuple(tag.corners[i, :].astype(int))
                        cv2.line(display_image, pt1, pt2, (0, 255, 0), 2)
                    cv2.circle(display_image, (center_x, center_y), 5, (0, 0, 255), -1)
            else:
                print("未检测到AprilTag。")

            # 重置检测标志
            detect_apriltag = False

        # 显示图像
        cv2.imshow('RealSense AprilTag Detector', display_image)

        key = cv2.waitKey(1)
        # 按下'q'或ESC退出
        if key & 0xFF == ord('q') or key == 27:
            print("正在关闭程序...")
            cv2.destroyAllWindows()
            break

finally:
    # 停止数据流
    pipeline.stop()
    print("相机已停止。")

