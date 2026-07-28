# 通过鼠标点击获取深度信息，可以缩放图像，配合标定使用

import pyrealsense2 as rs
import numpy as np
import cv2
import os

# 全局变量
scale = 1.0
point = (320, 240)  # 初始点击点
view_x, view_y = 0.0, 0.0  # 视图左上角在原图的坐标
image_w, image_h = 640, 480  # 图像尺寸
print_info = False  # 是否打印信息的标志


def mouse_callback(event, x, y, flags, param):
    global scale, point, view_x, view_y, print_info, image_w, image_h

    if event == cv2.EVENT_LBUTTONDOWN:
        point = (x, y)
        print_info = True

    if event == cv2.EVENT_MOUSEWHEEL:
        # 1. 计算鼠标在原图上的坐标
        view_w_old = image_w / scale
        view_h_old = image_h / scale
        img_mx = view_x + x * (view_w_old / image_w)
        img_my = view_y + y * (view_h_old / image_h)

        # 2. 更新缩放比例
        old_scale = scale
        if flags > 0:
            scale += 0.2  # 增大缩放步长
        else:
            scale -= 0.2
        scale = max(1.0, scale)  # 最小为1倍, 防止过度缩小

        if abs(scale - old_scale) < 0.01:
            return

        # 3. 计算新视图的左上角
        new_view_w = image_w / scale
        new_view_h = image_h / scale
        view_x = img_mx - x * (new_view_w / image_w)
        view_y = img_my - y * (new_view_h / image_h)

        # 4. 限制视图边界, 防止移出图像范围
        view_x = max(0.0, min(view_x, image_w - new_view_w))
        view_y = max(0.0, min(view_y, image_h - new_view_h))

def read_points_from_file(file_path):
    points = []
    with open(file_path, 'r') as file:
        for line in file:
            x, y, z = map(float, line.strip().split())
            points.append([x, y, z])
    return np.array(points, dtype=np.float32)


# 读取源坐标系和目标坐标系的点集
# src_points = read_points_from_file('camPosT_1.txt')
# dst_points = read_points_from_file('robPosT_1.txt')
# src_points = read_points_from_file('camPosLeft.txt')
# dst_points = read_points_from_file('robPosLeft.txt')
# 计算仿射变换矩阵
affine_matrix = cv2.estimateAffine3D(src_points, dst_points)
if affine_matrix is not None:
    print("Affine transformation matrix:\n", affine_matrix)
    extended_affine_matrix = np.vstack((affine_matrix[1], [0, 0, 0, 1]))
    print("Extended affine transformation matrix (4x4):\n", extended_affine_matrix)
else:
    print("Failed to estimate the affine transformation matrix")
    exit(1)


camPosFile = "camPos.txt"
if os.path.exists(camPosFile):
    os.remove(camPosFile)

# 枚举所有可用的 RealSense 设备
ctx = rs.context()
devices = ctx.query_devices()
print(len(devices) ,"====")
if len(devices) < 1:
    print("可用相机数量不足 3 个，请检查连接。")
    exit(1)

# 获取第三个相机的序列号（索引为 2）
print(devices[0],"====-=-=")
selected_device = devices[0]
serial_number = selected_device.get_info(rs.camera_info.serial_number)

# 配置深度和颜色流
pipeline = rs.pipeline()
config = rs.config()
config.enable_device(serial_number)
# 获取设备产品线以设置支持的分辨率
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("本示例需要带颜色传感器的深度相机")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 开始串流
profile = pipeline.start(config)

# 获取深度传感器的深度比例
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("深度比例: ", depth_scale)

# 获取颜色流的内参
color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
color_intrinsics = color_profile.get_intrinsics()

# 创建对齐对象
# rs.align 用于将深度帧与其他帧对齐
# "align_to" 是我们计划将深度帧对齐到的流类型
align_to = rs.stream.color
align = rs.align(align_to)

# 创建窗口并设置鼠标回调
cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
cv2.setMouseCallback('RealSense', mouse_callback)

try:
    while True:

        # 等待一对连贯的帧: 深度和颜色
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            continue

        # 应用孔洞填充滤波器
        # hole_filling = rs.hole_filling_filter()
        # filled_depth_frame = hole_filling.process(aligned_depth_frame).as_depth_frame()

        # 将图像转换为numpy数组
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # 根据view_x, view_y和scale来裁剪图像
        view_w = image_w / scale
        view_h = image_h / scale

        x1, y1 = int(view_x), int(view_y)
        x2, y2 = int(view_x + view_w), int(view_y + view_h)

        cropped_image = color_image[y1:y2, x1:x2]
        display_image = cv2.resize(cropped_image, (image_w, image_h))

        # 仅在鼠标点击时打印信息
        if print_info:
            # 将屏幕点击坐标转换为原图坐标
            original_x = int(view_x + point[0] * (view_w / image_w))
            original_y = int(view_y + point[1] * (view_h / image_h))

            # 获取深度值
            depth_in_meters = aligned_depth_frame.get_distance(original_x, original_y)

            # 如果深度为0，则在邻近区域搜索有效深度
            if depth_in_meters == 0:
                search_radius = 2  # 搜索半径为2，即5x5的区域
                for r in range(1, search_radius + 1):
                    for i in range(-r, r + 1):
                        for j in range(-r, r + 1):
                            if i == 0 and j == 0:
                                continue
                            nx, ny = original_x + j, original_y + i
                            if 0 <= nx < image_w and 0 <= ny < image_h:
                                new_depth = aligned_depth_frame.get_distance(nx, ny)
                                if new_depth > 0:
                                    depth_in_meters = new_depth
                                    print(f"在({nx}, {ny})找到有效深度: {depth_in_meters:.3f}m")
                                    break
                        if depth_in_meters > 0:
                            break
                    if depth_in_meters > 0:
                        break

            print(depth_in_meters,";===000000")
            # 反投影到3D
            if depth_in_meters > 0:
                point_3d = rs.rs2_deproject_pixel_to_point(color_intrinsics, [original_x, original_y], depth_in_meters)
                print(f"点坐标: ({original_x}, {original_y}), 深度: {depth_in_meters:.3f}m, 3D坐标: {point_3d}")
                result2txt = str(point_3d[0]*1000) + " " + str(point_3d[1]*1000) + " " + str(point_3d[2]*1000)

                point_a_new = np.array([point_3d[0]*1000, point_3d[1]*1000, point_3d[2]*1000, 1], dtype=np.float32).reshape(4, 1)
                point_b_new = extended_affine_matrix @ point_a_new
                x_new, y_new, z_new = point_b_new[:3, 0]
                print("===xyz:",x_new,",",y_new,",",z_new)
                # with open(camPosFile, 'a') as file_handle:  # .txt可以不自己新建,代码会自动新建
                #     file_handle.write(result2txt)  # 写入
                #     file_handle.write('\n')
                # file_handle.close()
            else:
                print(f"点坐标: ({original_x}, {original_y}), 深度: {depth_in_meters:.3f}m (无有效深度)")

            print_info = False  # 重置标志

        # 在点击点绘制一个圆圈
        cv2.circle(display_image, point, 4, (0, 0, 255), -1)

        # 显示图像
        cv2.imshow('RealSense', display_image)
        key = cv2.waitKey(1)

        # 按下esc或'q'关闭图像窗口
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:

    # 停止串流
    pipeline.stop()

