import pyrealsense2 as rs

# 创建管道
pipeline = rs.pipeline()

# 获取设备列表
context = rs.context()
devices = context.query_devices()

if devices.size() == 0:
    print("未检测到 Intel RealSense 相机")
else:
    for i in range(devices.size()):
        dev = devices[i]
        sn = dev.get_info(rs.camera_info.serial_number)
        print(f"相机 {i+1} 序列号: {sn}")