# from ultralytics import YOLO
# # 加载预训练的 YOLOv11n 模型
# model = YOLO('/home/zz/Downloads/ultralytics-8.3.169/runs/detect/train2/weights/best.pt')
# source = '/home/zz/Downloads/ultralytics-8.3.169/data/VOCdevkit/test/images/103.png' #更改为自己的图片路径
# # 运行推理，并附加参数
# model.predict(source, save=True)


# from ultralytics import YOLO
#
# # 加载预训练的 YOLOv11n 模型
# model = YOLO('/home/zz/Downloads/ultralytics-8.3.169/runs/train/exp/weights/best.pt')
#
# # 更改为此处的文件夹路径，该文件夹应包含你想要进行推理的所有图片
# source_folder = '/home/zz/Downloads/ultralytics-8.3.169/data/VOCdevkit/test/images/'
#
# # 运行推理，并附加参数，save=True 表示保存结果到运行目录下的 'runs/detect' 文件夹中
# results = model.predict(source=source_folder, save=True)
#
# # 如果你想进一步处理结果，可以遍历 results 变量获取详细信息
# for result in results:
#     # 处理每个图片的结果
#     print(result)


import cv2
from ultralytics import YOLO
import os
import random

# 加载预训练的 YOLOv11n 模型
model = YOLO('/home/dobotpc2/Downloads/ultralytics-8.3.169/runs/train/exp2/weights/best.pt')

# 文件夹路径，包含要进行推理的所有图片
source_folder = '/home/dobotpc2/Downloads/ultralytics-8.3.169/data/VOCdevkit/test/images/'

# 获取文件夹内所有图片文件
image_files = [os.path.join(source_folder, f) for f in os.listdir(source_folder) if
               f.lower().endswith(('.png', '.jpg', '.jpeg'))]

# 为每个类别随机生成颜色
num_classes = len(model.names)
colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for _ in range(num_classes)]

for image_path in image_files:
    # 读取原始图像
    img = cv2.imread(image_path)
    # 运行推理，并附加参数
    results = model.predict(source=image_path, save=False)
    result = results[0]

    boxes = result.boxes  # 获取检测框信息
    for box in boxes:
        score = box.conf.item()  # 获取检测分数
        if score > 0.6:  # 判断分数是否大于 0.6
            cls = int(box.cls.item())  # 获取检测类别
            name = result.names[cls]  # 获取类别名称
            xyxy = box.xyxy[0].cpu().numpy().astype(int)  # 获取检测框坐标并转为整数

            # 根据类别获取对应的颜色
            color = colors[cls]

            # 绘制检测框
            cv2.rectangle(img, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), color, 2)

            # 计算中心点坐标
            center_x = int((xyxy[0] + xyxy[2]) / 2)
            center_y = int((xyxy[1] + xyxy[3]) / 2)

            # 绘制中心点
            cv2.circle(img, (center_x, center_y), 5, color, -1)

            # 准备显示的文本
            text = f"{name}: {score:.2f}"
            # 绘制文本背景，使用对应的颜色
            (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(img, (xyxy[0], xyxy[1] - text_height - 5), (xyxy[0] + text_width, xyxy[1]), color, -1)
            # 绘制文本
            cv2.putText(img, text, (xyxy[0], xyxy[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            print(f"检测目标: {name}, 分数: {score:.2f}, 中心点坐标: ({center_x}, {center_y})")

    # 显示图像
    cv2.imshow('YOLOv11 Inference', img)

    # 等待按下空格键
    while True:
        key = cv2.waitKey(0)
        if key == 32:  # 空格键的 ASCII 码是 32
            # cv2.destroyWindow('YOLOv11 Inference')  # 只关闭当前窗口
            break

# 处理完所有图片后关闭所有窗口
cv2.destroyAllWindows()