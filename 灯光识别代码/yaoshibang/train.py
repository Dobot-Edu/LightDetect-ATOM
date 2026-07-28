# import warnings
# # warnings.filterwarnings('ignore')
# from ultralytics import YOLO
#
# if __name__ == '__main__':
#   model = YOLO('ultralytics/cfg/models/11/yolo11s.yaml')
#   model.load('yolo11s.pt')  #注释则不加载
#   results = model.train(
#     data='data.yaml',  #数据集配置文件的路径
#     epochs=300,  #训练轮次总数
#     batch=16,  #批量大小，即单次输入多少图片训练
#     imgsz=640,  #训练图像尺寸
#     workers=8,  #加载数据的工作线程数
#     device= 0,  #指定训练的计算设备，无nvidia显卡则改为 'cpu'
#     optimizer='SGD',  #训练使用优化器，可选 auto,SGD,Adam,AdamW 等
#     amp= True,  #True 或者 False, 解释为：自动混合精度(AMP) 训练
#     cache=False  # True 在内存中缓存数据集图像，服务器推荐开启
# )


import warnings
warnings.filterwarnings('ignore')  # 可选：忽略警告
from ultralytics import YOLO

if __name__ == '__main__':
    # 加载模型配置（可选加载预训练权重）
    model = YOLO('ultralytics/cfg/models/11/yolo11s.yaml')
    model.load('yolo11s.pt')  # 如果从头训练可注释此行

    # 开始训练
    results = model.train(
        data='data.yaml',              # 数据集配置文件路径
        epochs=300,                    # 总训练轮数（建议 300~500）
        batch=16,                      # 批量大小（根据显存调整）
        imgsz=640,                     # 图像尺寸（640 是标准）
        workers=8,                     # 数据加载线程数
        device=0,                      # 使用 GPU（0 表示第一块显卡）
        optimizer='SGD',               # 优化器（AdamW 更稳定）
        lr0=0.01,                      # 初始学习率（SGD 常用 0.01，AdamW 用 0.001）
        lrf=0.01,                      # 最终学习率（学习率衰减到原来的 0.01 倍）
        momentum=0.937,                # 动量（SGD 推荐 0.937）
        weight_decay=0.0005,           # 权重衰减（L2 正则化）
        warmup_epochs=3,               # 学习率预热轮数
        warmup_momentum=0.8,           # 预热阶段 momentum
        warmup_bias_lr=0.1,            # 预热阶段 bias 的学习率
        box=0.05,                      # 检测框损失系数（可调）
        cls=0.5,                       # 分类损失系数
        dfl=1.0,                       # DFL 损失系数
        label_smoothing=0.1,           # 标签平滑（缓解过拟合）
        hsv_h=0.015, hsv_s=0.7, hsv_v=0.4,  # 颜色空间增强
        degrees=0.0, translate=0.1, scale=0.5, shear=0.0, perspective=0.001,  # 几何变换增强
        flipud=0.5, fliplr=0.5,        # 上下/左右翻转
        mosaic=1.0,                    # Mosaic 数据增强
        mixup=0.2,                     # Mixup 增强（缓解过拟合）
        val=True,                      # 是否在每个 epoch 后验证
        amp=True,                      # 自动混合精度训练（加速训练）
        cache=False,                   # 是否缓存图像（内存充足时设为 True）
        project='runs/train',          # 训练结果保存路径
        name='exp',                    # 保存文件夹名称
    )