
import numpy as np
import pinocchio as pin
from spatialmath import SE3, SO3
from pinocchio.visualize import MeshcatVisualizer
import tkinter as tk
from tkinter import ttk, filedialog
import time
from threading import Thread
import os

# 导入您的机器人模型
from atom import robot_model
from atom.robot_model import Arm_IK as robot_model

class TrajectoryPlayer:
    def __init__(self):
        # 初始化机器人模型
        self.robot = robot_model()
        
        # 初始化关节角度 - 只使用双臂的14个关节
        self.joint_angles_left_arm = np.zeros(7)
        self.joint_angles_right_arm = np.zeros(7)
        
        # 轨迹数据
        self.trajectory_data = None
        self.current_index = 0
        self.is_playing = False
        self.play_speed = 1.0
        
        # 可视化器
        self.viz = None
        self.visualization_enabled = False
        
        print(f"机器人模型关节数量: {self.robot.interface_model.nq}")
        
    def load_trajectory_file(self, file_path):
        """加载轨迹文件"""
        try:
            self.trajectory_data = np.loadtxt(file_path)
            print(f"成功加载轨迹文件，共 {len(self.trajectory_data)} 个轨迹点")
            print(f"每个轨迹点包含 {self.trajectory_data.shape[1] if len(self.trajectory_data.shape) > 1 else 1} 个关节角度")
            return True
        except Exception as e:
            print(f"加载轨迹文件失败: {e}")
            return False
    
    def setup_visualization(self):
        """设置meshcat可视化"""
        try:
            self.viz = MeshcatVisualizer(
                self.robot.interface_model, 
                self.robot.interface_geom_model, 
                self.robot.interface_geom_model
            )
            self.viz.initViewer(loadModel=True)
            self.viz.loadViewerModel()
            self.viz.displayVisuals(True)
            self.viz.displayCollisions(False)
            
            # 显示坐标系
            frame_ids = [
                self.robot.interface_model.getFrameId("right_wrist_yaw_joint"),
                self.robot.interface_model.getFrameId("left_wrist_yaw_joint"),
                self.robot.interface_model.getFrameId("torso_link")
            ]
            self.viz.displayFrames(True, frame_ids)
            
            self.visualization_enabled = True
            print("Meshcat可视化已启动")
            return True
        except Exception as e:
            print(f"启动可视化失败: {e}")
            return False
    
    def parse_trajectory_point(self, trajectory_point):
        """解析轨迹点数据，只提取双臂的14个关节"""
        # 轨迹点格式：左腿6 + 右腿6 + 腰1 + 左臂7 + 右臂7 + 头2 = 29个关节
        # 我们只需要左臂(13-19)和右臂(20-26)的关节
        if len(trajectory_point) >= 27:  # 至少需要27个数据点才能获取右臂关节
            # 左臂关节 (13-19)
            self.joint_angles_left_arm = trajectory_point[13:20]
            
            # 右臂关节 (20-26)
            self.joint_angles_right_arm = trajectory_point[20:27]
            
            print(f"左臂关节: {np.rad2deg(self.joint_angles_left_arm)}")
            print(f"右臂关节: {np.rad2deg(self.joint_angles_right_arm)}")
            
            return True
        else:
            print(f"轨迹点数据不足，期望至少27个，实际{len(trajectory_point)}个")
            return False
    
    def update_robot_pose(self):
        """更新机器人姿态 - 只更新双臂"""
        if self.visualization_enabled and self.viz is not None:
            # 组合双臂关节角度（14个关节）
            arm_joints = np.concatenate([self.joint_angles_left_arm, self.joint_angles_right_arm])
            
            # 检查关节数量是否匹配
            if len(arm_joints) == self.robot.interface_model.nq:
                self.viz.display(arm_joints)
            else:
                print(f"关节数量不匹配: 期望{self.robot.interface_model.nq}, 实际{len(arm_joints)}")
                # 如果数量不匹配，尝试使用中性位置并只设置可用的关节
                neutral_q = pin.neutral(self.robot.interface_model)
                if len(arm_joints) <= len(neutral_q):
                    neutral_q[:len(arm_joints)] = arm_joints
                    self.viz.display(neutral_q)
    
    def play_trajectory(self):
        """播放轨迹"""
        if self.trajectory_data is None:
            print("没有加载轨迹文件")
            return
        
        self.is_playing = True
        self.current_index = 0
        
        def play_loop():
            while self.is_playing and self.current_index < len(self.trajectory_data):
                try:
                    # 获取当前轨迹点
                    trajectory_point = self.trajectory_data[self.current_index]
                    
                    # 解析轨迹点数据（只提取双臂关节）
                    if self.parse_trajectory_point(trajectory_point):
                        # 更新可视化
                        self.update_robot_pose()
                        
                        # 更新界面显示
                        if hasattr(self, 'update_display_callback'):
                            self.update_display_callback()
                    
                    self.current_index += 1
                    time.sleep(0.05 / self.play_speed)  # 控制播放速度
                    
                except Exception as e:
                    print(f"播放轨迹时出错: {e}")
                    break
            
            self.is_playing = False
            print("轨迹播放完成")
        
        # 在单独线程中播放
        play_thread = Thread(target=play_loop, daemon=True)
        play_thread.start()
    
    def stop_trajectory(self):
        """停止播放"""
        self.is_playing = False
    
    def set_play_speed(self, speed):
        """设置播放速度"""
        self.play_speed = max(0.1, min(5.0, speed))  # 限制速度范围
    
    def get_current_joint_angles_degrees(self):
        """获取当前关节角度（度数）"""
        # 分别获取左右臂关节角度
        left_arm_degrees = np.rad2deg(self.joint_angles_left_arm)
        right_arm_degrees = np.rad2deg(self.joint_angles_right_arm)
        
        # 格式化为逗号分隔的字符串
        left_arm_string = ", ".join([f"{angle:.6f}" for angle in left_arm_degrees])
        right_arm_string = ", ".join([f"{angle:.6f}" for angle in right_arm_degrees])
        
        return left_arm_string, right_arm_string

class TrajectoryPlayerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ATOM双臂机器人")
        self.root.geometry("900x750")  # 调整窗口高度
        
        # 初始化轨迹播放器
        self.player = TrajectoryPlayer()
        self.player.update_display_callback = self.update_display
        
        # 创建界面
        self.create_widgets()
        
    def create_widgets(self):
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 控制面板
        control_frame = ttk.LabelFrame(main_frame, text="控制面板", padding="10")
        control_frame.pack(fill=tk.X, pady=5)
        
        # 文件加载按钮
        file_frame = ttk.Frame(control_frame)
        file_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(file_frame, text="加载轨迹文件", 
                  command=self.load_trajectory_file).pack(side=tk.LEFT, padx=5)
        
        self.file_label = ttk.Label(file_frame, text="未加载文件")
        self.file_label.pack(side=tk.LEFT, padx=5)
        
        # 可视化控制
        viz_frame = ttk.Frame(control_frame)
        viz_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(viz_frame, text="启动可视化", 
                  command=self.start_visualization).pack(side=tk.LEFT, padx=5)
        
        # 播放控制
        play_frame = ttk.Frame(control_frame)
        play_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(play_frame, text="播放轨迹", 
                  command=self.play_trajectory).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(play_frame, text="停止", 
                  command=self.stop_trajectory).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(play_frame, text="重置", 
                  command=self.reset_trajectory).pack(side=tk.LEFT, padx=5)
        
        # 速度控制
        speed_frame = ttk.Frame(control_frame)
        speed_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(speed_frame, text="播放速度:").pack(side=tk.LEFT)
        
        self.speed_var = tk.DoubleVar(value=1.0)
        speed_scale = ttk.Scale(speed_frame, from_=0.1, to=5.0, 
                               orient=tk.HORIZONTAL, variable=self.speed_var,
                               command=self.update_speed)
        speed_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        self.speed_label = ttk.Label(speed_frame, text="1.0x")
        self.speed_label.pack(side=tk.LEFT)
        
        # 关节坐标操作区域
        coordinates_frame = ttk.LabelFrame(main_frame, text="关节坐标操作", padding="10")
        coordinates_frame.pack(fill=tk.X, pady=5)
        
        # 获取坐标按钮
        get_coord_frame = ttk.Frame(coordinates_frame)
        get_coord_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(get_coord_frame, text="获取当前关节坐标", 
                  command=self.get_current_joint_coordinates).pack(side=tk.LEFT, padx=5)
        
        # 左右臂坐标显示区域
        arms_coord_frame = ttk.Frame(coordinates_frame)
        arms_coord_frame.pack(fill=tk.X, pady=5)
        
        # 左臂坐标区域
        left_arm_frame = ttk.LabelFrame(arms_coord_frame, text="左臂关节坐标 (7个关节)", padding="5")
        left_arm_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        # 左臂按钮和文本框
        left_buttons_frame = ttk.Frame(left_arm_frame)
        left_buttons_frame.pack(fill=tk.X, pady=2)
        
        ttk.Button(left_buttons_frame, text="复制左臂坐标", 
                  command=lambda: self.copy_arm_coordinates('left')).pack(side=tk.LEFT, padx=2)
        
        self.left_arm_text = tk.Text(left_arm_frame, height=4, width=45)
        self.left_arm_text.pack(fill=tk.BOTH, expand=True, pady=2)
        
        left_scrollbar = ttk.Scrollbar(left_arm_frame, orient=tk.VERTICAL, command=self.left_arm_text.yview)
        left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.left_arm_text.configure(yscrollcommand=left_scrollbar.set)
        
        # 右臂坐标区域
        right_arm_frame = ttk.LabelFrame(arms_coord_frame, text="右臂关节坐标 (7个关节)", padding="5")
        right_arm_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)
        
        # 右臂按钮和文本框
        right_buttons_frame = ttk.Frame(right_arm_frame)
        right_buttons_frame.pack(fill=tk.X, pady=2)
        
        ttk.Button(right_buttons_frame, text="复制右臂坐标", 
                  command=lambda: self.copy_arm_coordinates('right')).pack(side=tk.LEFT, padx=2)
        
        self.right_arm_text = tk.Text(right_arm_frame, height=4, width=45)
        self.right_arm_text.pack(fill=tk.BOTH, expand=True, pady=2)
        
        right_scrollbar = ttk.Scrollbar(right_arm_frame, orient=tk.VERTICAL, command=self.right_arm_text.yview)
        right_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.right_arm_text.configure(yscrollcommand=right_scrollbar.set)
        
        # 状态显示
        status_frame = ttk.LabelFrame(main_frame, text="状态信息", padding="10")
        status_frame.pack(fill=tk.X, pady=5)
        
        self.status_label = ttk.Label(status_frame, text="就绪")
        self.status_label.pack(anchor=tk.W)
        
        # 关节角度显示
        joints_frame = ttk.LabelFrame(main_frame, text="双臂关节角度", padding="10")
        joints_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # 创建左右分栏
        paned_window = ttk.PanedWindow(joints_frame, orient=tk.HORIZONTAL)
        paned_window.pack(fill=tk.BOTH, expand=True)
        
        # 左臂关节显示
        left_frame = ttk.Frame(paned_window)
        paned_window.add(left_frame, weight=1)
        
        ttk.Label(left_frame, text="左臂关节角度 (°)", font=("Arial", 10, "bold")).pack(pady=5)
        
        self.left_arm_labels = []
        for i in range(7):
            frame = ttk.Frame(left_frame)
            frame.pack(fill=tk.X, pady=2, padx=10)
            ttk.Label(frame, text=f"关节 {i+1}:").pack(side=tk.LEFT)
            label = ttk.Label(frame, text="0.000", width=10)
            label.pack(side=tk.RIGHT)
            self.left_arm_labels.append(label)
        
        # 右臂关节显示
        right_frame = ttk.Frame(paned_window)
        paned_window.add(right_frame, weight=1)
        
        ttk.Label(right_frame, text="右臂关节角度 (°)", font=("Arial", 10, "bold")).pack(pady=5)
        
        self.right_arm_labels = []
        for i in range(7):
            frame = ttk.Frame(right_frame)
            frame.pack(fill=tk.X, pady=2, padx=10)
            ttk.Label(frame, text=f"关节 {i+1}:").pack(side=tk.LEFT)
            label = ttk.Label(frame, text="0.000", width=10)
            label.pack(side=tk.RIGHT)
            self.right_arm_labels.append(label)
    
    def get_current_joint_coordinates(self):
        """获取当前关节坐标并显示在文本框中"""
        try:
            left_arm_string, right_arm_string = self.player.get_current_joint_angles_degrees()
            
            # 更新左臂文本框
            self.left_arm_text.delete(1.0, tk.END)
            self.left_arm_text.insert(1.0, left_arm_string)
            
            # 更新右臂文本框
            self.right_arm_text.delete(1.0, tk.END)
            self.right_arm_text.insert(1.0, right_arm_string)
            
            self.update_status("已获取当前关节坐标")
        except Exception as e:
            self.update_status(f"获取关节坐标失败: {e}")
    
    def copy_arm_coordinates(self, arm_type):
        """复制指定手臂的坐标到剪贴板"""
        try:
            if arm_type == 'left':
                coordinates = self.left_arm_text.get(1.0, tk.END).strip()
                message = "左臂坐标"
            elif arm_type == 'right':
                coordinates = self.right_arm_text.get(1.0, tk.END).strip()
                message = "右臂坐标"
            else:
                return
            
            if coordinates:
                self.root.clipboard_clear()
                self.root.clipboard_append(coordinates)
                self.update_status(f"{message}已复制到剪贴板")
            else:
                self.update_status(f"{message}文本框为空，请先获取关节坐标")
        except Exception as e:
            self.update_status(f"复制{arm_type}坐标到剪贴板失败: {e}")
    
    def load_trajectory_file(self):
        """加载轨迹文件"""
        file_path = filedialog.askopenfilename(
            title="选择轨迹文件",
            filetypes=[("文本文件", "*.txt"), ("所有文件", "*.*")]
        )
        
        if file_path:
            success = self.player.load_trajectory_file(file_path)
            if success:
                self.file_label.config(text=os.path.basename(file_path))
                self.update_status(f"已加载轨迹文件: {os.path.basename(file_path)}，共 {len(self.player.trajectory_data)} 个轨迹点")
            else:
                self.update_status("加载轨迹文件失败")
    
    def start_visualization(self):
        """启动可视化"""
        success = self.player.setup_visualization()
        if success:
            self.update_status("可视化已启动")
        else:
            self.update_status("启动可视化失败")
    
    def play_trajectory(self):
        """播放轨迹"""
        if self.player.trajectory_data is None:
            self.update_status("请先加载轨迹文件")
            return
        
        if not self.player.visualization_enabled:
            self.update_status("请先启动可视化")
            return
        
        self.player.play_trajectory()
        self.update_status("正在播放轨迹...")
    
    def stop_trajectory(self):
        """停止播放"""
        self.player.stop_trajectory()
        self.update_status("已停止播放")
    
    def reset_trajectory(self):
        """重置轨迹"""
        self.player.current_index = 0
        self.player.stop_trajectory()
        self.update_status("已重置轨迹")
        def play_trajectory(self):
         """播放轨迹"""
        if self.player.trajectory_data is None:
            self.update_status("请先加载轨迹文件")
            return
        
        if not self.player.visualization_enabled:
            self.update_status("请先启动可视化")
            return
        
        self.player.play_trajectory()
        self.update_status("正在播放轨迹...")
    def update_speed(self, value):
        """更新播放速度"""
        speed = float(value)
        self.player.set_play_speed(speed)
        self.speed_label.config(text=f"{speed:.1f}x")
    
    def update_display(self):
        """更新界面显示"""
        # 更新左臂关节角度显示
        for i, angle in enumerate(self.player.joint_angles_left_arm):
            self.left_arm_labels[i].config(text=f"{np.rad2deg(angle):.3f}")
        
        # 更新右臂关节角度显示
        for i, angle in enumerate(self.player.joint_angles_right_arm):
            self.right_arm_labels[i].config(text=f"{np.rad2deg(angle):.3f}")
        
        # 更新进度信息
        if self.player.trajectory_data is not None:
            progress = (self.player.current_index / len(self.player.trajectory_data)) * 100
            self.update_status(f"播放进度: {progress:.1f}% - 轨迹点: {self.player.current_index}/{len(self.player.trajectory_data)}")
    
    def update_status(self, message):
        """更新状态信息"""
        self.status_label.config(text=message)
        self.root.update()

def main():
    # 创建主窗口
    root = tk.Tk()
    
    # 创建GUI
    app = TrajectoryPlayerGUI(root)
    
    # 启动主循环
    root.mainloop()

if __name__ == "__main__":
    main()