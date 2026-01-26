import numpy as np
import cv2
import csv
import os
import rclpy
from rclpy.node import Node
import pygame
import time
from threading import Thread
from communication.msg import ActuatorCmds

# 全局变量
keyboard_use = True
keyboard_opened = False
date_collect_cmd = False
data_group_count = 0
frame_count = 0
exit_flag = False

# 创建数据保存目录
dataset_dir = "collected_data"
csv_dir = os.path.join(dataset_dir, "csv")
camera0_imgs_dir = os.path.join(dataset_dir, "hand_imgs")  # 第一个摄像头
camera1_imgs_dir = os.path.join(dataset_dir, "head_imgs")  # 第二个摄像头

os.makedirs(dataset_dir, exist_ok=True)
os.makedirs(csv_dir, exist_ok=True)
os.makedirs(camera0_imgs_dir, exist_ok=True)
os.makedirs(camera1_imgs_dir, exist_ok=True)

# ROS数据
ros_control_enabled = False
ros_target_q = np.zeros(31, dtype=np.double)
ros_msg_count = 0

# 初始化多个摄像头
cameras = []
camera_names = ["hand_camera", "head_camera"]
for i in range(2):
    if i==1 :
        cap = cv2.VideoCapture(0)
    else:
        cap = cv2.VideoCapture(2)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cameras.append(cap)
        print(f"摄像头 {i} 已打开 - 分辨率: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    else:
        print(f"警告: 摄像头 {i} 无法打开")
        cameras.append(None)

if all(cam is None for cam in cameras):
    print("错误: 没有可用的摄像头")
    exit()
def delete_last_group():
    """删除上一组数据"""
    global data_group_count
    
    if data_group_count == 0:
        print("没有可删除的数据组")
        return
    
    group_to_delete = data_group_count
    
    # 删除CSV文件
    csv_file = os.path.join(csv_dir, f"{group_to_delete}.csv")
    if os.path.exists(csv_file):
        os.remove(csv_file)
        print(f"已删除CSV文件: {csv_file}")
    
    # 删除图片文件
    deleted_count = 0
    for img_dir, img_type in [(camera1_imgs_dir, "head"), (camera0_imgs_dir, "hand")]:
        for filename in os.listdir(img_dir):
            if filename.startswith(f"{group_to_delete}_"):
                img_path = os.path.join(img_dir, filename)
                os.remove(img_path)
                deleted_count += 1
    
    print(f"已删除第 {group_to_delete} 组数据 (共删除 {deleted_count} 张图片)")
    data_group_count -= 1
# 键盘控制
if keyboard_use:
    pygame.init()
    try:
        screen = pygame.display.set_mode((200, 100))
        pygame.display.set_caption("Keyboard Control")
        keyboard_opened = True
        print("按C开始采集,按V停止采集,按ESC退出")
    except Exception as e:
        print(f"无法初始化键盘：{e}")

    def handle_keyboard_input():
        global exit_flag, date_collect_cmd
        
        c_key_pressed = False
        v_key_pressed = False
        z_key_pressed = False
        while not exit_flag:
            keys = pygame.key.get_pressed()
            
            if keys[pygame.K_c]:
                if not c_key_pressed and not date_collect_cmd:
                    date_collect_cmd = True
                    print("开始数据采集")
                    c_key_pressed = True
            else:
                c_key_pressed = False
            
            if keys[pygame.K_v]:
                if not v_key_pressed and date_collect_cmd:
                    date_collect_cmd = False
                    print("停止数据采集")
                    v_key_pressed = True
            else:
                v_key_pressed = False
                
            if keys[pygame.K_z]:
                if not z_key_pressed:
                    delete_last_group()
                    z_key_pressed = True
            else:
                z_key_pressed = False
                
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    exit_flag = True
            
            pygame.time.delay(50)

    if keyboard_opened:
        keyboard_thread = Thread(target=handle_keyboard_input, daemon=True)
        keyboard_thread.start()

def save_data(group_id, frame_id, target_qs, frames):
    """保存多个摄像头的图像和CSV数据"""
    img_dirs = [camera0_imgs_dir, camera1_imgs_dir]
    
    # 保存每个摄像头的图像
    for i, (frame, img_dir) in enumerate(zip(frames, img_dirs)):
        if frame is not None:
            img_filename = f"{group_id}_{frame_id}.jpg"
            img_path = os.path.join(img_dir, img_filename)
            success = cv2.imwrite(img_path, frame)
            
            if not success:
                print(f"警告: 摄像头{i}图像保存失败 {img_path}")
                return False
    
    # 保存CSV
    csv_filename = f"{group_id}.csv"
    csv_path = os.path.join(csv_dir, csv_filename)
    file_exists = os.path.isfile(csv_path)
    
    with open(csv_path, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        if not file_exists:
            writer.writerow(['image_name', 'l1_pos', 'l2_pos', 'l3_pos', 'l4_pos', 'l5_pos', 'l6_pos', 'l7_pos', 'l8_pos'])
        
        image_name = f"{group_id}_{frame_id}"
        writer.writerow([
            image_name, 
            target_qs[15], target_qs[16], target_qs[17], target_qs[18], target_qs[19],
            target_qs[20], target_qs[21], target_qs[22]
        ])
    
    return True

class ActuatorCmdsSubscriber(Node):
    def __init__(self):
        super().__init__('actuator_cmds_subscriber')
        self.subscription = self.create_subscription(
            ActuatorCmds,
            '/hardware/arm_actuators_cmds',
            self.listener_callback,
            10)
        self.get_logger().info('ActuatorCmds订阅者已启动')

    def listener_callback(self, msg):
        global ros_control_enabled, ros_target_q, ros_msg_count
        ros_control_enabled = True
        ros_msg_count += 1
        
        if ros_msg_count % 100 == 0:
            self.get_logger().info(f'已接收 {ros_msg_count} 条ROS消息')
        
        ros_target_q[15] = msg.pos[0]
        ros_target_q[16] = msg.pos[1]
        ros_target_q[17] = msg.pos[2]
        ros_target_q[18] = msg.pos[3]
        ros_target_q[19] = msg.pos[4]
        ros_target_q[20] = msg.pos[10]
        ros_target_q[21] = msg.pos[11]
        ros_target_q[22] = msg.pos[12]

def ros_thread_func():
    """ROS2线程函数"""
    rclpy.init()
    node = ActuatorCmdsSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # 启动ROS2线程
    ros_thread = Thread(target=ros_thread_func, daemon=True)
    ros_thread.start()
    
    for i, cam in enumerate(cameras):
        print(f"  摄像头{i}: {'已打开' if cam and cam.isOpened() else '未打开'}")
    print(f"  键盘控制: {'已启用' if keyboard_opened else '未启用'}")
    
    # 等待ROS消息
    time.sleep(2)
    
    prev_collect_cmd = False
    last_save_time = time.time()
    save_interval = 0.05  # 20Hz采集频率
    
    camera_frame_count = 0
    last_status_time = time.time()
    
    try:
        while not exit_flag:
            # 读取所有摄像头
            frames = []
            all_frames_valid = True
            
            for i, cam in enumerate(cameras):
                if cam and cam.isOpened():
                    ret, frame = cam.read()
                    if ret:
                        frames.append(frame)
                    else:
                        print(f"警告: 摄像头{i}读取失败")
                        frames.append(None)
                        all_frames_valid = False
                else:
                    frames.append(None)
                    all_frames_valid = False
            
            if not any(f is not None for f in frames):
                print("错误: 所有摄像头都无法读取")
                break
            
            camera_frame_count += 1
            
            # 显示所有摄像头画面
            for i, frame in enumerate(frames):
                if frame is not None:
                    display_frame = frame.copy()
                    
                    # 在画面上显示状态
                    status_text = f"Cam{i} | ROS: {'ON' if ros_control_enabled else 'OFF'} | Collecting: {'YES' if date_collect_cmd else 'NO'} | Group: {data_group_count} | Frame: {frame_count}"
                    cv2.putText(display_frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    cv2.imshow(camera_names[i], display_frame)
            
            # 检查采集命令状态变化
            if date_collect_cmd != prev_collect_cmd:
                if date_collect_cmd:
                    data_group_count += 1
                    frame_count = 0
                    last_save_time = time.time()
                    print(f"\n===== 开始采集第 {data_group_count} 组数据 =====")
                    print(f"ROS状态: {ros_control_enabled}")
                    print(f"当前ROS数据样例: {ros_target_q[15:23]}")
                else:
                    print(f"\n===== 第 {data_group_count} 组采集完成,共 {frame_count} 帧 =====\n")
                prev_collect_cmd = date_collect_cmd
            
            # 如果正在采集且有ROS数据，按固定频率保存
            current_time = time.time()
            if date_collect_cmd and ros_control_enabled and all_frames_valid and (current_time - last_save_time >= save_interval):
                if save_data(data_group_count, frame_count, ros_target_q, frames):
                    frame_count += 1
                    last_save_time = current_time
                    
                    if frame_count % 20 == 0:
                        print(f"已采集 {frame_count} 帧 (ROS消息数: {ros_msg_count})")
            
            # 每5秒打印一次状态
            if current_time - last_status_time >= 5.0:
                print(f"\n状态更新:")
                print(f"  摄像头帧数: {camera_frame_count}")
                print(f"  ROS消息数: {ros_msg_count}")
                print(f"  ROS已启用: {ros_control_enabled}")
                print(f"  正在采集: {date_collect_cmd}")
                if date_collect_cmd:
                    print(f"  当前组号: {data_group_count}, 已采集帧数: {frame_count}")
                last_status_time = current_time
            
            # 按ESC退出
            if cv2.waitKey(1) & 0xFF == 27:
                exit_flag = True
                break
    
    except KeyboardInterrupt:
        print("\n收到中断信号")
    finally:
        # 释放所有摄像头
        for i, cam in enumerate(cameras):
            if cam:
                cam.release()
                print(f"摄像头{i}已释放")
        cv2.destroyAllWindows()
        exit_flag = True
        print("\n程序结束")
        print(f"总计:")
        print(f"  摄像头帧数: {camera_frame_count}")
        print(f"  ROS消息数: {ros_msg_count}")
        print(f"  已保存数据组: {data_group_count}")