import numpy as np
import sys
import rclpy
import time
import pygame
from threading import Thread  #键盘线程控制
from rclpy.node import Node
import communication.msg as bxiMsg
exit_flag = False  # 退出标志
keyboard_use = True  # 是否使用键盘控制
if keyboard_use:
    pygame.init()  # 初始化pygame
    try:
        screen = pygame.display.set_mode((200, 100))  # 创建一个小窗口以捕获键盘事件
        keyboard_opened=True
    except Exception as e:
        print("无法打开键盘控制窗口，键盘控制不可用:", e)
        keyboard_opened=False
 
    def handle_keyboard_input():
        global exit_flag
        while not exit_flag:
            keys = pygame.key.get_pressed()

            if keys[pygame.K_c]:
                print("前进")
            if keys[pygame.K_d]:
                print("后退")
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    exit_flag = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        exit_flag = True
            pygame.time.delay(50)  # 50ms延迟，减少CPU使用
    if keyboard_opened and keyboard_use:
        keyboard_thread = Thread(target=handle_keyboard_input)
        keyboard_thread.start()

joint_nominal_pos = np.array([   # 指定的固定关节角度
    0.0, 0.0, 0.0,
    0,0.0,-0.3,0.6,-0.3,0.0,
    0,0.0,-0.3,0.6,-0.3,0.0,
    0.1,0.0,0.0,-0.3,0.0,     # 左臂放在大腿旁边
    0.1,0.0,0.0,-0.3,0.0,
    0,0,0,
    0,0,0],    # 右臂放在大腿旁边
    dtype=np.float32)

joint_kp = np.array([     # 指定关节的kp，和joint_name顺序一一对应
    0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    0,0,15,
    0,0,0,], dtype=np.float32)

joint_kd = np.array([  # 指定关节的kd，和joint_name顺序一一对应
    0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,
    0,0,0.5,
    0,0,0], dtype=np.float32)
# joint_kp = np.array([     # 指定关节的kp，和joint_name顺序一一对应
#     0,0,0,
#     0,0,0,0,0,0,
#     0,0,0,0,0,0,
#     40,50,20,50,20,
#     0,0,0,0,0,
#     20,20,50,
#     0,0,0,], dtype=np.float32)

# joint_kd = np.array([  # 指定关节的kd，和joint_name顺序一一对应
#     0,0,0,
#     0,0,0,0,0,0,
#     0,0,0,0,0,0,
#     1.0,1.0,0.8,1.0,0.8,
#     0,0,0,0,0,
#     0.5,0.3,1,
#     0,0,0], dtype=np.float32)

class WristControlNode(Node):
    def __init__(self):
        super().__init__('wrist_control_node')
       
        # 创建发布器
        self.qpos_publisher = self.create_publisher(
            bxiMsg.ActuatorCmds, 
            "/hardware/arm_actuators_cmds",
            10
        )
        self.timer = self.create_timer(0.01, self.process_and_publish)  # 100Hz
        self.arm_qpos = np.zeros(16,dtype=np.float32)
        self.start_time = time.time()

    def process_and_publish(self):
        """处理目标角度并发布到ROS2主题"""
        try:
            msg = bxiMsg.ActuatorCmds()
            
            soft_k = np.clip((time.time() - self.start_time) / 3.0, 0.1, 1.0)
            soft_kp = joint_kp[-16:] * soft_k
            kp_mask =  [0,0,0,0,0,
                        0,0,0,0,0,
                        0,0,1,
                        0,0,0]
                
            msg.kp = (soft_kp * kp_mask).tolist()
            msg.kd = joint_kd[-16:].tolist()
            msg.pos = self.arm_qpos.tolist()
            msg.vel = np.zeros_like(self.arm_qpos).tolist()
            msg.torque = np.zeros_like(self.arm_qpos).tolist()
            
            self.qpos_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"处理和发布数据时出错: {e}")

    def destroy_node(self):
        """销毁节点时的清理工作"""
        self.listening = False
        if hasattr(self, 'udp_receiver'):
            self.udp_receiver.stop()
        super().destroy_node()