"""
右手数据到机械臂映射的ROS2节点
基于ESP32 UDP数据，映射到机械臂，并通过ROS2发布qpos数据
"""

import socket
import json
import numpy as np
import signal
import sys
import time
import threading
import struct
import select
import os
import fcntl
from math import radians, degrees
from scipy.spatial.transform import Rotation
from queue import Queue
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from threading import Thread  #键盘线程控制

import communication.msg as bxiMsg
import termios
import tty
import pygame
# 摇杆阈值（摇杆触发时的值）
UDP_HOST = "0.0.0.0"  # 监听所有网络接口
UDP_PORT = 8080       # UDP端口，与您的接收器保持一致
TIMEOUT = 2.0         # UDP接收超时时间（秒）
STICK_THRESHOLD = 30000  # 摇杆最大值约为32767
GRIPPER_STEP = 0.5       # 夹爪每次移动的步长
ANGLE_UNIT = "degrees"  # "degrees" 或 "radians" - ESP32发送的角度单位
keyboard_use = True  # 是否使用键盘控制
if keyboard_use:
    pygame.init()  # 初始化pygame
    try:
        # 设置窗口用于接收键盘输入
        screen = pygame.display.set_mode((200, 100))
        pygame.display.set_caption("Keyboard Control")
        keyboard_opened = True
  
        print("空格键: 停止所有运动")
    except Exception as e:
        print(f"无法初始化键盘：{e}")
    # 键盘线程退出标志
    exit_flag = False
    def handle_keyboard_input():
        """处理键盘输入的线程函数"""
        global key_press_flag ,node
        node = WristControlNode()
        key_press_flag=False  # 防止按键连发标志
        while not exit_flag:
            # 获取键盘输入
            keys = pygame.key.get_pressed()
            
            if keys[pygame.K_a]:
                node.control_mode='auto'
            if keys[pygame.K_d]:
                node.control_mode='manual'
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    exit_flag = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        exit_flag = True
            
            pygame.time.delay(50)  # 50ms延迟,减少CPU使用

    # 启动键盘输入处理线程
    if keyboard_opened and keyboard_use:
        keyboard_thread = Thread(target=handle_keyboard_input)
        keyboard_thread.start()

JOINT_DIRECTION = {
    1: 1,     # 第1个关节：正方向 (1) 或反方向 (-1)
    2: 1,    # 第2个关节：反方向
    3: 1,     # 第3个关节：正方向
    4: -1,    # 第4个关节：反方向
    5: 1,     # 第5个关节：正方向
    6: 1,     # 第6个关节：正方向
    7: 1,     # 第7个关节：正方向
    8: 1,     # 第8个关节：正方向 (如果有的话)

    9: -1,     # 第1个关节：正方向 (1) 或反方向 (-1)
    10: -1,    # 第2个关节：反方向
    11: 1,     # 第3个关节：正方向
    12: 1,    # 第4个关节：反方向
    13: 1,     # 第5个关节：正方向
    14: 1,     # 第6个关节：正方向
    15: 1,     # 第7个关节：正方向
    16: 1,     # 第8个关节：正方向 (如果有的话)
}

joint_name = (
    "waist_y_joint", "waist_x_joint", "waist_z_joint",
    "l_hip_z_joint", "l_hip_x_joint", "l_hip_y_joint", "l_knee_y_joint", "l_ankle_y_joint", "l_ankle_x_joint",
    "r_hip_z_joint", "r_hip_x_joint", "r_hip_y_joint", "r_knee_y_joint", "r_ankle_y_joint", "r_ankle_x_joint",
    "l_shld_y_joint", "l_shld_x_joint", "l_shld_z_joint", "l_elb_y_joint", "l_elb_z_joint",
    "r_shld_y_joint", "r_shld_x_joint", "r_shld_z_joint", "r_elb_y_joint", "r_elb_z_joint",
    "l_wrist_y_joint", "l_wrist_x_joint", "l_hand_joint",
    "r_wrist_y_joint", "r_wrist_x_joint", "r_hand_joint",    
)   

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
    100,100,50,70,50,
    0,0,0,0,0,
    20,20,20,
    0,0,0], dtype=np.float32)

joint_kd = np.array([  # 指定关节的kd，和joint_name顺序一一对应
    0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    5.0,5.0,1.0,2.0,0.8,
    0,0,0,0,0,
    0.4,0.5,0.4,
    0,0,0], dtype=np.float32)

# joint_kp = np.array([     # 指定关节的kp，和joint_name顺序一一对应
#     500,500,300,
#     100,100,100,150,30,10,
#     100,100,100,150,30,10,
#     40,50,15,40,15,
#     40,50,15,40,15,
#     15,15,10,
#     15,15,10,], dtype=np.float32)

# joint_kd = np.array([  # 指定关节的kd，和joint_name顺序一一对应
#     5,5,3,
#     2,2,2,2.5,1,1,
#     2,2,2,2.5,1,1,
#     1.0,1.0,0.8,1.0,0.8,
#     1.0,1.0,0.8,1.0,0.8,
#     0.4,0.4,0.5,
#     0.4,0.4,0.5], dtype=np.float32)
# 参考参数
last_angle = {}

class ESP32UDPReceiver:
    """ESP32 UDP数据接收器"""
    
    def __init__(self, host, port, timeout=2.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket = None
        self.running = False
        self.data_queue = Queue()
        self.latest_data = None  # 添加最新数据缓存
        self.stats = {
            'packets_received': 0,
            'packets_error': 0,
            'last_receive_time': 0,
            'last_timestamp': 0
        }
        
    def start(self):
        """启动UDP接收器"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.settimeout(self.timeout)
            self.socket.bind((self.host, self.port))
            self.running = True
            
            # 启动接收线程
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            
            print(f"UDP接收器已启动，监听 {self.host}:{self.port}")
            return True
            
        except Exception as e:
            print(f"启动UDP接收器失败: {e}")
            return False
    
    def stop(self):
        """停止UDP接收器"""
        self.running = False
        if self.socket:
            self.socket.close()
    
    def _receive_loop(self):
        """UDP接收循环"""
        while self.running:
            try:
                data, addr = self.socket.recvfrom(1024)
                self.stats['packets_received'] += 1
                self.stats['last_receive_time'] = time.time()
                # 解析JSON数据
                try:
                    message = data.decode('utf-8')
                    json_data = json.loads(message)
                    
                    # 更新时间戳
                    if 'timestamp' in json_data:
                        self.stats['last_timestamp'] = json_data['timestamp']
                    
                    # 更新最新数据
                    self.latest_data = json_data
                    
                    # 将数据放入队列
                    if not self.data_queue.full():
                        self.data_queue.put(json_data)
                    
                except (json.JSONDecodeError, UnicodeDecodeError) as e:
                    self.stats['packets_error'] += 1
                        
            except socket.timeout:
                # 超时是正常的，继续循环
                continue
            except Exception as e:
                if self.running:  # 只在运行时打印错误
                    self.stats['packets_error'] += 1
    
    def get_latest_data(self):
        """获取最新的数据"""
        return self.latest_data
    
    def get_stats(self):
        """获取统计信息"""
        return self.stats.copy()

def parse_esp32_data(json_data, num_joints):
    """解析ESP32发送的数据"""
    try:
        # 检查输入数据是否为None
        if json_data is None:
            # 返回默认值而不是None
            return np.zeros(num_joints, dtype=np.float32)
        
        angles = None
        
        # 支持多种数据格式
        if 'sensors' in json_data:
            # 格式: {"timestamp": ms, "sensors": [{"id": 1, "angle": 90.0}, ...]}
            sensors = json_data['sensors']
            angles = []
            
            # 按传感器ID排序
            sensors_sorted = sorted(sensors, key=lambda x: x.get('id', 0))
            
            for sensor in sensors_sorted:
                sensor_id = sensor.get('id', 0)
                angle = sensor.get('angle', 0.0)
                
                direction = JOINT_DIRECTION.get(sensor_id+1, 1)
                final_angle = angle * direction

                angles.append(final_angle)
        else:
            # 如果没有sensors字段，返回默认值
            angles = [0.0] * num_joints
        
        # 转换为numpy数组
        angles = np.array(angles, dtype=float)
        
        # 角度单位转换
        if ANGLE_UNIT == "degrees":
            angles = np.radians(angles)  # 度转弧度
        
        # 确保角度数量匹配
        if len(angles) > num_joints:
            angles = angles[:num_joints]
        elif len(angles) < num_joints:
            # 补零
            padding = np.zeros(num_joints - len(angles))
            angles = np.concatenate([angles, padding])
        
        return angles.astype(np.float32)
        
    except Exception as e:
        print(f"解析ESP32数据失败: {e}")
        # 返回默认值而不是None
        return np.zeros(num_joints, dtype=np.float32)

class WristControlNode(Node):
    """ROS2节点，用于接收ESP32数据并发布机械臂关节位置"""
    def __init__(self):
        super().__init__('wrist_control_node')
        
        # 创建发布器
        self.qpos_publisher = self.create_publisher(
            bxiMsg.ActuatorCmds, 
            "/hardware/arm_actuators_cmds",
            10
        )
        # 控制标志
        self.listening = True
        self.current_pos = np.zeros(16, dtype=np.float32)
        self.radians=np.zeros(16,dtype=np.float32)
        self.current_pos[:] = joint_nominal_pos[-16:]  # 使用标准姿态初始化

        # 控制模式：'auto' 或 'manual'
        self.control_mode = 'auto'

        # 初始化UDP接收器（只创建一次）
        self.udp_receiver = ESP32UDPReceiver(UDP_HOST, UDP_PORT, TIMEOUT)
        self.udp_started = False

        # 创建定时器来处理数据和发布
        self.timer = self.create_timer(0.001, self.process_and_publish)  # 100Hz
        self.exec_times = []
        self.display_counter = 0
        # 缓启动
        self.last_qpos = None
        self.smooth_factor = 0.01  # 平滑因子，值越小越平滑
        self.max_angle_step = 0.3  # 最大单步角度变化（弧度）
        self.start_time = time.time()
        
    def display_esp32_data(self, esp32_pos, latest_data):
        """原地刷新显示ESP32数据"""
        # 清除当前行并回到行首
        sys.stdout.write('\r' + ' ' * 100 + '\r')
        
        # 格式化显示esp32_pos数据
        if esp32_pos is not None and len(esp32_pos) > 0:
            angles_deg = esp32_pos
            angles_str = ' | '.join([f"J{i}:{angle:6.1f}rad" for i, angle in enumerate(angles_deg)])
            
            # 显示统计信息
            stats = self.udp_receiver.get_stats()
            timestamp = latest_data.get('timestamp', 0) if latest_data else 0
            
            # 组合显示信息
            display_info = f"ESP32: {angles_str} | {stats['packets_received']}包 | {timestamp}"
            
            # 原地显示（不换行）
            sys.stdout.write(display_info)
            sys.stdout.flush()
        else:
            sys.stdout.write(" 等待ESP32数据...")
            sys.stdout.flush()
    def angle_difference(self, target, current):
        """计算两个角度之间的最短差值，考虑-π到π的循环性"""
        diff = target - current
        
        # 将差值规范化到[-π, π]范围内
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
            
        return diff
    def smooth_angle_transition(self, target_angles, current_angles=None):
        """平滑角度过渡，正确处理-π到π的跳跃"""
        target_angles = np.array(target_angles, dtype=np.float32)
        
        # 使用self.last_qpos作为当前角度
        if self.last_qpos is None:
            self.last_qpos = target_angles.copy()
            return target_angles.copy()
        
        current_angles = self.last_qpos  # 使用上一次的角度作为当前角度
        smooth_angles = np.zeros_like(target_angles)
        
        for i in range(len(target_angles)):
            # 检测是否存在异常跳变
            angle_diff = self.angle_difference(target_angles[i], current_angles[i])
        
            # 限制最大单步变化
            if abs(angle_diff) > self.max_angle_step:
                angle_diff = np.sign(angle_diff) * self.max_angle_step
            
            # 应用平滑因子
            smooth_step = angle_diff * self.smooth_factor
            
            # 计算新角度
            new_angle = current_angles[i] + smooth_step
            
            # 规范化到[-π, π]范围
            smooth_angles[i] = new_angle
    
        # 更新历史角度
        self.last_qpos = smooth_angles.copy()
        
        return smooth_angles
    
    def process_and_publish(self):
        """处理数据并发布qpos"""
        start_time = time.time()
        global key_press_flag
        try:
            # 根据控制模式计算qpos
            if self.control_mode == 'auto' and self.udp_started:
                # 自动模式：使用ESP32数据
                latest_data = self.udp_receiver.get_latest_data()
                esp32_pos = parse_esp32_data(latest_data, 16)
                
                # 改进的角度限制和跳变检测
                for i in range(len(esp32_pos)):
                    # 额外的跳变检测
                    if self.last_qpos is not None and i < len(self.last_qpos):
                        angle_diff = abs(esp32_pos[i] - self.last_qpos[i])
                        # 如果单个关节变化超过90度，保持上一个值
                        if angle_diff > np.pi:
                                esp32_pos[i] = self.last_qpos[i]
                # 映射到机械臂关节
                new_radians = self.radians.copy()
                new_radians[0] = esp32_pos[0]
                new_radians[1] = esp32_pos[1]
                new_radians[2] = esp32_pos[2]
                new_radians[3] = esp32_pos[3]
                new_radians[4] = esp32_pos[4]
                new_radians[10] = esp32_pos[5]
                new_radians[11] = esp32_pos[6]
                new_radians[12] = esp32_pos[7]
                
                # new_radians[0] = esp32_pos[8]
                # new_radians[1] = esp32_pos[9]
                # new_radians[2] = esp32_pos[10]
                # new_radians[3] = esp32_pos[11]
                # new_radians[4] = esp32_pos[12]
                # new_radians[10] = esp32_pos[13]
                # new_radians[11] = esp32_pos[14]
                # new_radians[12] = esp32_pos[15]

                # new_radians[5] = esp32_pos[0]
                # new_radians[6] = esp32_pos[1]
                # new_radians[7] = esp32_pos[2]
                # new_radians[8] = esp32_pos[3]
                # new_radians[9] = esp32_pos[4]
                # new_radians[13] = esp32_pos[5]
                # new_radians[14] = esp32_pos[6]
                # new_radians[15] = esp32_pos[7]
                # print(f"esp32_pos:{esp32_pos[4]}")
                # 应用平滑过渡
                if(new_radians[12]>1):
                    new_radians[12]=-3.14
                # if(new_radians[15]>1):
                #     new_radians[15]=-3.14
                self.radians = self.smooth_angle_transition(new_radians)
                self.radians[12]=self.radians[12]*2
                # self.radians[15]=self.radians[15]*2
           
                qpos = self.radians
                
                self.display_counter += 1
                if self.display_counter % 5 == 0:
                    self.display_esp32_data(self.radians, latest_data)
                    
            elif self.control_mode == 'manual' or not self.udp_started:
                new_radians = self.radians.copy()
                new_radians[0] = 0.785
                new_radians[1] = 0.0
                new_radians[2] = 0.0
                new_radians[3] = -0.785-1.56
                new_radians[4] = -1.56
                new_radians[10] = 0.0
                new_radians[11] = 0.0
                new_radians[12] = 0.0
                self.radians = self.smooth_angle_transition(new_radians)
                qpos = self.radians
            else:
                # 如果没有ESP32数据，保持当前位置
                qpos = self.radians if hasattr(self, 'radians') else np.zeros(16, dtype=np.float32)
                
            # 创建消息
            msg = bxiMsg.ActuatorCmds()
            
            # 缓启动
            soft_k = np.clip((time.time() - self.start_time) / 3.0, 0.1, 1.0)
            soft_kp = joint_kp[-16:] * soft_k
             
            kp_mask =  [1,1,1,1,1,
                        0,0,0,0,0,
                        1,1,1,
                        0,0,0]
            
            msg.kp = (soft_kp * kp_mask).tolist()
            msg.kd = joint_kd[-16:].tolist()
            msg.pos = qpos.tolist()
            msg.vel = np.zeros_like(qpos).tolist()
            msg.torque = np.zeros_like(qpos).tolist()
            
            # 发布消息
            self.qpos_publisher.publish(msg)
            # exec_time = (time.time() - start_time) * 1000  # 毫秒
            # self.exec_times.append(exec_time)
            # if len(self.exec_times) > 100:
            #     avg_time = sum(self.exec_times) / len(self.exec_times)
            #     max_time = max(self.exec_times)
            #     print(f"执行时间 - 平均: {avg_time:.2f}ms, 最大: {max_time:.2f}ms")
            #     self.exec_times.clear()
        except Exception as e:
            self.get_logger().error(f"处理和发布数据时出错: {e}")
    
    def destroy_node(self):
        """销毁节点时的清理工作"""
        self.listening = False
        if hasattr(self, 'udp_receiver'):
            self.udp_receiver.stop()
        super().destroy_node()

def main(args=None):
    """主函数"""
    global node
    rclpy.init(args=args)
    
    try:
        node = WristControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()