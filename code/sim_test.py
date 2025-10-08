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

import communication.msg as bxiMsg
import termios
import tty

# 摇杆阈值（摇杆触发时的值）
STICK_THRESHOLD = 30000  # 摇杆最大值约为32767
GRIPPER_STEP = 0.5       # 夹爪每次移动的步长
ANGLE_UNIT = "degrees"  # "degrees" 或 "radians" - ESP32发送的角度单位
SMOOTH_FACTOR = 0.2   # 平滑因子 (0-1)，值越小越平滑
MAX_ANGLE_CHANGE = 0.2  # 单步最大角度变化（弧度），防止突变

JOINT_DIRECTION = {
    1: 1,     # 第1个关节：正方向 (1) 或反方向 (-1)
    2: -1,    # 第2个关节：反方向
    3: -1,     # 第3个关节：正方向
    4: -1,    # 第4个关节：反方向
    5: -1,     # 第5个关节：正方向
    6: -1,     # 第6个关节：正方向
    7: -1,     # 第7个关节：正方向
    8: -1,     # 第8个关节：正方向 (如果有的话)
}

JOINT_ZERO_OFFSETS = {
    1: 0,     # 第1个关节零点偏移: 0度
    2: 0,    # 第2个关节零点偏移: 90度 (当ESP32发送90度时，机械臂关节为0度)
    3: 0,     # 第3个关节零点偏移: 0度
    4: 0,     # 第4个关节零点偏移: 0度
    5: 0,     # 第5个关节零点偏移: 0度
    6: 0,     # 第6个关节零点偏移: 0度
    7: 0,     # 第7个关节零点偏移: 0度
    8: 0,     # 第8个关节零点偏移: 0度 (如果有的话)
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
    40,50,15,40,15,
    0,0,0,0,0,
    15,15,10,
    0,0,0,], dtype=np.float32)

joint_kd = np.array([  # 指定关节的kd，和joint_name顺序一一对应
    0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    1.0,1.0,0.8,1.0,0.8,
    0,0,0,0,0,
    0.4,0.4,0.5,
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
last_angle = {}
def limit_angle_range(angle, min_angle=-np.pi, max_angle=np.pi, joint_id=None):
    """将角度限制在指定范围内，支持循环限幅"""
    global last_angle


    reasonable_min = radians(-1000)
    reasonable_max = radians(1000)

    if angle > reasonable_max or angle < reasonable_min:
        # 如果是突变值，返回上一次的正常值
        if joint_id in last_angle:
            return last_angle[joint_id]
        else:
            # 如果没有历史值，返回0
            last_angle[joint_id] = 0.0
            return 0.0
        
    range_size = max_angle - min_angle
    while angle < min_angle:
        angle += range_size
    while angle > max_angle:
        angle -= range_size

    last_angle[joint_id] = angle
    return angle

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
        self.selected_joint = 0  # 当前选中的关节
    
        # 创建定时器来处理数据和发布
        self.timer = self.create_timer(0.01, self.process_and_publish)  # 100Hz

        self.gripper_l_real = -2.5
        self.gripper_r_real = -2.5
        self.display_counter = 0
        # 缓启动
        self.last_qpos = None
        self.smooth_factor = 0.1  # 平滑因子，值越小越平滑
        self.max_angle_step = 0.2  # 最大单步角度变化（弧度）
        self.start_time = time.time()

    
    def process_and_publish(self):
        """处理数据并发布qpos"""
            # 启动UDP接收器（只启动一次）
            # 根据控制模式计算qpos
        # 这里可以添加数据处理逻辑
        self.radians[0]=20
        self.radians[1]=20
        self.radians[2]=20
        self.radians[3]=20
        self.radians[4]=20
        self.radians[10]=0
        self.radians[11]=0
        self.radians[12]=0
        #左臂
        qpos=self.radians
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
        
        
        def destroy_node(self):
            """销毁节点时的清理工作"""
            self.listening = False
            super().destroy_node()

def main(args=None):
    """主函数"""
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