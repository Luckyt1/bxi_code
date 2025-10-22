from threading import Thread  #键盘线程控制
from rclpy.node import Node
import time
import rclpy
import numpy as np
import communication.msg as bxiMsg
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