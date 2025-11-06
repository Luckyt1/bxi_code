#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from communication.msg import MotionCommands

import threading
import math


class CmdVelToMotion(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_motion')

        # === 参数声明 ===
        self.declare_parameter('mode', 1)
        self.declare_parameter('filter_alpha_xy', 0.03)
        self.declare_parameter('filter_alpha_yaw', 0.05)
        self.declare_parameter('dead_zone', 0.05)
        self.declare_parameter('max_vel_x', 1.0)
        self.declare_parameter('min_vel_x', -0.5)
        self.declare_parameter('max_vel_y', 0.4)
        self.declare_parameter('min_vel_y', -0.4)
        self.declare_parameter('max_yaw_rate', 0.6)
        self.declare_parameter('min_yaw_rate', -0.6)

        # === 读取参数 ===
        self.mode = self.get_parameter('mode').value
        self.alpha_xy = self.get_parameter('filter_alpha_xy').value
        self.alpha_yaw = self.get_parameter('filter_alpha_yaw').value
        self.dead_zone = self.get_parameter('dead_zone').value
        self.max_vel_x = self.get_parameter('max_vel_x').value
        self.min_vel_x = self.get_parameter('min_vel_x').value
        self.max_vel_y = self.get_parameter('max_vel_y').value
        self.min_vel_y = self.get_parameter('min_vel_y').value
        self.max_yaw = self.get_parameter('max_yaw_rate').value
        self.min_yaw = self.get_parameter('min_yaw_rate').value

        # === 状态变量（线程安全）===
        self.lock = threading.Lock()
        self.raw_x = 0.0
        self.raw_y = 0.0
        self.raw_yaw = 0.0
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_yaw = 0.0

        # === ROS 接口 ===
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.motion_pub = self.create_publisher(MotionCommands, 'motion_commands', 10)

        # 20Hz 定时发布
        self.timer = self.create_timer(0.05, self.publish_filtered_command)  # 50ms

        self.get_logger().info(f"CmdVelToMotion node started with mode={self.mode}")

    # === 死区处理 ===
    def apply_dead_zone(self, value, dead_zone):
        return 0.0 if abs(value) < dead_zone else value

    # === 限幅 ===
    def clamp(self, value, min_val, max_val):
        return max(min_val, min(max_val, value))

    # === 订阅回调 ===
    def cmd_vel_callback(self, msg: Twist):
        with self.lock:
            # 读取原始速度
            self.raw_x = msg.linear.x
            self.raw_y = msg.linear.y
            self.raw_yaw = msg.angular.z

            # 死区
            self.raw_x = self.apply_dead_zone(self.raw_x, self.dead_zone)
            self.raw_y = self.apply_dead_zone(self.raw_y, self.dead_zone)
            self.raw_yaw = self.apply_dead_zone(self.raw_yaw, self.dead_zone * 2)

            # 限幅
            self.raw_x = self.clamp(self.raw_x, self.min_vel_x, self.max_vel_x)
            self.raw_y = self.clamp(self.raw_y, self.min_vel_y, self.max_vel_y)
            self.raw_yaw = self.clamp(self.raw_yaw, self.min_yaw, self.max_yaw)

    # === 定时发布（滤波后）===
    def publish_filtered_command(self):
        with self.lock:
            # 一阶低通滤波
            self.filtered_x = self.raw_x * self.alpha_xy + self.filtered_x * (1.0 - self.alpha_xy)
            self.filtered_y = self.raw_y * self.alpha_xy + self.filtered_y * (1.0 - self.alpha_xy)
            self.filtered_yaw = self.raw_yaw * self.alpha_yaw + self.filtered_yaw * (1.0 - self.alpha_yaw)

            # 构造消息
            msg = MotionCommands()
            msg.vel_des.x = self.filtered_x
            msg.vel_des.y = self.filtered_y
            msg.yawdot_des = self.filtered_yaw
            msg.mode = self.mode

            # 其他按钮默认 0
            msg.btn_6 = 0
            msg.btn_7 = 0

            self.motion_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
