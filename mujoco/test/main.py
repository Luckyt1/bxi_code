import mujoco
import mujoco.viewer
import numpy as np
import time
import os

import socket
import json
import threading
from queue import Queue
import sys

# ------------------------------------------------------------------------------------
# 用户操作指南
# 1. 准备文件:
#    - 使用 `compile your_arm.urdf your_arm.xml` 命令将您的URDF转换为XML。
#    - 确保XML文件中有为关节定义的 <actuator> (执行器)。
#
# 2. 修改文件名:
#    - 将下面的 `XML_FILENAME` 和 `URDF_FILENAME` 变量的值更改为您的文件名。
#
# 3. ESP32设置:
#    - 确保ESP32发送JSON格式: {"timestamp": ms, "sensors": [{"id": 1, "angle": 90.0}, ...]}
#    - 或简化格式: {"angles": [angle1, angle2, ...]}
#    - 角度单位：度数（代码会自动转换为弧度）
#
# 4. 运行代码:
#    - 在终端中运行 `python test.py`。
#
# 预期效果:
# - 仿真启动后，机械臂会根据ESP32发送的角度数据实时更新姿态。
# - 如果UDP连接断开，仿真会保持最后一个有效姿态。
# ------------------------------------------------------------------------------------

# --- 配置 ---
XML_FILENAME = "../models/mjmodel.xml"
URDF_FILENAME = "../models/arm.SLDASM.urdf"

# UDP通信设置
UDP_HOST = "0.0.0.0"  # 监听所有网络接口
UDP_PORT = 8080       # UDP端口，与您的接收器保持一致
TIMEOUT = 2.0         # UDP接收超时时间（秒）

# 调试选项
DEBUG_MODE = True     # 设置为True可查看详细信息
PRINT_UDP_DATA = True # 设置为True可查看UDP接收的数据
UDP_STATS = True      # 显示UDP统计信息

# 控制参数
SMOOTH_FACTOR = 0.2   # 平滑因子 (0-1)，值越小越平滑
MAX_ANGLE_CHANGE = 0.2  # 单步最大角度变化（弧度），防止突变
ANGLE_UNIT = "degrees"  # "degrees" 或 "radians" - ESP32发送的角度单位

JOINT_DIRECTION = {
    1: -1,     # 第1个关节：正方向 (1) 或反方向 (-1)
    2: 1,    # 第2个关节：反方向
    3: 1,     # 第3个关节：正方向
    4: 1,    # 第4个关节：反方向
    5: 1,     # 第5个关节：正方向
    6: 1,     # 第6个关节：正方向
    7: -1,     # 第7个关节：正方向
    8: -1,     # 第8个关节：正方向 (如果有的话)
}

JOINT_ZERO_OFFSETS = {
    1: 0,     # 第1个关节零点偏移: 0度
    2: -90,    # 第2个关节零点偏移: 90度 (当ESP32发送90度时，机械臂关节为0度)
    3: 0,     # 第3个关节零点偏移: 0度
    4: 0,     # 第4个关节零点偏移: 0度
    5: 0,     # 第5个关节零点偏移: 0度
    6: 0,     # 第6个关节零点偏移: 0度
    7: 0,     # 第7个关节零点偏移: 0度
    8: 0,     # 第8个关节零点偏移: 0度 (如果有的话)
}

class ESP32UDPReceiver:
    """ESP32 UDP数据接收器"""
    
    def __init__(self, host, port, timeout=2.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket = None
        self.running = False
        self.data_queue = Queue()
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
                    
                    if PRINT_UDP_DATA:
                        print(f"收到来自 {addr} 的数据: {json_data}")
                    
                    # 将数据放入队列
                    if not self.data_queue.full():
                        self.data_queue.put(json_data)
                    
                except (json.JSONDecodeError, UnicodeDecodeError) as e:
                    self.stats['packets_error'] += 1
                    if PRINT_UDP_DATA:
                        print(f"数据解析错误: {e}, 原始数据: {data}")
                        
            except socket.timeout:
                # 超时是正常的，继续循环
                continue
            except Exception as e:
                if self.running:  # 只在运行时打印错误
                    self.stats['packets_error'] += 1
                    if DEBUG_MODE:
                        print(f"UDP接收错误: {e}")
    
    def get_latest_data(self):
        """获取最新的数据"""
        latest_data = None
        # 清空队列，只保留最新数据
        while not self.data_queue.empty():
            latest_data = self.data_queue.get()
        return latest_data
    
    def get_stats(self):
        """获取统计信息"""
        return self.stats.copy()


def validate_files(xml_path, urdf_path):
    """验证文件是否存在"""
    if not os.path.exists(xml_path):
        raise FileNotFoundError(
            f"错误: 在 '{xml_path}' 找不到XML文件。\n"
            f"请确保文件名正确，并且文件路径存在。"
        )
    
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(
            f"错误: 在 '{urdf_path}' 找不到URDF文件。\n"
            f"请确保文件名正确，并且文件路径存在。"
        )


def load_models(xml_path, urdf_path):
    """加载MuJoCo和Pinocchio模型"""
    # 加载MuJoCo模型
    try:
        mj_model = mujoco.MjModel.from_xml_path(xml_path)
        mj_data = mujoco.MjData(mj_model)
    except Exception as e:
        raise RuntimeError(f"加载MuJoCo模型失败: {str(e)}\n请检查XML文件格式。")
    
    # 加载Pinocchio模型（可选，主要用于验证）
    try:
        pin_model = pin.buildModelFromUrdf(urdf_path)
        pin_data = pin_model.createData()
    except Exception as e:
        print(f"警告: 加载Pinocchio模型失败: {str(e)}")
        pin_model = None
        pin_data = None
    
    return mj_model, mj_data, pin_model, pin_data


def validate_model_compatibility(mj_model):
    """验证模型兼容性"""
    if mj_model.nu == 0:
        print("警告: MuJoCo模型中没有找到执行器，将直接控制关节位置")
        return mj_model.nq  # 返回关节数量
    
    return mj_model.nu


def smooth_angle_transition(current_angles, target_angles, smooth_factor, max_change):
    """平滑角度过渡，防止突变"""
    if current_angles is None:
        return target_angles
    
    # 计算角度差
    angle_diff = target_angles - current_angles
    
    # 限制最大变化量
    angle_diff = np.clip(angle_diff, -max_change, max_change)
    
    # 平滑过渡
    new_angles = current_angles + smooth_factor * angle_diff
    
    return new_angles


def parse_esp32_data(json_data, num_joints):
    """解析ESP32发送的数据"""
    try:
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

                zero_offset = JOINT_ZERO_OFFSETS.get(sensor_id, 0)
                adjusted_angle = angle - zero_offset
                
                direction = JOINT_DIRECTION.get(sensor_id, 1)
                final_angle = adjusted_angle * direction

                angles.append(final_angle)
                # 设置关节零点偏移
        
        # 转换为numpy数组
        angles = np.array(angles, dtype=float)
        
        # 角度单位转换
        if ANGLE_UNIT == "degrees":
            angles = np.radians(angles)  # 度转弧度
        
        # 确保角度数量匹配
        if len(angles) > num_joints:
            angles = angles[:num_joints]
            if DEBUG_MODE:
                print(f"裁剪角度数组到 {num_joints} 个关节")
        elif len(angles) < num_joints:
            # 补零
            padding = np.zeros(num_joints - len(angles))
            angles = np.concatenate([angles, padding])
            if DEBUG_MODE:
                print(f"补零到 {num_joints} 个关节")
        
        return angles
        
    except Exception as e:
        print(f"解析ESP32数据失败: {e}")
        return None


def main():
    """主仿真函数"""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(current_dir, XML_FILENAME)
    urdf_path = os.path.join(current_dir, URDF_FILENAME)
    
    # 验证文件存在
    validate_files(xml_path, urdf_path)
    
    # 加载模型
    mj_model, mj_data, pin_model, pin_data = load_models(xml_path, urdf_path)
    
    # 验证模型兼容性
    num_joints = validate_model_compatibility(mj_model)
    
    if DEBUG_MODE:
        print(f"模型信息:")
        print(f"  MuJoCo模型广义坐标数量 (nq): {mj_model.nq}")
        print(f"  MuJoCo模型执行器数量 (nu): {mj_model.nu}")
        print(f"  将接收 {num_joints} 个关节的角度数据")
        print(f"  角度单位: {ANGLE_UNIT}")
        print("-" * 50)
    
    # 启动UDP接收器
    udp_receiver = ESP32UDPReceiver(UDP_HOST, UDP_PORT, TIMEOUT)
    if not udp_receiver.start():
        print("无法启动UDP接收器，退出程序")
        return
    
    # 初始化控制变量
    last_print_time = time.time()
    print_interval = 3.0  # 每3秒打印一次统计信息
    
    current_joint_angles = None
    last_data_time = 0
    connection_timeout = 10.0  # 连接超时时间
    
    # 启动仿真
    with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
            # 获取最新的UDP数据
            latest_data = udp_receiver.get_latest_data()
            
            if latest_data is not None:
                # 解析角度数据
                target_angles = parse_esp32_data(latest_data, num_joints)
                
                if target_angles is not None:
                    # 平滑角度过渡
                    current_joint_angles = smooth_angle_transition(
                        current_joint_angles, 
                        target_angles, 
                        SMOOTH_FACTOR, 
                        MAX_ANGLE_CHANGE
                    )
                    
                    # 应用角度到仿真
                    if mj_model.nu > 0:
                        # 有执行器的情况
                        mj_data.qpos[:mj_model.nu] = current_joint_angles[:mj_model.nu]
                    else:
                        # 直接控制关节位置
                        mj_data.qpos[:num_joints] = current_joint_angles
                    
                    last_data_time = time.time()
                    
                    if PRINT_UDP_DATA:
                        angles_deg = np.degrees(current_joint_angles)
                        print(f"应用角度: {np.round(angles_deg, 1)}°")
            
            # 检查连接状态
            current_time = time.time()
            if current_time - last_data_time > connection_timeout and last_data_time > 0:
                if DEBUG_MODE and (current_time - last_print_time >= print_interval):
                    print("⚠️  警告: ESP32连接超时，使用最后已知姿态")
            
            # 执行仿真步（不进行物理仿真，只更新可视化）
            # 手动设置速度和加速度为零，保持静态显示
            mj_data.qvel[:] = 0
            mj_data.qacc[:] = 0
            
            # 前向运动学计算（更新连杆位置）
            mujoco.mj_forward(mj_model, mj_data)
            
            # 同步查看器
            viewer.sync()
            
            # 打印统计信息
            if UDP_STATS and (current_time - last_print_time >= print_interval):
                stats = udp_receiver.get_stats()
                print(f"\n=== ESP32通信统计 (时间: {current_time:.1f}s) ===")
                print(f"接收包数: {stats['packets_received']}")
                print(f"错误包数: {stats['packets_error']}")
                
                if stats['last_receive_time'] > 0:
                    time_since_last = current_time - stats['last_receive_time']
                    print(f"最后接收: {time_since_last:.1f}s前")
                else:
                    print("最后接收: 无数据")
                
                if current_joint_angles is not None:
                    angles_deg = np.degrees(current_joint_angles)
                    print(f"当前关节角度 (度): {np.round(angles_deg, 1)}")
                    print(f"当前关节角度 (弧度): {np.round(current_joint_angles, 3)}")
                else:
                    print("尚未接收到有效角度数据")
                
                if stats['last_receive_time'] > 0:
                    connection_status = "✅ 连接正常" if time_since_last < 3.0 else "⚠️  连接不稳定"
                    print(f"连接状态: {connection_status}")
                
                if stats['last_timestamp'] > 0:
                    print(f"ESP32时间戳: {stats['last_timestamp']}ms")
                
                print("-" * 30)
                last_print_time = current_time
            
            # 控制帧率
            time.sleep(0.01)  # 100 Hz更新率
    
    # 清理资源
    udp_receiver.stop()
    print("仿真已结束")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序异常: {str(e)}")
        if "Wayland" in str(e):
            print("提示: 尝试设置环境变量 GDK_BACKEND=x11 解决Wayland兼容性问题")
        elif "GLFW" in str(e):
            print("提示: 可能是图形界面问题，请确保X11转发正常工作")