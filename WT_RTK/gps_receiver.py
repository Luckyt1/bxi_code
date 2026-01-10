import socket
import time
import threading
from datetime import datetime

# ROS2 依赖库
import rclpy
from rclpy.node import Node
import sensor_msgs.msg

class GPSRequestNode(Node):
    def __init__(self):
        super().__init__('gps_request_node')
        # 创建 GPS 发布者，话题通常使用 'gps/fix'
        self.publisher_ = self.create_publisher(
            sensor_msgs.msg.NavSatFix, 
            "gps/fix", 
            10
        )
        self.current_gps_data = None
        self.data_lock = threading.Lock()
        self.quality = "0"   
        # 定时器频率调整为 10Hz (0.1s)，对于 GPS 来说通常足够
        # 1000Hz (0.001s) 会造成不必要的 CPU 占用
        self.timer = self.create_timer(0.1, self.process_and_publish)
        
    def update_gps_data(self, data,type):
        """供外部线程调用的数据更新接口"""
        if data:
            with self.data_lock:
                if(type == 'WTRTK'):
                    self.current_gps_data = data
                if(type == 'GNGGA'):
                    self.quality=data.get('quality')

    def process_and_publish(self):
        """定时发布最新的 GPS 数据"""
        msg = sensor_msgs.msg.NavSatFix()
        
        # 线程安全地获取数据
        current_data = None
        with self.data_lock:
            current_data = self.current_gps_data
            
        if current_data:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps_link"
            
            # 填充位置信息
            if current_data.get('latitude') is not None:
                msg.latitude = float(current_data['latitude'])
            if current_data.get('longitude') is not None:
                msg.longitude = float(current_data['longitude'])
                
            # 填充高度信息
            try:
                if current_data.get('altitude') not in [None, "N/A"]:
                     msg.altitude = float(current_data['altitude'])
            except ValueError:
                pass

            # 填充状态信息 (根据 satellites 数量简单判断)
            try:
                num_sats = int(self.quality)
                if num_sats == 4 :
                    msg.status.status = 2 # STATUS_FIX
                elif num_sats ==5:
                    msg.status.status = 1 # STATUS_SBAS_FIX
                elif num_sats in [1,2]:
                    msg.status.status = 0 # STATUS_FIX
                else:
                    msg.status.status = -1 # STATUS_NO_FIX
            except ValueError:
                pass
                
            self.publisher_.publish(msg)
        
        
def parse_wtrtk(wtrtk_data):
    """
    解析WTRTK格式的GPS数据
    
    参数:
        wtrtk_data: WTRTK格式字符串，例如:
        $WTRTK,0.000,0.000,0.000,0.000,0.033,-0.099,85.607,1,0,0,28,24,0.00,1,32.0218285784,118.8566535316,44.3265,104.92,23.08*75
    
    WTRTK数据结构:
        [0] $WTRTK - 帧头
        [1] 差分X距离（米）
        [2] 差分Y距离（米）
        [3] 差分Z距离（米）
        [4] 差分R距离（米）
        [5] 角度X
        [6] 角度Y
        [7] 角度Z（±180°）
        [8] 定向状态（0:初始化,1:单点,2:码差分,4:固定解,5:浮点解）
        [9] 保留
        [10] 保留
        [11] 无线信号质量
        [12] 无线通讯数据量
        [13] 运动航向角（0-360°）
        [14] 定位标志（0:未对准,1:已对准,2:二次对准）
        [15] 纬度（度）- 惯导定位
        [16] 经度（度）- 惯导定位
        [17] GPS高度（米）- 惯导高度
        [18] 定向航向角（4GA专用）
        [19] 俯仰角（4GA专用）
        [*] 校验位
    
    返回:
        解析后的字典，包含时间、纬度、经度、卫星数等信息
    """
    try:
        parts = wtrtk_data.split(',')
        if len(parts) < 18 or not parts[0].endswith('WTRTK'):
            return None
        
        # 解析纬度（已经是度格式，不需要转换）
        latitude = None
        if len(parts) > 19 and parts[19]:
            try:
                latitude = float(parts[19])*0.01
            except ValueError:
                pass
        
        # 解析经度（已经是度格式，不需要转换）
        longitude = None
        if len(parts) > 21 and parts[21]:
            try:
                longitude = float(parts[21])*0.01
            except ValueError:
                pass
        
        # 解析高度
        altitude = "N/A"
        if len(parts) > 17 and parts[17]:
            try:
                altitude = parts[17]
            except:
                pass
        
        # 定向状态作为质量指标
        # 0:初始化, 1:单点定位, 2:码差分, 4:固定解, 5:浮点解
        quality = parts[8] if len(parts) > 8 else "N/A"
        
        # 定位标志
        positioning_flag = parts[14] if len(parts) > 14 else "N/A"
        
        # 无线信号质量作为卫星数的替代
        num_satellites = parts[11] if len(parts) > 11 else "N/A"
        
        # 角度信息
        angle_x = parts[5] if len(parts) > 5 else "N/A"
        angle_y = parts[6] if len(parts) > 6 else "N/A"
        angle_z = parts[7] if len(parts) > 7 else "N/A"
        
        # 航向角
        heading = parts[13] if len(parts) > 13 else "N/A"
        orientation_heading = parts[18] if len(parts) > 18 else "N/A"
        
        # 俯仰角（最后一个字段，需要去掉校验和）
        pitch = "N/A"
        if len(parts) > 19:
            pitch_str = parts[19].split('*')[0]
            pitch = pitch_str if pitch_str else "N/A"
        
        # 获取当前时间作为 UTC 时间
        utc_time = datetime.now().strftime('%H:%M:%S')
        
        return {
            'utc_time': utc_time,
            'latitude': latitude,
            'longitude': longitude,
            'quality': quality,
            'satellites': num_satellites,  # 实际是无线信号质量
            'hdop': positioning_flag,  # 定位标志
            'altitude': altitude,
            'angle_x': angle_x,
            'angle_y': angle_y,
            'angle_z': angle_z,
            'heading': heading,
            'orientation_heading': orientation_heading,
            'pitch': pitch
        }
    except Exception as e:
        print(f"解析WTRTK数据失败: {e}")
        return None


def parse_gngga(gngga_data):
    """
    解析GNGGA格式的GPS数据
    
    参数:
        gngga_data: GNGGA格式字符串，例如:
        $GNGGA,064325.70,3112.75641353,N,12129.81289439,E,5,12,2.8,41.8053,M,11.8073,M,0.7,151*5D
    
    返回:
        解析后的字典，包含时间、纬度、经度、卫星数等信息
    """
    try:
        parts = gngga_data.split(',')
        if len(parts) < 15 or not parts[0].endswith('GGA'):
            return None
        
        # 解析时间 (HHMMSS.ss)
        time_str = parts[1]
        if len(time_str) >= 6:
            hours = time_str[0:2]
            minutes = time_str[2:4]
            seconds = time_str[4:]
            utc_time = f"{hours}:{minutes}:{seconds}"
        else:
            utc_time = "N/A"
        
        # 解析纬度
        lat_str = parts[2]
        lat_dir = parts[3]
        if lat_str:
            lat_deg = float(lat_str[:2])
            lat_min = float(lat_str[2:])
            latitude = lat_deg + lat_min / 60.0
            if lat_dir == 'S':
                latitude = -latitude
        else:
            latitude = None
        
        # 解析经度
        lon_str = parts[4]
        lon_dir = parts[5]
        if lon_str:
            lon_deg = float(lon_str[:3])
            lon_min = float(lon_str[3:])
            longitude = lon_deg + lon_min / 60.0
            if lon_dir == 'W':
                longitude = -longitude
        else:
            longitude = None
        
        # 其他信息
        quality = parts[6] if len(parts) > 6 else "N/A"
        num_satellites = parts[7] if len(parts) > 7 else "N/A"
        hdop = parts[8] if len(parts) > 8 else "N/A"
        altitude = parts[9] if len(parts) > 9 else "N/A"
        
        return {
            'utc_time': utc_time,
            'latitude': latitude,
            'longitude': longitude,
            'quality': quality,
            'satellites': num_satellites,
            'hdop': hdop,
            'altitude': altitude
        }
    except Exception as e:
        print(f"解析GPS数据失败: {e}")
        return None


def gps_receiver_server(host='0.0.0.0', port=5000, save_to_file=True, log_filename='gps_log.txt', gps_node=None):
    """
    GPS数据接收服务器
    
    参数:
        host: 监听地址，默认0.0.0.0（监听所有网卡）
        port: 监听端口，默认5000
        save_to_file: 是否保存到文件，默认True
        log_filename: 日志文件名，默认gps_log.txt
    """
    server_socket = None
    
    try:
        # 创建TCP socket
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # 绑定地址和端口
        server_socket.bind((host, port))
        
        # 开始监听（最多5个等待连接）
        server_socket.listen(5)
        
        print(f"GPS接收服务器启动成功")
        print(f"监听地址: {host}:{port}")
        print(f"等待客户端连接...\n")
        
        while True:
            # 等待客户端连接
            client_socket, client_address = server_socket.accept()
            print(f"客户端已连接: {client_address}")
            
            # 为每个客户端创建一个处理线程
            client_thread = threading.Thread(
                target=handle_client,
                args=(client_socket, client_address, save_to_file, log_filename, gps_node),
                daemon=True
            )
            client_thread.start()
    
    except KeyboardInterrupt:
        print("\n服务器被用户中断")
    except Exception as e:
        print(f"服务器错误: {e}")
    finally:
        if server_socket:
            server_socket.close()
            print("服务器已关闭")


def handle_client(client_socket, client_address, save_to_file=True, log_filename='gps_log.txt', gps_node=None):
    """
    处理单个客户端连接
    
    参数:
        client_socket: 客户端socket对象
        client_address: 客户端地址
        save_to_file: 是否保存到文件
        log_filename: 日志文件名
    """
    buffer = ""
    
    try:
        while True:
            # 接收数据
            data = client_socket.recv(1024)
            
            if not data:
                print(f"客户端 {client_address} 断开连接")
                break
            
            # 解码数据
            try:
                decoded_data = data.decode('utf-8')
                buffer += decoded_data
                
                # 处理完整的行
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    type=''
                    if line:
                        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                        
                        # 如果是WTRTK数据，解析并显示
                        if 'GNGGA' in line:
                            parsed_gngga = parse_gngga(line)
                            if parsed_gngga:
                                if gps_node:
                                    gps_node.update_gps_data(parsed_gngga, type='GNGGA')
                                    print(f"  定位状态(UTC): {parsed_gngga['quality']}")
        
                        if 'WTRTK' in line:
                            parsed = parse_wtrtk(line)
                            if parsed:
                                if gps_node:
                                    gps_node.update_gps_data(parsed,type='WTRTK')
                                    print(f"  时间(UTC): {parsed['utc_time']}")
                                    print(f"  经度(): {parsed['longitude']}")
                                    print(f"  纬度(): {parsed['latitude']}")
                                    
                        print()  # 空行分隔
            
            except UnicodeDecodeError:
                print(f"接收到无法解码的数据: {data}")
    
    except Exception as e:
        print(f"处理客户端 {client_address} 时出错: {e}")
    finally:
        client_socket.close()
        print(f"客户端 {client_address} 连接已关闭\n")


if __name__ == "__main__":
    # 配置参数
    HOST = '0.0.0.0'  # 监听所有网卡，如果只想本地测试可以改为'127.0.0.1'
    PORT = 5000       # 端口号，需要与发送端的TARGET_PORT一致
    SAVE_TO_FILE = True  # 是否保存GPS数据到文件
    LOG_FILE = 'gps_log.txt'  # 日志文件名
    
    # 初始化 ROS2
    rclpy.init()
    gps_node = GPSRequestNode()
    
    # 在单独线程中启动GPS接收服务器
    server_thread = threading.Thread(
        target=gps_receiver_server,
        args=(HOST, PORT, SAVE_TO_FILE, LOG_FILE, gps_node),
        daemon=True
    )
    server_thread.start()
    
    try:
        # 运行 ROS 节点
        rclpy.spin(gps_node)
    except KeyboardInterrupt:
        pass
    finally:
        gps_node.destroy_node()
        rclpy.shutdown()
