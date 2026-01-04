import socket
import time
import threading
from datetime import datetime


def save_gps_data(data, filename='gps_log.txt'):
    """
    保存GPS数据到文件
    
    参数:
        data: GPS数据字符串
        filename: 保存的文件名
    """
    try:
        with open(filename, 'a', encoding='utf-8') as f:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            f.write(f"[{timestamp}] {data}\n")
    except Exception as e:
        print(f"保存文件失败: {e}")


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


def gps_receiver_server(host='0.0.0.0', port=5000, save_to_file=True, log_filename='gps_log.txt'):
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
        if save_to_file:
            print(f"数据将保存到: {log_filename}")
        print(f"等待客户端连接...\n")
        
        while True:
            # 等待客户端连接
            client_socket, client_address = server_socket.accept()
            print(f"客户端已连接: {client_address}")
            
            # 为每个客户端创建一个处理线程
            client_thread = threading.Thread(
                target=handle_client,
                args=(client_socket, client_address, save_to_file, log_filename),
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


def handle_client(client_socket, client_address, save_to_file=True, log_filename='gps_log.txt'):
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
                    
                    if line:
                        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                        print(f"[{timestamp}] 接收到: {line}")
                        
                        # 如果是GNGGA数据，解析并显示
                        if 'GNGGA' in line:
                            parsed = parse_gngga(line)
                            if parsed:
                                print(f"  时间(UTC): {parsed['utc_time']}")
                                if parsed['latitude'] and parsed['longitude']:
                                    print(f"  位置: {parsed['latitude']:.6f}°N, {parsed['longitude']:.6f}°E")
                                print(f"  卫星数: {parsed['satellites']}, 质量: {parsed['quality']}, 高度: {parsed['altitude']}m")
                        
                        # 保存到文件
                        if save_to_file:
                            save_gps_data(line, log_filename)
                        
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
    
    # 启动GPS接收服务器
    gps_receiver_server(host=HOST, port=PORT, save_to_file=SAVE_TO_FILE, log_filename=LOG_FILE)
