import serial
import time
import threading
import socket

def read_com20(port='COM20', baudrate=460800, timeout=1):
    """
    读取COM20端口的数据（仅接收模式）
    
    参数:
        port: 串口号，默认COM20
        baudrate: 波特率，默认460800
        timeout: 超时时间(秒)，默认1秒
    """
    try:
        # 打开串口
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        
        print(f"成功打开串口 {port}")
        print(f"波特率: {baudrate}")
        print(f"开始读取数据...\n")
        
        # 持续读取数据
        while True:
            if ser.in_waiting > 0:  # 检查是否有数据可读
                # 读取一行数据
                data = ser.readline()
                
                try:
                    # 尝试解码为字符串
                    decoded_data = data.decode('utf-8').strip()
                    print(f"接收到: {decoded_data}")
                except UnicodeDecodeError:
                    # 如果解码失败，显示原始字节数据
                    print(f"接收到(原始): {data}")
            
            time.sleep(0.01)  # 短暂延迟，避免CPU占用过高
            
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print(f"串口 {port} 已关闭")


def send_data(ser, data):
    """
    通过串口发送数据
    
    参数:
        ser: 串口对象
        data: 要发送的数据（字符串或字节）
    """
    try:
        if isinstance(data, str):
            # 如果是字符串，编码为字节并添加换行符
            ser.write((data + '\n').encode('utf-8'))
        else:
            # 如果已经是字节，直接发送
            ser.write(data)
        print(f"已发送: {data}")
        return True
    except Exception as e:
        print(f"发送失败: {e}")
        return False


def read_and_send_com20(port='COM20', baudrate=460800, timeout=1):
    """
    读取和发送COM20端口的数据（交互模式）
    支持同时接收数据和通过键盘输入发送数据
    
    参数:
        port: 串口号，默认COM20
        baudrate: 波特率，默认460800
        timeout: 超时时间(秒)，默认1秒
    """
    try:
        # 打开串口
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        
        print(f"成功打开串口 {port}")
        print(f"波特率: {baudrate}")
        print(f"交互模式已启动")
        print(f"输入消息并按回车发送，按Ctrl+C退出\n")
        
        # 标志位，用于控制线程
        running = True
        
        def read_thread():
            """读取串口数据的线程"""
            while running:
                try:
                    if ser.in_waiting > 0:
                        data = ser.readline()
                        try:
                            decoded_data = data.decode('utf-8').strip()
                            print(f"\n[接收] {decoded_data}")
                            print(">>> ", end='', flush=True)  # 重新显示输入提示符
                        except UnicodeDecodeError:
                            print(f"\n[接收-原始] {data}")
                            print(">>> ", end='', flush=True)
                    time.sleep(0.01)
                except:
                    break
        
        # 启动接收线程
        recv_thread = threading.Thread(target=read_thread, daemon=True)
        recv_thread.start()
        
        # 主线程处理发送
        while True:
            try:
                message = input(">>> ")
                if message:
                    send_data(ser, message)
            except EOFError:
                break
            
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        running = False
        time.sleep(0.1)  # 等待线程结束
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print(f"串口 {port} 已关闭")


def read_and_forward_com20(port='COM20', baudrate=460800, timeout=1, 
                           target_ip='127.0.0.1', target_port=5000):
    """
    读取COM20端口的GPS数据并通过socket转发到指定IP
    
    参数:
        port: 串口号，默认COM20
        baudrate: 波特率，默认460800
        timeout: 超时时间(秒)，默认1秒
        target_ip: 目标IP地址，默认127.0.0.1
        target_port: 目标端口，默认5000
    """
    sock = None
    try:
        # 打开串口
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        
        print(f"成功打开串口 {port}")
        print(f"波特率: {baudrate}")
        
        def connect_server():
            """尝试连接到服务器，如果失败则一直等待"""
            while True:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(5)  # 设置连接超时
                    s.connect((target_ip, target_port))
                    s.settimeout(None)  # 恢复阻塞模式或根据需要设置
                    print(f"已连接到 {target_ip}:{target_port}")
                    return s
                except socket.error:
                    print(f"等待服务器 {target_ip}:{target_port} 上线...")
                    time.sleep(2)  # 等待2秒后重试

        # 初始连接
        sock = connect_server()
        
        print(f"开始读取并转发GPS数据...\n")
        
        # 持续读取并转发数据
        while True:
            if ser.in_waiting > 0:  # 检查是否有数据可读
                # 读取一行数据
                data = ser.readline()
                
                try:
                    # 尝试解码为字符串
                    decoded_data = data.decode('utf-8').strip()
                    
                    # 检查是否是NMEA格式数据（以$开头）
                    if decoded_data.startswith('$'):
                        print(f"接收到GPS数据: {decoded_data}")
                        
                        # 通过socket转发数据
                        try:
                            sock.sendall((decoded_data + '\n').encode('utf-8'))
                            print(f"已转发到 {target_ip}:{target_port}")
                        except socket.error as e:
                            print(f"Socket发送失败: {e}")
                            print(f"连接断开，尝试重新连接...")
                            try:
                                sock.close()
                            except:
                                pass
                            # 重新连接
                            sock = connect_server()

                    else:
                        print(f"接收到: {decoded_data}")
                        
                except UnicodeDecodeError:
                    # 如果解码失败，显示原始字节数据
                    print(f"接收到(原始): {data}")
            
            time.sleep(0.01)  # 短暂延迟，避免CPU占用过高
            
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except socket.error as e:
        print(f"Socket连接错误: {e}")
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        # 关闭socket连接
        if sock:
            try:
                sock.close()
                print(f"Socket连接已关闭")
            except:
                pass
        
        # 关闭串口
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print(f"串口 {port} 已关闭")


if __name__ == "__main__":
    # 可以根据需要修改这些参数
    PORT = '/dev/ttyUSB0'
    BAUDRATE = 460800  # 根据实际设备修改波特率，常见值: 9600, 115200, 57600等
    
    # Socket转发配置
    TARGET_IP = '192.168.88.83'  # 修改为目标IP地址
    TARGET_PORT = 5000  # 修改为目标端口
    
    # 选择模式：
    # 1. 仅接收模式：read_com20(port=PORT, baudrate=BAUDRATE)
    # 2. 交互模式（可发送和接收）：read_and_send_com20(port=PORT, baudrate=BAUDRATE)
    # 3. 读取并转发GPS数据：read_and_forward_com20(port=PORT, baudrate=BAUDRATE, target_ip=TARGET_IP, target_port=TARGET_PORT)
    
    # 默认使用GPS转发模式
    read_and_forward_com20(port=PORT, baudrate=BAUDRATE, target_ip=TARGET_IP, target_port=TARGET_PORT)
