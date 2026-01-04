import serial
import time
import threading

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


if __name__ == "__main__":
    # 可以根据需要修改这些参数
    PORT = 'COM20'
    BAUDRATE = 460800  # 根据实际设备修改波特率，常见值: 9600, 115200, 57600等
    
    # 选择模式：
    # 1. 仅接收模式：read_com20(port=PORT, baudrate=BAUDRATE)
    # 2. 交互模式（可发送和接收）：read_and_send_com20(port=PORT, baudrate=BAUDRATE)
    
    # 默认使用交互模式
    read_and_send_com20(port=PORT, baudrate=BAUDRATE)
