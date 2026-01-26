import socket
import cv2
import struct
import pickle
import numpy as np

# 配置 - 修改为发送端的IP地址
HOST = '192.168.88.120'  # 摄像头设备的IP
PORT = 9999

# 连接
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((HOST, PORT))
print(f"已连接到 {HOST}:{PORT}")
 
data = b""
payload_size = struct.calcsize("L")

try:
    while True:
        # 接收数据大小
        while len(data) < payload_size:
            packet = client.recv(4096)
            if not packet:
                break
            data += packet
        
        packed_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack("L", packed_size)[0]
        
        # 接收图像数据
        while len(data) < msg_size:
            data += client.recv(4096)
        
        frame_data = data[:msg_size]
        data = data[msg_size:]
        
        # 解码
        frames = pickle.loads(frame_data)
        frame0 = cv2.imdecode(frames['cam0'], cv2.IMREAD_COLOR)
        frame1 = cv2.imdecode(frames['cam1'], cv2.IMREAD_COLOR)
        
        # 显示
        cv2.imshow('Hand', frame0)
        cv2.imshow('Head', frame1)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
except KeyboardInterrupt:
    print("\n关闭...")
finally:
    client.close()
    cv2.destroyAllWindows()