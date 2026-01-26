import socket
import cv2
import struct
import pickle

# 配置
HOST = '0.0.0.0'
PORT = 9999

# 创建socket
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)
# 打开摄像头
cap0 = cv2.VideoCapture(0)
cap1 = cv2.VideoCapture(2)

# 降低分辨率
cap0.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 清空缓冲
cap0.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap1.set(cv2.CAP_PROP_BUFFERSIZE, 1)

conn, addr = server.accept()
print(f"已连接: {addr}")

try:
    while True:
        ret0, frame0 = cap0.read()
        ret1, frame1 = cap1.read()
        
        if ret0 and ret1:
            # JPEG压缩
            _, buf0 = cv2.imencode('.jpg', frame0, [cv2.IMWRITE_JPEG_QUALITY, 80])
            _, buf1 = cv2.imencode('.jpg', frame1, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
            # 打包数据
            data = pickle.dumps({'cam0': buf0, 'cam1': buf1})
            size = struct.pack("L", len(data))
            
            # 发送
            conn.sendall(size + data)
            
except KeyboardInterrupt:
    print("\n关闭...")
finally:
    cap0.release()
    cap1.release()
    conn.close()
    server.close()