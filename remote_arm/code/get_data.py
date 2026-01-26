import socket
import cv2
import struct
import pickle
import time
class CameraReceiver:
    def __init__(self, host='192.168.88.120', port=9999, retry=True, retry_interval=2):
        self.host = host
        self.port = port
        self.retry = retry
        self.retry_interval = retry_interval
        self.client = None
        self.data = b""
        self.size = struct.calcsize("L")
        print(f"已连接到 {host}:{port}")
        self._connect()
        
    def _connect(self):
        while True:
            try:
                self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client.connect((self.host, self.port))
                print(f"已连接到 {self.host}:{self.port}")
                break
            except ConnectionRefusedError:
                if self.retry:
                    print(f"等待服务端启动... ({self.host}:{self.port})")
                    time.sleep(self.retry_interval)
                else:
                    print(f"无法连接到 {self.host}:{self.port}")
                    raise
            except Exception as e:
                print(f"连接错误: {e}")
                if not self.retry:
                    raise
                time.sleep(self.retry_interval)

    def get_frames(self):
        # 接收大小
        while len(self.data) < self.size:
            self.data += self.client.recv(4096)
        
        msg_size = struct.unpack("L", self.data[:self.size])[0]
        self.data = self.data[self.size:]
        
        # 接收数据
        while len(self.data) < msg_size:
            self.data += self.client.recv(4096)
        
        frames = pickle.loads(self.data[:msg_size])
        self.data = self.data[msg_size:]
        
        return cv2.imdecode(frames['hand'], cv2.IMREAD_COLOR), \
               cv2.imdecode(frames['head'], cv2.IMREAD_COLOR)
    
    def run(self):
        try:
            while True:
                hand, head = self.get_frames()
                cv2.imshow('Hand', hand)
                cv2.imshow('Head', head)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        except KeyboardInterrupt:
            print("\n关闭...")
        finally:
            self.client.close()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    CameraReceiver().run()