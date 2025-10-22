import cv2

cameras = []
camera_names = ["hand_camera", "head_camera"]
for i in range(2):
    if i==1 :
        cap = cv2.VideoCapture(0)
    else:
        cap = cv2.VideoCapture(2)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cameras.append(cap)
        print(f"摄像头 {i} 已打开 - 分辨率: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    else:
        print(f"警告: 摄像头 {i} 无法打开")
        cameras.append(None)

if all(cam is None for cam in cameras):
    print("错误: 没有可用的摄像头")
    exit()
    # 释放摄像头资源的函数