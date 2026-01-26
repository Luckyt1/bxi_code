import pygame
from threading import Thread  #键盘线程控制
keyboard_use = True  # 是否使用键盘控制
# 键盘控制设置
def setup_keyboard_control():
    if keyboard_use:
        pygame.init()  # 初始化pygame
        try:
            screen = pygame.display.set_mode((200, 100))  # 创建一个小窗口以捕获键盘事件
            keyboard_opened=True
        except Exception as e:
            print("无法打开键盘控制窗口，键盘控制不可用:", e)
            keyboard_opened=False
        exit_flag = False  # 退出标志
        def handle_keyboard_input():
            while not exit_flag:
                keys = pygame.key.get_pressed()

                if keys[pygame.K_w]:
                    print("前进")
                if keys[pygame.K_s]:
                    print("后退")
                if keys[pygame.K_a]:
                    print("左转")
                if keys[pygame.K_d]:
                    print("右转")

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        exit_flag = True
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            exit_flag = True
                pygame.time.delay(50)  # 50ms延迟，减少CPU使用
        if keyboard_opened and keyboard_use:
            keyboard_thread = Thread(target=handle_keyboard_input)
            keyboard_thread.start()