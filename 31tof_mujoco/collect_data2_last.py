import math
import pygame
import numpy as np
import onnxruntime as ort  # 添加ONNX Runtime
import mujoco
import mujoco.viewer
from collections import deque###双段数列，左右均可添加
from threading import Thread  #键盘线程控制
from scipy.spatial.transform import Rotation as R
import cv2  # 导入OpenCV
import time  # 添加时间模块用于帧率计算
import os  # 添加os模块用于文件操作
import csv  # 添加csv模块用于保存csv文件
import threading
import rclpy
from rclpy.node import Node
from communication.msg import ActuatorCmds, MotionCommands
################################################keyboard_controller
# 全局变量：速度命令和键盘控制标志
x_vel_cmd, y_vel_cmd, yaw_vel_cmd = 0.0, 0.0, 0.0  # x方向速度，y方向速度，偏航角速度
keyboard_use = True  # 是否使用键盘控制
keyboard_opened = False  # 键盘是否已初始化

date_collect_cmd = False  # 键盘是否已初始化

# 数据采集相关全局变量
data_collecting = False  # 当前是否正在采集数据
data_group_count = 0  # 当前数据组编号
frame_count = 0  # 当前组内帧计数
data_collect_counter = 0  # 数据采集计数器
data_collect_interval = 50  # 数据采集间隔（1000Hz / 20Hz = 50步）

# 创建数据保存目录
dataset_dir = "collected_data"
csv_dir = os.path.join(dataset_dir, "csv")
hand_imgs_dir = os.path.join(dataset_dir, "hand_imgs")
head_imgs_dir = os.path.join(dataset_dir, "head_imgs")

# 创建目录
os.makedirs(dataset_dir, exist_ok=True)
os.makedirs(csv_dir, exist_ok=True)
os.makedirs(hand_imgs_dir, exist_ok=True)
os.makedirs(head_imgs_dir, exist_ok=True)

# 添加:自动检测已有数据组数的函数
def detect_existing_data_groups():
    """检测已存在的数据组数量"""
    global data_group_count
    
    # 检查CSV目录中的文件
    csv_files = []
    if os.path.exists(csv_dir):
        csv_files = [f for f in os.listdir(csv_dir) if f.endswith('.csv')]
    
    if not csv_files:
        data_group_count = 0
        print("未检测到已有数据,从第0组开始采集")
        return
    
    # 提取所有组号
    group_numbers = []
    for csv_file in csv_files:
        try:
            group_num = int(csv_file.replace('.csv', ''))
            group_numbers.append(group_num)
        except ValueError:
            continue
    
    if group_numbers:
        # 找到最大组号,下一组从max+1开始
        max_group = max(group_numbers)
        data_group_count = max_group + 1
        print(f"检测到已有 {len(group_numbers)} 组数据")
        print(f"最大组号: {max_group}")
        print(f"将从第 {data_group_count} 组开始采集")
        
        # 显示已有数据的统计信息
        for group_num in sorted(group_numbers):
            csv_path = os.path.join(csv_dir, f"{group_num}.csv")
            if os.path.exists(csv_path):
                with open(csv_path, 'r') as f:
                    frame_count_in_group = sum(1 for _ in f) - 1  # 减去表头
                print(f"  第 {group_num} 组: {frame_count_in_group} 帧")
    else:
        data_group_count = 0
        print("未检测到有效数据组,从第0组开始采集")
# 调用检测函数
detect_existing_data_groups()
print(f"数据将保存到: {dataset_dir}")

def delete_last_group():
    """删除上一组数据"""
    global data_group_count
    
    if data_group_count == 0:
        print("没有可删除的数据组")
        return
    
    group_to_delete = data_group_count - 1
    
    # 删除CSV文件
    csv_file = os.path.join(csv_dir, f"{group_to_delete}.csv")
    if os.path.exists(csv_file):
        os.remove(csv_file)
        print(f"已删除CSV文件: {csv_file}")
    
    # 删除图片文件
    deleted_count = 0
    for img_dir, img_type in [(head_imgs_dir, "head"), (hand_imgs_dir, "hand")]:
        for filename in os.listdir(img_dir):
            if filename.startswith(f"{group_to_delete}_"):
                img_path = os.path.join(img_dir, filename)
                os.remove(img_path)
                deleted_count += 1
    
    print(f"已删除第 {group_to_delete} 组数据 (共删除 {deleted_count} 张图片)")
    data_group_count -= 1
# 键盘控制初始化
if keyboard_use:
    pygame.init()  # 初始化pygame
    try:
        # 设置窗口用于接收键盘输入
        screen = pygame.display.set_mode((200, 100))
        pygame.display.set_caption("Keyboard Control")
        keyboard_opened = True
        print("键盘控制已启动。使用以下按键控制机器人：")
        print("W/S: 前进/后退")
        print("A/D: 左移/右移")
        print("Q/E: 左转/右转")
        print("C/V: 开始/停止数据采集")
        print("Z: 删除上一组数据")
        print("空格键: 停止所有运动")
    except Exception as e:
        print(f"无法初始化键盘：{e}")
    
    # 键盘线程退出标志
    exit_flag = False

    def handle_keyboard_input():
        """处理键盘输入的线程函数"""
        global exit_flag, x_vel_cmd, y_vel_cmd, yaw_vel_cmd, date_collect_cmd, data_collecting
        
        max_lin_vel = 0.6  # 最大线速度 (m/s)
        max_ang_vel = 1.2  # 最大角速度 (rad/s)
        
        z_key_pressed = False  # 添加Z键按下状态标志
        
        while not exit_flag:
            # 获取键盘输入
            keys = pygame.key.get_pressed()
            
            # 重置速度命令
            x_vel_cmd = 0.0
            y_vel_cmd = 0.0
            yaw_vel_cmd = 0.0
            
            # 前进/后退 (W/S)
            if keys[pygame.K_w]:
                x_vel_cmd = max_lin_vel
            if keys[pygame.K_s]:
                x_vel_cmd = -max_lin_vel
                
            # 左移/右移 (A/D)
            if keys[pygame.K_a]:
                y_vel_cmd = max_lin_vel+0.4
            if keys[pygame.K_d]:
                y_vel_cmd = -max_lin_vel-0.4
                
            # 左转/右转 (Q/E)
            if keys[pygame.K_q]:
                yaw_vel_cmd = max_ang_vel
            if keys[pygame.K_e]:
                yaw_vel_cmd = -max_ang_vel
        
            # 开始采集/停止采集 (C/V)
            if keys[pygame.K_c]:
                if not date_collect_cmd:  # 如果之前没有开始采集
                    date_collect_cmd = True
                    print("开始数据采集")
            if keys[pygame.K_v]:
                if date_collect_cmd:  # 如果之前正在采集
                    date_collect_cmd = False
                    print("停止数据采集")
        
            # 删除上一组数据 (Z键)
            if keys[pygame.K_z]:
                if not z_key_pressed:
                    delete_last_group()
                    z_key_pressed = True
            else:
                z_key_pressed = False
            
            # 空格键停止所有运动
            if keys[pygame.K_SPACE]:
                x_vel_cmd, y_vel_cmd, yaw_vel_cmd = 0.0, 0.0, 0.0
            
            # 处理退出事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    exit_flag = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        exit_flag = True
            
            pygame.time.delay(50)  # 50ms延迟,减少CPU使用

    # 启动键盘输入处理线程
    if keyboard_opened and keyboard_use:
        keyboard_thread = Thread(target=handle_keyboard_input)
        keyboard_thread.start()

class cmd:
    """命令类，用于存储速度命令"""
    vx = 0.0  # x方向速度
    vy = 0.0  # y方向速度
    dyaw = 0.0  # 偏航角速度

def quaternion_to_euler_array(quat):
    """将四元数转换为欧拉角（滚转、俯仰、偏航）
    
    参数:
        quat: 四元数 [x, y, z, w]
        
    返回:
        numpy数组: [roll, pitch, yaw] 弧度制
    """
    # 确保四元数格式正确 [x, y, z, w]
    x, y, z, w = quat
    
    # 滚转 (x轴旋转)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)
    
    # 俯仰 (y轴旋转)
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)  # 限制在[-1,1]范围内避免数值误差
    pitch_y = np.arcsin(t2)
    
    # 偏航 (z轴旋转)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)
    
    # 返回滚转、俯仰、偏航的numpy数组（弧度）
    return np.array([roll_x, pitch_y, yaw_z])

def get_obs(data, model):
    q = data.qpos.astype(np.double)  # 关节位置12
    dq = data.qvel.astype(np.double)  # 关节速度12
    quat = data.sensor('Body_Quat').data[[1, 2, 3, 0]].astype(np.double)  # 身体方向四元数
    r = R.from_quat(quat)  # 创建旋转对象，旋转矩阵

    v = r.apply(data.qvel[:3], inverse=True).astype(np.double)  # 基座坐标系中的基座速度3
    omega = data.sensor('Body_Gyro').data.astype(np.double)  # 身体角速度imu
    gvec = r.apply(np.array([0., 0., -1.]), inverse=True).astype(np.double)  # 基座坐标系中的重力向量
    
    # 获取足部位置和力
    foot_positions = []
    foot_forces = []
    #model.nbody 是 MuJoCo 模型中身体（body）的数量
    for i in range(model.nbody):
        #遍历模型中的每一个身体，通过索引 i 访问每个身体，然后获取其名称。
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        #print(body_name)
        if 'l_ankle_x_link' or 'r_ankle_x_joint' in body_name:  # 根据模型名称识别足部
            foot_positions.append(data.xpos[i][2].copy().astype(np.double))  # z轴位置h
            #data.cfrc_ext[i][2] 获取作用在身体i上的外部接触力在z轴的分量
            foot_forces.append(data.cfrc_ext[i][2].copy().astype(np.double))  # z轴力
            
        if 'base_link' in body_name:  # 根据模型名称识别基座
            base_pos = data.xpos[i][:3].copy().astype(np.double)  # 基座位置xyz3
 
    return (q, dq, quat, v, omega, gvec, base_pos, foot_positions, foot_forces)#12,12,4,3,1,1,3,1,3

def pd_control_all(target_q, q, kp, target_dq, dq, kd, nominal_pos):
    torque_out = (target_q + nominal_pos - q ) * kp - dq * kd
    return torque_out

def get_camera_image(camera_name,model, data, renderer):
    """获取相机图像"""
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if camera_id == -1:
        print(f"警告: 未找到相机 {camera_name}")
        return None

    # 使用渲染器获取相机图像
    renderer.update_scene(data, camera=camera_name)
    img = renderer.render()
    return img

def save_data(group_id, frame_id, qs, head_img, hand_img):
    """保存数据到文件"""
    # 保存图片
    head_img_filename = f"{group_id}_{frame_id}.jpg"
    hand_img_filename = f"{group_id}_{frame_id}.jpg"
    
    head_img_path = os.path.join(head_imgs_dir, head_img_filename)
    hand_img_path = os.path.join(hand_imgs_dir, hand_img_filename)
    
    # 转换颜色格式并保存
    if head_img is not None:
        cv2.imwrite(head_img_path, cv2.cvtColor(head_img, cv2.COLOR_RGB2BGR))
    if hand_img is not None:
        cv2.imwrite(hand_img_path, cv2.cvtColor(hand_img, cv2.COLOR_RGB2BGR))
    
    # 保存到CSV文件
    csv_filename = f"{group_id}.csv"
    csv_path = os.path.join(csv_dir, csv_filename)
    
    # 检查文件是否存在，如果不存在则创建并写入表头
    file_exists = os.path.isfile(csv_path)
    
    with open(csv_path, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        if not file_exists:
            writer.writerow(['image_name', 'l1_pos', 'l2_pos', 'l3_pos', 'l4_pos', 'l5_pos','l6_pos','l7_pos','l8_pos','progress'])
        
        # 写入数据行
        image_name = f"{group_id}_{frame_id}"
        writer.writerow([image_name, qs[15], qs[16], qs[17], qs[18], qs[19], qs[20], qs[21], qs[22], 0.0])
            
def run_mujoco(policy, cfg, env_cfg):
    # 手动指定MuJoCo模型路径
    mujoco_model_path = cfg.sim_config.mujoco_model_path  
    print(f"从以下路径加载MuJoCo xml模型: {mujoco_model_path}")
    
    # 加载模型xml
    model = mujoco.MjModel.from_xml_path(mujoco_model_path)
    # 设置仿真时间步长
    model.opt.timestep = cfg.sim_config.dt
    # 模型数据
    data = mujoco.MjData(model)
    data.qpos[10:22] = cfg.robot_config.default_dof_pos  # 设置初始关节位置，取前12个，我们前三个腰部

    # 执行一步仿真以初始化
    mujoco.mj_step(model, data)
    # 创建查看器
    try:
        viewer = mujoco.viewer.launch_passive(model, data)
    except Exception as e:
        print(f"MuJoCo Viewer 初始化失败: {e}")
        return

    # 设置查看器相机参数
    viewer.cam.distance = 3.0  # 相机距离
    viewer.cam.azimuth = 90    # 方位角
    viewer.cam.elevation = -15  # 仰角

    # 确保渲染循环正常运行
    try:
        # 主仿真循环
        for _ in range(int(cfg.sim_config.sim_duration / cfg.sim_config.dt)):
            # 执行仿真步
            mujoco.mj_step(model, data)

            # 渲染MuJoCo查看器
            if viewer.is_running():
                viewer.sync()  # 使用 sync 方法更新渲染
            else:
                print("MuJoCo Viewer 已关闭")
                break

            # 创建渲染器
            renderer = mujoco.Renderer(model)
            
            camera_names=['head_camera', 'left_hand_camera', 'right_hand_camera']
            
            camera_images =[None] * len(camera_names)

            for i in range(len(camera_names)):
                cv2.namedWindow(camera_names[i], cv2.WINDOW_NORMAL)
                #cv2.resizeWindow(camera_names[i], 1200, 800)
                cv2.resizeWindow(camera_names[i], 640, 480)

            # 初始化变量
            target_q = np.zeros((env_cfg.env.num_actions), dtype=np.double)  # 目标关节位置12
            action = np.zeros((env_cfg.env.num_actions), dtype=np.double)  # 动作12[0]

            ###############################################################################################
            # 创建历史观测队列（用于帧堆叠）
            hist_obs = deque()#双队列
            for _ in range(env_cfg.env.frame_stack):
                hist_obs.append(np.zeros([1, env_cfg.env.num_single_obs], dtype=np.double))

            count_lowlevel = 1  # 低层控制计数器

            # 设置numpy打印选项
            np.set_printoptions(formatter={'float': '{:0.4f}'.format})
            
            # 添加帧率计算变量
            mujoco_step_count = 0
            opencv_frame_count = 0
            start_time = time.time()
            last_print_time = start_time
            print_interval = 5.0  # 每2秒打印一次帧率信息
            
            # 数据采集相关变量
            global data_collecting, data_group_count, frame_count, data_collect_counter, date_collect_cmd
            prev_collect_cmd = date_collect_cmd  # 上一次的采集命令状态
            ##################################################################################################
            
            # 主仿真循环
            for _ in range(int(cfg.sim_config.sim_duration / cfg.sim_config.dt)):
                # 获取观测值
                q, dq, quat, v, omega, gvec, base_pos, foot_positions, foot_forces = get_obs(data, model)#获取mujoco变量

                qs = np.zeros(len(joint_nominal_pos), dtype=np.double)#31[0]
                dqs = np.zeros(len(joint_nominal_pos), dtype=np.double)#31[0]  

                qs[:23] = q[7:30] 
                qs[23:31] = q[31:39] 
                dqs[:23] = dq[6:29]  
                dqs[23:31] = dq[30:38]  
                
                q = q[10:22] 
                dq = dq[9:21]  
                
                # 1000Hz -> 100Hz 控制频率转换（每10步执行一次高层控制）
                #decimation = 10  # 控制频率降采样因子
                if count_lowlevel % cfg.sim_config.decimation == 0:
                    ####### 仅站立模式相关逻辑 #######
                    if hasattr(env_cfg.commands, "sw_switch"):
                        vel_norm = np.sqrt(x_vel_cmd**2 + y_vel_cmd**2 + yaw_vel_cmd**2)  # 速度命令范数
                        if env_cfg.commands.sw_switch and vel_norm <= env_cfg.commands.stand_com_threshold:
                            count_lowlevel = 0  # 重置计数器
                            
                    # 构建观测向量num_single_obs=47+1
                    obs = np.zeros([1, env_cfg.env.num_single_obs], dtype=np.float32)
                    eu_ang = quaternion_to_euler_array(quat)  # 四元数转欧拉角
                    eu_ang[eu_ang > math.pi] -= 2 * math.pi  # 角度归一化到[-π, π]
                    
                    # 命令相关观测
                    #num_commands = 5  # 命令维度：sin_pos, cos_pos, vx, vy, vz
                    if env_cfg.env.num_commands == 5:
                        # 5维命令：正弦、余弦、线速度x、线速度y、角速度
                        obs[0, 0] = math.sin(2 * math.pi * count_lowlevel * cfg.sim_config.dt / env_cfg.rewards.cycle_time)
                        obs[0, 1] = math.cos(2 * math.pi * count_lowlevel * cfg.sim_config.dt / env_cfg.rewards.cycle_time)
                        obs[0, 2] = x_vel_cmd * env_cfg.normalization.obs_scales.lin_vel
                        obs[0, 3] = y_vel_cmd * env_cfg.normalization.obs_scales.lin_vel
                        obs[0, 4] = yaw_vel_cmd * env_cfg.normalization.obs_scales.ang_vel
                    if env_cfg.env.num_commands == 3:
                        # 3维命令：线速度x、线速度y、角速度
                        obs[0, 0] = x_vel_cmd * env_cfg.normalization.obs_scales.lin_vel
                        obs[0, 1] = y_vel_cmd * env_cfg.normalization.obs_scales.lin_vel
                        obs[0, 2] = yaw_vel_cmd * env_cfg.normalization.obs_scales.ang_vel
                    
                    ##########################################################################################################obs    
                    # 关节相关观测5：12
                    obs[0, env_cfg.env.num_commands:env_cfg.env.num_commands+env_cfg.env.num_actions] = (
                        q - cfg.robot_config.default_dof_pos) * env_cfg.normalization.obs_scales.dof_pos
                    obs[0, env_cfg.env.num_commands+env_cfg.env.num_actions:env_cfg.env.num_commands+2*env_cfg.env.num_actions] = (
                        dq * env_cfg.normalization.obs_scales.dof_vel)
                    obs[0, env_cfg.env.num_commands+2*env_cfg.env.num_actions:env_cfg.env.num_commands+3*env_cfg.env.num_actions] = action
                    
                    # 基座状态观测
                    obs[0, env_cfg.env.num_commands+3*env_cfg.env.num_actions:env_cfg.env.num_commands+3*env_cfg.env.num_actions+3] = omega
                    obs[0, env_cfg.env.num_commands+3*env_cfg.env.num_actions+3:env_cfg.env.num_commands+3*env_cfg.env.num_actions+6] = eu_ang

                    # 限制观测值范围
                    obs = np.clip(obs, -env_cfg.normalization.clip_observations, env_cfg.normalization.clip_observations)
                    hist_obs.append(obs)
                    hist_obs.popleft()
                    policy_input = np.zeros([1, env_cfg.env.num_observations], dtype=np.float32)
                    for i in range(env_cfg.env.frame_stack):
                        policy_input[0, i * env_cfg.env.num_single_obs : (i + 1) * env_cfg.env.num_single_obs] = hist_obs[i][0, :]
                    
                    # ONNX模型推理获取动作
                    # 获取输入和输出名称
                    input_name = policy.get_inputs()[0].name
                    output_name = policy.get_outputs()[0].name
                    
                    # 运行推理
                    action_output = policy.run([output_name], {input_name: policy_input})[0]
                    action[:] = action_output[0]  # 提取动作
                     
                    action = np.clip(action, -env_cfg.normalization.clip_actions, env_cfg.normalization.clip_actions)# 动作值裁剪范围(-100,100)

                    target_q = action * env_cfg.control.action_scale  # 缩放动作到目标关节位置缩小一半
                    #print(f"预测目标关节位置: {target_q}")
                    
                target_dqs = np.zeros(31, dtype=np.double)#31[0]
                target_qs = np.zeros(31, dtype=np.double)#31[0]
                
                target_qs[0:3] = [0,0,0]#腰部
                target_qs[3:15] = target_q#腿部
                target_qs[15:22] = ros_target_q[15:22]-joint_nominal_pos[15:22]#左手7
                target_qs[22:23] = ros_target_q[22:23]-joint_nominal_pos[22:23]#左手夹爪-0.01~0.012(关)0.01
                target_qs[23:30] = [0.785,0,0,-0.785-1.56,1.56,0,0]-joint_nominal_pos[23:30]#右手7
                target_qs[30:31] = [-0.02]-joint_nominal_pos[22:23]#右手右手夹爪
                
                #全身扭矩
                taus_all = pd_control_all(target_qs, qs, cfg.robot_config.kps,target_dqs, dqs, cfg.robot_config.kds, joint_nominal_pos.copy())  # 计算扭矩
                #限制双腿扭矩输出
                taus_all[3:15] = np.clip(taus_all[3:15], -cfg.robot_config.tau_limit, cfg.robot_config.tau_limit)  # 限制扭矩范围-500 500
                
                data.ctrl = taus_all #输入扭矩到mujoco
                
                applied_tau = data.actuator_force  # 读取mujoco实际应用的扭矩

                # 执行仿真步
                mujoco.mj_step(model, data)
                
                # 更新Mujoco步数计数
                mujoco_step_count += 1
                
                #渲染mujoco
                if count_lowlevel%30==0:
                    viewer.sync()  

                #显示相机
                if count_lowlevel % 50 == 0:
                    for i in range(len(camera_names)):
                        camera_images[i] = get_camera_image(camera_names[i],model, data, renderer)
                        camera_images[i] = cv2.resize(camera_images[i], (640, 480))
                    for i in range(len(camera_names)):
                        if camera_images[i] is not None:
                            cv2.imshow(camera_names[i], cv2.cvtColor(camera_images[i], cv2.COLOR_RGB2BGR))
                    cv2.waitKey(1)
                    # 更新OpenCV帧数计数
                    opencv_frame_count += 1
                    
                # 数据采集逻辑
                # 检查采集命令状态变化
                if date_collect_cmd != prev_collect_cmd:
                    if date_collect_cmd:  # 开始采集
                        data_collecting = True
                        frame_count = 0
                        data_collect_counter = 0
                        print(f"开始采集第 {data_group_count} 组数据")
                    else:  # 停止采集
                        data_collecting = False
                        print(f"停止采集第 {data_group_count} 组数据，共采集 {frame_count} 帧")
                        data_group_count += 1  # 准备下一组
                    
                    prev_collect_cmd = date_collect_cmd
                
                # 如果正在采集数据，按照20Hz频率采集
                if data_collecting:
                    data_collect_counter += 1
                    if data_collect_counter >= data_collect_interval:
                        # 获取相机图像
                        head_img = get_camera_image('head_camera', model, data, renderer)
                        hand_img = get_camera_image('left_hand_camera', model, data, renderer)
                        
                        if head_img is not None and hand_img is not None:
                            # 调整图像大小
                            head_img = cv2.resize(head_img, (640, 480))
                            hand_img = cv2.resize(hand_img, (640, 480))
                            
                            # 保存数据
                            save_data(data_group_count, frame_count,qs, head_img, hand_img)
                            
                            # 更新帧计数
                            frame_count += 1
                            
                            # 每采集50帧打印一次进度
                            if frame_count % 50 == 0:
                                print(f"第 {data_group_count} 组数据已采集 {frame_count} 帧")
                        
                        # 重置计数器
                        data_collect_counter = 0
                
                count_lowlevel += 1
                
                # 定期打印帧率信息
                current_time = time.time()
                elapsed_time = current_time - last_print_time
                if elapsed_time >= print_interval:
                    # 计算Mujoco刷新率
                    mujoco_fps = mujoco_step_count / elapsed_time
                    # 计算OpenCV帧率
                    opencv_fps = opencv_frame_count / elapsed_time
                    
                    print(f"\n=== 性能统计 (每{print_interval}秒更新) ===")
                    print(f"Mujoco刷新率: {mujoco_fps:.1f} Hz (步数: {mujoco_step_count})")
                    print(f"OpenCV显示帧率: {opencv_fps:.1f} Hz (帧数: {opencv_frame_count})")
                    
                    if data_collecting:
                        print(f"数据采集: 进行中 - 第 {data_group_count} 组, 已采集 {frame_count} 帧")
                    else:
                        print(f"数据采集: 未进行")
                    print(f"理论最大刷新率: {1/cfg.sim_config.dt:.0f} Hz")
                    print(f"实际刷新率/理论值: {mujoco_fps/(1/cfg.sim_config.dt)*100:.1f}%")
                    print("================================\n")
                    
                    # 重置计数器和时间
                    mujoco_step_count = 0
                    opencv_frame_count = 0
                    last_print_time = current_time

        # 清理资源
        if viewer.is_running():
            viewer.close()
        cv2.destroyAllWindows()
        global exit_flag
        exit_flag = True
    except Exception as e:
        print(f"MuJoCo 渲染循环失败: {e}")
    finally:
        # 清理资源
        if viewer.is_running():
            viewer.close()
        cv2.destroyAllWindows()
        exit_flag = True  # 设置退出标志以停止键盘线程
class ActuatorCmdsSubscriber(Node):
    def __init__(self):
        super().__init__('actuator_cmds_subscriber')
        self.subscription = self.create_subscription(
            ActuatorCmds,
            '/hardware/arm_actuators_cmds',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量警告
        
        # 初始化全局控制变量
        global ros_control_enabled, ros_target_q
        ros_control_enabled = False
        ros_target_q = np.zeros(31, dtype=np.double)
        self.get_logger().info('ActuatorCmds订阅者已启动')

    def listener_callback(self, msg):
        global ros_control_enabled, ros_target_q
        
        # 启用ROS控制
        ros_control_enabled = True
        
        # 重置控制参数
        ros_target_q.fill(0.0)
        
        ros_target_q[15] = msg.pos[0]
        ros_target_q[16] = msg.pos[1]
        ros_target_q[17] = msg.pos[2]
        ros_target_q[18] = msg.pos[3]
        ros_target_q[19] = msg.pos[4]
        
        ros_target_q[20] = msg.pos[10]
        ros_target_q[21] = -msg.pos[11]
        ros_target_q[22] = (msg.pos[12]+2)/200#-4~0(闭合)#左手夹爪-0.1~0.012(关)0.01
        
        ros_target_q[23] = msg.pos[5]
        ros_target_q[24] = msg.pos[6]
        ros_target_q[25] = msg.pos[7]
        ros_target_q[26] = msg.pos[8]
        ros_target_q[27] = msg.pos[9]
        
        ros_target_q[28] = msg.pos[13]
        ros_target_q[29] = msg.pos[14]
        ros_target_q[30] = msg.pos[15]

def start_ros2_node():
    """启动ROS2节点的函数"""
    # 创建两个订阅者
    rclpy.init()
    actuator_subscriber = ActuatorCmdsSubscriber()
    
    # 创建执行器来同时运行两个节点
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(actuator_subscriber)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        actuator_subscriber.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    
    robot_name = "elf31"

    dof_num = 31

    dof_use = 12

    ankle_y_offset = 0.0

    joint_name = (
        "waist_y_joint",
        "waist_x_joint",
        "waist_z_joint",
        
        "l_hip_z_joint",   # 左腿_髋关节_z轴
        "l_hip_x_joint",   # 左腿_髋关节_x轴
        "l_hip_y_joint",   # 左腿_髋关节_y轴
        "l_knee_y_joint",   # 左腿_膝关节_y轴
        "l_ankle_y_joint",   # 左腿_踝关节_y轴
        "l_ankle_x_joint",   # 左腿_踝关节_x轴

        "r_hip_z_joint",   # 右腿_髋关节_z轴    
        "r_hip_x_joint",   # 右腿_髋关节_x轴
        "r_hip_y_joint",   # 右腿_髋关节_y轴
        "r_knee_y_joint",   # 右腿_膝关节_y轴
        "r_ankle_y_joint",   # 右腿_踝关节_y轴
        "r_ankle_x_joint",   # 右腿_踝关节_x轴

        "l_shld_y_joint",   # 左臂_肩关节_y轴
        "l_shld_x_joint",   # 左臂_肩关节_x轴
        "l_shld_z_joint",   # 左臂_肩关节_z轴
        "l_elb_y_joint",   # 左臂_肘关节_y轴
        "l_elb_z_joint",   # 左臂_肘关节_y轴
        
        "r_shld_y_joint",   # 右臂_肩关节_y轴   
        "r_shld_x_joint",   # 右臂_肩关节_x轴
        "r_shld_z_joint",   # 右臂_肩关节_z轴
        "r_elb_y_joint",    # 右臂_肘关节_y轴
        "r_elb_z_joint",    # 右臂_肘关节_y轴
        
        "l_wrist_y_joint",
        "l_wrist_x_joint",
        "l_hand_joint",

        "r_wrist_y_joint",
        "r_wrist_x_joint",
        "r_hand_joint",
        )   

    joint_nominal_pos = np.array([   # 指定的固定关节角度
        0.0, 0.0, 0.0,
        0,0.0,-0.3,0.6,-0.3,0.0,
        0,0.0,-0.3,0.6,-0.3,0.0,
        
        0.9,0.0,0.0,-0.3,0.0,
        0.0,0.0,0.0,# 左臂放在大腿旁边 (Y=0 肩平, X=0 前后居中, Z=0 不旋转, 肘关节微弯)-0.03,-0.025
        
        0.9,0,0.0,0.-0.3,0,
        0.0,0.0,0.0],    # 右臂放在大腿旁边 (Y=0 肩平, X=0 前后居中, Z=0 不旋转, 肘关节微弯)
        dtype=np.float32)

    onnx_model_path = 'policy/model.onnx'  # 修改为您的ONNX模型文件路径

    class env_cfg_bxi():

        class env():
            frame_stack = 15  # 历史观测帧数
            num_single_obs = (47+1)  # 单帧观测数
            num_observations = int(frame_stack * num_single_obs)  # 总观测空间 (66×47)
            num_actions = (12+0)  # 动作数
            num_commands = 5 # sin[2] vx vy vz

        class init_state():

            default_joint_angles = {
                'l_hip_z_joint': 0.0,
                'l_hip_x_joint': 0.0,
                'l_hip_y_joint': -0.3,
                'l_knee_y_joint': 0.6,
                'l_ankle_y_joint': -0.3,
                'l_ankle_x_joint': 0.0,
                
                'r_hip_z_joint': 0.0,
                'r_hip_x_joint': 0.0,
                'r_hip_y_joint': -0.3,
                'r_knee_y_joint': 0.6,
                'r_ankle_y_joint': -0.3,
                'r_ankle_x_joint': 0.0,
            }

        class control():
            action_scale = 0.5
            
        class commands():
            stand_com_threshold = 0.05 # if (lin_vel_x, lin_vel_y, ang_vel_yaw).norm < this, robot should stand
            sw_switch = True # use stand_com_threshold or not

        class rewards:
            cycle_time = 0.7

        class normalization:
            class obs_scales:
                lin_vel = 2.
                ang_vel = 1.
                dof_pos = 1.
                dof_vel = 0.05
                quat = 1.
            clip_observations = 100.
            clip_actions = 100.

    env_cfg= env_cfg_bxi()
    
    class Sim2simCfg():
        """仿真配置类"""
        class sim_config:
            """仿真配置"""
            mujoco_model_path = "model/elf2_31_arm/elf2_31_arm.xml"  # 手动指定MuJoCo模型路径
            sim_duration = 5000.0  # 仿真持续时间（秒）
            dt = 0.001  # 仿真时间步长（秒）
            decimation = 10  # 控制频率降采样因子

        class robot_config:
            joint_kp = np.array([     # 指定关节的kp，和joint_name顺序一一对应
                500,500,300,
                100,100,100,150,50,30,
                100,100,100,150,50,30,
                100,100,100,100,100,
                100,100,100,
                100,100,100,100,100,
                100,100,100], dtype=np.float32)

            joint_kd = np.array([  # 指定关节的kd，和joint_name顺序一一对应
                5,5,3,
                2,2,2,2.5,1,1,
                2,2,2,2.5,1,1,
                1,1,0.8,1,0.8,
                1,1,1,
                1,1,0.8,1,0.8,
                1,1,1], dtype=np.float32)

            ####
            kps=joint_kp
            kds=joint_kd
            tau_limit = 500. * np.ones(env_cfg.env.num_actions, dtype=np.double)  # 关节扭矩限制 num_actions=12
            ####
            default_dof_pos = np.array(list(env_cfg.init_state.default_joint_angles.values())) 
    # 加载ONNX模型
    print(f"从以下路径加载ONNX模型: {onnx_model_path}")
    
    # 创建ONNX Runtime会话
    # 可以选择不同的执行提供者，例如CUDA（如果可用）或CPU
    #providers = ['CPUExecutionProvider']  # 使用CPU执行
    providers = ['CUDAExecutionProvider']  # 使用GPU执行
    
    try:
        policy = ort.InferenceSession(onnx_model_path, providers=providers)
        print("ONNX模型加载成功")
        print(f"输入名称: {policy.get_inputs()[0].name}")
        print(f"输出名称: {policy.get_outputs()[0].name}")
        print(f"输入形状: {policy.get_inputs()[0].shape}")
        print(f"输出形状: {policy.get_outputs()[0].shape}")
    except Exception as e:
        print(f"加载ONNX模型失败: {e}")
        exit(1)

    print("启动ROS2订阅者...")

    ros2_thread = threading.Thread(target=start_ros2_node, daemon=True)
    ros2_thread.start()
    print("ROS2订阅者已启动，等待接收ActuatorCmds和MotionCommands消息...")
    # 运行MuJoCo仿真
    run_mujoco(policy, Sim2simCfg(), env_cfg)