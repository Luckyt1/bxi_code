#!/usr/bin/env python3
#导入环境变量
import os
os.environ['HYDRA_FULL_ERROR'] = '1'
os.environ['CUDA_VISIBLE_DEVICES'] = '0'

import hydra
import numpy as np
import torch
import dill
from omegaconf import OmegaConf
import pathlib
# 导入自定义模块
from config.utils38 import (
    dict_apply,
    interpolate_image_batch,
)
from controller.policy.dexgraspvla_controller import DexGraspVLAController
import logging
import cv2

# 初始化日志系统
def log_init():
    """配置并初始化日志记录器"""
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)

    # 定义日志格式
    format = "[%(levelname)s %(filename)s:%(lineno)d] %(message)s"
    handler = logging.StreamHandler()
    handler.setLevel(logging.INFO)
    formatter = logging.Formatter(format)
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    return logger

logger = log_init()

class Robot:
    """机器人控制主类，负责模型推理和轨迹预测"""
    def __init__(self,task_name):
        """
        根据配置初始化机器人系统
        参数:
            config: 包含以下关键字段的配置对象
                - controller_checkpoint_path: 控制器模型路径
                - device: 计算设备 (cpu/cuda)
                - executions_per_action_chunk: 每个动作块执行次数
        """
        # 加载预训练控制器模型
        config=OmegaConf.load(os.path.join("config", task_name + ".yaml"))
        checkpoint_path = config.controller_checkpoint_path
        payload = torch.load(checkpoint_path, pickle_module=dill)
        
        # 更新模型配置中的本地权重路径
        payload["cfg"]["policy"]["obs_encoder"]["model_config"]["head"][
            "local_weights_path"
        ] = "dinov2/checkpoints/dinov2_vitb14_pretrain.pth"
        
        cfg = payload["cfg"]
        cls = hydra.utils.get_class(cfg._target_)
        workspace = cls(cfg)
        workspace.load_payload(payload, exclude_keys=None, include_keys=None)
        
        # 初始化控制器
        self.controller: DexGraspVLAController
        self.controller = workspace.model

        # 配置参数
        self.device = config.device
        self.executions_per_action_chunk = config.executions_per_action_chunk
        self.n_latency_steps = config.n_latency_steps

        # 设置模型为评估模式并转移到指定设备
        self.controller.eval()
        logger.info(f'Using device: {self.device if torch.cuda.is_available() else "cpu"}')
        self.controller.to(self.device if torch.cuda.is_available() else "cpu")

    def prepare_observation(self, head_image_path, wrist_image_path, joint_angles):
        """准备观察数据"""
        # 读取图像
        # head_image = cv2.imread(head_image_path)
        # wrist_image = cv2.imread(wrist_image_path)
        
        #存入rgb格式图像
        head_image = head_image_path
        wrist_image = wrist_image_path
        
        if head_image is None:
            raise ValueError(f"无法加载头部图像: {head_image_path}")
        if wrist_image is None:
            raise ValueError(f"无法加载腕部图像: {wrist_image_path}")
            
        # 转换为numpy数组并确保数据类型正确
        proprioception = np.array(joint_angles, dtype=np.float32)
        
        # 图像预处理
        rgb_head = interpolate_image_batch(head_image[None, ...]).unsqueeze(0)
        rgb_wrist = interpolate_image_batch(wrist_image[None, ...]).unsqueeze(0)
        
        logger.info("观察数据已准备完成")
        return {
            "rgb": rgb_head,  # (1,1,3,H,W)
            "right_cam_img": rgb_wrist,  # (1,1,3,H,W)
            "right_state": torch.from_numpy(proprioception)
            .unsqueeze(0)
            .unsqueeze(0),  # (1,1,7)
        }

    def predict_trajectory(self, obs: dict) -> np.ndarray:
        """使用控制器模型预测轨迹"""
        # 将观察数据转移到模型设备
        obs = dict_apply(obs, lambda x: x.to(self.controller.device))
        
        # 模型推理
        with torch.no_grad():
            actions = self.controller.predict_action(obs_dict=obs)  # (B,64,action_dim)
        
        # 处理动作数据
        #n_latency_steps = 3  # 延迟补偿步数3
        trajectory = (
            actions.detach()
            .cpu()
            .numpy()[
                0, self.n_latency_steps : self.n_latency_steps + self.executions_per_action_chunk
            ]  # (executions_per_action_chunk, action_dim)
        )
        
        logger.info(f"预测轨迹形状: {trajectory.shape}")
        return trajectory

    def run_inference(self, head_image_path, wrist_image_path, joint_angles):
        """运行一次完整的推理过程"""
        logger.info("开始模型推理...")
        
        # 准备观察数据
        obs = self.prepare_observation(head_image_path, wrist_image_path, joint_angles)
        
        # 预测轨迹
        trajectory = self.predict_trajectory(obs)
        
        # 输出轨迹信息
        #logger.info(f"预测轨迹 :\n{trajectory}")

        for i, action in enumerate(trajectory):
            # 假设前6个值是关节角，最后1个是夹爪状态
            joint_angles = action[:(len(action)-1)]
            gripper_state = action[(len(action)-1)]
            # print(f"步骤 {i+1}:")
            # print(f"  关节角: {joint_angles}")
            # print(f"  夹爪状态: {gripper_state}")
            # print()
        
        return trajectory

