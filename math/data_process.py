import numpy as np
radians=np.zeros(16,dtype=np.float32)
class math_data_process:
    def __init__(self):
        self.last_qpos = None  # 用于存储上一次的角度值
        self.smooth_factor = 0.2  # 平滑因子，范围0-1，值越小越平滑
        self.max_angle_step = np.pi / 18  # 最大单步变化，单位为弧度（这里设为10度）

    def angle_difference(self, target, current):
        """计算两个角度之间的最小差值，考虑周期性"""
        diff = target - current
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff
    
    def smooth_angle_transition(self, target_angles, current_angles=None):
        """平滑角度过渡，正确处理-π到π的跳跃"""
        target_angles = np.array(target_angles, dtype=np.float32)
        
        # 使用self.last_qpos作为当前角度
        if self.last_qpos is None:
            self.last_qpos = target_angles.copy()
            return target_angles.copy()
        
        current_angles = self.last_qpos  # 使用上一次的角度作为当前角度
        smooth_angles = np.zeros_like(target_angles)
        
        for i in range(len(target_angles)):
            # 检测是否存在异常跳变
            angle_diff = self.angle_difference(target_angles[i], current_angles[i])
            
            # 如果角度变化过大，可能是传感器错误或通信错误，保持当前值
            if abs(angle_diff) > 1.5*np.pi:  # 度以上的突变认为是异常
                smooth_angles[i] = current_angles[i]
                continue
            
            # 限制最大单步变化
            if abs(angle_diff) > self.max_angle_step:
                angle_diff = np.sign(angle_diff) * self.max_angle_step
            
            # 应用平滑因子
            smooth_step = angle_diff * self.smooth_factor
            
            # 计算新角度
            new_angle = current_angles[i] + smooth_step
            
            # 规范化到[-π, π]范围
            while new_angle > np.pi:
                new_angle -= 2 * np.pi
            while new_angle < -np.pi:
                new_angle += 2 * np.pi
                
            smooth_angles[i] = new_angle

        # 更新历史角度
        self.last_qpos = smooth_angles.copy()
        
        return smooth_angles