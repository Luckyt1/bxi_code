import numpy as np
import mujoco
import mujoco.viewer
import pinocchio as pin
from pathlib import Path
import time
import communication.msg as bxiMsg
from rclpy.node import Node
from sensor_msgs.msg import JointState
import rclpy
joint_nominal_pos = np.array([   # 指定的固定关节角度
    0.0, 0.0, 0.0,
    0,0.0,-0.3,0.6,-0.3,0.0,
    0,0.0,-0.3,0.6,-0.3,0.0,
    0.1,0.0,0.0,-0.3,0.0,
    0,0,0,0, # 左臂放在大腿旁边
    0.1,0.0,0.0,-0.3,0.0,
    0,0,0,0],    # 右臂放在大腿旁边
    dtype=np.float32)

joint_kp = np.array([     # 指定关节的kp，和joint_name顺序一一对应
    0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    40,50,20,50,20,
    0,0,0,0,
    40,50,20,50,20,
    0,0,0,0,], dtype=np.float32)

joint_kd = np.array([  # 指定关节的kd，和joint_name顺序一一对应
    0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    1.0,1.0,0.8,1.0,0.8,
    0.5,0.3,1,0,
    0,0,0,0,0, 
    0,0,0,0], dtype=np.float32)
class WristControlNode(Node):
    def __init__(self):
        super().__init__('wrist_control_node')
       
        # 创建发布器
        self.qpos_publisher = self.create_publisher(
            bxiMsg.ActuatorCmds, 
            "/simulation/arm_actuators_cmds",
            10
        )
        
        # 创建订阅器 - 订阅关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/simulation/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.timer = self.create_timer(0.01, self.process_and_publish)  # 100Hz
        self.arm_qpos = np.zeros(18,dtype=np.float32)
        self.received_joint_pos = None  # 存储接收到的关节位置
        self.start_time = time.time()

    def process_and_publish(self):
        """处理目标角度并发布到ROS2主题"""
        try:
            msg = bxiMsg.ActuatorCmds()

            msg.kp = joint_kp[-18:].tolist()
            msg.kd = joint_kd[-18:].tolist()
            msg.pos = self.arm_qpos.tolist()
            msg.vel = np.zeros_like(self.arm_qpos).tolist()
            msg.torque = np.zeros_like(self.arm_qpos).tolist()
            
            self.qpos_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"处理和发布数据时出错: {e}")
    
    def joint_state_callback(self, msg):
        """关节状态回调函数 - 接收 /simulation/joint_states"""
        try:
            # 提取位置信息
            self.received_joint_pos = np.array(msg.position)
            
            # 可以根据需要提取特定关节的位置（例如左臂关节15-21）
            left_arm_pos = self.received_joint_pos[15:22]
                # self.get_logger().info(f"左臂关节位置: {left_arm_pos}", throttle_duration_sec=1.0)
            
        except Exception as e:
            self.get_logger().error(f"处理关节状态时出错: {e}")


    def destroy_node(self):
        """销毁节点时的清理工作"""
        self.listening = False
        if hasattr(self, 'udp_receiver'):
            self.udp_receiver.stop()
        super().destroy_node()

class MuJoCoGravityCompensator:
    """MuJoCo重力补偿器"""
    
    def __init__(self, mjcf_path: str):
        """初始化"""
        # 加载MuJoCo模型
        self.mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.mj_data = mujoco.MjData(self.mj_model)
        
        # 加载Pinocchio模型
        self.pin_model = pin.buildModelFromMJCF(mjcf_path)
        self.pin_data = self.pin_model.createData()
        # self.pin_model.gravity.linear = np.array([0., 0., -9.81])
    
        
        # 控制模式
        self.gravity_comp_enabled = True
        
        print("✓ 初始化完成")
        print(f"  总关节数: {self.pin_model.nv}")
        print(f"  左臂关节数: 7")
    
    def get_gravity_compensation(self) -> np.ndarray:
        """计算左臂重力补偿力矩"""
        q = self.mj_data.qpos.copy()
        tau_full = pin.computeGeneralizedGravity(
            self.pin_model, self.pin_data, q
        )
        return tau_full[15:22]
    
    def apply_gravity_compensation(self):
        """应用重力补偿"""
        if self.gravity_comp_enabled:
            tau = self.get_gravity_compensation()
            self.mj_data.ctrl[15:22] = tau
            return tau
        else:
            self.mj_data.ctrl[15:22] = 0
            return np.zeros(7)

count = 0
def run_simple_demo(compensator):
    """简单演示 - 自动运行"""
    print("\n开始重力补偿演示...")
    print("按 Ctrl+C 停止\n")
    global count
    with mujoco.viewer.launch_passive(
        compensator.mj_model, 
        compensator.mj_data
    ) as viewer:
        
        # 设置相机
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -15
        
        # 设置左臂初始姿态
        compensator.mj_data.qpos[15:22] = [0.0, 0.5, 0.0, -1.0, 0.0, 0.0, 0.0]
        # compensator.mj_data.qpos[0:7] = [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0]
        print(len(compensator.mj_data.qpos))
        try:
            while viewer.is_running():
                step_start = time.time()
                
                # 应用重力补偿
                tau = compensator.apply_gravity_compensation()
                
                # 仿真步进
                mujoco.mj_step(compensator.mj_model, compensator.mj_data)
                
                if(count % 20 == 0):
                    viewer.sync()   
                
                # 控制频率
                time_until_next_step = compensator.mj_model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                count += 1
                    
        except KeyboardInterrupt:
            print("\n演示已停止")


def main():
    """主函数"""
    mjcf_path = Path(__file__).parent.parent / "models" / "31tof_model" / "elf2_31_arm" / "elf2_31_arm.urdf"
    # 初始化补偿器
    compensator = MuJoCoGravityCompensator(str(mjcf_path))
    run_simple_demo(compensator)
    try:
        node = WristControlNode()
    except Exception as e:
        print(f"创建节点时出错: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
            rclpy.shutdown()
if __name__ == "__main__":
    main()