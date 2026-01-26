import numpy as np
import mujoco
import mujoco.viewer
import pinocchio as pin
from pathlib import Path
import time
import communication.msg as bxiMsg
from sensor_msgs.msg import JointState
from rclpy.node import Node
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
    20,20,20,0,
    0,0,0,0,0,
    0,0,0,0,], dtype=np.float32)

joint_kd = np.array([  # 指定关节的kd，和joint_name顺序一一对应
    0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    1.0,1.0,0.8,1.0,0.8,
    0.5,0.3,1,0,
    0,0,0,0,0, 
    0,0,0,0], dtype=np.float32)

mjcf_path = Path(__file__).parent.parent / "models" / "31tof_model" / "elf2_31_arm" / "elf2_31_arm.urdf"
class MuJoCoGravityCompensator:
    """MuJoCo重力补偿器"""
    
    def __init__(self, mjcf_path: str):
        """初始化"""
        self.pin_model = pin.buildModelFromMJCF(mjcf_path)
        self.pin_data = self.pin_model.createData()
    
        self.qpos = joint_nominal_pos.copy()
        # 控制模式
        self.gravity_comp_enabled = True
        
        print("✓ 初始化完成")
        print(f"  总关节数: {self.pin_model.nv}")
        print(f"  左臂关节数: 7")
    
    def get_gravity_compensation(self, q: np.ndarray) -> np.ndarray:
        """计算左臂重力补偿力矩"""
        tau_full = pin.computeGeneralizedGravity(
            self.pin_model, self.pin_data, q
        )
        return tau_full[15:22]
    
    def apply_gravity_compensation(self):
        """应用重力补偿"""
        if self.gravity_comp_enabled:
            tau = self.get_gravity_compensation(self.qpos)
            self.mj_data.ctrl[15:22] = tau
            return tau
        else:
            self.mj_data.ctrl[15:22] = 0
            return np.zeros(7)

class WristControlNode(Node):
    def __init__(self):
        super().__init__('wrist_control_node')
       
        # 创建发布器
        self.qpos_publisher = self.create_publisher(
            bxiMsg.ActuatorCmds, 
            "/simulation/arm_actuators_cmds",
            10
        )
        self.timer = self.create_timer(0.01, self.process_and_publish)  # 100Hz
        self.arm_qpos = np.zeros(18,dtype=np.float32)
        self.received_joint_pos = None
        self.left_arm_pos = np.zeros(7,dtype=np.float32)
        self.start_time = time.time()
        
        # 创建重力补偿器作为类成员变量
        self.compensator = MuJoCoGravityCompensator(str(mjcf_path))
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/simulation/joint_states',
            self.joint_state_callback,
            10
        )
        
    def process_and_publish(self):
        """处理目标角度并发布到ROS2主题"""
        try:
            msg = bxiMsg.ActuatorCmds()
            
            # 使用类成员变量的compensator
            tau = self.compensator.apply_gravity_compensation()
            
            msg.kp = joint_kp[-18:].tolist()
            msg.kd = joint_kd[-18:].tolist()
            msg.pos = self.arm_qpos.tolist()
            msg.vel = np.zeros_like(self.arm_qpos).tolist()
            # 前7个是左臂重力补偿力矩，后11个补零
            msg.torque = tau.tolist() + [0] * 11
            
            self.qpos_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"处理和发布数据时出错: {e}")
    def joint_state_callback(self, msg):
        """关节状态回调函数 - 接收 /simulation/joint_states"""
        try:
            # 提取位置信息
            self.received_joint_pos = np.array(msg.position)
            
            # 将接收到的关节位置赋值给compensator的qpos
            self.compensator.qpos = self.received_joint_pos.copy()
            
            # 可以根据需要提取特定关节的位置（例如左臂关节15-21）
            self.left_arm_pos = self.received_joint_pos[15:22]
        except Exception as e:
            self.get_logger().error(f"处理关节状态时出错: {e}")

    def destroy_node(self):
        """销毁节点时的清理工作"""
        self.listening = False
        if hasattr(self, 'udp_receiver'):
            self.udp_receiver.stop()
        super().destroy_node()
def main():
    """主函数"""
    rclpy.init()
    try:
        node = WristControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n程序正常退出")
    except Exception as e:
        print(f"运行时出错: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()