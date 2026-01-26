import numpy as np
import mujoco
import mujoco.viewer
import pinocchio as pin
from pathlib import Path
import time
import communication.msg as bxiMsg
from sensor_msgs.msg import JointState
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import rclpy
joint_nominal_pos = np.array([ 
    0,0,0,1,0,0,0,                          # 指定的固定关节角度
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
    0,0,0,0,0,
    0,0,0,0,
    0,0,0,0,0,
    0,0,0,0,], dtype=np.float32)

joint_kd = np.array([  # 指定关节的kd，和joint_name顺序一一对应
    0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,
    0,0,0,0,
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
        
        print("✓ 重力补偿器初始化完成")
        print(f"  模型配置维度 (nq): {self.pin_model.nq}")
        print(f"  模型速度维度 (nv): {self.pin_model.nv}")
 
    def get_gravity_compensation(self, q: np.ndarray) -> np.ndarray:
        """计算左臂重力补偿力矩"""
        tau_full = pin.computeGeneralizedGravity(
            self.pin_model, self.pin_data, q
        )
        return tau_full[21:28].astype(np.float32)
    
    def apply_gravity_compensation(self):
        """应用重力补偿"""
        tau = self.get_gravity_compensation(self.qpos)
        return tau

class WristControlNode(Node):
    def __init__(self):
        super().__init__('wrist_control_node')
       
        # 创建发布器
        self.qpos_publisher = self.create_publisher(
            bxiMsg.ActuatorCmds, 
            "/hardware/arm_actuators_cmds",
            10
        )
        self.timer = self.create_timer(0.01, self.process_and_publish)  # 100Hz
        self.arm_qpos = np.zeros(16,dtype=np.float32)
        self.received_joint_pos = None
        self.start_time = time.time()
        
        # 创建重力补偿器作为类成员变量
        self.compensator = MuJoCoGravityCompensator(str(mjcf_path))
        self.step=0
        # 配置QoS策略 - 设置为BEST_EFFORT以匹配发布者
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建订阅器，使用兼容的QoS
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/hardware/joint_states',
            self.joint_state_callback,
            qos_profile
        )
        
    def process_and_publish(self):
        """处理目标角度并发布到ROS2主题"""
        try:
            msg = bxiMsg.ActuatorCmds()
            
            # 使用类成员变量的compensator
            direction = np.array([-1.5, -1, -1, -1, -1, -1,-1], dtype=np.float32)
            tau = self.compensator.apply_gravity_compensation()
            
            msg.kp = joint_kp[-16:].tolist()
            msg.kd = joint_kd[-16:].tolist()
            msg.pos = self.arm_qpos.tolist()
            msg.vel = np.zeros_like(self.arm_qpos, dtype=np.float32).tolist()
            
            # 创建完整的力矩数组：前5个和9-12位置是重力补偿，其余补零
            torque_array = np.zeros(16, dtype=np.float32)
            torque_array[:5] = (tau * direction)[:5]   # 前5个
            torque_array[10:12] = (tau * direction)[5:7]
            print(torque_array)
            msg.torque = torque_array.tolist()
            
            self.qpos_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"处理和发布数据时出错: {e}")
    # def joint_state_callback(self, msg):
    #     """关节状态回调函数 - 接收 /simulation/joint_states"""
    #     try:
    #         # 提取位置信息
    #         self.received_joint_pos = np.array(msg.position, dtype=np.float32)
            
    #         # 添加7个freejoint参数（baselink的位置和姿态）
    #         prefix_values = np.array([0, 0, 0, 1, 0, 0, 0], dtype=np.float32)
    #         self.compensator.qpos = np.concatenate([prefix_values, self.received_joint_pos])
                
    #     except Exception as e:
    #         self.get_logger().error(f"处理关节状态时出错: {e}")
    def joint_state_callback(self, msg):
        """关节状态回调函数 - 接收 /simulation/joint_states"""
        try:
            # 提取位置信息 (31位)
            pos = np.array(msg.position, dtype=np.float32)
           
            # 在28和31位置后各补一个0
            self.received_joint_pos = np.insert(pos, [28, 31], [0, 0])
            
            # 整体交换位置：[29,28,27,26] 和 [25,24,23,22,21]
            temp = self.received_joint_pos.copy()
            self.received_joint_pos[20:24] = temp[25:29]  # 21-25 <- 26-30 (即29,28,27,26,加一位)
            self.received_joint_pos[24:29] = temp[20:25]  # 26-29 <- 21-24 (即25,24,23,22)
            
            # if self.step % 50 ==0:
            #     print(self.received_joint_pos)
            # 添加7个freejoint参数（baselink的位置和姿态）
            prefix_values = np.array([0, 0, 0, 1, 0, 0, 0], dtype=np.float32)
            self.compensator.qpos = np.concatenate([prefix_values, self.received_joint_pos])
            self.step+=1    
        except Exception as e:
            self.get_logger().error(f"处理关节状态时出错: {e}")
    def destroy_node(self):
        """销毁节点时的清理工作"""
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