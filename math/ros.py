import communication.msg as bxiMsg
import rclpy
from rclpy.node import Node
import numpy as np
joint_nominal_pos = np.array([   # 指定的固定关节角度
    0.0, 0.0, 0.0,
    0,0.0,-0.3,0.6,-0.3,0.0,
    0,0.0,-0.3,0.6,-0.3,0.0,
    0.1,0.0,0.0,-0.3,0.0,     # 左臂放在大腿旁边
    0.1,0.0,0.0,-0.3,0.0,
    0,0,0,
    0,0,0],    # 右臂放在大腿旁边
    dtype=np.float32)

joint_kp = np.array([     # 指定关节的kp，和joint_name顺序一一对应
    0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    40,50,20,50,20,
    0,0,0,0,0,
    20,20,50,
    0,0,0,], dtype=np.float32)

joint_kd = np.array([  # 指定关节的kd，和joint_name顺序一一对应
    0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    1.0,1.0,0.8,1.0,0.8,
    0,0,0,0,0,
    0.5,0.3,1,
    0,0,0], dtype=np.float32)
class WristControlNode(Node):
    def __init__(self):
        super().__init__('wrist_control_node')
        self.qpos_publisher = self.create_publisher(
                    bxiMsg.ActuatorCmds, 
                    "/hardware/arm_actuators_cmds",
                    10
                )
        self.timer = self.create_timer(0.001, self.process_and_publish)
        
    def process_and_publish(self):
        msg = bxiMsg.ActuatorCmds()
        msg.kp = joint_kp[-16:].tolist()
        msg.kd = joint_kd[-16:].tolist()
        msg.pos = joint_nominal_pos.tolist()
        msg.vel = np.zeros_like(joint_nominal_pos).tolist()
        msg.torque = np.zeros_like(joint_nominal_pos).tolist()
        self.qpos_publisher.publish(msg)
def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = WristControlNode()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()
if __name__ == "__main__":
    main()