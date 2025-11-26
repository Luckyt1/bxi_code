from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import math
import threading
from tf2_ros import Buffer, TransformListener

def create_pose(navigator, x, y, z, qx, qy, qz, qw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose

def navigate_to(navigator, pose, tf_node, tf_buffer, description="目标点"):
    print(f'\n--- 开始前往 {description} ---')
    navigator.goToPose(pose)

    while not navigator.isTaskComplete():
        # 刷新 TF 数据
        rclpy.spin_once(tf_node, timeout_sec=0.0)
        
        # 获取当前位置和姿态
        status_str = ""
        try:
            if tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.0)):
                t = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                p = t.transform.translation
                q = t.transform.rotation
                status_str += f"Pos:[{p.x:.2f}, {p.y:.2f}, {p.z:.2f}] Quat:[{q.x:.2f}, {q.y:.2f}, {q.z:.2f}, {q.w:.2f}] "
        except Exception:
            pass
        
        # 获取导航反馈
        if status_str:
            print(status_str, end='\r')
    
    return navigator.getResult()

def main():
    rclpy.init()
    
    # 1. 设置 TF 监听器 (用于获取机器人当前位置)
    tf_node = rclpy.create_node('tf_helper_node')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, tf_node)
    # 2. 创建导航器
    navigator = BasicNavigator() 

    # 等待 Nav2 完全启动
    # navigator.waitUntilNav2Active()

    waypoints = [
       ( -4.1571, 13.209, -0.37133, 0.0, 0.0, 0.0, 1.0),
        (-1.3976, 2.8667, -0.26238, 0.0, 0.0, 0.0, 1.0),
    ]
    for i, wp in enumerate(waypoints):
        x, y, z, qx, qy, qz, qw = wp
        
        # --- 正常前往目标 ---
        goal_pose = create_pose(navigator, x, y, z, qx, qy, qz, qw)
        # 执行端
        
        result = navigate_to(navigator, goal_pose, tf_node, tf_buffer, description=f"第 {i+1} 个目标点")

        if result == TaskResult.SUCCEEDED:
            print(f'\n第 {i+1} 个目标点到达！')
        elif result == TaskResult.CANCELED:
            print(f'\n第 {i+1} 个目标点任务被取消')
            break
        elif result == TaskResult.FAILED:
            print(f'\n第 {i+1} 个目标点导航失败')
            break

    print('\n所有任务结束。')
    tf_node.destroy_node()
    # 退出
    rclpy.shutdown()

if __name__ == '__main__':
    main()