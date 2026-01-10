import requests
import json
import math
from shapely.geometry import LineString
import geopandas as gpd
import math
# import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Path
class GeoConverter:
    def __init__(self, origin_lat, origin_lon):
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        # 地球半径 (米)
        self.R = 6378137.0 

    def latlon_to_xy(self, lat, lon):
        """
        将经纬度转换为 map 坐标系下的 x, y (单位: 米)
        x: 东向 (East)
        y: 北向 (North)
        """
        # 将角度转换为弧度
        rad_lat = math.radians(lat)
        rad_lon = math.radians(lon)
        rad_origin_lat = math.radians(self.origin_lat)
        rad_origin_lon = math.radians(self.origin_lon)

        # 计算纬度差 (y轴 - 北向)
        # 1度纬度对应的距离约为 111320 米
        delta_lat = rad_lat - rad_origin_lat
        y = delta_lat * self.R

        # 计算经度差 (x轴 - 东向)
        # 经度对应的距离随纬度变化，需要乘以 cos(纬度)
        delta_lon = rad_lon - rad_origin_lon
        x = delta_lon * self.R * math.cos(rad_origin_lat)

        return x, y
def shp_to_geojson():
    data = gpd.read_file('/home/tang/tang_ws/navigation2/data/121.496885_121.496885_路径.shp')
    
    path_geometry = data.geometry.iloc[0]

    waypoints = list(path_geometry.coords)

    return waypoints
class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
    def publish_rviz_path(self, poses):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = poses
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'已发布 nav_msgs/Path 用于可视化，包含 {len(poses)} 个点')
    def send_goal(self,poses):
        
        self.publish_rviz_path(poses)
        server_reached = self._action_client.wait_for_server(timeout_sec=10.0)
        if not server_reached:
            self.get_logger().error('动作服务器未启动！')
            return
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('目标被拒绝')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    def get_result_callback(self, future):
        result = future.result().result
        print('导航任务完成!')
        rclpy.shutdown()
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'当前正在导航到第 {feedback.current_waypoint + 1} 个目标点')
        
    
def main():
    rclpy.init()
    try:
        waypoints =shp_to_geojson()
        first_pt=waypoints[0]
        origin_lat=first_pt[1]
        origin_lon=first_pt[0]
        converter=GeoConverter(origin_lat, origin_lon)
        # for lat,lon in waypoints:
        #     x,y=converter.latlon_to_xy(lon,lat)
        #     print(f"经度: {lon}, 纬度: {lat} => x: {x:.2f} m, y: {y:.2f} m")
        nav_poses=[]
        
        node = rclpy.create_node('waypoint_navigator')
        current_time=node.get_clock().now().to_msg()
        node.destroy_node()
        for pt in waypoints:
            origin_lat=first_pt[1]
            origin_lon=first_pt[0]
            x,y=converter.latlon_to_xy(pt[1],pt[0])
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            # 显式设置为 0，表示使用最新可用的变换
            pose.header.stamp.sec = 0
            pose.header.stamp.nanosec = 0
            
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            nav_poses.append(pose)
        sender = WaypointNavigator()
        sender.send_goal(nav_poses)
        rclpy.spin(sender)
            
    except Exception as e:
        print(f"发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
  # 请替换为自己的Key
if __name__ == "__main__":
    main()