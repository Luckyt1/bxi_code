import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    localizer_pkg = get_package_share_directory('localizer')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    # 实机模式：关闭仿真时间
    use_sim_time = False
    
    # 配置文件路径
    pcl2scan_params_file = PathJoinSubstitution([FindPackageShare('localizer'), 'config', 'pointcloud_to_laserscan.yaml'])
    ekf_config_file = PathJoinSubstitution([FindPackageShare('localizer'), 'config', 'ekf_gps.yaml'])
    
    return LaunchDescription([
        # === 1. TF 树构建 ===
        # map -> odom -> base_link -> 传感器
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_baselink',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='baselink_to_livox',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame'],
            output='screen'
        ),
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='baselink_to_gps',
            arguments=['-0.1', '0', '0.8', '0', '0', '0', 'base_link', 'gps_link'],
            output='screen'
        ),

        # === 2. 激光雷达避障：点云转2D扫描 ===
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/livox/lidar'),
                ('scan', '/scan')
            ],
            parameters=[pcl2scan_params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # === 3. GPS 定位：NavSat Transform ===
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='navsat_transform_node',
                    name='navsat_transform',
                    output='screen',
                    parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
                    remappings=[
                        ('imu', '/livox/imu'),
                        ('gps/fix', '/fix'),
                        ('odometry/filtered', '/odometry/global')
                    ]
                ),
            ]
        ),

        # === 4. GPS 定位：EKF 融合 ===
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node_map',
                    output='screen',
                    parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
                    remappings=[
                        ('odometry/filtered', '/odometry/global')
                    ]
                ),
            ]
        ),

        # === 5. 可视化 ===
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.join(nav2_bringup_pkg, 'rviz', 'nav2_default_view.rviz')],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'
                )
            ]
        )
    ])