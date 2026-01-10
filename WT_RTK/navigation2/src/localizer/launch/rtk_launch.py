import os
import launch
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
    nav2_params_path = PathJoinSubstitution([FindPackageShare('localizer'), 'config', 'nav2_params.yaml'])
    return LaunchDescription([
        # === 1. TF 树构建 ===
        # map -> odom -> base_link -> 传感器
        # 注意：删除了 odom_to_baselink 的静态变换，因为这应由 EKF 发布
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
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='baselink_to_imu',
            arguments=['-0.1', '0', '0.8', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),

        # === 3. 激光雷达避障：点云转2D扫描 ===
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

        # === 4. GPS 定位：NavSat Transform ===
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='navsat_transform_node',
                    name='navsat_transform',
                    output='screen',
                    # arguments=['--ros-args', '--log-level', 'debug'],
                    parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
                    remappings=[
                        ('imu/data', '/imu/rtk'),
                        ('gps/fix', '/fix'),
                        ('odometry/filtered', '/odometry/global')
                    ]
                ),
            ]
        ),

        # === 5. GPS 定位：EKF 融合 ===
        TimerAction(
            period=8.0,
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
        
        # === 6. Navigation2 启动 ===
        TimerAction(
                    period=10.0,
                    actions=[
                        launch.actions.IncludeLaunchDescription(
                            launch.launch_description_sources.PythonLaunchDescriptionSource(
                                os.path.join( nav2_bringup_pkg, 'launch', 'navigation_launch.py')
                            ),
                            launch_arguments={
                                'use_sim_time': 'false',
                                'params_file': nav2_params_path,
                                'autostart': 'true',
                                'use_composition': 'False',
                            }.items()
                        )
                    ]
                ),

        # === 5. 可视化 ===
        TimerAction(
            period=12.0,
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