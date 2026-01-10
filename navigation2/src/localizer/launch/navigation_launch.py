import os
import launch
import launch_ros.actions
from launch.actions import TimerAction
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('localizer')
    nav2_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    map_yaml_path = PathJoinSubstitution([FindPackageShare('localizer'), 'config', 'map.yaml'])
    nav2_params_path = PathJoinSubstitution([FindPackageShare('localizer'), 'config', 'nav2_params.yaml'])
    pcl_to_scan_params_path = PathJoinSubstitution([FindPackageShare('localizer'), 'config', 'pointcloud_to_laserscan.yaml'])

    return LaunchDescription([
        # 1. 启动 fastlio2 + localizer_node (立即启动 - 基础传感器)
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'localizer_launch.py')
            )
        ),

        # 在 navigation_launch.py 中，localizer_launch 之后加入：
        # 1.5 水平投影坐标系广播节点（延迟 3 秒，等待 map→body 稳定）
        TimerAction(
            period=3.0,
            actions=[
                launch_ros.actions.Node(
                    package='localizer',
                    executable='frame_projector_node',
                    name='frame_projector_node',
                    output='screen'
                )
            ]
        ),

        # 2. pointcloud → laserscan (延迟3秒 - 等待点云数据)
        TimerAction(
            period=3.0,
            actions=[
                launch_ros.actions.Node(
                    package='pointcloud_to_laserscan',
                    executable='pointcloud_to_laserscan_node',
                    name='pointcloud_to_laserscan',
                    remappings=[
                        ('cloud_in', '/fastlio2/body_cloud'),
                        ('scan', '/scan')
                    ],
                    parameters=[pcl_to_scan_params_path],
                    output='screen'
                )
            ]
        ),
        
        # 5. Navigation2 核心系统 (延迟8秒 - 等待地图和传感器数据就绪)
        TimerAction(
            period=4.0,
            actions=[
                launch.actions.IncludeLaunchDescription(
                    launch.launch_description_sources.PythonLaunchDescriptionSource(
                        os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': nav2_params_path,
                        'autostart': 'true',
                        'use_composition': 'False',
                    }.items()
                )
            ]
        ),

        # 6. RVIZ 可视化 (延迟10秒 - 等待所有核心组件启动)
        TimerAction(
            period=4.0,
            actions=[
                launch_ros.actions.Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2_nav',
                    output='screen',
                    arguments=['-d', os.path.join(nav2_dir, 'rviz', 'nav2_default_view.rviz')]
                )
            ]
        ),
        
        # 7. 激活 localizer_node (延迟12秒 - 在导航栈完全启动后)
        TimerAction(
            period=5.0,
            actions=[
                launch_ros.actions.Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['localizer_node']}
                    ]
                )
            ]
        ),
        
        # 可选：启动 cmd_vel_to_motion (延迟15秒 - 如果需要)
        # TimerAction(
        #     period=15.0,
        #     actions=[
        #         launch_ros.actions.Node(
        #             package='localizer',
        #             executable='cmd_vel_to_motion',
        #             name='cmd_vel_to_motion',
        #             output='screen',
        #             parameters=[nav2_params_path]
        #         )
        #     ]
        # ),
    ])
