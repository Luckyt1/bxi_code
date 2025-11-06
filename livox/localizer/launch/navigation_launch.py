import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('localizer')
    nav2_dir = get_package_share_directory('nav2_bringup')

    # 布尔值：用于参数传递
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    map_yaml_path = PathJoinSubstitution([FindPackageShare('localizer'), 'config', 'map.yaml'])
    nav2_params_path = PathJoinSubstitution([FindPackageShare('localizer'), 'config', 'nav2_params.yaml'])
    pcl_to_scan_params_path = PathJoinSubstitution([FindPackageShare('localizer'), 'config', 'pointcloud_to_laserscan.yaml'])

    return LaunchDescription([
        # 1. 启动 fastlio2 + localizer_node
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'localizer_launch.py')
            )
        ),

        # 2. pointcloud → laserscan
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
        ),

        # 3. map_server
        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_yaml_path}
            ]
        ),

        # 4. lifecycle manager
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        ),

        # 5. Navigation2 核心（关键：所有 launch_arguments 必须是字符串！）
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_path,
                'autostart': 'true',        # 字符串
                'use_composition': 'False', # 字符串！不是 False
            }.items()
        ),

        # 6. RVIZ
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_nav',
            output='screen',
            arguments=['-d', os.path.join(nav2_dir, 'rviz', 'nav2_default_view.rviz')]
        ),
    ])
