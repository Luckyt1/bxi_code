import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 声明参数对象
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 2. 路径定义
    # 注意：不再使用 .perform()，直接传递 Substitution 对象给 parameters 更加安全和规范
    localizer_config_path = PathJoinSubstitution(
        [FindPackageShare("localizer"), "config", "localizer.yaml"]
    )

    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )

    return LaunchDescription(
        [
            # 3. 必须显式声明接收该参数，否则 IncludeLaunchDescription 传进来的值会被丢弃
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation clock if true'
            ),

            Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[
                    # 传入配置文件路径
                    {"config_path": lio_config_path},
                    # 关键修复：开启仿真时间同步，解决 NO Effective Points 问题
                    {"use_sim_time": use_sim_time}
                ],
            ),
            Node(
                package="localizer",
                namespace="localizer",
                executable="localizer_node",
                name="localizer_node",
                output="screen",
                parameters=[
                    {"config_path": localizer_config_path},
                    {"use_sim_time": use_sim_time}
                ],
            ),
            # 已删除：内部的 rviz2 启动。
            # 由 rtk_launch.py 统一管理可视化，避免冲突。
        ]
    )