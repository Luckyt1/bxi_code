from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    启动语音监听节点
    功能：监听并显示语音识别结果
    """
    voice_listener = Node(
        package="wheeltec_mic_ros2",
        executable="voice_listener",
        output='screen',
        name='voice_listener'
    )

    ld = LaunchDescription()
    ld.add_action(voice_listener)
    
    return ld
