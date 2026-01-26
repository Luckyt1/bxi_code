#!/bin/bash

# 快速启动脚本
echo "=== 快速启动脚本 ==="
echo "请选择要执行的命令:"
echo "1) 启动xpm遥控器"
echo "2) “执行遥控器节点"
echo "3)"source message bag

read -p "请输入选项 (1-6): " choice

case $choice in
    1)
        echo "启动xpm遥控器"
        source ../bxi_ros2_pkg/setup.bash 
        cd ..
        python3 arm_test.py
        # 替换为你的开发服务器启动命令
        # npm start
        # python manage.py runserver
        # ./your_server
        ;;
    2)
        echo "执行遥控器节点"11
        cd bxi_rl_controller_ros2_example/
        source install/setup.bash
        ros2 launch remote_controller remote_conroller_launch.py 
        ;;
  
    3)
        echo "source message bag"
        source ../bxi_ros2_pkg/setup.bash 
        ;;
esac