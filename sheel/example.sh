#!/bin/bash

# 1. 加载基础环境 (ROS 2 系统环境)
source /opt/ros/humble/setup.bash

# 2. 设置显示权限 (防止 RViz 报错)


echo "请选择要执行的操作："
echo "1 - 编译项目 (Build)"
echo "2 - 启动导航 (Run)"
echo "3 - 启动far_planner"
echo "4 - 启动可视化权限"
read -p "输入数字: " num

case $num in
    1)
        # 编译所有包
        cd autonomous_exploration_development_environment/
        colcon build 
        ;;
    2)
        cd autonomous_exploration_development_environment/
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        ros2 launch vehicle_simulator system_real_robot.launch.py 
        ;;
    3)
        cd far_planner/
        source install/setup.bash
        ros2 launch far_planner far_planner.launch 
        ;;
    4)
        export DISPLAY=:0
        xhost +local:root > /dev/null 2>&1
        ;;
    *)
        echo "输入错误"
        ;;
esac
