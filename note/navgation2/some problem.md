-  出现gazebo版本不兼容的问题，一种解决办法模板
```
步骤1：移除冲突的软件包  
sudo apt remove gazebo  
sudo apt remove gz-tools2

步骤2：清理未使用的依赖  
sudo apt autoremove

步骤3：然后尝试安装 turtlebot3-gazebo  
sudo apt install ros-humble-turtlebot3-gazebo
```
- 构建 ` navagation2` （humble版本）
```
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws/src
git clone https://github.com/ros-planning/navigation2.git --branch humble
cd ~/nav2_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
colcon build --symlink-install
```
- test
```
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

```
