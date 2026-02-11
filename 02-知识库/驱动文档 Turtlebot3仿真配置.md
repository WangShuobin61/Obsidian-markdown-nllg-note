```bash
1.安装turtlebot3相关功能包:
sudo apt-get install ros-humble-turtlebot3* -y

2.下载gazebo模型:
mkdir -p ~/.gazebocd ~/.gazebo/
git clone https://github.com/osrf/gazebo_models models
rm -rf models/.git

3.设置环境变量:
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc

4.运行仿真:
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

5.运行键盘控制节点
ros2 run turtlebot3_teleop teleop_keyboard

6.运行建图节点
ros2 launch turtlebot3_cartographer cartographer.launch.py

7.保存地图
ros2 run nav2_map_server map_saver_cli -f ~/hourse

8.运行导航节点:
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/home/wangshuobin/hourse.yaml
```

![[微信图片_20251212165533_121_2.png]]