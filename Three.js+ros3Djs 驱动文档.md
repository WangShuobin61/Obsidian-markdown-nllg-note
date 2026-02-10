五个终端五条命令

![[1770610168511.png]]


```bash
# 启动 rosbridge WebSocket 服务，使网页等外部程序可以通过 WebSocket 与 ROS2 通信
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 进入 Web 可视化服务工程目录，加载工作空间环境，并运行 URDF 发布节点用于网页端显示机器人模型
cd /home/tl/web_server
source install/setup.bash
ros2 run simple_web_viz urdf_publisher

# 安装 ROS2 的 rosbridge-server 软件包，提供 WebSocket 与 ROS2 通信的桥接功能
sudo apt install ros-$ROS_DISTRO-rosbridge-server

# 在当前目录启动一个简单的 HTTP 静态文件服务器，监听 8080 端口并允许局域网访问
python3 -m http.server 8080 --bind 0.0.0.0
```
