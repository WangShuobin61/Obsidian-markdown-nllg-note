# ROS Noetic + Hesai Pandar40P 激光雷达 Docker 容器化运行教程

> 适用版本：ROS Noetic / Ubuntu 20.04 Docker 镜像
> 适用设备：Hesai Pandar40P 激光雷达
> 运行方式：Docker 容器 + host 网络 + 本机 GUI 显示
> 示例雷达 IP：192.168.20.13
> 示例本机雷达网口：enp6s0
> 示例本机网口 IP：192.168.20.100

---

## 1. 背景与方案说明

本教程以 ROS Noetic 和 Hesai Pandar40P 激光雷达为例，说明如何在 Docker 容器中运行激光雷达 ROS 驱动，并通过 RViz 在本机 GUI 桌面显示点云。

采用 Docker 容器化运行的主要目的：

1. 避免在宿主机反复配置 ROS 环境
2. 方便将配置好的 ROS 和雷达驱动整体打包迁移
3. 在不同工控机或国产化系统上尽量保持一致的运行环境
4. 通过 host 网络模式让容器直接使用本机网口接收激光雷达数据
5. 通过 X11 显示挂载让容器中的 RViz 直接显示到宿主机桌面

本教程中的关键设计是：

* 使用官方 ROS Noetic desktop-full 镜像
* 容器启动时使用 `--network host` 共享宿主机网口
* 容器启动时使用 `--privileged` 提供硬件访问权限
* 通过 `DISPLAY` 和 `/tmp/.X11-unix` 挂载实现 GUI 显示
* 使用 Hesai 官方 ROS 驱动运行 Pandar40P 雷达
* 通过 `roslaunch hesai_ros_driver start.launch` 启动驱动和 RViz

---

## 2. 硬件连接与网口确认

### 第一步：连接激光雷达

将 Hesai Pandar40P 激光雷达网线插入工控机或电脑的一个有线网口。

雷达上电后，系统中会出现对应的有线网卡。

---

### 第二步：确认雷达接在哪个网口

在宿主机中执行：

```bash
ip -br link
```

然后可以通过插拔雷达网线观察哪个网口状态发生变化。

例如插拔后发现变化的网口是：

```bash
enp6s0
```

则说明激光雷达连接在本机的 `enp6s0` 网口上。

---

## 3. 配置宿主机雷达网口 IP

### 第一步：确认雷达 IP

本教程示例中 Pandar40P 雷达 IP 为：

```bash
192.168.20.13
```

因此本机雷达网口需要配置到同一网段，例如：

```bash
192.168.20.100
```

---

### 第二步：临时配置网口 IP

在宿主机执行：

```bash
sudo ifconfig enp6s0 192.168.20.100
```

其中：

```bash
enp6s0
```

替换为你实际连接雷达的网口名。

---

### 第三步：确认 IP 是否配置成功

执行：

```bash
ifconfig
```

找到对应网口 `enp6s0`，应能看到类似：

```bash
inet 192.168.20.100
```

说明本机雷达网口 IP 已经设置成功。

也可以用：

```bash
ip addr show enp6s0
```

确认网口 IP。

---

### 第四步：测试宿主机是否能 ping 通雷达

执行：

```bash
ping 192.168.20.13
```

如果能看到类似输出：

```bash
64 bytes from 192.168.20.13
```

说明宿主机与激光雷达网络链路已经打通。

注意：有些雷达可能禁用 ping。如果 ping 不通，也可以通过 UDP 抓包判断是否有点云数据。

---

## 4. 安装并拉取 ROS Noetic Docker 镜像

### 第一步：确认 Docker 可用

执行：

```bash
docker --version
docker info
```

如果能正常输出 Docker 版本和服务信息，说明 Docker 已经安装并运行。

---

### 第二步：拉取官方 ROS Noetic desktop-full 镜像

执行：

```bash
docker pull osrf/ros:noetic-desktop-full
```

该镜像包含：

* Ubuntu 20.04 用户空间
* ROS Noetic
* RViz
* rqt
* 常用 ROS desktop-full 工具

查看镜像：

```bash
docker images
```

应能看到：

```bash
osrf/ros   noetic-desktop-full
```

---

## 5. 允许 Docker 使用宿主机 GUI

容器中的 RViz、xeyes 等图形程序需要显示到宿主机桌面，因此需要放行 X11。

### 第一步：设置 DISPLAY

在宿主机执行：

```bash
export DISPLAY=:0
```

查看：

```bash
echo $DISPLAY
```

应输出：

```bash
:0
```

---

### 第二步：放行本地 Docker 图形访问

执行：

```bash
xhost +local:root
xhost +local:docker
```

如果提示没有 `xhost` 命令，则安装：

```bash
sudo apt update
sudo apt install x11-xserver-utils -y
```

然后重新执行：

```bash
xhost +local:root
xhost +local:docker
```

---

## 6. 创建支持网口和 GUI 的 ROS 容器

执行以下命令创建容器：

```bash
docker run -dit \
  --name ros_lidar_gui \
  --network host \
  --privileged \
  -e DISPLAY=:0 \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:noetic-desktop-full \
  bash
```

参数说明：

| 参数                                 | 作用                    |
| ---------------------------------- | --------------------- |
| `-dit`                             | 后台运行容器，并保持交互终端        |
| `--name ros_lidar_gui`             | 指定容器名称                |
| `--network host`                   | 共享宿主机网络，容器可直接使用本机网口   |
| `--privileged`                     | 提供较高权限，方便访问硬件资源       |
| `-e DISPLAY=:0`                    | 将宿主机显示器传给容器           |
| `-e QT_X11_NO_MITSHM=1`            | 避免部分 Qt 图形程序显示异常      |
| `-v /tmp/.X11-unix:/tmp/.X11-unix` | 挂载 X11 图形显示通道         |
| `osrf/ros:noetic-desktop-full`     | 使用 ROS Noetic 完整桌面版镜像 |
| `bash`                             | 容器启动后保持 bash 进程       |

---

## 7. 进入容器并检查网口

### 第一步：进入容器

```bash
docker exec -it ros_lidar_gui bash
```

进入后终端应变成类似：

```bash
root@主机名:/#
```

---

### 第二步：安装常用网络工具

容器内执行：

```bash
apt update
apt install -y net-tools iproute2 iputils-ping tcpdump vim git cmake build-essential
```

---

### 第三步：查看容器内网口

执行：

```bash
ifconfig
```

或者：

```bash
ip a
```

因为创建容器时使用了：

```bash
--network host
```

所以容器里应该能看到和宿主机一致的网口，包括：

```bash
enp6s0
```

并且该网口 IP 应该为之前设置的：

```bash
192.168.20.100
```

---

### 第四步：测试容器是否能访问雷达

容器内执行：

```bash
ping 192.168.20.13
```

如果能正常返回，说明容器已经可以访问激光雷达。

---

## 8. 验证 Docker GUI 是否正常

### 第一步：安装 xeyes

容器内执行：

```bash
apt update
apt install -y x11-apps
```

---

### 第二步：运行 xeyes

```bash
xeyes
```

如果宿主机桌面弹出“小眼睛”窗口，说明 Docker GUI 已经打通。

如果没有弹出，回到宿主机重新执行：

```bash
export DISPLAY=:0
xhost +local:root
xhost +local:docker
```

然后重新进入容器测试。

---

## 9. 配置 ROS Noetic 环境

容器内执行：

```bash
source /opt/ros/noetic/setup.bash
```

为了以后每次进入容器自动加载 ROS 环境，可以写入 `.bashrc`：

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

如果出现 ROS Noetic 生命周期提醒，不影响当前运行。若不想显示该提醒，可以执行：

```bash
echo "export DISABLE_ROS1_EOL_WARNINGS=1" >> ~/.bashrc
source ~/.bashrc
```

---

## 10. 安装 Hesai 驱动依赖

### 安装 Boost

容器内执行：

```bash
apt update
apt install -y libboost-all-dev
```

---

### 安装 YAML

容器内执行：

```bash
apt update
apt install -y libyaml-cpp-dev
```

---

### 安装 catkin 相关工具

如果执行 `catkin_make` 时提示命令不存在，则安装：

```bash
apt update
apt install -y ros-noetic-catkin python3-catkin-tools
```

然后重新加载 ROS 环境：

```bash
source /opt/ros/noetic/setup.bash
```

检查：

```bash
which catkin_make
```

正常应输出类似：

```bash
/opt/ros/noetic/bin/catkin_make
```

---

## 11. 克隆 HesaiLidar_ROS_2.0 驱动

进入容器的 root 目录：

```bash
cd ~
```

克隆 Hesai 官方 ROS 驱动：

```bash
git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
```

如果项目已经提前复制到容器中，也可以直接使用已有目录。

---

## 12. 创建 ROS 工作空间并编译

### 第一步：创建 catkin 工作空间

```bash
cd ~
mkdir -p hesai_ws/src
```

---

### 第二步：复制驱动源码到 src 目录

假设驱动目录为：

```bash
~/HesaiLidar_ROS_2.0
```

执行：

```bash
cp -r ~/HesaiLidar_ROS_2.0 ~/hesai_ws/src/
```

如果你的驱动目录叫：

```bash
~/HeSai_pander40p
```

则执行：

```bash
cp -r ~/HeSai_pander40p ~/hesai_ws/src/
```

---

### 第三步：进入工作空间根目录

```bash
cd ~/hesai_ws
```

注意：`catkin_make` 应该在工作空间根目录执行，也就是包含 `src` 文件夹的目录。

---

### 第四步：编译

```bash
source /opt/ros/noetic/setup.bash
catkin_make
```

编译成功后加载工作空间环境：

```bash
source devel/setup.bash
```

为了每次进入容器自动加载该工作空间，也可以写入：

```bash
echo "source ~/hesai_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 13. 修改 Hesai 驱动配置文件

### 第一步：查找配置文件

进入驱动目录后查找配置文件：

```bash
find ~/hesai_ws/src -name "*.yaml" -o -name "*.json"
```

常见配置文件可能为：

```bash
config.yaml
```

或者位于：

```bash
hesai_ros_driver/config/
```

---

### 第二步：修改雷达 IP

找到配置项：

```yaml
device_ip_address: 192.168.1.201
```

将其修改为实际雷达 IP。

本教程示例中雷达 IP 为：

```yaml
device_ip_address: 192.168.20.13
```

---

### 第三步：确认端口配置

Pandar40P 常见配置为：

```yaml
udp_port: 2368
ptc_port: 9347
```

如果配置文件中有对应字段，应确认与雷达实际端口一致。

---

### 第四步：处理 multicast 配置

如果运行时报错：

```bash
Multicast IP error, set correct multicast ip address or keep it empty
```

说明配置文件中的组播 IP 配置不正确。

如果当前不使用组播，应将相关字段设置为空，例如：

```yaml
multicast_ip: ""
```

或者将占位符内容删除。

---

### 第五步：处理 channel fov 配置

如果运行时报错：

```bash
channel fov file does not exist: Your channel fov filter file path
```

说明配置文件中存在无效的通道视场角过滤文件路径。

如果当前不用该功能，可以将对应路径设置为空，或关闭该功能。例如：

```yaml
channel_fov_file: ""
```

具体字段名以实际配置文件为准。

---

## 14. 启动 Hesai 雷达 ROS 驱动

进入工作空间：

```bash
cd ~/hesai_ws
source devel/setup.bash
```

启动驱动：

```bash
roslaunch hesai_ros_driver start.launch
```

正常情况下终端会出现类似信息：

```bash
Hesai Lidar ROS
Hesai Lidar SDK
SocketSource::Open succeed
Lidar::Receive Udp Thread start to run
PtcClient::PtcClient() 192.168.20.13 9347
```

如果 launch 文件中包含 RViz，宿主机桌面会自动弹出 RViz 窗口。

---

## 15. 验证 UDP 点云数据

如果能 ping 通雷达，但 RViz 没有点云，可以用 tcpdump 验证 UDP 数据是否进入容器。

容器内执行：

```bash
tcpdump -i enp6s0 udp port 2368
```

如果持续刷出 UDP 数据，说明雷达点云数据已经进入容器。

也可以查看 PTC 端口：

```bash
tcpdump -i enp6s0 port 9347
```

---

## 16. 常见问题与解决方法

### 1. docker exec 进入容器时报错 container is not running

原因：容器已经停止。

查看：

```bash
docker ps -a
```

启动：

```bash
docker start ros_lidar_gui
```

进入：

```bash
docker exec -it ros_lidar_gui bash
```

如果启动后立刻退出，可以用：

```bash
docker start -ai ros_lidar_gui
```

---

### 2. 创建容器时报错 name is already in use

原因：已有同名容器。

查看：

```bash
docker ps -a
```

如果旧容器不要了：

```bash
docker rm -f ros_lidar_gui
```

然后重新创建。

---

### 3. 容器内 ping 命令不存在

安装：

```bash
apt update
apt install -y iputils-ping
```

---

### 4. 容器内 ifconfig 命令不存在

安装：

```bash
apt update
apt install -y net-tools
```

也可以直接用：

```bash
ip a
```

---

### 5. catkin_make 命令不存在

先加载 ROS：

```bash
source /opt/ros/noetic/setup.bash
```

如果仍然没有：

```bash
apt update
apt install -y ros-noetic-catkin python3-catkin-tools
```

---

### 6. xeyes 或 RViz 无法显示

宿主机执行：

```bash
export DISPLAY=:0
xhost +local:root
xhost +local:docker
```

创建容器时必须包含：

```bash
-e DISPLAY=:0
-v /tmp/.X11-unix:/tmp/.X11-unix
```

---

### 7. Multicast IP error

错误示例：

```bash
Multicast IP error, set correct multicast ip address or keep it empty
```

解决方法：检查配置文件中的 multicast 字段。如果不使用组播，将其设置为空：

```yaml
multicast_ip: ""
```

---

### 8. channel fov file does not exist

错误示例：

```bash
channel fov file does not exist: Your channel fov filter file path
```

解决方法：检查配置文件中是否存在占位符路径。如果不用该功能，将路径设置为空或关闭功能。

---

### 9. Packet with invalid delimiter

错误示例：

```bash
Packet with invaild delimiter
```

常见原因：

1. 雷达型号配置不正确
2. UDP 端口配置不正确
3. 收到的不是当前驱动期望的数据格式
4. 配置文件与雷达型号不匹配

建议检查：

```bash
grep -R "Pandar" -n ~/hesai_ws/src
grep -R "udp" -n ~/hesai_ws/src
grep -R "2368" -n ~/hesai_ws/src
```

确认：

```bash
雷达型号：Pandar40P
雷达 IP：192.168.20.13
本机 IP：192.168.20.100
UDP 端口：2368
PTC 端口：9347
```

---

## 17. 保存当前配置好的容器为镜像

如果容器中已经安装好依赖、编译好驱动、修改好配置，可以将当前容器保存为新镜像。

先退出容器：

```bash
exit
```

宿主机执行：

```bash
docker commit ros_lidar_gui ros_lidar_gui_saved:v1
```

查看镜像：

```bash
docker images
```

应能看到：

```bash
ros_lidar_gui_saved   v1
```

以后可以直接使用该镜像创建容器，不需要重新安装依赖。

---

## 18. 使用已保存镜像重新创建容器

如果已经保存了镜像：

```bash
ros_lidar_gui_saved:v1
```

可以用以下命令重新创建容器：

```bash
docker run -dit \
  --name ros_lidar_gui \
  --network host \
  --privileged \
  -e DISPLAY=:0 \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros_lidar_gui_saved:v1 \
  bash
```

进入：

```bash
docker exec -it ros_lidar_gui bash
```

---

## 19. 将镜像导出并迁移到其他电脑

### 第一步：导出镜像

```bash
docker save ros_lidar_gui_saved:v1 | gzip > ros_lidar_gui_saved_v1.tar.gz
```

---

### 第二步：传输到其他电脑

可以使用 U 盘拷贝，也可以通过 scp：

```bash
scp ros_lidar_gui_saved_v1.tar.gz 用户名@目标电脑IP:/home/用户名/
```

---

### 第三步：在目标电脑导入镜像

目标电脑执行：

```bash
docker load < ros_lidar_gui_saved_v1.tar.gz
```

查看镜像：

```bash
docker images
```

---

### 第四步：在目标电脑运行容器

```bash
docker run -dit \
  --name ros_lidar_gui \
  --network host \
  --privileged \
  -e DISPLAY=:0 \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros_lidar_gui_saved:v1 \
  bash
```

进入容器：

```bash
docker exec -it ros_lidar_gui bash
```

---

## 20. 常用 Docker 命令速查

查看镜像：

```bash
docker images
```

查看正在运行的容器：

```bash
docker ps
```

查看所有容器：

```bash
docker ps -a
```

启动容器：

```bash
docker start ros_lidar_gui
```

进入容器：

```bash
docker exec -it ros_lidar_gui bash
```

停止容器：

```bash
docker stop ros_lidar_gui
```

删除容器：

```bash
docker rm ros_lidar_gui
```

强制删除容器：

```bash
docker rm -f ros_lidar_gui
```

保存容器为镜像：

```bash
docker commit ros_lidar_gui ros_lidar_gui_saved:v1
```

导出镜像：

```bash
docker save ros_lidar_gui_saved:v1 | gzip > ros_lidar_gui_saved_v1.tar.gz
```

导入镜像：

```bash
docker load < ros_lidar_gui_saved_v1.tar.gz
```

查看 Docker 占用空间：

```bash
docker system df
```

---

## 21. 总结

完整流程为：

1. 将 Pandar40P 激光雷达接入本机网口
2. 使用 `ip -br link` 确认雷达对应网口
3. 将本机雷达网口配置为与雷达同网段 IP
4. 使用 `docker pull osrf/ros:noetic-desktop-full` 拉取 ROS Noetic 完整桌面镜像
5. 使用 `--network host` 和 `--privileged` 创建容器
6. 使用 `DISPLAY` 和 `/tmp/.X11-unix` 共享宿主机 GUI
7. 在容器中安装网络工具、Boost、YAML、catkin 工具
8. 克隆 HesaiLidar_ROS_2.0 驱动
9. 创建 catkin 工作空间并编译
10. 修改 config.yaml 中的雷达 IP 和相关参数
11. 运行 `roslaunch hesai_ros_driver start.launch`
12. 使用 RViz 查看点云
13. 将配置好的容器 commit 成镜像，方便后续迁移和复用

本方案已经验证的关键点包括：

* 容器可以共享宿主机雷达网口
* 容器可以 ping 通 Pandar40P 雷达
* 容器可以通过 X11 打开本机 GUI
* ROS Noetic 可以在 Docker 中运行 Hesai 雷达驱动
* RViz 可以在宿主机桌面显示容器内的点云结果
