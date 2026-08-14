# Ubuntu 16.04 + ROS1 Kinetic Docker 可行性测试记录

## 1. 测试结论

**测试成功。**

已经完成以下验证：

- 成功拉取 `ros:kinetic-ros-base-xenial` 基础镜像。
- 成功构建自定义镜像 `ros1-kinetic-test:16.04`。
- 成功启动并进入容器。
- 容器操作系统确认为 Ubuntu 16.04.7 LTS。
- ROS 发行版确认为 ROS1 Kinetic。
- `roscore` 和 `catkin_make` 均可用。
- 空 catkin 工作空间成功完成构建。

因此，使用 Docker 部署 **Ubuntu 16.04 + ROS1 Kinetic ros-base** 环境在当前 amd64 宿主机上可行。

## 2. 测试环境

- 宿主机：Ubuntu 22.04.5 LTS
- 宿主机架构：x86_64 / amd64
- Docker Engine：29.6.2
- 容器系统：Ubuntu 16.04.7 LTS（Xenial）
- ROS：ROS1 Kinetic
- ROS 镜像变体：`ros-base`
- 基础镜像：`ros:kinetic-ros-base-xenial`
- 生成镜像：`ros1-kinetic-test:16.04`
- 宿主机代理：FlClash，HTTP/Mixed 端口 `7890`

## 3. Dockerfile 设计

当前 `Dockerfile` 只负责验证基础 ROS1 环境，不包含 Qt、GUI 和业务项目源码。

构建过程中会自动检查：

1. 容器系统版本必须为 Ubuntu 16.04。
2. ROS 发行版必须为 Kinetic。
3. `roscore` 必须存在。
4. `catkin_make` 必须存在。
5. `/root/catkin_ws` 空工作空间必须能够正常编译。

容器默认启动命令是：

```dockerfile
CMD ["roscore"]
```

ROS Master 默认使用端口 `11311`。

## 4. 首次构建失败原因

首次执行：

```bash
docker build --pull -t ros1-kinetic-test:16.04 .
```

出现错误：

```text
failed to resolve source metadata
Head "https://registry-1.docker.io/...": i/o timeout
```

这不是 Dockerfile 或 ROS 镜像标签错误，而是 Docker daemon 无法直接连接 Docker Hub。

宿主机使用 FlClash，但 Docker daemon 是 systemd 系统服务，不会自动继承桌面应用或当前终端的代理设置。

通过 FlClash 代理测试 Docker Hub：

```bash
curl -x http://127.0.0.1:7890 \
  -I --connect-timeout 10 \
  https://registry-1.docker.io/v2/
```

返回：

```text
HTTP/1.1 200 Connection established
HTTP/2 401
```

这里的 `401` 是正常结果，说明已经成功连接 Docker Hub，只是请求未携带认证信息。

## 5. Docker daemon 代理配置

创建配置目录：

```bash
sudo mkdir -p /etc/systemd/system/docker.service.d
```

编辑配置文件：

```bash
sudo vim /etc/systemd/system/docker.service.d/http-proxy.conf
```

配置内容：

```ini
[Service]
Environment="HTTP_PROXY=http://127.0.0.1:7890"
Environment="HTTPS_PROXY=http://127.0.0.1:7890"
Environment="NO_PROXY=localhost,127.0.0.1,::1"
```

重新加载并重启 Docker：

```bash
sudo systemctl daemon-reload
sudo systemctl restart docker
```

检查代理是否生效：

```bash
systemctl show docker --property=Environment
```

注意：如果输出中的 `NO_PROXY` 末尾出现多余的 `>`，应检查并修正 `/etc/systemd/system/docker.service.d/http-proxy.conf`。

## 6. 拉取基础镜像

```bash
docker pull ros:kinetic-ros-base-xenial
```

成功结果：

```text
Status: Downloaded newer image for ros:kinetic-ros-base-xenial
docker.io/library/ros:kinetic-ros-base-xenial
```

## 7. 构建测试镜像

```bash
cd /home/nllg/my_project/docker_test

docker build \
  -t ros1-kinetic-test:16.04 \
  .
```

构建成功的关键输出：

```text
[+] Building 6.5s (7/7) FINISHED
naming to docker.io/library/ros1-kinetic-test:16.04
unpacking to docker.io/library/ros1-kinetic-test:16.04
```

查看镜像：

```bash
docker image ls
```

已生成：

```text
ros1-kinetic-test:16.04
ros:kinetic-ros-base-xenial
```

## 8. 进入临时测试容器

```bash
docker run --rm -it \
  --name ros1-kinetic-shell \
  ros1-kinetic-test:16.04 \
  bash
```

容器默认进入工作目录：

```text
/root/catkin_ws
```

工作空间中已经存在：

```text
build  devel  src
```

验证命令：

```bash
cat /etc/os-release
echo "$ROS_DISTRO"
rosversion -d
command -v roscore
command -v catkin_make
```

实际验证结果：

```text
PRETTY_NAME="Ubuntu 16.04.7 LTS"
VERSION_ID="16.04"
kinetic
kinetic
/opt/ros/kinetic/bin/roscore
/opt/ros/kinetic/bin/catkin_make
```

执行 `exit` 后容器正常退出，命令退出码为 `0`：

```bash
exit
```

由于使用了 `--rm`，退出后临时容器会自动删除，镜像仍然保留。

## 9. 后台运行 ROS Master

启动 `roscore`：

```bash
docker run -d --rm \
  --name ros1-kinetic \
  -p 11311:11311 \
  ros1-kinetic-test:16.04
```

查看运行状态：

```bash
docker ps
```

查看日志：

```bash
docker logs ros1-kinetic
```

进入正在运行的容器，并加载 ROS 环境：

```bash
docker exec -it ros1-kinetic \
  bash -c 'source /opt/ros/kinetic/setup.bash && exec bash'
```

进入后可执行：

```bash
rosnode list
rostopic list
```

停止容器：

```bash
docker stop ros1-kinetic
```

## 10. 常用管理命令

查看镜像：

```bash
docker image ls
```

查看运行中的容器：

```bash
docker ps
```

查看所有容器：

```bash
docker ps -a
```

删除测试镜像：

```bash
docker image rm ros1-kinetic-test:16.04
```

取消 Docker daemon 代理：

```bash
sudo rm /etc/systemd/system/docker.service.d/http-proxy.conf
sudo systemctl daemon-reload
sudo systemctl restart docker
```

## 11. 当前范围与限制

本次只证明基础容器环境可行，尚未包含：

- 业务项目源码
- 业务 ROS package
- Qt 或其他 GUI 框架
- RViz、Gazebo、rqt
- USB、串口、CAN、摄像头等硬件设备映射
- 宿主机与多容器之间的 ROS 网络配置
- 项目运行时依赖

Ubuntu 16.04 和 ROS Kinetic 均已停止维护。该方案适合遗留项目复现、迁移验证和受控环境部署，不建议直接作为面向公网的新生产环境。