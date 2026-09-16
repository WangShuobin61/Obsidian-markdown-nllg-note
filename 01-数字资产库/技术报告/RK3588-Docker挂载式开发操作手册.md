# RK3588 Docker 挂载式开发操作手册

> 适用对象：参与本项目的 Qt、ROS 2、底盘、导航、建图和现场部署同事，尤其是第一次接触本项目容器开发方式的成员。  
> 适用平台：RK3588（ARM64/aarch64）+ 国产 Linux/Ubuntu 用户空间 + Docker + X11。  
> 当前环境：ROS 2 Humble、Qt 6.10.3、FluentUI、GCC/CMake/Ninja。  
> 项目路径：`/home/kylin/Desktop/app_sourc/gas_control_bringup`。

## 1. 先理解这套开发模式

本项目采用“环境进镜像，代码和产物留在宿主机”的开发方式：

| 内容 | 保存位置 |
| --- | --- |
| ROS 2 Humble、Qt 6.10.3、FluentUI、编译器、系统依赖 | Docker 镜像 |
| Qt/C++/QML 项目源码 | RK3588 宿主机 |
| 外部 ROS 2 工作区源码 | RK3588 宿主机 |
| Qt 的 `build/` 产物 | RK3588 宿主机项目目录 |
| ROS 的 `build/install/log` | RK3588 宿主机 ROS 工作区 |
| 数据库、配置、截图等运行数据 | RK3588 宿主机持久化目录 |
| X11 socket、串口、GPU | 启动容器时挂载或透传 |

镜像可以理解为统一的开发电脑环境，容器是临时启动的工作进程，宿主机目录才是
源码和构建成果的长期保存位置。

执行 `docker/run.sh` 的运行链路如下：

```text
宿主机执行 run.sh
        ↓
脚本计算 UID/GID、挂载目录、X11、串口和渲染参数
        ↓
脚本执行 docker run --rm ...
        ↓
镜像中的 entrypoint.sh 加载 ROS 2 Humble 和已有工作区 overlay
        ↓
执行 bash、cmake、colcon 或 Qt 可执行文件
        ↓
命令结束后临时容器删除，宿主机挂载目录中的文件继续保留
```

`run.sh` 没有替代 Docker。它只是把容易输错的一长串 `docker run` 参数集中管理。

## 2. 当前有效的 Docker 文件

```text
gas_control_bringup/
├── docker/
│   ├── Dockerfile                    # 当前 ARM64 环境镜像定义
│   ├── run.sh                        # 所有开发操作的统一入口
│   ├── entrypoint.sh                 # 容器启动时加载 ROS 环境
│   ├── build_qt_module.sh            # QtBase 构建脚本
│   ├── build_qt_addon_module.sh      # 其他 Qt 模块构建脚本
│   └── config/
│       └── ros_profile.json          # 容器内运行配置
├── CMakePresets.json                 # Qt ARM64 Debug/Release 预设
├── build/                            # Qt ARM64 构建产物，位于宿主机
├── src/                              # Qt/C++/QML 源码
└── sx_gas_control_ros/               # 当前有意保留的不完整 ROS 参考工作区
```

项目根目录的 `Dockerfile` 是旧的“把源码复制进镜像”方案，只作历史参考。当前
RK3588 挂载式开发必须使用 `docker/Dockerfile`，并通过 `docker/run.sh` 操作。

## 3. 开发机首次检查

进入项目根目录：

```bash
cd /home/kylin/Desktop/app_sourc/gas_control_bringup
```

确认 CPU 架构：

```bash
uname -m
```

RK3588 应输出：

```text
aarch64
```

确认当前用户可以访问 Docker：

```bash
docker info
```

确认环境镜像存在：

```bash
docker image ls
```

应至少看到：

```text
gas-control-env   humble-qt6-arm64
```

执行项目自检：

```bash
bash docker/run.sh check
```

重点检查输出：

```text
machine=aarch64
qt=6.10.3
ros=humble
app_mount=ok
```

`ros_mount=missing` 或 `ros_overlay=not-built` 不一定是错误，只表示尚未通过
`ROS_WS_HOST` 挂载并构建外部 ROS 工作区。

## 4. `run.sh` 命令总览

查看帮助：

```bash
bash docker/run.sh help
```

| 命令 | 用途 | 容器退出后保留的内容 |
| --- | --- | --- |
| `image` | 构建/更新环境镜像 | Docker 镜像和构建缓存 |
| `shell` | 进入交互式开发容器 | 宿主机挂载目录中的修改 |
| `build debug` | 编译 Qt Debug 版本 | 项目 `build/` |
| `build release` | 编译 Qt Release 版本 | 项目 `build/` |
| `ros-build` | 编译挂载的 ROS 工作区 | ROS 工作区 `build/install/log` |
| `run debug` | 运行 Qt Debug 程序 | 数据目录及日志 |
| `run release` | 运行 Qt Release 程序 | 数据目录及日志 |
| `check` | 检查架构、Qt、ROS 和挂载 | 无 |

除 `image` 使用 `docker build` 外，其他命令都会执行一次新的 `docker run --rm`。
它们不是反复进入同一个长期容器。

以 ROS 同事最常用的命令为例：

```bash
bash docker/run.sh ros-build
```

- `bash`：使用 Bash 解释器运行脚本；
- `docker/run.sh`：从项目根目录定位统一入口脚本；
- `ros-build`：传给脚本的第一个参数，是本项目自定义的操作名称，不是 Docker
  官方子命令；
- 脚本识别 `ros-build` 后，才会在内部拼装并执行 `docker run ... colcon build`。

因此命令行中虽然没有直接写出 `docker run`、`-v` 和 `--network host`，这些参数
仍然存在，只是集中写在 `run.sh` 中。

## 5. 宿主机与容器目录映射

默认映射如下：

| 宿主机 | 容器内 | 权限 | 用途 |
| --- | --- | --- | --- |
| 项目根目录 | `/workspace/gas_control_bringup` | `rw` | Qt 源码和构建产物 |
| `$HOME/.local/share/GasControl` | `/home/gas/.local/share/GasControl` | `rw` | 数据库、截图、配置 |
| `ROS_WS_HOST` 指定目录 | `/workspace/ros` | `rw` | 外部 ROS 工作区 |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | `rw` | X11 图形窗口 |
| `$HOME/.Xauthority` | `/tmp/gas.Xauthority` | `ro` | X11 身份认证 |

`-v` 是 `--volume` 的简写，语法为：

```text
-v 宿主机路径:容器路径:权限
```

例如：

```bash
-v /home/kylin/ros2_ws:/workspace/ros:rw
```

容器中的 `/workspace/ros/build` 实际就是宿主机
`/home/kylin/ros2_ws/build`。所以容器删除后，编译结果仍然存在。

## 6. Qt 同事的日常开发流程

### 6.1 编译 Debug 版本

```bash
cd /home/kylin/Desktop/app_sourc/gas_control_bringup
bash docker/run.sh build debug
```

产物位于宿主机：

```text
build/ARM64_Qt_6_10_3-Debug/app_gas_control_bringup
```

### 6.2 运行 Debug 版本

```bash
bash docker/run.sh run debug
```

### 6.3 编译并运行 Release 版本

```bash
bash docker/run.sh build release
bash docker/run.sh run release
```

Release 产物位于：

```text
build/ARM64_Qt_6_10_3-Release/app_gas_control_bringup
```

### 6.4 修改不同文件后的处理

| 修改内容 | 是否重新编译 | 建议操作 |
| --- | --- | --- |
| QML | Debug 下通常不需要重新编译 | 关闭并重新启动 Debug 程序 |
| C++/头文件 | 需要 | `bash docker/run.sh build debug` |
| CMakeLists/CMakePresets | 需要重新配置和编译 | `bash docker/run.sh build debug` |
| Qt、FluentUI、系统库版本 | 需要更新环境镜像 | `bash docker/run.sh image ...` |
| `docker/config/ros_profile.json` | 不需要编译 | 重启程序后生效 |

Debug 版本会把项目自身的 QML URL 重定向到宿主机挂载的 `src/`，便于快速修改；
这不等于运行中的窗口会自动热更新，通常仍需重启应用。

## 7. ROS 同事的日常开发流程

### 7.1 准备外部 ROS 2 工作区

推荐把完整 ROS 工作区放在项目目录外，例如：

```text
/home/kylin/ros2_ws/
├── src/
├── build/       # colcon 自动生成
├── install/     # colcon 自动生成
└── log/         # colcon 自动生成
```

项目自带的 `sx_gas_control_ros/` 当前有意保持不完整，不能默认当作完整业务工作区。

### 7.2 设置工作区挂载

仅对当前命令设置：

```bash
ROS_WS_HOST=/home/kylin/ros2_ws \
bash docker/run.sh check
```

同一个终端中持续使用：

```bash
export ROS_WS_HOST=/home/kylin/ros2_ws
bash docker/run.sh check
```

设置后，宿主机 `/home/kylin/ros2_ws` 会挂载为容器内 `/workspace/ros`。

### 7.3 编译整个 ROS 工作区

```bash
export ROS_WS_HOST=/home/kylin/ros2_ws
bash docker/run.sh ros-build
```

脚本最终在容器内执行：

```bash
cd /workspace/ros
colcon build --symlink-install
```

其中：

- `colcon` 是 ROS 2 工作区构建工具；
- `build` 是 colcon 的构建子命令；
- `--symlink-install` 适合开发阶段，资源和 Python 文件尽量使用符号链接；
- C++ 代码仍然需要正常编译。

### 7.4 只编译指定 ROS 包

```bash
bash docker/run.sh ros-build \
    --packages-select sx_gas_qt_compat
```

编译指定包及其依赖链：

```bash
bash docker/run.sh ros-build \
    --packages-up-to sx_gas_bringup
```

限制并行任务数：

```bash
bash docker/run.sh ros-build \
    --parallel-workers 2
```

`ros-build` 后面的参数会原样转交给 `colcon build`。

### 7.5 进入交互式 ROS 开发容器

```bash
export ROS_WS_HOST=/home/kylin/ros2_ws
bash docker/run.sh shell
```

进入后可以执行：

```bash
cd /workspace/ros
colcon build --symlink-install
source install/setup.bash
ros2 pkg list
ros2 topic list
```

容器入口已经自动加载：

```bash
source /opt/ros/humble/setup.bash
```

如果 `/workspace/ros/install/setup.bash` 在容器启动前已经存在，入口也会自动加载
该 overlay。第一次构建刚刚生成 `install/` 时，可以在当前 shell 手动执行：

```bash
source /workspace/ros/install/setup.bash
```

### 7.6 不要在容器里再次运行 `docker/run.sh`

进入 `bash docker/run.sh shell` 后，已经处于容器内部。此时直接运行 `colcon`、
`cmake`、`ros2` 即可，不要再次执行：

```bash
bash docker/run.sh ros-build
```

后者会尝试在容器里再次访问 Docker daemon，当前方案没有挂载 Docker socket。

## 8. 联调 Qt 与 ROS

Qt 程序需要访问外部 ROS 工作区时，运行命令也必须携带同一个 `ROS_WS_HOST`：

```bash
export ROS_WS_HOST=/home/kylin/ros2_ws
bash docker/run.sh run debug
```

启动顺序建议：

1. 完成 `bash docker/run.sh ros-build`；
2. 启动底盘、传感器、导航或建图 ROS 节点；
3. 使用 `ros2 topic list`、`ros2 topic echo` 验证中间件；
4. 再启动 `bash docker/run.sh run debug`；
5. 检查 Qt 界面中的 ROS 状态。

容器使用 `--network host`，因此 DDS 不经过 Docker bridge/NAT。所有联调进程仍需
使用一致的 `ROS_DOMAIN_ID`。当前 Qt 硬件配置在
`docker/config/ros_profile.json` 中指定 `domain_id: 30`。

当前配置中的主要话题包括：

```text
/cmd_vel_key
/odom
/imu/data
/scan
/point_cloud_raw
/diagnostics
/gimbal_cmd
/joint_states
```

修改话题名、启动包或工作区路径前，应先核对
`docker/config/ros_profile.json`，不要在多处重复硬编码。

## 9. 当前 ROS 镜像能力边界

基础镜像为：

```text
ros:humble-ros-base-jammy
```

当前 Dockerfile 在基础镜像上增加了项目所需的 `rclcpp`、常用消息类型、DDS RMW、
Qt 和编译工具，但目前没有安装完整的：

- RViz2；
- Nav2；
- SLAM Toolbox；
- Gazebo；
- 特定雷达、相机和底盘驱动。

需要这些依赖时，应把对应 apt 包写进 `docker/Dockerfile`，例如：

```text
ros-humble-rviz2
ros-humble-navigation2
ros-humble-nav2-bringup
ros-humble-slam-toolbox
```

然后重新构建环境镜像。不要只在临时容器中执行 `apt install` 后就认为依赖已经
保存，因为 `docker run --rm` 的容器退出后，这些安装会一起消失。

为避免修改基础依赖层后触发耗时的 Qt 重新编译，新增的纯运行期 ROS 软件包建议
放在 Dockerfile 最终 `environment` 阶段的独立 `RUN apt-get ...` 层中。

ROS 包需要的系统依赖也应经过代码评审后固化到 Dockerfile。交互式容器中的
`rosdep install` 适合验证依赖清单，但不是永久保存依赖的方式。

## 10. GUI、X11 与渲染模式

### 10.1 默认启动

```bash
bash docker/run.sh run debug
```

脚本优先挂载宿主机 `.Xauthority`，一般不需要执行 `xhost`。

如果明确提示找不到 Xauthority，可以临时执行：

```bash
xhost +local:docker
```

测试完成后可收回：

```bash
xhost -local:docker
```

### 10.2 默认软件渲染

默认模式等价于：

```bash
GAS_RENDER_MODE=software bash docker/run.sh run debug
```

这里使用 Mesa 软件 OpenGL，虽然由 CPU 渲染，但仍支持 FluentUI 所需的着色器。
不要设置：

```bash
QT_QUICK_BACKEND=software
```

Qt Quick 的纯软件 adaptation 不支持 FluentUI 的部分 ShaderEffect，可能出现
“任务栏有图标、窗口已经全屏并获得焦点，但内容完全透明”。

### 10.3 尝试 RK3588 GPU

```bash
GAS_RENDER_MODE=gpu bash docker/run.sh run debug
```

GPU 模式会按实际存在的设备透传 `/dev/dri/card*`、`/dev/dri/renderD*` 和
`/dev/mali*`。RK3588 内核驱动与容器内 EGL/OpenGL 用户态库必须兼容；设备节点
存在不代表硬件加速一定可用。

## 11. 串口和其他硬件

默认宿主机串口：

```text
/dev/ttyCH343USB0
```

脚本仅在设备存在时透传，并把容器内路径固定为：

```text
/dev/ttyCH343USB0
```

宿主机设备名称不同时：

```bash
MCU_DEVICE=/dev/ttyUSB0 bash docker/run.sh run debug
```

脚本会自动添加设备 GID，不使用 `--privileged`。摄像头、CAN、GPIO 等设备目前
没有通用自动透传逻辑，需要按实际设备在 `run.sh` 中显式增加，不能为了省事直接
改成 `--privileged`。

## 12. 数据和文件属主

容器使用宿主机当前用户的 UID/GID 运行。当前 RK3588 上对应：

```text
uid=1000(gas) gid=1000(gas)
```

因此容器生成的 Qt 构建产物、ROS 构建产物和运行数据，在宿主机上仍属于普通用户，
不会变成 `root:root`。

不要使用下面的方式编译：

```bash
sudo bash docker/run.sh build debug
sudo bash docker/run.sh ros-build
```

否则挂载目录可能出现 root 所有的文件。

默认持久化数据目录：

```text
/home/kylin/.local/share/GasControl
```

使用其他目录：

```bash
GAS_DATA_HOST=/home/kylin/gas_data \
bash docker/run.sh run release
```

## 13. 环境镜像何时需要重新构建

以下修改不需要重建镜像：

- 修改 Qt/C++/QML 业务源码；
- 修改外部 ROS 工作区源码；
- 重新执行 CMake 或 colcon；
- 修改运行数据和大部分业务配置。

以下修改需要重建镜像：

- 新增 apt 系统包；
- 新增 Nav2、RViz2、SLAM Toolbox 等 ROS 二进制依赖；
- 修改 Qt 版本或模块；
- 修改 FluentUI；
- 修改 `docker/Dockerfile` 或 `entrypoint.sh`。

构建当前镜像：

```bash
cd /home/kylin/Desktop/app_sourc/gas_control_bringup
bash docker/run.sh image \
    --build-arg QT_BUILD_JOBS=2 \
    --build-arg QT_SOURCE_BASE=https://mirrors.ustc.edu.cn/qtproject/official_releases/qt/6.10/6.10.3/submodules
```

Qt 在 RK3588 上从源码编译耗时较长。当前已经给完整 Qt/FluentUI 构建阶段保留标签：

```text
gas-control-buildcache:qt6.10.3-fluentui-arm64
```

开发阶段不要执行：

```bash
docker system prune -a
```

该命令可能删除有用的 Qt 构建缓存，使下次重新经历长时间源码编译。

## 14. 镜像和项目迁移

当前正式环境镜像标签：

```text
gas-control-env:humble-qt6-arm64
```

导出镜像：

```bash
docker image save gas-control-env:humble-qt6-arm64 \
    | gzip -1 \
    > gas-control-env_humble-qt6-arm64_arm64.tar.gz
```

导入镜像：

```bash
docker load -i gas-control-env_humble-qt6-arm64_arm64.tar.gz
```

镜像只包含开发环境，不包含挂载式方案中的项目源码、Qt 可执行文件、外部 ROS
工作区和运行数据。这些内容必须另外通过 Git、压缩包或 `rsync` 传输。

打包项目源码但排除本机构建缓存：

```bash
tar -czf /home/kylin/Desktop/gas_control_bringup_source.tar.gz \
    --exclude='gas_control_bringup/build' \
    --exclude='gas_control_bringup/.cache' \
    --exclude='gas_control_bringup/.venv' \
    --exclude='gas_control_bringup/compile_commands.json' \
    -C /home/kylin/Desktop/app_sourc \
    gas_control_bringup
```

## 15. 常用排查命令

查看运行中的容器：

```bash
docker ps
```

进入已经运行的容器：

```bash
docker exec -it 容器ID bash
```

查看容器日志：

```bash
docker logs 容器ID
docker logs -f 容器ID
```

查看镜像：

```bash
docker image ls
```

查看 Docker 实际磁盘占用：

```bash
docker system df -v
```

确认容器身份和架构：

```bash
bash docker/run.sh shell
whoami
id
uname -m
```

预期为 `gas`、UID/GID 1000 和 `aarch64`。

## 16. 常见故障

### 16.1 无法连接 Docker daemon

典型提示：

```text
permission denied while trying to connect to the Docker daemon socket
```

检查：

```bash
systemctl status docker
id
ls -l /var/run/docker.sock
```

用户加入 `docker` 组后必须重新登录图形会话或 SSH 会话。不要用 `sudo` 绕过后
继续编译，否则会生成 root 所有的构建产物。

### 16.2 `I have no name!`

说明容器以某个数字 UID/GID 运行，但镜像 `/etc/passwd` 中没有对应用户记录。
当前新镜像已经创建 `gas` 用户。确认使用的不是旧镜像：

```bash
docker image ls gas-control-env:humble-qt6-arm64
bash docker/run.sh shell
whoami
```

### 16.3 有任务栏图标但窗口透明

检查是否人为设置了错误后端：

```bash
env | grep QT_QUICK_BACKEND
```

不要使用 `QT_QUICK_BACKEND=software`。退出旧容器后重新执行：

```bash
bash docker/run.sh run debug
```

### 16.4 ROS 工作区未就绪

典型提示：

```text
工作空间环境：/workspace/ros/install/setup.bash
```

如果尚未设置 `ROS_WS_HOST` 或第一次还没完成 colcon，这是预期状态。执行：

```bash
export ROS_WS_HOST=/home/kylin/ros2_ws
bash docker/run.sh ros-build
```

### 16.5 找不到 ROS 包

依次检查：

```bash
echo "$ROS_WS_HOST"
bash docker/run.sh check
bash docker/run.sh shell
test -f /workspace/ros/install/setup.bash && echo ready
ros2 pkg list
```

如果缺少的是 Nav2、RViz2、SLAM 或设备驱动，先确认它是否真的安装在镜像中。

### 16.6 `exec format error`

一般表示把 x86_64 可执行文件或镜像带到了 ARM64。检查：

```bash
file build/ARM64_Qt_6_10_3-Debug/app_gas_control_bringup
docker image inspect \
    --format '{{.Architecture}}' \
    gas-control-env:humble-qt6-arm64
```

应分别包含 `ARM aarch64` 和 `arm64`。

### 16.7 提示另一个程序实例正在运行

项目使用单实例保护，且容器使用宿主机 IPC。先关闭已有 Qt 程序，再重新执行
`bash docker/run.sh run debug`。

## 17. 新人推荐操作顺序

第一次接手项目时严格按以下顺序操作：

```bash
# 1. 进入项目
cd /home/kylin/Desktop/app_sourc/gas_control_bringup

# 2. 查看镜像
docker image ls

# 3. 检查 Qt/ROS/ARM64 环境
bash docker/run.sh check

# 4. 编译 Qt Debug
bash docker/run.sh build debug

# 5. 运行纯界面模式
bash docker/run.sh run debug

# 6. 指定完整 ROS 工作区
export ROS_WS_HOST=/home/kylin/ros2_ws

# 7. 检查 ROS 挂载
bash docker/run.sh check

# 8. 编译 ROS 工作区
bash docker/run.sh ros-build

# 9. 启动 Qt + ROS 联调
bash docker/run.sh run debug
```

判断“改完以后该执行什么”的简单规则：

```text
改 QML                → 重启 Debug 程序
改 Qt/C++             → build debug → run debug
改 ROS 源码           → ros-build → 启动/联调
加系统或 ROS 二进制包 → 修改 Dockerfile → image → 再构建业务代码
改 Qt/FluentUI        → 修改 Dockerfile → image（可能耗时很长）
```

## 18. 团队约定

1. 所有日常操作从项目根目录执行；
2. 使用 `docker/run.sh` 作为统一入口，避免每个人维护不同的 `docker run` 命令；
3. 不使用 `sudo` 生成构建产物；
4. 不使用 `--privileged` 代替精确设备透传；
5. 不在临时容器里永久安装依赖；依赖必须进入 Dockerfile；
6. ROS 工作区路径通过 `ROS_WS_HOST` 明确指定；
7. Qt、ROS 和配置修改分别走对应构建流程，不混用 x86 与 ARM64 构建目录；
8. 修改 Dockerfile、运行参数或挂载路径时同步更新本文档；
9. 清理镜像前先执行 `docker system df -v`，保留 Qt/FluentUI 构建缓存；
10. 交付时同时记录镜像标签、源码版本、ROS 工作区版本和配置版本。
