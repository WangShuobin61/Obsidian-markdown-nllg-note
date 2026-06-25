# GAS_CONTROL Docker 容器化部署指南

> 适用版本：Qt 6.10.3 / ROS 2 Humble / Ubuntu 22.04  
> 目标平台：x86_64（含兆芯 KX7000 或 海光HG3350 处理器 国产操作系统（银河麒麟或通信UOS）工控机）  
> 编写日期：2026-06-22

---

## 1. 背景与方案说明

本项目依赖 Qt 6.10.3 + FluentUI 第三方 QML 插件 + ROS 2 Humble，在国产化工控机（兆芯 KX7000，预装 UOS / 银河麒麟等系统）上缺少上述依赖，无法直接运行。

选用 Docker 容器化方案：把完整的 Ubuntu 22.04 用户空间、Qt 6.10.3（含 FluentUI 插件）、ROS 2 环境连同编译好的应用程序打包为一个镜像文件，镜像在任意安装了 Docker Engine 的机器上均可直接运行，彻底屏蔽底层系统差异。

### 关键设计决策

- **Qt 整套注入**：将开发机 `/opt/Qt/6.10.3/gcc_64`（含 FluentUI `.so`）打成 tarball 复制进镜像，路径与开发机一致（`/opt/Qt/6.10.3/gcc_64`），`CMakePresets.json` 无需任何修改即可在容器内外使用同一套预设。
- **镜像内做 Release 构建**：`Dockerfile` 里用 Ninja + GCC 做 Release 编译，QML 文件全部编译进二进制，不依赖任何宿主机路径；Debug build 因 `SourceUrlInterceptor` 机制依赖宿主机源码路径，**在容器内不可直接运行**。
- **多阶段构建（multi-stage build）保护源码**：`Dockerfile` 分为 `builder` 与 `runtime` 两个 stage。源码、Qt tarball、编译工具链（clang/cmake/ninja/git）全部留在 builder 阶段；交付的 runtime 镜像里只有可执行文件、`.so`、`.qm` 翻译文件和 Qt 运行时。**甲方拿到镜像看不到源码，也无法在镜像内重新编译**。

---

## 2. 项目相关文件说明


| 文件                             | 说明                           |
| ------------------------------ | ---------------------------- |
| `Dockerfile`                   | 镜像构建描述文件                     |
| `docker/prepare_qt.sh`         | 在开发机上将 Qt 打包为 tarball 的一次性脚本 |
| `docker/qt6103_gcc64.tar.gz`   | Qt 打包产物（**不提交 git**，需本地生成）   |
| `scripts/docker_entrypoint.sh` | 容器启动时执行，source ROS 环境并启动程序   |
| `.dockerignore`                | 排除无关目录，放行 Qt tarball 进入构建上下文 |


---

## 3. 开发机：首次构建镜像（完整流程）

> 前提：开发机已安装 Qt 6.10.3 至 `/opt/Qt/6.10.3`，已安装 Docker Engine。

### 第一步：生成 Qt tarball（约需 2 分钟，执行一次即可）

```bash
cd /home/wangshuobin/project/gas_control_ws
bash docker/prepare_qt.sh
```

**做了什么**：把 `/opt/Qt/6.10.3/gcc_64`（2.3GB，含 FluentUI 插件）压缩为 `docker/qt6103_gcc64.tar.gz`（约 500MB），作为 Docker 构建上下文的一部分。

> 此文件被 `.gitignore` 排除，不会提交到仓库。换机器时需重新生成。

### 第二步：构建 Docker 镜像（约需 10～15 分钟）

```bash
docker build -t gas_control .
```

**做了什么**：按 `Dockerfile` 描述依次执行两个 stage：

`builder` 阶段（带源码、工具链）：

1. 基于 `ros:humble`（Ubuntu 22.04 + ROS 2 Humble）
2. 安装编译工具链（clang、ninja、cmake）和 Qt 运行所需系统库、中文字体
3. 解压 Qt 6.10.3（含 FluentUI 插件）到 `/opt/Qt/6.10.3/gcc_64`
4. 复制源码，以 Release 模式编译整个项目

`runtime` 阶段（最终交付层，仅含可执行文件）：

1. 重新基于 `ros:humble`，仅装运行时所需系统库
2. 从 builder 复制 `/opt/Qt/6.10.3/gcc_64` 和 `build/docker-release/` 编译产物
3. 设置容器启动入口

最终 `gas_control:latest` 这个 tag 指向的是 **runtime 阶段的镜像**，builder 阶段不会产生独立 tag，但其层缓存仍然保留在 Docker 中（用于下次增量 build 加速）。

构建成功后可查看：

```bash
docker images gas_control
```

### 第三步：理解 build 命令的几个关键行为

**1）`docker build` 是增量构建**

每条 Dockerfile 指令对应一层（layer），Docker 会缓存每一层。下次 build 时，只要 **基础镜像 + 上文层 + 这条指令的内容 + 它要 COPY 的文件哈希** 都没变，这层就直接走缓存，几毫秒过。所以日常只改源码时：

- 前面装 apt 包、解 Qt tar 这些大头全部命中缓存
- 只重跑"COPY 源码 + cmake build + 复制产物到 runtime"这几层
- 通常 1～2 分钟完成

只要你修改了某条指令上游的内容，这条及之后的所有层缓存都会失效，因此尽量把"少变动"的层（apt install、Qt 解压）放在 Dockerfile 前面，"频繁变动"的层（COPY 源码）放后面。这一点当前 Dockerfile 已经做到。

**2）`-t gas_control:latest` 不是覆盖，是改 tag 指向**

Docker 镜像本身按内容寻址（image ID 是哈希），不可变。重新 build 时：

- 生成的是**新的 image ID**
- `gas_control:latest` 这个 tag 被重新指向新 ID
- 旧 ID 失去 tag，变成"悬空镜像（dangling image）"，仍占磁盘

所以每次 build 完建议清理：

```bash
docker image prune -f          # 清理悬空镜像
```

如果想保留多个版本对比，可以打带版本号的 tag：

```bash
docker build -t gas_control:v0.2 -t gas_control:latest .
```

**3）一条 `docker build` 默认跑完所有 stage**

Dockerfile 里有两个 `FROM`（`builder` 和 `runtime`），默认会顺序跑完，最终 tag 指向最后一个 stage（runtime）。中间 builder 没有独立 tag，但它的层缓存留在 Docker 内部，下次 build 时复用。

**4）只想构建到 builder 阶段（不打包 runtime）**

排查编译错误，或者想进 builder 看 `build/docker-release/` 产物时，用 `--target` 指定停在哪个 stage：

```bash
# 只跑 builder，不进 runtime 阶段
docker build --target builder -t gas_control:builder .

# 进入 builder 镜像查看编译产物
docker run -it --rm gas_control:builder /bin/bash
# 容器里能看到 /root/gas_control_ws/src/... 全套源码和 build/docker-release/
```

完整 build runtime 时，`gas_control:builder` 这个 tag 不会自动消失，需要时手动删：

```bash
docker rmi gas_control:builder
```

### 第四步：后续 Dockerfile 修改后的重新构建

```bash
docker build -t gas_control .
docker image prune -f
```

只改 Dockerfile 末尾几行（比如调整 `CMD`、`ENTRYPOINT`）时，前面所有缓存层全部命中，几秒钟搞定。

---

## 4. 运行容器内的应用程序

Qt GUI 程序需要访问宿主机的 X11 显示服务。

### 第一步：放行 X11（每次重启后执行一次）

```bash
xhost +local:docker
```

### 第二步：启动容器并运行程序

```bash
docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    gas_control
```

程序窗口会弹出在宿主机桌面上。`--rm` 表示容器退出后自动删除，不会留下残留容器。

---

## 5. 进入容器排查问题

如果程序没有弹出窗口，用以下命令进入容器手动诊断：

```bash
docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --entrypoint /bin/bash \
    gas_control
```

进入后手动运行：

```bash
source /opt/ros/humble/setup.bash
/opt/app/app_gas_control_bringup
```

观察终端报错。常见问题：


| 报错                                        | 原因                                        | 解决                                                |
| ----------------------------------------- | ----------------------------------------- | ------------------------------------------------- |
| `cannot connect to X server`              | 未执行 `xhost +local:docker` 或 DISPLAY 变量未传入 | 确认 `xhost` 命令已执行，`-e DISPLAY=$DISPLAY` 参数已加       |
| `No such file or directory`（QML 路径含宿主机路径） | 跑了 Debug build 而不是 Release build          | 确认运行的是 `/opt/app/app_gas_control_bringup`         |
| 中文显示方块                                    | 镜像缺中文字体                                   | Dockerfile 里已加 `fonts-noto-cjk`，重新 `docker build` |


## 后续如何在 Docker 里开发

关键概念：镜像里那份编译产物只是"交付样品"，日常开发不用它。开发时把宿主机源码挂载进容器，在容器里编译运行，产物直接落回宿主机硬盘。

启动开发容器（GUI 程序需要 X11，先放行显示）：

```bash
xhost +local:docker

docker run -it --rm \
    --name gas_dev \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/root/gas_control_ws \
    -v $HOME/.local/share/GasControl:$HOME/.local/share/GasControl \
    --entrypoint /bin/bash \
    gas_control
```

进容器后，用你平时一模一样的命令：

```bash
cd /root/gas_control_ws/src/gas_control_bringup
cmake --preset clang_debug
cmake --build --preset clang_debug
./build/Clang14_Qt_6_10_3-Debug/app_gas_control_bringup
```

`-v $(pwd):/root/gas_control_ws` 是核心：容器里的 /root/gas_control_ws 就是你宿主机当前目录。宿主机用编辑器改 .cpp/.qml，容器里立刻可见；容器里编译生成的 build/、compile_commands.json 也直接出现在宿主机。改代码→容器内重新 build→运行，循环往复。

**退出容器**：输入 `exit` 或按 `Ctrl+D`，容器立刻停止并因`--rm` 被自动删除。

---

## 6. 国产化工控机（KX7000）：仅运行，不开发

> 目标：把程序部署到兆芯 KX7000 工控机上让其运行，无需在工控机上进行任何编译或开发。

工控机只需安装 Docker Engine，不需要 Qt、ROS 2 或任何开发工具。

### 6.1 在工控机上安装 Docker Engine

工控机预装系统为 UOS / 银河麒麟等国产系统（x86_64 架构），通用安装方式：

```bash
curl -fsSL https://get.docker.com | sudo bash
sudo usermod -aG docker $USER
# 重新登录后 docker 命令无需 sudo
```

> 若工控机无法访问公网，参考 Docker 官方文档的离线安装方式，或向管理员申请内网镜像源。

### 6.2 将镜像传输到工控机

在**开发机**上导出镜像为文件：

```bash
docker save gas_control:latest | gzip > gas_control.tar.gz
# 约 5～6GB，复制到 U 盘或通过内网传输至工控机
```

在**工控机**上导入：

```bash
docker load < gas_control.tar.gz
```

### 6.3 运行程序

工控机上运行的是镜像内已编译好的 **Release 版本**。它的路径在 `Dockerfile` 末尾通过 `CMD` 写死：

```
/opt/app/app_gas_control_bringup
```

所以**不需要手动指定可执行文件路径**，`docker run gas_control` 启动时会自动走 `ENTRYPOINT`（初始化 ROS + Qt 环境）再执行这个 `CMD`。

> 注意：工控机场景**不要挂载源码目录**（不加 `-v $(pwd):/root/gas_control_ws`）。镜像本身**没有源码**（多阶段构建已剔除），可执行文件位于 `/opt/app/`，挂载源码也不会替换它。

#### 方式一：直接启动（推荐，日常使用）

```bash
# 放行 X11（每次重启后执行一次）
xhost +local:docker

# 启动程序（自动运行 CMD 里的 Release 可执行文件）
docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /opt/gas_control_data:/root/.local/share/GasControl \
    gas_control
```

各参数含义：


| 参数                                                       | 作用                   |
| -------------------------------------------------------- | -------------------- |
| `-it`                                                    | 交互式终端，能看到程序日志输出      |
| `--rm`                                                   | 容器退出后自动删除，不留残留容器     |
| `-e DISPLAY=$DISPLAY`                                    | 把宿主机显示器传给容器，GUI 才能显示 |
| `-v /tmp/.X11-unix:/tmp/.X11-unix`                       | X11 显示套接字，GUI 渲染通道   |
| `-v /opt/gas_control_data:/root/.local/share/GasControl` | 持久化数据库到工控机硬盘（见 6.5）  |


程序窗口会直接显示在工控机屏幕上。

#### 方式二：后台常驻运行（开机自启 / 无人值守场景）

```bash
docker run -d --restart unless-stopped \
    --name gas_control \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /opt/gas_control_data:/root/.local/share/GasControl \
    gas_control
```

- `-d`：后台运行，不占用当前终端
- `--restart unless-stopped`：程序崩溃或工控机重启后自动拉起（适合工业值守）
- `--name gas_control`：给容器固定名字，方便管理

后台模式下查看日志、停止、重启：

```bash
docker logs -f gas_control     # 实时查看程序输出
docker stop gas_control        # 停止
docker start gas_control       # 重新启动
docker rm -f gas_control       # 删除容器（更新版本前需先删）
```

> `--restart unless-stopped` 配合工控机的 Docker 开机自启（`systemctl enable docker`），即可实现"开机后程序自动运行"。

#### 进入容器排查（程序起不来时）

```bash
docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --entrypoint /bin/bash \
    gas_control
```

进入后手动运行，观察报错：

```bash
source /opt/ros/humble/setup.bash
/opt/app/app_gas_control_bringup
```

> **工控机不需要**项目源码、Qt tarball，也不需要执行任何编译命令。镜像内已包含完整的可执行程序。

### 6.4 版本更新

开发机重新构建新版本镜像后，重新走 6.2 流程，导出新 `gas_control.tar.gz`，在工控机上重新 `docker load`，旧镜像会被替换。

```bash
# 后台常驻模式：先删旧容器，再 load 新镜像，重新 run
docker rm -f gas_control
docker load < gas_control.tar.gz
docker image prune -f          # 清理旧的悬空镜像
# 然后重新执行 6.3 的启动命令
```

### 6.5 应用数据持久化

程序通过 `QStandardPaths::GenericDataLocation` + `GasControl` 子目录写入数据库，容器内实际路径为 `/root/.local/share/GasControl/app.db`。该路径**不挂载就会随 `--rm` 一起丢失**。

上面命令中的 `-v /opt/gas_control_data:/root/.local/share/GasControl` 把这份数据落到工控机的 `/opt/gas_control_data/app.db`，容器删除、镜像更新后数据依然保留。首次运行前可手动建好目录：

```bash
sudo mkdir -p /opt/gas_control_data
```

---

## 7. 同事开发机：接手开发

> 目标：同事在自己的机器上克隆代码，使用 Docker 作为编译和运行环境继续开发，无需在宿主机上安装 Qt 或 ROS。

### 7.1 安装 Docker Engine

```bash
curl -fsSL https://get.docker.com | sudo bash
sudo usermod -aG docker $USER
```

### 7.2 克隆代码并准备 Qt tarball

```bash
git clone <repo_url>
cd gas_control_ws
```

`docker/qt6103_gcc64.tar.gz` 不在 git 仓库里，需要单独获取，二选一：

- **方式 A（推荐）**：从现有开发机复制 `docker/qt6103_gcc64.tar.gz`（约 500MB）到新机器的 `docker/` 目录下，**无需安装 Qt**。
- **方式 B**：新机器已安装 Qt 6.10.3 到 `/opt/Qt/6.10.3`（含 FluentUI 插件），执行 `bash docker/prepare_qt.sh` 自行生成。

### 7.3 构建镜像

```bash
docker build -t gas_control .
# 约需 10～15 分钟，此后增量修改代码不需要重新构建镜像
```

### 7.4 日常开发工作流

宿主机用 IDE 编辑代码，用 Docker 容器编译和运行验证。

**启动开发容器**（挂载源码目录）：

```bash
xhost +local:docker
docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/root/gas_control_ws \
    --entrypoint /bin/bash \
    gas_control
```

**容器内编译和运行**：

```bash
source /opt/ros/humble/setup.bash
cd /root/gas_control_ws/src/gas_control_bringup

# 首次配置（之后只需 build）
cmake -S . -B build/docker-release -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DQt6_DIR=/opt/Qt/6.10.3/gcc_64/lib/cmake/Qt6

# 每次改完代码后：增量编译
cmake --build build/docker-release -j$(nproc)

# 运行验证
./build/docker-release/app_gas_control_bringup
```

`-v $(pwd):/root/gas_control_ws` 的效果：容器内 `/root/gas_control_ws` 就是宿主机当前目录，宿主机改的文件容器里立刻可见，容器编译出的 `build/` 也直接落在宿主机硬盘上。

### 7.5 宿主机本地开发 vs 容器 Release 构建的区别


|              | 宿主机 `cmake --preset clang_debug` | 容器内 Release 构建 |
| ------------ | -------------------------------- | -------------- |
| 前提           | 宿主机已装 Qt 6.10.3 + clang          | 只需 Docker      |
| QML 加载       | 从源码目录热重载（修改 QML 无需重编）            | 编译进 qrc 二进制    |
| 能在 KX7000 运行 | 不能                               | 能（随镜像部署）       |
| 适用场景         | IDE 补全、快速迭代                      | 交付验证、生产部署      |


---

## 8. 常用维护命令速查

```bash
# 查看本地镜像
docker images

# 查看正在运行的容器
docker ps

# 删除旧的悬空镜像（build 后清理）
docker image prune -f

# 进入正在运行的容器
docker exec -it <容器ID> /bin/bash

# 导出镜像
docker save gas_control:latest | gzip > gas_control.tar.gz

# 导入镜像
docker load < gas_control.tar.gz
```

