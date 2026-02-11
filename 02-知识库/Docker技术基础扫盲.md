[[中国信息测评中心自主可控要求 清单]]
# Docker 基础快速扫盲 Q& %% A %% 

## Q1：docker的打包和我使用cmake构建生成build、install，以及和我使用linuxdeployqt打包工具，有什么区别吗

这是一个非常棒的问题，触及了软件交付的核心。简单来说，它们是 三个不同层级 的操作，彼此有联系但目的不同。

让我们把你的软件比作一家“餐厅”。

### 1. CMake 构建/Install —— “装修厨房并做菜”

- 是什么 ：这是源代码变成可执行程序的过程。
  - cmake ：相当于设计图纸，规划厨房怎么建。
  - make ：相当于装修队进场，把代码编译成二进制文件（做菜）。
  - make install ：把做好的菜（可执行文件）和餐具（库文件）整齐地摆盘到一个目录下（比如 /usr/local/bin 或 ./install ）。
- 局限性 ：做出来的菜 只能在当前厨房（当前操作系统环境）吃 。如果你把这盘菜端到另一家没装修好的店（没有安装对应 Qt 库或 glibc 的电脑），客人就没法吃。
- 你的现状 ：你在 Ubuntu 22.04 上 make install 出来的程序，直接拷贝到 UOS 上是跑不起来的，因为 UOS 缺少这道菜需要的“餐具”（依赖库）。
### 2. linuxdeployqt —— “打包外卖”
- 是什么 ：这是一个 依赖收集工具 。
  - 它会扫描你的可执行文件（那盘菜），看看它需要哪些库（餐具），然后把这些库（Qt 库、系统库）一股脑全部塞进一个文件夹（AppDir），甚至打包成一个单文件（AppImage）。
- 目的 ：让你的程序可以“带走吃”。
- 关系 ：它发生在 make install 之后 。你先得有菜，才能打包外卖。
- 局限性 ： 它非常依赖底层系统 （glibc 版本）。
  - 这就是你当前遇到的最大痛点：你在 Ubuntu 22.04（glibc 2.35）上用 linuxdeployqt 打包的 AppImage，拿到 UOS V20（基于 Debian 10，glibc 2.28）上运行， 大概率会报错 version 'GLIBC_2.29' not found 。
  - 因为外卖盒（打包）虽然带了餐具（Qt 库），但它假设客人桌子（操作系统内核/glibc）是新的，结果 UOS 的桌子太旧了，放不下。
### 3. Docker 打包 —— “移动餐车”
- 是什么 ：这是一个 虚拟化交付 。
  - 它不光打包菜（程序）和餐具（Qt 库），它连 整个厨房的环境 （Ubuntu 22.04 的用户空间、文件系统、所有依赖）都打包进去了。
- 关系 ：
  - Docker 打包流程 包含了 CMake 构建流程。
  - Dockerfile 里的 RUN cmake ... && make ... 就是在容器内部执行第一步的操作。
  - Docker 替代了 linuxdeployqt 。既然我把整个厨房都搬过去了，我就不需要再费劲去挑哪些餐具需要打包（不需要 linuxdeployqt ），因为容器里什么都有。
- 优势 ： 彻底屏蔽底层差异 。
  - 不管你的宿主机是 UOS、CentOS 还是 Fedora，只要装了 Docker 引擎，你的“移动餐车”就能跑。
  - 容器内部依然是 Ubuntu 22.04 环境，所以你编译出来的程序能完美运行，根本感觉不到外面其实是 UOS。
### 总结对比表
方式 产物 能在 UOS 上跑吗？ 为什么？ 典型用途 CMake Install 一堆分散的文件 不能 缺库，缺环境，版本不兼容 本地开发调试 linuxdeployqt AppImage 单文件 很难 虽带了 Qt 库，但被底层的 glibc 版本卡死 同架构分发 (如 Ubuntu 发给 Ubuntu) Docker Docker 镜像 完美 自带完整 OS 环境，不依赖 UOS 的库 跨系统/跨版本部署 (工业标准)

### 你的最佳实践路线
针对你的项目（Qt 6 + ROS 2 Humble -> 部署到 UOS）：

1. 放弃 linuxdeployqt ：不要试图在 Ubuntu 22.04 上打包 AppImage 给 UOS 用，glibc 版本回退问题会让你崩溃。
2. 拥抱 Docker ：
   - 编写 Dockerfile。
   - 在 Dockerfile 里写上 cmake 和 make install 的命令。
   - 这样构建出来的 Docker 镜像，里面就包含了已经编译好、安装好的程序。
   - 把这个镜像丢给 UOS，它就能跑了。
   所以， Docker 打包 = ( 基础镜像 Ubuntu 22.04 ) + ( 在里面执行 CMake 构建 ) + ( 设置启动入口 ) 。它包含了前者的功能，但提供了更高级的封装。

 利用 Docker 屏蔽底层操作系统的差异 。

UOS V20 基于 Debian 10，内核和 glibc 较老，直接运行 ROS 2 Humble（基于 Ubuntu 22.04）和最新的 Qt 6.10 非常困难。Docker 是目前最完美的解决方案。



## Q2:docker的容器目录包括在容器内运行的构建等操作，是用户可干预的吗

这是一个非常好的问题，触及了 Docker 的核心机制。

简单直接的回答是： 在 docker build 构建过程中，用户不可直接干预（它是全自动的黑盒）；但在构建完成后，或者通过特定方式启动时，用户完全可以读取和干预。

针对您选中的代码段 COPY . /app 和 RUN cmake ... ，我们可以从三个阶段来理解“可见性”和“干预性”：

### 1. 构建阶段 (Build Time) —— "黑盒模式"

当您运行 docker build 时：

- 不可干预 ：第 67 行的 RUN cmake ... 是在一个临时的中间容器里自动执行的。您 不能 在这个过程中暂停、进去修改文件或手动敲命令。它就像一个批处理脚本，要么全部成功，要么报错停止。
- 不可见 ：构建生成的文件（如 /app/build/docker-release 里的 .o 文件和可执行文件）都封装在镜像的“层”（Layer）里，您无法直接在宿主机的文件夹里看到它们。

### 2. 运行阶段 (Run Time) —— "进入现场"

构建完成后，您可以通过启动容器来“干预”和“查看”：

- 进入容器查看 ：
  您随时可以启动一个 shell 进入容器内部，就像通过 SSH 连到另一台电脑一样：

  ```
  # 启动并进入容器终端
  docker run -it ros2_qt6_test:latest /bin/bash
  
  # 进去后，您可以自由操作：
  cd /app/build/docker-release
  ls -l          # 查看构建产物
  ./appqml_test  # 手动运行程序
  rm -rf *       #甚至可以删除文件（只影响当前容器，不影响镜像）
  ```

- 复制文件出来 ：
  如果您想把容器里编译好的文件拿出来给宿主机用：

  ```
  # 把容器里的 /app/build 拷贝到当前目录
  docker cp <容器ID>:/app/build ./local_build_output
  ```

###  ★★★ 3. 开发阶段 (Development) —— "实时干预" (挂载)  ★★★★★

如果您希望 "我在宿主机改代码，容器里立刻生效" 或者 "容器里编译生成的文件，我在宿主机能直接看到" ，那么 COPY 指令是不够的，您需要使用 挂载 (Volume) 。

这是开发时的标准做法：

```bash
# -v $(pwd):/app  --> 把当前的宿主机目录“映射”到容器的 /app 目录
docker run -it \
    -v $(pwd):/app \
    ros2_qt6_test:latest \
    /bin/bash
```

在这种模式下：

1. 完全透明 ：容器里的 /app 就是您宿主机的当前目录。
2. 双向干预 ：
   - 您在宿主机修改 main.cpp ，容器里看到的 main.cpp 也会立刻变。
   - 您在容器里运行 cmake 生成的 compile_commands.json ，会直接出现在您的宿主机硬盘上（这正是您之前解决 Ninja 问题时可能需要的）。

### 总结

- Dockerfile 里的命令 ：是自动化的“固化”过程，不可实时干预。
- 使用 -v 挂载启动 ：可以让容器目录变成用户完全可读、可写、可干预的“透明”目录。



## Q3: Docker 常用命令科普

Docker 的使用逻辑可以类比为“做菜”：

- Dockerfile = 菜谱 （定义怎么做）
- Image (镜像) = 做好的菜 （打包好的环境，只读模板）
- Container (容器) = 端上桌的菜 （正在运行的实例，可读写）
  以下是开发中最高频使用的命令：

  1. 镜像管理 (Image)

- 构建镜像

  ```
  # -t: 给镜像起个名字 (tag)
  # .: Dockerfile 所在的当前目录
  docker build -t my_image_name .
  ```

- 查看本地镜像

  ```
  docker images
  ```

- 删除镜像

  ```
  # 删除名为 my_image_name 的镜像
  docker rmi my_image_name
  ``` 2. 容器生命周期 (Container)
  ```

- 启动容器 (Run) —— 最常用的命令

  ```
  # -it: 交互模式 (进入容器终端)
  # --rm: 容器停止后自动删除 (适合临时测试)
  # -d: 后台运行 (Detached mode)
  # -v: 挂载目录 (主机目录:容器目录)
  # -p: 端口映射 (主机端口:容器端口)
  
  # 示例：启动并进入终端，退出后自动删除
  docker run -it --rm ros2_qt6_test /bin/bash
  ```

- 查看运行中的容器

  ```
  docker ps       # 查看正在运行的
  docker ps -a    # 查看所有的（包括已停止的）
  ```

- 停止与启动

  ```
  docker stop <容器ID>   # 停止容器
  docker start <容器ID>  # 重新启动已停止的容器
  ``` 3. 调试与操作
  ```

- 进入正在运行的容器 (非常重要)
  如果容器已经在后台运行（比如挂掉或者正在跑服务），你想进去看看：

  ```
  # exec: 在容器里执行命令
  # /bin/bash: 启动一个 shell
  docker exec -it <容器ID> /bin/bash
  ```

- 查看日志 程序跑不起来，或者后台输出了什么？

  ```
  # -f: 实时跟随输出 (Follow)
  docker logs -f <容器ID>
  ``` 4. 清理空间
  Docker 用久了会占用大量磁盘，可以使用修剪命令：
  ```

```
docker system prune
# 会删除所有停止的容器、未被使用的网络、悬空的镜像
```


### 总结：你的开发流

1. 修改代码。
2. docker build -t ros2_qt6_test . (重新构建镜像)
3. docker run ... (运行测试)
4. 如果出错， docker run -it ... /bin/bash 进入容器内部调试。



```
#检查Qt官方仓库源码可用版本
docker run --rm ros2_qt6_test pip3 install aqtinstall && aqt list-qt linux desktop
```

```
5.9.0 5.9.1 5.9.2 5.9.3 5.9.4 5.9.5 5.9.6 5.9.7 5.9.8 5.9.9
5.10.0 5.10.1
5.11.0 5.11.1 5.11.2 5.11.3
5.12.0 5.12.1 5.12.2 5.12.3 5.12.4 5.12.5 5.12.6 5.12.7 5.12.8 5.12.9 5.12.10 5.12.11 5.12.12
5.13.0 5.13.1 5.13.2
5.14.0 5.14.1 5.14.2
5.15.0 5.15.1 5.15.2
6.0.0 6.0.1 6.0.2 6.0.3 6.0.4
6.1.0 6.1.1 6.1.2 6.1.3
6.2.0 6.2.1 6.2.2 6.2.3 6.2.4
6.3.0 6.3.1 6.3.2
6.4.0 6.4.1 6.4.2 6.4.3
6.5.0 6.5.1 6.5.2 6.5.3
6.6.0 6.6.1 6.6.2 6.6.3
6.7.0 6.7.1 6.7.2 6.7.3
6.8.0 6.8.1 6.8.2 6.8.3
6.9.0 6.9.1 6.9.2 6.9.3
6.10.0 6.10.1 6.10.2
6.11.0
```



## Q4: 锐评一下docker技术~

好，来点不留情面的 😄
 我给你一个**工程师视角的“锐评版 Docker”**——优点不吹，缺点不藏。

------

### 一句话总评

> **Docker 是“把环境问题推迟到更高级别解决”的伟大发明，同时也是“把系统复杂度偷偷转移给运维和新同学”的罪魁祸首。**

它不是银弹，但它确实改变了软件工程。

------

### 👍 Docker 吹得动的地方（不是假优点）

1️⃣ 对开发者：**环境地狱终结者**

- “我这能跑，你那不行” 👉 直接失效

- Qt / ROS / CUDA / Python / Node 混合地狱 👉 一个镜像全收

- 新人入职：

  ```
  docker run
  ```

  > 第一天就能干活

**锐评**：
 Docker 是程序员版的“标准集装箱”，不优雅，但极其有用。

------

2️⃣ 对 CI/CD：**工业级复制粘贴**

- 构建环境=镜像 hash
- 测试、发布、回滚都有“时间冻结感”
- 极大降低“线上=线下不一致”的概率

**锐评**：
 Docker 不是为了让你写得爽，而是为了**让系统可重复**。

------

3️⃣ 对历史包袱：**垃圾收容所**

- Ubuntu 16.04 + GCC 5 + Qt 5.12？
   👉 进容器，封印它
- 老项目不用升级，也不用死

**锐评**：
 Docker 是**技术债的冷冻库**，不是还债工具。

------

### 👎 Docker 的黑点（新手和老手都会踩）

1️⃣ 学 Docker ≠ 学 Linux

**这是最大的误区**

很多人以为：

> “我会 Docker 了 ≈ 我懂系统了”

实际是：

- 不懂 namespace / cgroup
- 不懂文件系统 overlay
- 不懂信号、PID 1、init
- 不懂网络、端口、iptables

结果就是：

- 容器死得莫名其妙
- 日志找不到
- `CTRL+C` 没反应
- “为啥我这个容器一 run 就退出？”

**锐评**：

> Docker 降低了门槛，也掩盖了基础。

------

2️⃣ Dockerfile = 新一代“祖传脚本”

你肯定见过：

```
RUN apt update && apt install -y a b c d e f g h i j
```

- 没版本
- 没注释
- 没可维护性
- 谁也不敢动

**锐评**：
 Dockerfile 是 **bash 的转世灵童**。

------

3️⃣ 镜像体积：嘴上说轻量，身体很诚实

- “基础镜像很小！”
   👉 然后装 gcc、cmake、qt、ros、python、node、java
- 最终：**5GB 起步**

**锐评**：
 Docker 让“磁盘不值钱”这件事，提前到来了。

------

4️⃣ 调试体验：反人类

- gdb？strace？perf？
   👉 要么没装，要么权限不够
- GUI / Qt / Wayland / X11
   👉 配一次骂一次

**锐评**：

> Docker 对调试的态度是：
>  **“你不配在线上调试。”**

------

5️⃣ 一旦上生产，复杂度指数级爆炸

单机 Docker：

> “还挺香”

一旦：

- docker-compose
- volume
- network
- k8s

直接变成：

- YAML 驱动工程
- 日志、监控、网络三位一体折磨

**锐评**：
 Docker 本身很简单，**生态复杂得要命**。

------

### 🧠 几个真实但扎心的结论

### ❗ Docker 并没有解决这些问题

- 软件质量
- 架构设计
- 依赖治理
- 运维能力不足

它只是：

> **让这些问题延后爆炸**

------

### ❗ Docker 特别适合你现在的使用场景（说人话版）

结合你之前聊的内容（Qt / ROS2 / UOS / 发行）：

✔ 非常适合：

- Qt / ROS2 构建环境隔离
- CI 编译机
- 解决“Qt 6.10 在哪都能跑”

❌ 不一定适合：

- 桌面 GUI 最终发行
- 普通用户直接运行
- 性能/硬实时场景

------

## 🧾 我的终极锐评

> **Docker 是现代软件工程的“空气”：**
>
> - 没它你会窒息
> - 吸多了你会头疼
> - 最危险的是：你会忘了它不是自然存在的



## Q3：在统信 UOS V20 上安装 Docker

结论：完全可以安装。 UOS V20 是标准的 Linux 发行版（Debian 系），Docker 的支持非常好。

详细步骤：

1. 开启开发者模式（关键第一步）

   - 在 UOS 系统中，打开“控制中心” -> “通用” -> “开发者模式”。
   - 点击“进入开发者模式”，通常需要登录 UOS 账号并重启。
   - 原因：默认情况下 UOS 禁止安装非应用商店的软件和获取 root 权限。

2. 准备环境与添加源 打开终端（Terminal），执行以下命令：

   ```
   # 1. 更新索引并安装基础工具
   sudo apt-get update
   sudo apt-get install -y apt-transport-https ca-certificates curl 
   gnupg2 software-properties-common
   
   # 2. 添加 Docker 官方 GPG 密钥 (使用阿里云镜像加速)
   curl -fsSL https://mirrors.aliyun.com/docker-ce/linux/debian/
   gpg | sudo apt-key add -
   
   # 3. 添加软件源
   # 注意：这里必须强制指定 "buster"，因为 UOS V20 对应 Debian 10 
   (Buster)
   sudo add-apt-repository "deb [arch=amd64] https://mirrors.aliyun.
   com/docker-ce/linux/debian buster stable"
   ```

3. 安装 Docker Engine

   ```
   sudo apt-get update
   sudo apt-get install -y docker-ce docker-ce-cli containerd.io
   ```

4. 启动并验证

   ```
   sudo systemctl start docker
   sudo systemctl enable docker  # 设置开机自启
   sudo docker run hello-world   # 测试运行
   ```

   如果看到 "Hello from Docker!"，说明安装成功。

## Q5：ROS 2 + Qt 6 的开发与 Docker 打包全流程*（有待商榷，目前不采用）*

### *<u>**（目前考虑采用使用统一的基础镜像源、Volume挂载不同工作空间，及单镜像启动容器，单容器实例多入口的方式进行开发和部署及其最终打包,见Q2）**</u>*

你的理解是正确的： “在 Ubuntu 22.04 开发，在 Docker 里打包，在 UOS 上通过 Docker 运行” 。

对于你提到的 上位机（Qt+ROS2） 和 下位机（纯 ROS2） 架构，我为你设计了一套标准化的工程流。

  1. 核心架构设计
     我们不需要把所有东西塞进一个巨大的镜像，建议采用 双镜像 或 单镜像多入口 策略。为了维护方便，我们这里采用 单镜像多入口 策略（因为基础环境都是 ROS2 Humble）。

- 开发环境 (Dev Host) : 你的 Ubuntu 22.04 笔记本/台式机。

- 部署环境 (Target Host) : 统信 UOS V20 机器（已安装 Docker）。

- 交付物 (Artifact) : 一个 Docker 镜像文件（例如 my_robot_system.tar ）。 

 2.详细操作流程 阶段一：开发 (Development)  继续在你的 Ubuntu 22.04 上开发。

- 下位机代码 : 放在 ~/ros2_ws/src/robot_control

- 上位机代码 : 放在 ~/ros2_ws/src/qt_hmi (Qt6 C++ 项目)\

```
workspace/
├── src/
│   ├── upper_gui/        # Qt6 + ROS2
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src/
│   │
│   ├── lower_controller/ # 纯 ROS2
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src/
│
├── docker/
│   ├── Dockerfile.upper
│   ├── Dockerfile.lower
│   └── entrypoint.sh
```

  你不需要在开发阶段关心 Docker，只需要确保代码在 Ubuntu 22.04 上能跑通。
   阶段二：打包 (Packaging)
  这是关键步骤。我们需要编写一个 Dockerfile ，它就像一个“配方”，告诉 Docker 如何把你的代码做成一道“菜”。

在你的工作空间根目录（例如 ~/ros2_ws ）创建一个 Dockerfile ：

```
# 使用 ROS 2 Humble 的官方基础镜像 (ros-base 版本体积更小，包含通信核心库)
# 如果不需要 Rviz 等 GUI 工具，推荐使用此版本以加速下载
FROM ros:humble-ros-base

# 设置环境变量，防止 apt-get 安装时出现交互式提示
ENV DEBIAN_FRONTEND=noninteractive

# 1. 替换 apt 源为阿里云源 (加速系统软件安装)
RUN sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list

# 2. 安装基本构建工具和系统依赖
# build-essential: 编译器等 (包含 gcc/g++)
# cmake, ninja-build: 构建工具
# python3-pip: 用于安装 aqtinstall
# libgl1-mesa-dev, libvulkan-dev: Qt Quick 需要的图形库
# libxkbcommon-dev, libdbus-1-3: Qt 运行时的依赖
# libxcb-*: Qt 在 Linux 上运行所需的 XCB 插件依赖
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    ninja-build \
    python3-pip \
    libglib2.0-0 \
    libgl1-mesa-dev \
    libvulkan-dev \
    libxkbcommon-dev \
    libxkbcommon-x11-0 \
    libdbus-1-3 \
    libxcb-cursor0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-sync1 \
    libxcb-xfixes0 \
    libxcb-xinerama0 \
    libxcb-xkb1 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade aqtinstall -i https://mirrors.aliyun.com/pypi/simple/

ARG QT_VERSION=6.10.1
ARG QT_HOST=linux
ARG QT_TARGET=desktop
ARG QT_AQT_ARCH=linux_gcc_64
ARG QT_DIR_ARCH=gcc_64
RUN aqt install-qt ${QT_HOST} ${QT_TARGET} ${QT_VERSION} ${QT_AQT_ARCH} \
    --outputdir /opt/Qt \
    -b https://download.qt.io

ENV PATH=/opt/Qt/${QT_VERSION}/${QT_DIR_ARCH}/bin:$PATH
ENV Qt6_DIR=/opt/Qt/${QT_VERSION}/${QT_DIR_ARCH}/lib/cmake/Qt6
ENV QT_PLUGIN_PATH=/opt/Qt/${QT_VERSION}/${QT_DIR_ARCH}/plugins
ENV QML2_IMPORT_PATH=/opt/Qt/${QT_VERSION}/${QT_DIR_ARCH}/qml
ENV LD_LIBRARY_PATH=/opt/Qt/${QT_VERSION}/${QT_DIR_ARCH}/lib:$LD_LIBRARY_PATH

# 3. 设置工作目录
WORKDIR /app

# 4. 复制项目源码到容器中
COPY . /app

# 5. 构建项目
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cmake -S /app -B /app/build/docker-release -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH=/opt/ros/humble \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      -DQt6_DIR=/opt/Qt/${QT_VERSION}/${QT_DIR_ARCH}/lib/cmake/Qt6 && \
    cmake --build /app/build/docker-release"

# 6. 设置容器入口点
# entrypoint.sh 用于在容器启动时加载 ROS 2 环境
COPY script/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# 默认运行构建好的程序
CMD ["./build/docker-release/appqml_test"]

```

同时创建一个 entrypoint.sh 脚本：

```
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash

# 根据传入参数决定启动谁
if [ "$1" = "gui" ]; then
    # 启动上位机 (Qt)
    exec ros2 run qt_hmi app_executable
elif [ "$1" = "control" ]; then
    # 启动下位机 (ROS2)
    exec ros2 launch robot_control start_robot.launch.py
else
    # 默认行为
    exec "$@"
fi
```

构建镜像命令（在开发机执行）：

```
docker build -t robot_system:v1.0 .
```

导出镜像为文件：

```
docker save -o robot_system_v1.0.tar robot_system:v1.0
```

阶段三：部署 (Deployment)
拿着这个 robot_system_v1.0.tar 文件（几 GB 大小）去 UOS 机器。

1. 导入镜像：

``` 
sudo docker load -i robot_system_v1.0.tar
```

2. 启动下位机 (控制程序)： 通常下位机不需要界面，后台运行即可。

--net=host 是 ROS2 通信的关键，让容器共用宿主机网络

允许 Docker 访问 X11 服务   

   ```
xhost +local:root
   ```

```
sudo docker run -it --rm \
       --name robot_gui \
       --net=host \
       --privileged \
       -e DISPLAY=$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       robot_system:v1.0 gui
       
# 1. 允许 Docker 连接本地 X Server
xhost +local:docker

# 2. 运行容器（带图形界面支持）
sudo docker run -it --rm \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ros2_qt6_test
```

3. 启动上位机 (Qt 界面)： 这是最麻烦的一步，因为 Docker 本身没有屏幕，它需要把画面“投射”到 UOS 的桌面上

**总结核心要点:**

1. 网络互通 ：使用 --net=host 参数。这样 Docker 里的上位机和下位机就像运行在同一台机器的两个进程一样，ROS 2 的 DDS 通信会自动发现对方，完全不需要配置 IP。
2. 图形界面 ：通过 -v /tmp/.X11-unix:/tmp/.X11-unix 和 -e DISPLAY 把 UOS 的显示接口借给 Docker 用。
3. 硬件访问 ：使用 --privileged 参数，确保下位机能访问 USB 转串口、CAN 卡或网口。
4. 开发分离 ：你依然在最舒服的 Ubuntu 22.04 上写代码，UOS 只是一个负责运行 Docker 的“载体”。
   这套流程是目前机器人行业交付标准做法，既保证了开发效率，又解决了现场部署的依赖地狱问题。