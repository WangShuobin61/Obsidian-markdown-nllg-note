# RK3588 ARM64 Docker 挂载式开发与部署

## 1. 采用的边界

新方案把内容分成两类：

| 位于环境镜像 | 位于 RK3588 宿主机并挂载 |
| --- | --- |
| Ubuntu 22.04 用户空间 | Qt 应用源码 |
| ROS 2 Humble 与二进制依赖 | ROS 自研工作区源码 |
| ARM64 Qt 6.10.3 SDK 与 FluentUI | CMake/colcon 编译产物 |
| GCC、Clang、CMake、Ninja、GStreamer | 数据库、截图、地图和用户配置 |
| X11、字体、软件渲染运行库 | 设备节点（按需透传，不是文件挂载） |

因此修改源码不需要重建镜像。镜像只在系统依赖、ROS 依赖、Qt SDK 或 FluentUI
变化时重建。可执行文件虽然保存在宿主机，仍要从容器内启动，因为它动态依赖
镜像中的 Qt、ROS 和系统库。

新方案由 `docker/run.sh` 临时生成隔离的 Docker build context，项目源码不会进入
context，也不会出现在任何镜像层中。根目录旧 `Dockerfile` 暂时保留作历史对照。

## 2. ARM64 是硬边界

当前 RK3588/银河麒麟机器的 `uname -m` 是 `aarch64`。x86 开发机上生成的：

- Qt `gcc_64` 目录；
- FluentUI 的 `.so`；
- Qt 应用和 ROS C++ 节点；
- x86 Docker 镜像；

都不能在这里直接运行。推荐在 RK3588 上用 ARM64 容器原生编译。这样 CMake 与
colcon 的产物自然落在 ARM 宿主机挂载目录，省去 sysroot、host Qt tools 和两套
工具链的维护。

## 3. Qt 与 FluentUI 在镜像构建阶段生成

不要复用 x86 在线安装器生成的 `gcc_64`。`docker/Dockerfile` 会在 ARM64 的
Ubuntu 22.04/ROS Humble 构建阶段从 Qt 官方源码依次编译以下 Qt 6.10.3 模块：

- `qtbase`；
- `qtshadertools`、`qtdeclarative`；
- `qtsvg`、`qt5compat`；
- `qtwebsockets`、`qtmultimedia`；
- `qttools`（项目的 LinguistTools 所需）。

Qt 安装到 `/opt/qt/6.10.3`。随后 Docker 构建会解压项目相邻目录中的
`FluentUI.zip`，按《FluentUI 构建与安装到 Qt 目录指南》的做法，先输出到普通
构建目录，再把完整 `FluentUI/` QML 模块复制到 Qt 的 `qml/` 目录。

已检查该压缩包：它是 FluentUI 1.7.7，Git 提交为
`7e33a2f672d18239ef49ab960075b1137a1d18e7`，并包含 Qt 6 所需的源码和资源。
Dockerfile 还会校验其 SHA-256
`270fa7bdf613c1fd18e60241fd5b015d35be063f22d83f450fddf0dfc6474727`。
最终 `environment` stage 只复制安装后的 Qt SDK 和 FluentUI，不包含它们的源码、
压缩包或构建树。

## 4. 构建环境镜像

```bash
cd /home/kylin/Desktop/app_sourc/gas_control_bringup
bash docker/run.sh image
```

脚本默认读取：

```text
/home/kylin/Desktop/app_sourc/FluentUI.zip
```

若位置变化，可设置 `FLUENTUI_ARCHIVE=/absolute/path/FluentUI.zip`。脚本会建立一个
临时 build context，其中只有 Dockerfile、三个辅助脚本和 FluentUI 压缩包；Qt
应用源码不会发送给 Docker daemon。

生成镜像：`gas-control-env:humble-qt6-arm64`。首次构建需要下载和原生编译 Qt，
耗时较长；`QT_BUILD_JOBS` 默认是 4，可按内存情况调整：

```bash
bash docker/run.sh image --build-arg QT_BUILD_JOBS=2
```

各 Qt 模块独立成层，中途失败后重试可以复用已经完成的模块。最终会检查 Qt
版本、Qt Core/FluentUI ELF 架构，以及 XCB 和 FluentUI 的动态库缺失项。

`qtbase` 使用独立且稳定的构建脚本；其余模块的脚本在 `qtbase` 层完成后才复制进
构建阶段。这样即使后续模块的配置参数需要修订，也不会使耗时的 `qtbase` 缓存
失效。只有完整构建成功后，Docker 才会把
`gas-control-env:humble-qt6-arm64` 标签赋给最终镜像；构建中途失败时看到的
`<none>:<none>` 是可供续建的中间镜像层，并非已经完成的环境镜像。

## 5. 编译和调试

当前 Qt 上位机源码不要求挂载完整 ROS 业务工作区。需要开发中间件业务时，再指定
任意宿主机 ROS 2 工作区：

```bash
export ROS_WS_HOST=/absolute/path/to/sx_gas_control_ws
```

先检查环境和挂载：

```bash
bash docker/run.sh check
```

编译该 ROS 工作区（产物写入宿主机工作区的 `build/install/log`）：

```bash
bash docker/run.sh ros-build
```

如果要把参数原样交给 colcon，例如只编译选定包：

```bash
bash docker/run.sh ros-build --packages-up-to sx_gas_bringup sx_gas_qt_compat
```

编译 Qt 应用（产物写入本项目宿主机 `build/`）：

```bash
bash docker/run.sh build debug
bash docker/run.sh build release
```

进入交互式容器：

```bash
bash docker/run.sh shell
```

`CMakePresets.json` 新增了 `arm64_container_debug` 和 `arm64_container_rel`，输出到
独立的 `build/ARM64_*` 目录；原 x86 preset 及其 `/opt/Qt/.../gcc_64` 路径保持
不变，因此不会把两种架构的 CMake cache 和目标文件混在一起。

## 6. 运行 GUI

```bash
bash docker/run.sh run debug
# 或
bash docker/run.sh run release
```

脚本默认挂载宿主机 `~/.Xauthority` 和 X11 socket，不需要先扩大 X11 访问范围。
如果宿主机没有 Xauthority，脚本会提示再执行：

```bash
xhost +local:docker
```

默认用 Mesa 软件 OpenGL，先保证不同 RK3588 BSP/国产系统上的可用性：

```bash
GAS_RENDER_MODE=software bash docker/run.sh run release
```

这里的 `software` 只设置 `LIBGL_ALWAYS_SOFTWARE=1`，仍使用 Qt Quick 默认的 OpenGL
场景图后端。不要设置 `QT_QUICK_BACKEND=software`：FluentUI 的模糊、遮罩等
`ShaderEffect` 在 Qt Quick 纯软件 adaptation 下无法正常绘制，典型现象就是 X11
已经创建并聚焦了全屏窗口、任务栏也有图标，但窗口内容完全透明。

确认板卡存在 `/dev/dri/*` 或 `/dev/mali*` 且容器内用户态 GPU 驱动兼容后，再试：

```bash
GAS_RENDER_MODE=gpu bash docker/run.sh run release
```

RK3588 的 Mali 内核驱动与用户态 EGL/OpenGL 库必须匹配；仅添加 `--device` 不保证
硬件加速一定可用。应先以软件渲染验收功能，再单独调优视频/GPU。

## 7. ROS DDS 与硬件权限

启动脚本使用 `--network host`，避免 ROS 2 DDS 发现和组播被 Docker bridge/NAT
隔离。串口仅在设备存在时透传，默认是：

```text
/dev/ttyCH343USB0
```

设备名不同时：

```bash
MCU_DEVICE=/dev/ttyUSB0 bash docker/run.sh run release
```

脚本会把这个宿主机设备映射成容器内配置固定使用的 `/dev/ttyCH343USB0`，并加入
设备 GID，不使用 `--privileged`。持久化目录默认是
宿主机 `~/.local/share/GasControl`，在容器内固定映射到
`/home/gas/.local/share/GasControl`。容器以宿主机 UID/GID 运行，因此构建产物和
数据库不会变成 root 所有。镜像构建时会以执行构建的宿主用户 UID/GID 创建
`gas` 用户，运行时同时设置 `USER=gas` 和 `LOGNAME=gas`；这样既保留宿主机文件
属主，又不会在交互 shell 中出现 `I have no name!`。

设置 `ROS_WS_HOST` 后，工作区固定映射到容器 `/workspace/ros`，入口会自动加载
已有的 `install/setup.bash`。未设置时不创建此挂载，Qt 程序仍可按“仅界面模式”
运行。容器版配置位于 `docker/config/ros_profile.json`，程序通过
`SX_GAS_QT_CONFIG` 显式加载它，不再依赖原开发机的 `/mnt/machine3/...` 路径。

## 8. 后续生产运行建议

调试期先使用 `docker/run.sh`。稳定后可增加 systemd 服务，核心命令仍应保持：

- 源码或 Release 产物目录 bind mount；
- 可选的 ROS 工作区或其 `install/` bind mount；
- 数据目录 bind mount；
- `--network host`；
- X11 cookie/socket；
- 精确的串口和 GPU 设备透传；
- 固定镜像 tag 或 digest。

生产环境若不希望部署源码，可以只把宿主机 Release 构建目录挂载为只读。它仍然
满足“程序产物在宿主机、镜像只保存环境”的边界。

## 9. RK3588 实机验证记录（2026-09-15）

首次构建在完成 `qtbase` 后配置 `qtshadertools` 时失败，原因为
`qt-configure-module` 的 CMake 参数前缺少 `--` 分隔符。现已将 QtBase 与附加模块
拆分为两个脚本并修正参数；重试时成功复用 QtBase 中间层，没有重新编译 QtBase。

最终环境镜像：

```text
gas-control-env:humble-qt6-arm64
架构：linux/arm64
Qt：6.10.3
ROS：humble
```

`bash docker/run.sh check` 已确认 Qt Core 和 FluentUI 插件均为 ARM aarch64，源码
挂载正常且动态库无缺失。`bash docker/run.sh build debug` 已在容器中成功编译，产物
保存在 RK3588 宿主机：

```text
build/ARM64_Qt_6_10_3-Debug/app_gas_control_bringup
```

该文件及构建目录归宿主用户 `kylin:kylin` 所有，证明挂载式构建不会生成 root
所有的产物。短时 GUI 启动测试已通过 Qt、FluentUI、QML 和 ROS 初始化；未设置
`ROS_WS_HOST` 时显示 ROS overlay 未就绪属于预期状态。

业务工程中的静态 `core` 库会被链接进共享 QML 模块；ARM64 链接器要求它使用
位置无关代码，因此 `src/core/CMakeLists.txt` 已为该目标启用
`POSITION_INDEPENDENT_CODE`。

首次实机 GUI 运行曾出现“任务栏有图标、全屏窗口已聚焦但画面透明”。通过
`wmctrl`/`xprop` 确认窗口几何为 `1920×1080`，根因是旧启动脚本注入了
`QT_QUICK_BACKEND=software`。现已改为 Mesa 软件 OpenGL，既可由 CPU 渲染，也
保留 FluentUI 所需的着色器能力。最终镜像同时补充了与宿主 UID/GID 对应的
`gas` 用户记录。
