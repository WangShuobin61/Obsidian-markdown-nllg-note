# 深度相机接入技术报告

**项目**：sx_cleansilt_qt  
**目标设备**：Intel RealSense™ D435  
**报告时间**：2026-08-14  
**状态**：架构完成，待现场设备验证

---

## 一、背景与目标

项目需要在主界面展示深度相机的 RGB 视频流。正式设备为 **Intel RealSense™ D435**，需前往现场才能接触到实机。为在开发阶段尽早完成架构验证，使用了手头可用的两台代替设备进行测试：

- **Orbbec Astra S**（USB ID `2bc5:0402`）—— 支持 OpenNI2 SDK
- **ASUS Xtion Pro Live / PrimeSense**（USB ID `1d27:0601`）—— 支持 OpenNI2 SDK

由于这两台设备都依赖厂商自有 SDK（OpenNI2），测试价值有限，且在开发主机上引发了严重的 USB 音频兼容问题（详见第二章）。综合评估后，决定移除测试设备的全部业务逻辑，架构直接对准 D435（librealsense2）。

---

## 二、USB 音频兼容性问题（已解决）

### 2.1 现象

Ubuntu 22.04 开机后接入两台 USB 深度相机，GNOME 桌面出现卡顿：文件管理器（Nautilus）和终端无响应，持续数分钟。拔掉相机后问题消失，完成 A/B 验证，确认为硬件触发。

### 2.2 根因分析

两台相机的 USB 复合接口中均暴露了音频端点。以 PrimeSense（`1d27:0601`）为例，其接口布局为：

```
1-2.2:1.0  厂商自定义接口
1-2.2:1.1  USB Audio Control  → snd-usb-audio
1-2.2:1.2  USB Audio Streaming → snd-usb-audio
```

故障链：

1. 开机时 udev 枚举设备，内核自动加载 `snd-usb-audio`
2. PulseAudio 尝试以 44.1 kHz 配置音频端点，设备拒绝
3. 内核持续报错：

```
usb 1-2.2: 2:1: cannot set freq 44100 to ep 0x84
usb 1-2.2: 2:0: usb_set_interface failed (-32)
```

4. PulseAudio 进入内核不可中断睡眠（D 状态），阻塞在 `usb_start_wait_urb`
5. GNOME 会话的应用启动、窗口管理、关闭流程全部被拖慢；Nautilus 陷入 `futex_wait_queue`，最终被 OOM killer 以退出码 137 终止

D-Bus 探测全程返回 `DBUS_RC=0`，磁盘容量正常，排除 D-Bus 整体挂死和磁盘满的可能性。

### 2.3 临时缓解措施

手动解绑 PrimeSense 音频接口：

```bash
echo '1-2.2:1.2' | sudo tee /sys/bus/usb/drivers/snd-usb-audio/unbind
echo '1-2.2:1.1' | sudo tee /sys/bus/usb/drivers/snd-usb-audio/unbind
```

### 2.4 根本解决

正式设备 D435 不暴露 USB 音频接口，此问题不复存在。测试设备已从项目中移除，无需制作 udev 规则。

---

## 三、架构设计

### 3.1 分层结构

```
QML 层（UI 状态机）
  └── UsbCameraView.qml          视图容器，四态状态机
        └── DepthCameraSink      QML_ELEMENT，桥接 C++ 与 QML
              └── CameraBackend  抽象接口（纯虚）
                    └── RealSenseCameraBackend   D435 后端实现
```

各层职责严格隔离：

| 层级 | 职责 |
|------|------|
| `UsbCameraView.qml` | 三态 UI（Loading/Success/Error）、控制栏交互、截图 |
| `DepthCameraSink` | QML 元素暴露，`videoSink` 注入，信号转发 |
| `CameraBackend` | 抽象接口：`play()`、`stop()`、`setVideoSink()`、`playingChanged`、`errorOccurred` |
| `RealSenseCameraBackend` | librealsense2 取流，独立 `QThread`，RGB8→RGBX8888 帧转换 |

### 3.2 状态机

```
idle
  ↓ 点击播放
connecting（Loading 态）
  ↓ 首帧到达（playingChanged 信号）
playing（Success 态）
  ↓ 点击暂停 / 设备断开 / EOS
idle / failed（Error 态，可点击重连）
```

### 3.3 帧流转路径

```
rs2::pipeline (独立 QThread)
  → rs2::video_frame (RGB8, 1280×720@30fps)
  → 手动 RGB8→RGBX8888 拷贝（每帧逐像素，填充 Alpha=0xFF）
  → QVideoFrame
  → QVideoSink::setVideoFrame()   （Qt 内部保证跨线程安全）
  → VideoOutput（QML）
```

### 3.4 设备标识约定（deviceUri）

| 值 | 语义 |
|----|------|
| `""` / `"0"` | 枚举第 0 台设备（默认） |
| `"1"` | 枚举第 1 台设备 |
| 序列号字符串 | 直接按序列号打开（多设备时推荐，确定性强） |

---

## 四、文件结构

### 4.1 相机功能模块

```
src/features/camera/
├── CMakeLists.txt                  按需链接 librealsense2（find_package QUIET）
├── UsbCameraView.qml               D435 RGB 视图，四态状态机
└── viewmodels/
    ├── camerabackend.h             抽象接口
    ├── depthcamerasink.h / .cpp    QML_ELEMENT 桥接层
    ├── realsensecamerabackend.h / .cpp   D435 后端
    └── mediastorage.h / .cpp       截图路径管理
```

### 4.2 UI 集成

```
src/ui/mainwindow/
├── MainWinFunction.qml             主标签页容器（含 D435 相机标签页）
└── tabs/
    ├── Camera1Tab.qml              D435 相机标签页（device: "0"）
    ├── DatabaseTab.qml
    ├── DemoTab1.qml
    └── DemoTab2.qml
```

### 4.3 已删除文件（测试设备）

```
src/features/camera/viewmodels/
├── openni2camerabackend.h          Orbbec / Xtion 后端（已删除）
└── openni2camerabackend.cpp        Orbbec / Xtion 后端（已删除）

src/ui/mainwindow/tabs/
└── Camera2Tab.qml                  ASUS Xtion 标签页（已删除）
```

---

## 五、构建配置

```cmake
# src/features/camera/CMakeLists.txt（关键部分）
find_package(realsense2 QUIET)

if(realsense2_FOUND)
    # 添加 realsensecamerabackend.h/cpp，定义 HAVE_REALSENSE
    target_link_libraries(camera PRIVATE realsense2::realsense2)
    target_compile_definitions(camera PRIVATE HAVE_REALSENSE)
else()
    # 编译通过，play() 发出 errorOccurred 而非崩溃
    message(WARNING "[camera] librealsense2 未找到，现场设备接入前无需安装")
endif()
```

**设计意图**：开发机无需安装 librealsense2 也能正常编译，`DepthCameraSink::play()` 在 `HAVE_REALSENSE` 未定义时直接发出 `errorOccurred`，UI 呈现 Error 态。现场安装 SDK 后重新构建即可激活。

---

## 六、现场接入指南（D435）

### 6.1 安装 librealsense2

在现场主机执行（需要网络或提前下载）：

```bash
# Ubuntu 22.04
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCD
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev
```

### 6.2 重新构建

```bash
cd /home/nllg/my_project/sx_cleansilt_qt
cmake --preset gcc_debug
cmake --build build/GCC_Qt_6_10_3-Debug
```

构建日志应出现：`[camera] librealsense2 后端已启用`

### 6.3 验证连接

```bash
# 确认设备识别
rs-enumerate-devices

# 独立测试取流
rs-color
```

### 6.4 运行应用

```bash
./build/GCC_Qt_6_10_3-Debug/app_cleansilt
```

进入主界面 → 点击「D435 相机」标签页 → 点击「播放」。

### 6.5 多台 D435（未来扩展）

当现场需要接入多台 D435 时，推荐按序列号指定：

```qml
// Camera1Tab.qml
UsbCameraView { device: "123456789012" }

// Camera2Tab.qml（届时重建）
UsbCameraView { device: "987654321098" }
```

序列号可通过 `rs-enumerate-devices` 获取。

---

## 七、已知限制

| 限制 | 说明 |
|------|------|
| 仅 RGB 流 | 深度流未接入；D435 深度数据可后续通过 `RS2_STREAM_DEPTH` 扩展 |
| 单一分辨率 | 固定 1280×720@30fps，不支持运行时切换 |
| 无录制功能 | 可在 pipeline 中增加 `tee` 分流实现 |
| 无帧率监控 | 可添加帧计数器和统计显示 |

---

## 八、开发阶段总结

### 过程回顾

1. **阶段一**：调研方案，使用 GStreamer + V4L2 对普通 USB 摄像头取流，验证 Qt Multimedia 帧推送链路
2. **阶段二**：适配 OpenNI2 SDK，接入 Orbbec Astra S 和 ASUS Xtion，调试 RGB 帧推送
3. **阶段三**：发现 USB 音频兼容性导致 GNOME 卡顿，定位根因，验证缓解措施
4. **阶段四**：评估测试价值，决定放弃 OpenNI2 路径；重构为 CameraBackend 抽象接口 + RealSense 实现；清理全部测试代码
5. **阶段五（当前）**：架构锁定，等待现场验证

### 结论

测试设备（Orbbec / Xtion）的测试不具备对 D435 的迁移价值，且引入了额外的环境噪音（USB 音频干扰）。正确做法是在架构层面直接瞄准目标设备，以 CMake `QUIET` 查找 + 编译时条件开关的方式管理 SDK 可用性，避免强依赖阻碍开发机构建。