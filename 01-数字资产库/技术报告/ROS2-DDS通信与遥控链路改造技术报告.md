# 任务 2+3：ROS 2 DDS 通信与遥控链路改造技术报告

> 项目：`gas_control_bringup`  
> 技术栈：Qt 6.10.3 / QML / FluentUI / ROS 2 Humble / rclcpp / rclpy  
> 完成日期：2026-07-24  
> 状态：已实施并通过 `clang_debug` 增量编译验证

---

## 1. 改造目标与最终结果

任务 2 与任务 3 合并实施，原因是两者共同构成上位机与下位机之间的双向控制闭环：

- 任务 2：下位机状态与里程通过 ROS 2 DDS 回传到 Qt 上位机。
- 任务 3：上位机底盘摇杆和云台控制通过 ROS 2 DDS 下发到下位机。

改造前存在两条 UDP 链路：

- `StateManager` 监听 UDP `45454`，接收状态 JSON。
- `JoystickBridge` 向 UDP `45455` 发送底盘和云台合并 JSON。

改造后统一为四条 ROS 2 话题：

- `/robot_state`：`std_msgs/msg/String`，载荷为 JSON，负责电池、网络、环境和气体浓度。
- `/odom`：`nav_msgs/msg/Odometry`，负责位置、累计里程和姿态。
- `/cmd_vel`：`geometry_msgs/msg/Twist`，负责底盘线速度和角速度。
- `/gimbal_cmd`：`std_msgs/msg/Float32`，负责云台 0～1 归一化位置。

最终完成内容：

1. Qt 主程序接入 `rclcpp`，ROS executor 在独立线程运行，不阻塞 Qt 主线程。
2. 删除状态接收 UDP 逻辑，保留并复用原 JSON 业务解析。
3. 接入 `/odom`，累计里程并将四元数转换为 roll/pitch/yaw。
4. 打通按时间和按距离两种气体采样模式。
5. 删除 `JoystickBridge` 和 UDP `45455` 出站链路。
6. 底盘导航球直接换算并发布 `Twist`。
7. 云台由悬浮摇杆改为小车设置卡片中的滑块，保存时持久化并发布。
8. 两个 Python 测试脚本由 socket 改为 rclpy 节点。
9. 完整 QML 缓存、C++ ViewModel 和主程序链接均已通过编译。

---

## 2. 实施过程与关键决策

### 2.1 接口约定确认

实施前先固定了两端通信契约：

- 状态编码采用 `std_msgs/String + JSON`，避免引入自定义 rosidl 消息包。
- 话题名采用 `/robot_state`、`/odom`、`/cmd_vel`、`/gimbal_cmd`。
- 云台量纲采用 0～1 归一化值。
- 底盘最大角速度默认采用 `2.0 rad/s`。
- `/robot_state` 保持原 `battery / signal / environment / gasConcentrations` 字段结构。

这使状态业务模型与 ROS 传输层解耦：后续即使更换状态消息类型，只需调整 `StateSubscriber`，无需修改 `StateManager` 以上的 UI 和业务代码。

### 2.2 构建层接线

根 `CMakeLists.txt` 可选查找以下 ROS 2 包：

```cmake
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(nav_msgs QUIET)
```

`src/viewmodels/CMakeLists.txt` 仅在依赖全部存在时链接 ROS 2，并定义 `HAVE_ROS=1`。因此同一套源码支持两种构建：

- 有 ROS 环境：启用 DDS 收发。
- 无 ROS 环境：ROS 适配器退化为空壳，Qt UI 仍可独立编译运行。

这种设计把 ROS 依赖收敛在 `viewmodels/rosVM`，没有扩散到 QML、数据库和其它业务模块。

### 2.3 ROS 生命周期接入

`main.cpp` 在加载 QML 引擎前创建并启动 `RosContext`：

```text
QGuiApplication 创建
    → 单实例检查
    → RosContext.start(argc, argv, "gas_control_bringup")
    → QQmlApplicationEngine 加载
    → app.exec()
    → main 返回，RosContext 析构并 stop()
```

在 QML 引擎前启动的原因是：`StateManager`、`CmdVelPublisher`、`GimbalPublisher` 都是 QML 单例，它们构造时必须能从 `RosContext` 获得已经可用的 ROS node。

### 2.4 传输层替换顺序

实施采用窄链路逐步替换：

1. 先接通 CMake 与 `RosContext`，确认 Qt + rclcpp 能编译和链接。
2. 新增 `StateSubscriber`，复用 `StateManager::updateFromJson()`，替换 UDP 状态接收。
3. 新增 `OdomSubscriber` 与 `StateManager` 里程/姿态属性。
4. 在 QML 中接通距离采样触发。
5. 新增 `CmdVelPublisher`，替换 `JoystickBridge`。
6. 新增 `GimbalPublisher`，移除悬浮云台球并改为设置滑块。
7. 最后改造 rclpy 测试脚本并整体编译。

这种顺序保证每一步都有独立的输入、输出和验证点，避免一次同时修改 UI、网络、状态和数据库而无法定位问题。

---

## 3. 最终架构与数据流

```mermaid
flowchart LR
    subgraph ROS_SIDE[下位机 / ROS 2 侧]
        StatePub[/robot_state\nString JSON]
        OdomPub[/odom\nOdometry]
        CmdSub[/cmd_vel\nTwist]
        GimbalSub[/gimbal_cmd\nFloat32]
    end

    subgraph QT_SIDE[Qt6 上位机]
        RosContext[RosContext\nnode + executor 线程]
        StateSubscriber[StateSubscriber]
        OdomSubscriber[OdomSubscriber]
        StateManager[StateManager]
        NavigationBall[NavigationBall.qml]
        CmdVelPublisher[CmdVelPublisher]
        CarSettings[CarSettingsCard.qml]
        GimbalPublisher[GimbalPublisher]
        BasicControl[BasicControlCard.qml]
        GasRecordManager[GasRecordManager]
    end

    StatePub --> StateSubscriber -->|QueuedConnection| StateManager
    OdomPub --> OdomSubscriber -->|QueuedConnection| StateManager
    NavigationBall --> CmdVelPublisher --> CmdSub
    CarSettings --> GimbalPublisher --> GimbalSub
    StateManager --> BasicControl --> GasRecordManager
    RosContext --> StateSubscriber
    RosContext --> OdomSubscriber
    RosContext --> CmdVelPublisher
    RosContext --> GimbalPublisher
```

### 3.1 线程边界

ROS 订阅回调运行在 `RosContext` 的 executor 线程。回调中禁止直接修改 QML 可见对象：

```text
ROS 回调线程
    → StateSubscriber.emit jsonReceived(QByteArray)
    → OdomSubscriber.emit odometryUpdated(纯数值)
    → Qt::QueuedConnection
    → Qt 主线程 StateManager 更新属性
    → QML 属性绑定刷新
```

`StateSubscriber` 和 `OdomSubscriber` 的头文件通过 pImpl 隐藏 ROS 类型，`rclcpp` 和消息 include 只存在于 `.cpp`，保证依赖边界清晰。

### 3.2 模块职责

- `RosContext`：全局唯一 ROS node、executor 和 spin 线程；负责启动、停止和无 ROS 构建兜底。
- `StateSubscriber`：订阅 `/robot_state`，将 `String.data` 转为 UTF-8 `QByteArray`，不处理业务字段。
- `OdomSubscriber`：订阅 `/odom`，完成里程累计和四元数转欧拉角，输出纯数值。
- `StateManager`：状态业务门面；解析 JSON、维护电池/网络/环境/气体/里程/位置/姿态属性。
- `CmdVelPublisher`：把导航球的角度和力度转换为 `Twist` 并发布。
- `GimbalPublisher`：把 UI 的 0～100 云台值夹紧并归一化为 0～1 后发布。
- `NavigationBall.qml`：只负责触摸输入、几何计算和调用底盘发布器。
- `BasicControlCard.qml`：负责采样任务按钮、时间触发和距离触发。
- `CarSettingsCard.qml`：负责小车参数草稿、取消、保存、INI 持久化和云台命令提交。
- `RightPanelPreferences.qml`：右侧控制参数的单一持久化来源，写入 INI 的 `[rightPanel]` 分类。
- `GasRecordManager`：气体记录任务状态机和数据库写入，不主动读取 QML 偏好。

---

## 4. ROS 2 话题契约

### `/robot_state`

- 方向：下位机 → 上位机。
- 类型：`std_msgs/msg/String`。
- QoS：`rclcpp::QoS(10)`。
- 内容：UTF-8 JSON。
- 当前兼容字段：

```json
{
  "battery": { "level": 88, "status": "discharging" },
  "signal": { "type": "wifi", "wifiStrength": 3 },
  "environment": { "temperature": 25.0, "humidity": 50.0 },
  "gasConcentrations": { "co": 0.0, "o2": 20.9 }
}
```

`StateManager` 仍兼容部分旧平铺字段及 `gases / concentrations` 备用气体字段名。

### `/odom`

- 方向：下位机 → 上位机。
- 类型：`nav_msgs/msg/Odometry`。
- QoS：`rclcpp::QoS(50)`。
- 使用字段：`pose.pose.position.x/y` 和 `pose.pose.orientation`。
- 输出结果：累计里程、`posX`、`posY`、roll、pitch、yaw。

### `/cmd_vel`

- 方向：上位机 → 下位机。
- 类型：`geometry_msgs/msg/Twist`。
- QoS：`rclcpp::QoS(10)`。
- 当前使用字段：`linear.x`、`angular.z`。

### `/gimbal_cmd`

- 方向：上位机 → 下位机。
- 类型：`std_msgs/msg/Float32`。
- QoS：`rclcpp::QoS(10)`。
- 量纲：0～1 归一化值。


## 5. 关键算法与业务逻辑

### 5.1 里程累计

`OdomSubscriber` 保存上一帧位置，首帧只建立基准，不把“初始位置到原点”的距离计入里程。后续每帧执行：

```text
dx = currentX - lastX
dy = currentY - lastY
odometer += hypot(dx, dy)
```

该里程表示节点启动后的二维轨迹累计长度，不是 `Odometry` 消息中某个直接字段。

当前实现特性：

- 只累计 X/Y 平面距离，不计 Z 轴。
- 节点重启后从 0 重新累计。
- 未对定位跳变做限幅；若 `/odom` 突然跳点，里程会把跳变量计入。
- 没有根据时间戳过滤乱序或重复消息。

### 5.2 四元数转姿态角

`OdomSubscriber` 使用 ZYX 顺序将四元数转换为 roll/pitch/yaw，并转换为角度：

```text
roll  = atan2(2(wx + yz), 1 - 2(x² + y²))
pitch = asin(clamp(2(wy - zx), -1, 1))
yaw   = atan2(2(wz + xy), 1 - 2(y² + z²))
```

`pitch` 输入先夹紧到 `[-1, 1]`，避免浮点误差导致 `asin()` 返回 NaN。三姿态角只用于实时展示，不写入数据库。

### 5.3 导航球到 Twist 的换算

导航球约定：

- `angle = 0°`：前方。
- 顺时针递增：`90°` 为右，`180°` 为后，`270°` 为左。
- `magnitude`：0～100。

发布器执行：

```text
speed     = clamp(magnitude, 0, 100) / 100
angleRad  = angle × π / 180
forward   = cos(angleRad)
turn      = sin(angleRad)
linear.x  = forward × speed × maxLinear
angular.z = -turn × speed × maxAngular
```

负号来自 REP-103 右手坐标系：从上往下看，右转对应负 `angular.z`。

松开导航球时调用：

```text
CmdVelPublisher.updateChassis(0, 0, false)
```

`active=false` 会直接发布全 0 `Twist`，实现回中停车，不依赖最后一次角度。

### 5.4 云台归一化与提交

云台 UI 使用 0～100 的整数滑块。保存时：

```text
CarSettingsCard.save()
    → RightPanelPreferences.setGimbalPosition(value)
    → QML Settings 写入 INI
    → GimbalPublisher.setValue(value)
    → clamp(value, 0, 100) / 100
    → 发布 Float32 到 /gimbal_cmd
```

云台采用“草稿 + 保存”而不是拖动即发布：

- 拖动滑块：只改变页面草稿。
- 点击“保存”：持久化并发布。
- 点击“取消更改”：从已保存值恢复，不发布。
- 点击全局重置：偏好恢复为 0%，卡片收到 `resetCompleted` 后重载草稿。

这种模式适合参数型推杆。如果未来需要连续实时操纵，应改回 `onMoved` 发布，不能同时保留“保存才生效”的语义。

### 5.5 状态 JSON 解析

`StateSubscriber` 不解析字段，只复制消息载荷。`StateManager::updateFromJson()` 负责：

1. 验证根节点是 JSON object。
2. 解析 `battery`、`signal`、`environment`。
3. 解析 `gasConcentrations`，并兼容 `gases`、`concentrations`。
4. 更新对应状态对象并发出 Qt 通知信号。
5. 电池、网络、环境写入 `QSettings` 快照；气体浓度保持实时态。

因此 ROS 传输和状态业务保持了明确分层：订阅器只负责“搬运”，`StateManager` 负责“解释”。

---

## 6. 气体记录与距离触发

### 6.1 任务状态机

`GasRecordManager` 持有以下状态：

```text
idle:    currentTaskId = 0, sampling = false
running: currentTaskId > 0, sampling = true
paused:  currentTaskId > 0, sampling = false
```

- `beginTask()`：无任务时新建；暂停态时恢复同一任务。
- `pauseTask()`：保留任务 ID，只停止采样。
- `endTask()`：结束数据库任务并回到 idle。
- `commitSample()`：只有 running 状态才写入。

### 6.2 按时间检测

`BasicControlCard.qml` 中的 `Timer` 仅在以下条件运行：

```text
GasRecordManager.sampling == true
detectionMode == 0
```

到点后抓取当前气体列表和当前累计里程：

```text
GasRecordManager.commitSample(
    StateManager.gasConcentrations,
    StateManager.odometer)
```

时间间隔来自 `RightPanelPreferences.detectionIntervalSec`。

### 6.3 按距离检测

开始距离模式任务时，QML 记录当前里程基准：

```text
lastSampledOdometer = StateManager.odometer
```

之后监听 `StateManager.odometryChanged`：

```text
if currentOdometer - lastSampledOdometer >= detectionIntervalM:
    commitSample(gases, currentOdometer)
    lastSampledOdometer = currentOdometer
```

触发逻辑放在 QML 而不是 C++ 的原因：

- 检测模式和间隔属于 `RightPanelPreferences`。
- 气体现值属于 `StateManager`。
- 任务状态属于 `GasRecordManager`。
- 如果 C++ 主动反查 QML 单例，会形成反向依赖和隐藏数据流。

QML 在边界处组合三方状态，C++ 模块各自保持单一职责。

### 6.4 当前数据库边界

数据库表已包含：

```text
odometer REAL NOT NULL DEFAULT 0.0
pos_x REAL
pos_y REAL
```

当前已真正写入的是 `odometer`。虽然 `StateManager.currentOdomSnapshot()` 已提供：

```text
{ odometer, pos_x, pos_y }
```

但 `GasRecordManager::commitSample()` 当前签名仍是：

```cpp
commitSample(const QVariantList &items, double odometer)
```

因此 `pos_x/pos_y` 目前保持 `NULL`。若要完整绑定采样时刻位置，后续应把接口改为接受 `QVariantMap snapshot`，并为 `GasRecord.posX/posY` 赋值。

---

## 7. 设置草稿与 INI 持久化

`RightPanelPreferences.qml` 使用 `QtCore.Settings`：

```text
category = "rightPanel"
```

实际写入应用 INI 的 `[rightPanel]` 分类。当前相关字段：

- `carSpeed`：默认 `0.5`。
- `gimbalPosition`：默认 `0.0`。
- `detectionMode`：默认 `1`，即按距离。
- `detectionIntervalSec`：默认 `5.0` 秒。
- `detectionIntervalM`：默认 `1.0` 米。
- `lowBatteryThreshold`：默认 `20`。
- `lowBatteryAction`：默认 `0`。

`CarSettingsCard.qml` 使用统一草稿模式：

```text
控件值与已保存偏好比较
    → dirty = true
    → 启用“保存”

点击保存
    → 调用各 setter
    → Settings 自动持久化
    → dirty 自动回落 false

点击取消更改
    → revert()
    → 控件恢复为 RightPanelPreferences 当前值
```

浮点参数使用 `eps = 0.001` 比较，避免 0.1 步进的二进制误差导致保存按钮长期处于脏状态。


## 8. 后续可调整参数

### 8.1 运行时可通过 UI/INI 调整

以下参数已经进入 `RightPanelPreferences`，可在小车设置卡片修改并保存：

- `carSpeed`：0～10 m/s，步进 0.1。
- `gimbalPosition`：0～100%，步进 1；保存时发布为 0～1。
- `detectionMode`：0=按时间，1=按距离。
- `detectionIntervalSec`：2～60 秒。
- `detectionIntervalM`：0.5～5.0 米，步进 0.1。
- `lowBatteryThreshold`：0～100%。
- `lowBatteryAction`：警告与返回策略。

### 8.2 C++ 当前默认参数

以下参数当前在 C++ 中定义，修改后需要重新编译：

- `CmdVelPublisher::m_maxLinear = 0.5 m/s`。
- `CmdVelPublisher::m_maxAngular = 2.0 rad/s`。
- `/cmd_vel` QoS 深度 `10`。
- `/gimbal_cmd` QoS 深度 `10`。
- `/robot_state` QoS 深度 `10`。
- `/odom` QoS 深度 `50`。
- 话题名 `/robot_state`、`/odom`、`/cmd_vel`、`/gimbal_cmd`。

### 8.3 测试脚本参数

`receive_joystick.py`：

- `--max-linear`：反算显示使用的线速度上限，默认 `0.5`。
- `--max-angular`：反算显示使用的角速度上限，默认 `2.0`。

`simulate_state_status.py`：

- `--interval`：状态循环发送间隔，默认 `2.0` 秒。
- `--odom-rate`：里程发布频率，默认 `10 Hz`。
- `--odom-step`：随机游走每步位移，默认 `0.05 m`。
- `--no-odom`：只发布状态，不发布里程。
- `--gas-drift`：气体随机浮动比例，默认 `0.02`。
- 电池、网络、温湿度和各气体值均可通过命令行参数或交互菜单修改。

---

## 9. 当前限制与建议调整

### 9.1 优先级高：把 `carSpeed` 同步到 `CmdVelPublisher.maxLinear`

当前 UI 保存 `RightPanelPreferences.carSpeed`，但没有调用：

```text
CmdVelPublisher.maxLinear = carSpeed
```

因此导航球实际换算仍使用发布器构造默认值 `0.5 m/s`。建议在 `CarSettingsCard.save()` 中同时更新：

```qml
CmdVelPublisher.maxLinear = speedSlider.value
```

或者让 `CmdVelPublisher` 在构造时从统一 C++ 配置服务读取，避免 QML 偏好和发布器运行值出现双源。

### 9.2 优先级高：位置快照完整落库

把 `GasRecordManager::commitSample(items, odometer)` 改为：

```text
commitSample(items, snapshot)
snapshot = { odometer, pos_x, pos_y }
```

然后在 `GasRecord` 中填充 `posX/posY`。数据库结构已经具备，无需迁移。

### 9.3 优先级中：话题与 QoS 参数化

当前话题名和 QoS 深度是编译期常量。建议新增 `RosCommunicationPreferences` 或配置结构，集中管理：

```text
robotStateTopic
odomTopic
cmdVelTopic
gimbalTopic
stateQosDepth
odomQosDepth
commandQosDepth
```

如果下位机里程使用 `SensorDataQoS / BEST_EFFORT`，上位机当前 `RELIABLE` 订阅可能发生 QoS 不兼容，需要联调时重点检查。

### 9.4 优先级中：里程跳变过滤

建议为 `/odom` 增加：

- 单帧最大位移阈值。
- 时间戳单调性检查。
- NaN/Inf 检查。
- 可选使用下位机直接提供的累计里程，而不是上位机二次积分。

### 9.5 优先级中：无 ROS 状态反馈

当前无 ROS 构建能够正常打开 UI，但界面没有统一显示 DDS 是否可用。建议让 `RosContext` 暴露 QML 只读属性：

```text
available
nodeName
lastError
```

并在系统消息或状态栏显示“ROS 未启用 / 节点初始化失败”。

### 9.6 优先级低：清理旧文档和冗余属性

- `doc/Technical report/joystick-implementation.md` 描述的是 UDP 双摇杆旧实现，已不符合现状，应标记为历史版本或重写。
- `NavigationBall.qml` 的 `side` 属性现在恒为 `chassis`，可以删除，进一步收敛组件接口。
- `BasicControlCard.qml` 中“遥控模式”仍启动外部 `teleop_keyboard`，与虚拟摇杆并存；应明确是备用入口还是后续删除。

---

## 10. 测试与联调

### 10.1 构建验证

已在 ROS 2 Humble 环境下完成：

```bash
source /opt/ros/humble/setup.bash
cmake --build build/Clang14_Qt_6_10_3-Debug
```

验证结果：

- `roscontext.cpp`、两个 subscriber、两个 publisher 编译成功。
- `statemanager.cpp` 编译成功。
- `NavigationBall.qml`、`BasicControlCard.qml`、`CarSettingsCard.qml` 等 QML cache 生成成功。
- `libviewmodels.so`、QML plugins 和 `app_gas_control_bringup` 链接成功。

### 10.2 Python 语法验证

```bash
python3 -m py_compile script/receive_joystick.py
python3 -m py_compile script/simulate_state_status.py
```

两个脚本均通过。

### 10.3 联调启动顺序

终端 1：启动指令接收节点。

```bash
source /opt/ros/humble/setup.bash
python3 script/receive_joystick.py --max-linear 0.5 --max-angular 2.0
```

终端 2：启动状态和里程模拟节点。

```bash
source /opt/ros/humble/setup.bash
python3 script/simulate_state_status.py --interactive --odom-rate 10 --odom-step 0.05
```

终端 3：启动上位机。

```bash
source /opt/ros/humble/setup.bash
./build/Clang14_Qt_6_10_3-Debug/app_gas_control_bringup
```

验证点：

1. 顶栏电池、网络、环境状态随 `/robot_state` 更新。
2. 气体浓度随 JSON 变化。
3. `StateManager.odometer` 持续增加，姿态角随随机 yaw 变化。
4. 时间模式按秒写入气体记录并带里程。
5. 距离模式达到设定米数后写入记录。
6. 底盘导航球拖动时接收节点输出 `linear.x/angular.z`，松手收到零速度。
7. 云台滑块拖动时不发布；点击小车设置“保存”后收到 0～1 值。
8. 重启应用后云台位置、检测模式和间隔从 INI 恢复。

---

## 11. 关键文件清单

### 构建与生命周期

- `CMakeLists.txt`：查找 ROS 2 依赖。
- `src/viewmodels/CMakeLists.txt`：可选链接、定义 `HAVE_ROS`。
- `src/app/main.cpp`：创建并启动 `RosContext`。

### ROS 适配层

- `src/viewmodels/rosVM/roscontext.h/.cpp`
- `src/viewmodels/rosVM/statesubscriber.h/.cpp`
- `src/viewmodels/rosVM/odomsubscriber.h/.cpp`
- `src/viewmodels/rosVM/cmdvelpublisher.h/.cpp`
- `src/viewmodels/rosVM/gimbalpublisher.h/.cpp`

### 状态与业务层

- `src/viewmodels/stateManageVM/statemanager.h/.cpp`
- `src/viewmodels/gasRecordVM/gasrecordmanager.h/.cpp`
- `src/viewmodels/gasRecordVM/gasrecordrepository.h/.cpp`

### QML 交互层

- `src/generalstyle/NavigationBall.qml`
- `src/generalstyle/RightPanelPreferences.qml`
- `src/rightsidebar/BasicControlCard.qml`
- `src/rightsidebar/CarSettingsCard.qml`
- `src/app/Main.qml`

### 测试工具

- `script/receive_joystick.py`
- `script/simulate_state_status.py`

---

## 12. 架构结论

本次改造的核心价值不是简单地把 UDP API 换成 ROS API，而是建立了稳定的模块边界：

```text
ROS 消息类型
    只存在于 rosVM .cpp
        ↓
Qt 信号与纯值
    进入 StateManager / QML 单例
        ↓
QML 在业务边界组合状态
        ↓
GasRecordManager / 数据库各自保持单一职责
```

入站链路以 `StateManager` 为业务门面，出站链路以两个 publisher 为命令适配器；ROS executor 与 Qt 主线程通过 queued signal 隔离。这个结构允许后续独立替换消息编码、话题名称、QoS、底盘速度模型或云台协议，而不需要把 ROS 依赖扩散到整个项目。
