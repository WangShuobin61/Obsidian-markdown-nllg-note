# Qt 作者新版本后续开发交接要求

## 1. 文档目的与开发基线

如果上位机开发者继续以以下目录中的作者新版本开发：

```text
/mnt/machine3/sx_GasControl_qt_new
```

必须把本文列出的 ROS 集成功能前向移植到新版本。作者新版本可以作为界面和业务页面的
上游基线，但不能直接覆盖当前已经可用的两个集成版本：

```text
/mnt/machine3/sx_GasControl_qt_simulation
/mnt/machine3/sx_GasControl_qt_hardware
```

正确方式是保留 `qt_new` 的新版界面，再按功能逐项移植、测试；不要整目录复制覆盖，也
不要只比较界面文件。ROS 端唯一接口契约是
[Qt/ROS 统一接口契约](qt_ros_unified_interface.md)。本文用于告诉 Qt 开发者“要改什么”，
如接口细节发生冲突，以统一接口契约和 ROS 消息定义为准。

## 2. 必须保留的作者新版界面行为

- 面向 1280×800、10.1 英寸触摸屏，程序启动后支持全屏运行。
- 左、中、右区域在小屏幕上不能互相遮挡，当前建议最小宽度为 220/680/220。
- 最大化按钮长按进入全屏，并能恢复进入全屏前的窗口状态。
- 作者新版中的“云台控制”实际表示履带车摆臂控制。用户可见中文统一改成“摆臂控制”，
  英文统一为 `Swing Arm`。
- 为减少无关改动，`GimbalPublisher`、`/gimbal_cmd` 等内部兼容名称暂时可以保留；不能
  因修改界面文字而更改 ROS 消息语义。
- 不要永久隐藏 `RvizSettingsCard`。路线编辑、地图显示和可视化入口必须继续可用。
- 保留现有登录、用户管理和控制界面进入流程。不能依赖只有某台开发机有效的隐藏快捷键；
  正常登录后必须有可见入口进入控制界面。测试账号属于部署数据，不应硬编码在源码。

## 3. 仿真版和真机版必须分开

最终必须产生两个清晰隔离的版本或构建 profile，不能在同一个界面中靠操作员临时选择
“仿真/真机”后复用危险默认值。

| 项目 | 仿真版 | 真机版 |
| --- | --- | --- |
| 当前参考实现 | `sx_GasControl_qt_simulation` | `sx_GasControl_qt_hardware` |
| ROS Domain | 120 | 30 |
| 点云 | `/lidar/points` | `/point_cloud_raw` |
| 系统启动 | `sx_gas_sim_env/corridor_robot.launch.py` | `sx_gas_bringup/hardware.launch.py` |
| 地图 | 仿真地图及过滤报告 | 只显示 `real_*.yaml` |
| 路线 | 用户仿真路线目录 | 待审目录与审批目录分离 |
| 气体/升降 | 允许明确的模拟数据 | 禁止模拟数据 |
| Gazebo | 只启动无界面 `gzserver`，不弹出 `gzclient` | 禁止启动 |
| Foxglove | Qt 启动后自动启动只读 Bridge | 由真机部署配置决定，不得隐式开放写权限 |

禁止把 domain 120、仿真世界、`use_sim_time=true`、模拟气体、模拟升降或仿真任务带入
真机版。真机版也不能把真实串口、审批任务和硬件路径带入仿真版。

## 4. 配置必须完全外置并经过模式校验

在 `src/gas_control_bringup/config/` 中保留：

```text
ros_profile.json
ros_profile.schema.json
```

程序需要使用 `RosProfileConfig` 统一读取配置，并支持环境变量
`SX_GAS_QT_CONFIG` 指定部署文件。指定文件不存在、类型错误、edition 不匹配或字段越界时，
程序应显示明确错误并以退出码 2 结束，不能回落到隐藏默认值。

这里必须区分“配置文件无效”和“ROS 运行环境暂不可用”：

- 找不到 `ros_profile.json`、JSON 格式错误、edition 不匹配、必填字段缺失、字段类型错误、
  数值越界或路径不是绝对路径，属于配置文件无效，仍应以退出码 2 结束；
- 配置结构有效，但 `ros.setup`、`ros.workspace` 或 `ros.workspace_setup` 指向的文件/目录
  当前不存在，属于 ROS 运行环境未就绪。真机版允许以“仅界面模式”启动，但所有依赖 ROS
  的控制入口必须禁用，后台调用也必须继续 fail-closed，不能尝试用隐藏默认路径启动节点。

以下内容必须外置：

- ROS setup、工作空间、`install/setup.bash`、Domain ID、rosbridge 端口；
- 地图、路线、世界、地形配置、任务和阈值路径；
- 所有 ROS launch 包名和 launch 文件名；
- 速度、里程计、IMU、雷达、点云、诊断、摆臂命令和反馈话题；
- 初始楼层、出生位姿、Foxglove 地址和端口；
- 运行探测周期、话题超时、lifecycle 探测周期；
- 摆臂控制器话题、角度范围、步进角度和反馈超时；
- 真机 STM32 串口和波特率。目前为 `/dev/ttyCH343USB0`、115200。

禁止使用“配置缺失后转换为 0”的实现。端口、角度、周期、初始楼层、位姿等关键数值
必须同时检查字段存在、JSON 类型和允许范围。

## 5. ROS 话题接口

| 方向 | 话题 | 类型 | Qt 要求 |
| --- | --- | --- | --- |
| Qt → ROS | `/cmd_vel_key` | `geometry_msgs/msg/Twist` | 人工遥控只进入速度仲裁，不得直连 `/cmd_vel_final` |
| Qt → ROS | `/gimbal_cmd` | `std_msgs/msg/Float32` | 摆臂归一化命令 `[0,1]`，不是检测仪升降柱 |
| ROS → Qt | `/odom` | `nav_msgs/msg/Odometry` | 统一显示里程计；真机中这是 EKF 融合结果 |
| ROS → Qt | `/imu/data` | `sensor_msgs/msg/Imu` | 显示 IMU 新鲜度和异常状态 |
| ROS → Qt | `/scan` | `sensor_msgs/msg/LaserScan` | 使用 SensorData QoS |
| ROS → Qt | `/lidar/points` 或 `/point_cloud_raw` | `sensor_msgs/msg/PointCloud2` | 按 edition 选择，使用 SensorData QoS |
| ROS → Qt | `/joint_states` | `sensor_msgs/msg/JointState` | 读取 `front_arm_joint` 实际角度和速度 |
| ROS → Qt | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 读取 MCU、摆臂、EKF 和其他驱动诊断 |

真机中 STM32 原始编码器数据位于 `/wheel/odom`，`robot_localization` 融合
`/wheel/odom + /imu/data` 后发布统一 `/odom`。Qt 只能订阅 `/odom`，不要重新在 Qt 内融合，
也不要启动 `odom_to_tf` 或建立 `/wheel/odom → /odom` relay。

STM32 的 `0xF0` 心跳是“STM32 → ROS”单向数据，Qt 和 ROS 都不能向 STM32回发心跳。
Qt 应读取 `/diagnostics` 中的 `mcu/serial`，至少显示：

- `serial_open`、`last_valid_frame_age_s`；
- `imu_frames`、`invalid_imu_frames`、`unknown_frames`、`errors`；
- `system_state`、`control_mode`；
- `gas_online_bitmap`、`motor_status_bitmap`、`current_mcu_error_code`。

打开串口不代表 STM32 在线；必须在超时内收到合法帧。没有收到真实气体帧时，Qt 不能把
全零浓度显示为“传感器正常”。

## 6. 虚拟摇杆和速度安全

- 摇杆保持在某一非零位置时，必须以约 10 Hz 持续发布最后速度，不能只发送一次。
- 松开、触摸取消、窗口失焦、控制页面关闭、ROS 断连和程序退出时都必须立即发布零速度。
- Qt 只能发布 `/cmd_vel_key`，由 `cmd_vel_mux` 仲裁后输出 `/cmd_vel_final`。
- 不允许建立全局 topic relay 绕过仲裁、Collision Monitor 或跨层控制权。
- 真机测试必须先验证硬件急停和人工接管；软件停止不能代替急停。

## 7. 摆臂控制必须有完整反馈和联锁

作者新版“云台控制”需要按摆臂语义接入现有 `GimbalPublisher`/适配器：

```text
Qt /gimbal_cmd [0,1]
  → swing_arm_adapter
  → /front_arm_position_controller/commands（弧度）
```

Qt 不能把“已发送命令”当作“摆臂已经到位”。界面必须基于 `/joint_states` 和
`/diagnostics` 显示：实际角度、速度、运动状态、是否回零、上下限、反馈超时、故障码和
可选电流。驱动器未提供电流时显示“未知”，不能伪造 0 A。

以下情况必须禁用滑块并拒绝命令：

- `front_arm_joint` 反馈超时；
- 摆臂诊断为 ERROR；
- 巡检 Action 正在占有摆臂；
- 跨层 Action 正在占有摆臂；
- 命令越界或配置无效。

摆臂和气体检测仪升降柱是两个机构。`/inspection/lift_to_height` 不能复用
`/gimbal_cmd`，界面文字和内部状态也不能混用。

## 8. 运行状态不能只看 QProcess

需要保留 `RuntimeStatusMonitor` 或等价实现，分别显示：

- 系统、建图、导航、路线编辑、巡检进程状态；
- 未启动、正在启动、等待就绪、已就绪、正在停止、故障；
- 里程计、IMU、雷达、点云、诊断、STM32 和摆臂反馈的新鲜度；
- 巡检自检通过项、失败项和摘要。

QProcess 仍在运行不等于 ROS 可用。Nav2“已就绪”必须确认 `/map_server`、
`/planner_server`、`/controller_server` lifecycle 均为 Active。配置错误、launch 退出、
话题超时和 lifecycle 未就绪必须向用户显示具体原因，不能用固定等待时间后强制显示成功。

## 9. 路线编辑、选择与执行

Qt 需要提供完整流程：

1. 选择地图并启动 Nav2，等待 lifecycle 就绪；
2. 点击“新建路线并编辑”，输入路线名；
3. 打开带正确 ROS 环境的 RViz，在地图上标点；
4. 支持撤销、清空、验证；
5. 只有 Nav2 分段验证通过才能保存；
6. 保存成功后刷新“已保存路线”并自动选中新路线；
7. 启动巡检、自检、执行所选路线，并显示 Action 进度和结果。

对应 ROS 接口：

| 名称 | 类型 |
| --- | --- |
| `/inspection/route/undo`、`clear`、`validate`、`save` | `std_srvs/srv/Trigger` |
| `/inspection/run_self_check` | `std_srvs/srv/Trigger` |
| `/inspection/run_mission` | `sx_gas_msgs/action/RunInspection` |
| `/floor_transition_navigator/navigate_to_floor` | `sx_gas_msgs/action/NavigateToFloor` |
| `/inspection/lift_to_height` | `sx_gas_msgs/action/LiftToHeight` |
| `/inspection/system_health` | `sx_gas_msgs/msg/SystemHealth` |
| `/inspection/lift/state` | `sx_gas_msgs/msg/LiftState` |

真机路线必须两阶段管理：新路线只能保存到 `route_staging_dir`；人工审核并移动到
`route_dir`（`approved_routes`）后才允许进入“审批路线”下拉框并执行。禁止覆盖同名待审
或已审批路线。仿真路线可以直接保存在用户仿真目录，但不能进入真机审批列表。

## 10. 地图与地形通道门禁

- 建图和 Nav2 不能同时运行。
- 地图原图不能被 Qt 覆盖；地形后处理必须生成新文件。
- 楼梯/斜坡过滤地图必须存在对应 `.report.yaml` 且结果为 PASS，才能用于地形导航。
- 真机只显示 `real_*.yaml`，仿真地图不能出现在真机列表。
- 真机缺少获批地形配置、任务或气体阈值时，只允许建图、单层导航、路线编辑和监视，
  自动巡检保持 fail-closed。
- RViz 必须从已经 source ROS 和工作空间的进程环境启动，否则会出现窗口打开但没有地图、
  TF 或插件的问题。

## 11. Foxglove 与进程生命周期

- 仿真版 Qt 打开后自动启动只读 Foxglove Bridge，当前默认 `0.0.0.0:8765`。
- “启动新走廊仿真系统”只启动无界面 Gazebo Server，不启动 `gzclient`。
- 常驻 ROS/仿真进程必须使用独立 `QProcess` 和独立 Unix 进程组。
- Qt 退出时按依赖倒序停止由本实例启动的路线编辑、巡检、Nav2、SLAM、系统和 Foxglove，
  并终止 launch 的后代节点，不能只杀死 launch 父进程。
- 路线编辑或巡检运行时拒绝停止 Nav2；Nav2、SLAM、路线编辑或巡检运行时拒绝停止底层
  系统。
- 用户主动停止和异常退出要区分，主动停止不能弹出伪故障。
- QML 不直接拼 shell 命令。参数通过结构化列表传给进程，日志输出需要限长并保留错误原因。

## 12. 建议前向移植的文件/模块

不要机械复制全部工程，优先按职责移植当前两个参考版本中的以下模块：

```text
src/gas_control_bringup/config/ros_profile.json
src/gas_control_bringup/config/ros_profile.schema.json
src/gas_control_bringup/src/viewmodels/rosVM/rosprofileconfig.*
src/gas_control_bringup/src/viewmodels/rosVM/rosprocesscontroller.*
src/gas_control_bringup/src/viewmodels/rosVM/runtimestatusmonitor.*
src/gas_control_bringup/src/viewmodels/rosVM/cmdvelpublisher.*
src/gas_control_bringup/src/viewmodels/rosVM/gimbalpublisher.*
src/gas_control_bringup/src/viewmodels/rosVM/odomsubscriber.*
src/gas_control_bringup/src/viewmodels/rosVM/roscontext.*
src/gas_control_bringup/src/rightsidebar/CarSettingsCard.qml
src/gas_control_bringup/src/rightsidebar/RvizSettingsCard.qml
src/gas_control_bringup/src/app/Main.qml
```

同时比较各目录的 `CMakeLists.txt`，确保新增 C++ 类、QML 类型、JSON 配置和资源会被构建、
复制到发布目录。不能只移植 QML 而遗漏后台状态机和配置校验。

## 13. 合并与版本管理要求

- 先给 `qt_new` 当前上游状态打标签或建立独立分支，再逐模块前向移植。
- 仿真版和真机版分别提交，提交信息注明接口版本和对应 ROS 工作区版本。
- 不覆盖两个参考版本的现有改动；遇到冲突按功能行为重新实现，不按文件新旧时间决定。
- ROS 话题、类型、Action 字段、摆臂方向或诊断键变化时，必须先修改
  `qt_ros_unified_interface.md` 和 `schema_version`，再同步两个 edition。
- 新增用户可见字符串后更新中英文翻译资源。
- 不把账号密码、串口权限提升命令、现场审批文件或绝对密钥写入源码。

## 14. 交付验收清单

开发者提交新版 Qt 前至少提供以下证据：

- 仿真版和真机版分别全量构建成功；
- 正确配置可启动，缺配置、错误 edition、错误类型和关键零值均失败关闭；
- 仿真版只在 domain 120，真机版只在 domain 30；
- 摇杆保持约 10 Hz，松手、失焦和退出均产生零速度；
- Qt 退出后无遗留 ROS、Gazebo、Foxglove 进程和端口监听；
- Nav2 未 Active 时不能编辑、验证或执行路线；
- 路线可新建、验证、保存、选择并执行，重名不能覆盖；
- 真机待审路线不能执行，审批路线可以选择；
- 摆臂反馈超时、越界、故障以及巡检/跨层占用时命令被拒绝；
- 真机 `/odom` 来自 EKF，IMU、雷达、STM32 和摆臂状态均有独立超时显示；
- STM32 离线或气体仪离线时巡检自检失败，不能用全零模拟数据通过；
- 仿真地图、真实地图、模拟传感器和审批任务没有跨 edition 混用；
- 新版功能与当前两个参考版本逐项回归，无“更新作者界面后丢失既有 ROS 功能”。

## 15. 2026-08-31 真机版“仅界面模式”修改记录

### 15.1 修改原因和边界

合并同事的真机分支后，`config/ros_profile.json` 中的 ROS 工作空间为同事机器上的固定路径：

```text
/mnt/machine3/sx_gas_control_ws
```

当前上位机开发电脑不存在该目录及其 `install/setup.bash`。旧逻辑把运行目录不存在当作配置
文件整体无效，程序在创建 QML 界面前输出以下错误并以退出码 2 结束：

```text
ROS profile rejected: "ROS环境或工作空间路径不存在"
```

本次修改的目标是让 Qt 上位机开发不依赖同事的 ROS 工作空间进度：ROS 未拉取、未编译或
部署路径暂时不可用时，界面仍能打开；真机控制功能保持安全关闭。本次没有拉取、更新、
编译或修改 `sx_gas_control_ros` 仓库，也没有把现有配置强行改成本机可能过期的 ROS 仓库。

### 15.2 代码修改

基础实现提交为 `f871fed`（`fix: allow UI-only startup without ROS workspace`），涉及以下文件：

- `src/viewmodels/rosVM/rosprofileconfig.cpp`
  - 保留 JSON 格式、edition、字段类型、数值范围、绝对路径等严格校验；
  - `ros.setup`、`ros.workspace`、`ros.workspace_setup` 不存在时不再判定整个 profile 无效；
  - 终端输出缺失的具体路径，并说明程序将以仅界面模式启动。
- `src/viewmodels/rosVM/rosprocesscontroller.cpp`
  - 在加载 ROS 环境前分别检查 ROS 安装环境、工作空间和工作空间 setup 文件；
  - 缺失时设置 `environmentReady=false` 和可供 QML 展示的 `environmentError`；
  - 不执行无效的 `source` 命令；原有 `startResident()`、`runOneShot()` 后台拒绝逻辑继续
    作为最终安全门禁；
  - source 路径改为通过 Bash 位置参数传入，路径中包含空格时也不会被错误拆分。
- `src/rightsidebar/BasicControlCard.qml`
  - 显示“仅界面模式”及缺失路径；
  - 禁用启动真机硬件、RViz、导航、建图、地形后处理、遥控、路线编辑/验证/保存、巡检和
    真机自检等 ROS 功能；
  - 若某个由本程序启动的 ROS 进程已经在运行，仍允许点击对应停止按钮；
  - 气体记录、日志和其他不需要启动 ROS 进程的上位机界面功能不因该模式统一禁用。
- `src/generalstyle/NavParamsCard.qml`
  - 显示仅界面模式错误原因；
  - ROS 环境未就绪时禁用“启动真机硬件 Profile”。
- `src/rightsidebar/RightFunction.qml`
  - 向 `BasicControlCard` 补充传递 `secondaryTextColor`，消除相关 QML 颜色赋值警告。

`environmentReady` 在 `RosProcessController` 创建时探测并在本次进程生命周期内保持不变。
ROS 工作空间准备完成后，应修正部署配置并重启上位机。可以通过
`SX_GAS_QT_CONFIG=/绝对路径/ros_profile.json` 使用本机部署配置，不需要为适配个人电脑提交
同事机器路径或覆盖公共配置。

### 15.3 验证结果

- `cmake --build build/Clang14_Qt_6_10_3-Debug --parallel 4` 构建成功，C++ 与 QML 均完成编译；
- 在 `/mnt/machine3/sx_gas_control_ws` 和对应 `install/setup.bash` 确实不存在的条件下，使用
  `QT_QPA_PLATFORM=offscreen` 启动测试，程序成功创建 ROS Context、加载主 QML 界面并持续
  运行，不再输出 `ROS profile rejected` 或提前返回退出码 2；
- 测试日志正确列出缺失的工作空间和 setup 文件；
- `ctest` 当前报告 `No tests were found`，因此本次只有构建检查和启动冒烟测试，没有自动化
  单元测试结果；
- 启动日志仍有 FluentUI 内部 `FluShortcutPicker` 的既有警告，与 ROS profile 拒绝和本次
  修改无关，不影响主界面加载。

后续恢复真机 ROS 功能时，应由 ROS 负责人提供已经构建、含 `install/setup.bash` 的工作空间
及匹配的 profile。不要仅为了消除界面提示而创建空目录、空 setup 文件或跳过后台门禁。

## 16. 当前权威参考

- ROS/Qt 接口：[Qt/ROS 统一接口契约](qt_ros_unified_interface.md)
- 仿真操作：[Qt + 仿真巡检流程](inspection_workflow_simulation.md)
- 真机操作：[Qt + 真机巡检流程](inspection_workflow_hardware.md)
- 仿真 Qt 变更：`/mnt/machine3/sx_GasControl_qt_simulation/QT_ROS_INTEGRATION_CHANGELOG.md`
- 真机 Qt 变更：`/mnt/machine3/sx_GasControl_qt_hardware/QT_ROS_INTEGRATION_CHANGELOG.md`
- 仿真 Qt 直接运行：`/mnt/machine3/sx_GasControl_qt_simulation/DIRECT_QT_WORKFLOW.md`
- 真机 Qt 直接运行：`/mnt/machine3/sx_GasControl_qt_hardware/DIRECT_QT_WORKFLOW.md`

注意：参考版本中的早期历史段落可能已经被后续章节取代，例如“真机摆臂只有命令没有
反馈”已经不再是当前目标状态。开发者应优先阅读变更记录最后的章节和本项目统一接口，
不能只依据旧段落回退功能。
