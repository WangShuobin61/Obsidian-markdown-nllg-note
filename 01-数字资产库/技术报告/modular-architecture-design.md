# gas_control_bringup 模块化架构设计技术报告

## 一、设计目标

`gas_control_bringup` 是基于 ROS2 工作空间的嵌入式气体调控机器人终端 UI 应用。项目从早期的单体 Qt Widgets 架构（见 `doc/src_old`）演进到当前的多模块 QML 架构，核心驱动力是：

- 界面逻辑与业务逻辑彻底解耦，QML 侧不持有任何 C++ 对象引用
- 各 UI 区域独立维护，互不干扰
- C++ ViewModel 层可独立编译、测试，不依赖具体界面实现

---

## 二、整体模块划图

```
gas_control_bringup/
├── src/
│   ├── app/                    ← 入口模块（URI: gascontrolbringup.app）
│   │   ├── main.cpp            ← QQmlApplicationEngine 启动
│   │   └── Main.qml            ← 根窗口，组合所有 UI 区域
│   │
│   ├── core/                   ← 纯 C++ 基础工具（无 QML）
│   │   └── algorithm/
│   │       └── SingleInstanceGuard  ← 进程单例守卫
│   │
│   ├── viewmodels/             ← C++ ViewModel 层（URI: gascontrolbringup.viewmodels）
│   │   ├── authManagerVM/      ← 认证子系统
│   │   │   ├── AuthManager     ← QML_SINGLETON 业务门面
│   │   │   ├── AuthRepository  ← SQLite 数据访问层
│   │   │   └── PasswordHasher  ← 加密工具类
│   │   ├── stateManageVM/      ← 设备状态子系统
│   │   │   ├── StateManager    ← QML_SINGLETON UDP 接收器
│   │   │   ├── BatteryState    ← 电池状态值对象
│   │   │   ├── SignalState     ← 网络信号值对象
│   │   │   └── EnvironmentState ← 环境传感器值对象
│   │   └── componentsVM/       ← UI 组件 ViewModel
│   │       └── CircularReveal  ← QML_ELEMENT 涟漪动画组件
│   │
│   ├── generalstyle/           ← 通用 UI 组件（URI: gascontrolbringup.generalstyle）
│   │   ├── LogIn.qml           ← 登录对话框
│   │   ├── UserSession.qml     ← QML_SINGLETON 会话状态
│   │   ├── ChangePasswordDialog.qml
│   │   ├── RegisterUserDialog.qml
│   │   ├── SettingsDialog.qml
│   │   └── HoveringCue.qml
│   │
│   ├── topbar/                 ← 顶栏区域（URI: gascontrolbringup.topbar）
│   │   ├── TopFunction.qml     ← 顶栏容器
│   │   ├── TopBatteryStatus.qml
│   │   ├── TopConnectionStatus.qml
│   │   ├── TopEnvironmentStatus.qml
│   │   ├── TopRuntimeStatus.qml
│   │   ├── TopUserStatus.qml
│   │   └── TopWindowControls.qml
│   │
│   ├── mainwindow/             ← 主内容区（URI: gascontrolbringup.mainwindow）
│   │   └── MainWinFunction.qml
│   ├── leftsidebar/            ← 左侧边栏（URI: gascontrolbringup.leftsidebar）
│   │   └── LeftFunction.qml
│   └── rightsidebar/           ← 右侧边栏（URI: gascontrolbringup.rightsidebar）
│       └── RightFunction.qml
│
└── script/
    └── simulate_state_status.py ← 状态模拟调试工具
```

---

## 三、层次划分原则

### 3.1 三层结构


| 层次              | 模块                                                                                | 职责                            |
| --------------- | --------------------------------------------------------------------------------- | ----------------------------- |
| **基础层**         | `core`                                                                            | 与 QML/UI 完全无关的 C++ 工具，可被任何层引用 |
| **ViewModel 层** | `viewmodels`                                                                      | 持有状态、处理业务逻辑，以 QML 可绑定属性暴露数据   |
| **视图层**         | `app` / `generalstyle` / `topbar` / `mainwindow` / `leftsidebar` / `rightsidebar` | 纯 QML，只做布局和交互，不含业务逻辑          |


依赖方向严格单向：视图层 → ViewModel 层 → 基础层，不允许反向依赖。

### 3.2 ViewModel 层内部分组

`viewmodels` 目录下按业务域再次分组：

- `authManagerVM`：认证、权限，对应登录/用户管理界面
- `stateManageVM`：设备实时状态，对应顶栏各状态指示器
- `componentsVM`：与具体业务无关的可复用 UI 组件（`CircularReveal`）

每个子目录是独立的关注点，新增业务域只需新建子目录，不影响已有代码。

---

## 四、状态下传模式：UDP → ViewModel → QML 绑定

设备状态的数据流是本项目最典型的模块协作示例：

```
外部设备 / simulate_state_status.py
        │ UDP JSON (port 45454)
        ▼
StateManager::readPendingDatagrams()
        │ 解析 JSON，更新 BatteryState / SignalState / EnvironmentState
        ▼
emit batteryChanged() / signalChanged() / environmentChanged()
        │ Qt 信号
        ▼
TopBatteryStatus.qml         ← StateManager.batteryLevel 属性绑定自动刷新
TopConnectionStatus.qml      ← StateManager.signalType / wifiStrength 绑定
TopEnvironmentStatus.qml     ← StateManager.temperature / humidity 绑定
```

QML 侧完全依赖属性绑定，不持有任何函数调用时序，状态到显示的映射是声明式的。Python 调试脚本 `simulate_state_status.py` 可在不连接真实硬件的情况下发送 JSON 包，驱动完整的 UI 响应链路。

---

## 五、认证状态流转

```
LogIn.qml
    │ AuthManager.verifyLogin(username, password)
    ▼
AuthManager（C++ QML_SINGLETON）
    │ 验证通过：emit authenticationChanged()
    ▼
UserSession.qml（QML_SINGLETON）
    │ 监听 AuthManager.isAuthenticated
    ▼
Main.qml 中的条件布局
    │ 根据 UserSession 状态显示/隐藏功能区
    ▼
各功能页面根据 AuthManager.currentRole 决定权限范围
```

认证结果从 C++ 层经信号传递到 QML 单例 `UserSession`，再由 `UserSession` 驱动整个 UI 的权限展示，各功能模块不直接查询认证状态，而是观察 `UserSession` 的派生状态。

---

## 六、CMake 模块化组织

每个 UI 区域对应一个独立的 `qt_add_qml_module` 目标，URI 以 `gascontrolbringup.` 为命名空间前缀统一组织：

```cmake
# 顶级 CMakeLists.txt 汇总
add_subdirectory(src/core)
add_subdirectory(src/generalstyle)
add_subdirectory(src/viewmodels)
add_subdirectory(src/topbar)
add_subdirectory(src/mainwindow)
add_subdirectory(src/leftsidebar)
add_subdirectory(src/rightsidebar)

target_link_libraries(app_gas_control_bringup
    PRIVATE core generalstyle viewmodels topbar mainwindow leftsidebar rightsidebar Qt6::Svg)
```

各子模块只 `link` 自己实际依赖的模块，依赖图清晰可审计。`viewmodels` 模块通过 `GLOB_RECURSE` 自动收集所有子目录的源文件，新增 ViewModel 文件无需修改 CMakeLists。

---

## 七、与旧架构的对比

旧架构（`doc/src_old`）是典型的单体 Qt Widgets 应用：


| 维度     | 旧架构                      | 当前架构                                |
| ------ | ------------------------ | ----------------------------------- |
| UI 框架  | Qt Widgets（`.ui` 文件）     | Qt Quick / QML                      |
| 状态管理   | 控件直接读写成员变量               | ViewModel + 属性绑定                    |
| 模块划分   | 单一可执行文件，文件级分离            | 多个独立 QML 模块，URI 隔离                  |
| 业务逻辑位置 | 混在 `mainwindow.cpp`      | 独立 ViewModel 类，可单元测试                |
| 硬件通信   | `rclcomm.cpp` 直接操作 UI 控件 | UDP → StateManager → 信号 → QML 绑定    |
| 认证     | 无                        | 完整三层（Manager / Repository / Hasher） |


当前架构的核心收益是：任何一个 QML 视图文件的改动不影响 C++侧，任何一个 C++ ViewModel 的改动不需要触碰 QML 文件，两侧可以独立迭代。