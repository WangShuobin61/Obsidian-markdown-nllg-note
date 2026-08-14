# FluShell 可复用应用外壳框架技术报告

> 适用版本：Qt 6.10.3 / Clang 14 / FluentUI for Qt6
> 涉及模块：ShellLayout、ShellWindowControls、ShellTitleBar、AppPreferences、Tour
> 编写日期：2026-08-04

---

## 1. 背景与目标

### 1.1 起因

`sx_GasControl_qt` 是一个 ROS 2 工作空间，其上位机核心功能全部集中在 `gas_control_bringup` 功能包中。随着功能模块趋于完整，其中若干与业务无关的基础设施（无边框标题栏、主题涟漪、鉴权门禁、设置中心、停靠区折叠）具备在其他 Qt/FluentUI 项目中复用的价值。

为降低后续新项目的起步成本，本次将上述基础设施提取为独立的最小可运行项目 **FluShell**，放置于 `~/my_project/FluShell`，脱离 ROS 工作空间组织方式，作为纯 Qt 项目独立维护。

### 1.2 为何不沿用 ROS 工作空间组织

| 维度 | ROS 工作空间 | 纯 Qt 项目 |
|---|---|---|
| 构建系统 | `colcon` + `ament_cmake` 包装 | 直接 `cmake --preset` |
| 依赖声明 | `package.xml` + `ament_cmake` | `CMakeLists.txt` + `find_package(Qt6)` |
| 适用范围 | 与 ROS 有硬依赖的程序包 | 任何 Qt6 桌面应用 |
| 复用难度 | 高（需要 ROS 环境） | 低（只需 Qt6 + FluentUI） |

FluShell 不依赖任何 ROS 组件，强制套用 ROS 工作空间只会引入无谓的编译约束。

---

## 2. 最终项目结构

```
~/my_project/FluShell/
├── CMakeLists.txt          # 顶层，定义 app_flushell 可执行目标
├── CMakePresets.json       # clang_debug / gcc_debug / clang_rel / gcc_rel 四套预设
├── res/
│   └── img/icon/icon.svg
├── src/
│   ├── app/
│   │   ├── main.cpp        # 单实例守卫、翻译装载、QML 引擎启动
│   │   └── Main.qml        # 根窗口：ShellLayout 挂载点（派生项目在此替换内容）
│   ├── auth/               # 登录 / 注册 / 用户管理 QML
│   ├── core/               # AppDatabase、AppPaths、SingleInstanceGuard、XlsxWriter
│   ├── shell/              # 框架壳层 QML（本次重点）
│   │   ├── ShellLayout.qml
│   │   ├── ShellTitleBar.qml
│   │   ├── ShellWindowControls.qml
│   │   ├── ShellUserStatus.qml
│   │   ├── SettingsDialog.qml
│   │   ├── Tour.qml
│   │   └── DemoPanel.qml
│   ├── theme/              # AppPreferences、HoveringCue、TouchScrollColumn、UserSession
│   └── viewmodels/         # AuthManager、SystemEventManager、AppController、CircularReveal
└── translations/
    ├── flushell_zh_CN.ts
    └── flushell_en_US.ts
```

---

## 3. 核心模块设计

### 3.1 ShellLayout — 可折叠三区布局

#### 问题溯源

原项目 `Main.qml` 的停靠区折叠逻辑存在**三个状态源**：

```
Window（持有 leftSidebarVisible / rightSidebarVisible / bottomBarVisible）
    ↓ 传给
TopWindowControls.qml（直接读写 hostWindow.leftSidebarVisible）← 反向依赖宿主窗口
    ↓ 另一条路
MainWinFunction.qml（自己再存一份 bottomBarVisible，由 Main 逐层灌）
```

这是典型的状态复散问题：同一个「布局折叠状态」同时存在于 Window 属性、标题栏按钮状态和内容区 prop，任何一处改变都要手动同步其余两处。

#### 解决方案

新建 `ShellLayout.qml`，将**布局状态的单一数据源**和**内容插槽**集中在同一个组件内：

```
AppPreferences（落盘）
      ↓ 初值
ShellLayout             ← 唯一状态源：leftVisible / rightVisible / bottomVisible
    ├── leftPane   : Component    ┐
    ├── centerPane : Component    ├── 派生项目注入，基座不认识内容
    ├── bottomPane : Component    │
    └── rightPane  : Component    ┘
           ↑ dockTarget
ShellWindowControls     ← 折叠按钮直接读写 ShellLayout，不经过 Window
```

关键属性与方法：

```qml
// 折叠状态（单一数据源）
property bool leftVisible:   AppPreferences.leftPaneVisible
property bool rightVisible:  AppPreferences.rightPaneVisible
property bool bottomVisible: AppPreferences.bottomPaneVisible

// 内容插槽（Component 注入，Loader 惰性装载）
property Component leftPane:   null
property Component centerPane: null
property Component bottomPane: null
property Component rightPane:  null

// 供标题栏按钮调用的切换方法
function toggleLeft()   { leftVisible = !leftVisible }
function toggleRight()  { rightVisible = !rightVisible }
function toggleBottom() { bottomVisible = !bottomVisible }

// 供宿主窗口推导 minimumWidth（收起侧栏后允许更窄）
readonly property int contentMinimumWidth: centerMinimumWidth
    + (leftVisible ? leftMinimumWidth : 0)
    + (rightVisible ? rightMinimumWidth : 0)
```

布局层次（水平 SplitView 嵌套垂直 SplitView）：

```
FluSplitLayout (水平)
├── leftSlot   (Loader ← leftPane，折叠时 active=false 真正释放资源)
├── centerSlot (FluSplitLayout 垂直)
│   ├── centerMainSlot  (Loader ← centerPane)
│   └── bottomSlot      (Loader ← bottomPane，折叠时 active=false)
└── rightSlot  (Loader ← rightPane，折叠时 active=false)
```

`hasLeftPane / hasRightPane / hasBottomPane` 由 `prop !== null` 推导：没有注入内容的区，对应的标题栏折叠按钮自动隐藏，不会出现「点了没反应」的死按钮。

#### 状态持久化

`onLeftVisibleChanged` / `onRightVisibleChanged` / `onBottomVisibleChanged` 直接回写 `AppPreferences`。重置全部偏好后，通过 `Connections { onResetCompleted }` 把默认值灌回来。

### 3.2 ShellWindowControls — 折叠按钮迁移

原 `TopWindowControls.qml` 的折叠按钮直接读写 `hostWindow.leftSidebarVisible`，这是反向依赖（控件依赖宿主实现细节）。新版通过 `dockTarget` 属性引用 `ShellLayout`：

```qml
// ShellWindowControls.qml
property var dockTarget: null   // 接受 ShellLayout 实例

FluIconButton {
    id: btnDockLeft
    // 没注入内容时整个按钮消失
    visible: dockTarget !== null && dockTarget.hasLeftPane
    iconColor: _dockColor(visible && dockTarget.leftVisible)
    text: visible && dockTarget.leftVisible ? qsTr("关闭左侧栏") : qsTr("展开左侧栏")
    onClicked: dockTarget.toggleLeft()
}
```

`dockTarget` 为 `null` 时三个折叠按钮整体隐身，向后兼容不使用 ShellLayout 的场景。

### 3.3 ShellTitleBar — dockTarget 透传

`ShellTitleBar` 新增 `property var dockTarget: null`，透传给 `ShellWindowControls`，并暴露 `tourTargetDockLeft / tourTargetDockRight / tourTargetDockBottom` 三个导览锚点，供 Tour 使用。

### 3.4 AppPreferences — 布局状态落盘

在 `QML Settings` 的 `[preferences]` 段新增三个持久化字段：

```qml
// AppPreferences.qml 新增部分
readonly property bool leftPaneVisible:   prefs.leftPaneVisible
readonly property bool rightPaneVisible:  prefs.rightPaneVisible
readonly property bool bottomPaneVisible: prefs.bottomPaneVisible

property Settings store: Settings {
    // ...（原有字段）
    property bool leftPaneVisible:   true
    property bool rightPaneVisible:  true
    property bool bottomPaneVisible: true
}

function setLeftPaneVisible(value)   { prefs.leftPaneVisible = value }
function setRightPaneVisible(value)  { prefs.rightPaneVisible = value }
function setBottomPaneVisible(value) { prefs.bottomPaneVisible = value }
```

`reset()` 中同步补充三个字段的默认值重置，`resetCompleted` 信号触发 `ShellLayout` 回灌。

### 3.5 Tour — 折叠步骤自适应

`Tour.qml` 新增 `targetDockLeft / targetDockRight / targetDockBottom` 三个属性，并使用计算属性 `dockSteps` 动态构建折叠步骤数组——**只有对应按钮实际可见时才纳入步骤**，避免派生项目未注入某停靠区时出现空指引：

```qml
readonly property var dockSteps: {
    const list = []
    if (targetDockLeft && targetDockLeft.visible)
        list.push({ title: qsTr("左侧栏"), ... })
    if (targetDockBottom && targetDockBottom.visible)
        list.push({ title: qsTr("底栏"), ... })
    if (targetDockRight && targetDockRight.visible)
        list.push({ title: qsTr("右侧栏"), ... })
    return list
}
steps: dockSteps.concat(baseSteps).concat(extraSteps)
```

---

## 4. FluShell 与 sx_GasControl_qt 的关系

```
~/my_project/
├── FluShell/               ← 独立纯 Qt 项目，无 ROS 依赖
│   └── src/shell/ShellLayout.qml  ← 四区可折叠布局
│
└── sx_GasControl_qt/
    └── src/gas_control_bringup/
        └── src/
            └── app/Main.qml    ← 原有项目（仍使用 FluSplitLayout 三栏手写布局）
```

两个项目**独立演进**，FluShell 不包含任何 ROS、气体记录或硬件驱动相关内容，只提供应用外壳能力。后续派生新项目时从 FluShell 克隆，替换 `Main.qml` 的 `shellLayout` 插槽内容即可。

---

## 5. 已知遗留告警

运行时控制台会出现：

```
qrc:/qt/qml/FluentUI/Controls/FluShortcutPicker.qml:15:5:
    Unable to assign [undefined] to FluHotkey*
```

**根因**：FluentUI 源码中 `FluShortcutPicker.qml` 第 15 行声明了 `property FluHotkey syncHotkey: undefined`，`undefined` 无法赋给 C++ 注册的 QObject 子类属性，QML 引擎在每次实例化时告警。

**影响**：零。`FluShortcutPicker` 在本项目的用途仅取 `current` 属性和 `onAccepted` 信号，从未使用 `syncHotkey`，该 handler 永远不会触发。

**修复评估**：正确修法需改 FluentUI 源码（加 null 检查），代价是维护 patch 版本并在每次上游升级时重新 merge，性价比极低。维持现状，忽略该告警。

---

## 6. 派生项目使用指南

### 6.1 最小改造步骤

1. 复制 `FluShell/` 目录，重命名（如 `MyApp/`）。
2. 在顶层 `CMakeLists.txt` 中修改 `project(myapp ...)` 和 URI 前缀（全局替换 `flushell` → `myapp`）。
3. 在 `Main.qml` 的 `ShellLayout` 四个插槽中替换占位 `DemoPanel` 为业务组件：

```qml
ShellLayout {
    id: shellLayout
    anchors.fill: parent
    visible: UserSession.isLoggedIn

    leftPane:   Component { MyLeftPanel {}   }   // 不需要的区不写，折叠按钮自动消失
    centerPane: Component { MyMainContent {} }
    bottomPane: Component { MyStatusBar {}   }
    rightPane:  Component { MyControlCard {} }

    // 尺寸约束（可覆写默认值）
    leftMinimumWidth:   280
    centerMinimumWidth: 720
    bottomHeightRatio:  0.25
}
```

4. 给 `ShellTitleBar` 接上 `dockTarget: shellLayout`（已在 `Main.qml` 模板中完成）。

### 6.2 扩展业务偏好

不要改 `AppPreferences.qml`，另建 `XxxPreferences.qml`（同样 `pragma Singleton`，同样 `Settings { category: "..." }`），保持基座偏好与业务偏好内聚分离。

---

## 7. 架构评估：为何不采用 Qt 插件式开发

在本次讨论中评估了 Qt 插件式（`QPluginLoader` + 纯虚接口）与编译期模块（当前方案）两种架构：

| 维度 | 编译期模块（当前方案） | Qt 插件式 |
|---|---|---|
| 新增功能成本 | 改 CMake + 改 Main.qml + 全量重编 | 编译一个 `.so`，放进目录重启生效 |
| 主程序耦合 | 直接依赖具体类型 | 只依赖抽象接口 |
| ABI 稳定性负担 | 无 | 重：接口变则所有插件必须同版本重编 |
| 单人/单产品效率 | 高 | 低（多付出接口设计 + 版本管理成本） |

**结论**：插件式的收益在于「主程序发布后第三方独立扩展」，当前场景是自己从 FluShell 克隆新项目（编译期复用），不满足插件式的价值前提。`ShellLayout` 的 `Component` 插槽已提供等价的可替换性，且有完整编译期类型检查，性价比更高。