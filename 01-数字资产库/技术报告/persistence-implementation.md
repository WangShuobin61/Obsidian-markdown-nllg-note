# 应用持久化实现技术报告

> 适用版本：gas_control_bringup（Qt 6.10.3 / QML / C++）  
> 编写日期：2026-03-14

---

## 1. 背景与目标

原始应用只持久化用户账号数据（SQLite `app.db`）。重启后：

- 主题（深/浅色、强调色、动画开关）恢复默认值
- 侧栏可见性恢复默认值
- 顶栏电量、信号、温湿度全部显示 `--` / 未知状态

本次改造在不改变已有存储路径和数据库结构的前提下，增加两类持久化能力：

| 类别 | 内容 | 存储载体 |
|------|------|----------|
| A 类：用户偏好 | 主题、摇杆、侧栏 | QML `Settings`（INI 文件）|
| B 类：系统状态快照 | 电量、信号、温湿度 | C++ `QSettings`（同一 INI 文件）|

---

## 2. 存储路径与文件布局

### 2.1 已有的 app.db 路径构造方式

`app.db` 由 `AuthRepository` 手动拼接路径：

```
QStandardPaths::GenericDataLocation + "/GasControl/app.db"
// → ~/.local/share/GasControl/app.db
```

这条路径与 `QApplication::organizationName` / `applicationName` 无关，因此可以安全地设置 org/app name 而不影响 `app.db` 的位置。

### 2.2 settings.ini 的落地位置

`main.cpp` 中配置：

```cpp
app.setOrganizationName("GasControl");
QSettings::setDefaultFormat(QSettings::IniFormat);
QSettings::setPath(
    QSettings::IniFormat,
    QSettings::UserScope,
    QStandardPaths::writableLocation(QStandardPaths::GenericDataLocation)
    // → ~/.local/share
);
```

`QSettings::UserScope` + `GenericDataLocation` + `organizationName` 组合后：

```
~/.local/share/GasControl/app_gas_control_bringup.ini
```

`app.db` 和 `settings.ini` 共处同一目录，结构清晰。

### 2.3 INI 文件 section 划分

```ini
[preferences]          ← A 类：QML Settings 托管
darkMode = ...
accentColor = ...
...

[state/battery]        ← B 类：C++ QSettings 托管
[state/signal]
[state/environment]
```

---

## 3. A 类：用户偏好持久化

### 3.1 模块：AppPreferences.qml（新建单例）

位置：`src/generalstyle/AppPreferences.qml`，注册为 QML singleton（与 `UserSession` 同类注册方式）。

**数据流：**

```
disk (settings.ini [preferences])
        ↕ QML Settings (store)
AppPreferences singleton
    ├── restore()    ← 启动时 Main.qml Component.onCompleted 调用
    ├── reset()      ← SettingsDialog "重置全部" 按钮调用
    └── Connections  ← FluTheme / UserSession 变化时自动回写盘
```

**持久化的 9 个字段：**

| 字段 | 绑定目标 | 默认值 |
|------|----------|--------|
| `darkMode` | `FluTheme.darkMode` | Light |
| `accentColor` | `FluTheme.accentColor`（存 hex 字符串）| Blue |
| `animationEnabled` | `FluTheme.animationEnabled` | true |
| `nativeText` | `FluTheme.nativeText` | false |
| `chassisJoystickEnabled` | `UserSession.chassisJoystickEnabled` | true |
| `gimbalJoystickEnabled` | `UserSession.gimbalJoystickEnabled` | true |
| `joystickSize` | `UserSession.joystickSize` | 50 |
| `leftSidebarVisible` | `mainWindow.leftSidebarVisible` | true |
| `rightSidebarVisible` | `mainWindow.rightSidebarVisible` | true |

### 3.2 防环机制

`restore()` 和 `reset()` 内部执行期间 `_restoring = true`，此时 `Connections` 监听器的回写分支被短路，避免"灌值 → 触发信号 → 回写盘 → 再触发"的循环。

### 3.3 强调色还原

`FluTheme.accentColor` 是一个 `FluAccentColor` 对象，不能直接序列化。存储时取 `.normal.toString()`（hex 字符串），还原时优先从 8 个预设色中匹配，未命中则用 `FluColors.createAccentColor(hex)` 动态创建，保证 `SettingsDialog` 中的色板对勾能正确高亮。

### 3.4 侧栏与 Main.qml

侧栏可见性的 **单一数据源在 `mainWindow`**，`AppPreferences` 只作为持久化镜像。

```
启动：Main.qml 读 AppPreferences.leftSidebarVisible 初始化 mainWindow.leftSidebarVisible
运行：mainWindow.leftSidebarVisible 变化 → onLeftSidebarVisibleChanged → AppPreferences.setLeftSidebarVisible()
重置：AppPreferences.resetCompleted 信号 → Main.qml 强制回读 AppPreferences 的新值
```

"重置后显式回读"这一步是必要的：`reset()` 把 `store.leftSidebarVisible` 写回 `true`，但 `mainWindow.leftSidebarVisible` 已经是 `true` 则不会触发 `Changed`，若不显式回读，只有盘上值更新、界面状态不变的情况下也能保持一致，但若用户在重置前把侧栏关掉了，`mainWindow.leftSidebarVisible` 是 `false`，此时 `reset()` 不会自动恢复它——故需要 `Connections { target: AppPreferences; function onResetCompleted() {...} }` 兜底。

---

## 4. B 类：系统状态快照持久化

### 4.1 模块：StateManager（已有 C++ 单例，新增快照读写）

**决策：哪些持久化，哪些不持久化**

| 状态 | 持久化 | 理由 |
|------|--------|------|
| 电量 level / status | ✅ | 硬件电量变化缓慢，快照值有参考意义 |
| 信号 type / wifiStrength | ✅ | 启动时可正确显示上次的连接类型 |
| 温度 / 湿度 | ✅ | 环境数据连续性好，短时间变化小 |
| 气体浓度 | ❌ | 保留硬编码默认值，直到下位机发来数据 |

**数据流：**

```
disk (settings.ini [state/*])
    ↑ saveBatterySnapshot()      ← updateBattery() 结尾调用
    ↑ saveSignalSnapshot()       ← updateSignal() 结尾调用
    ↑ saveEnvironmentSnapshot()  ← updateEnvironment() 结尾调用
    ↓ loadSnapshot()             ← 构造函数，信号连接之后、UDP 绑定之前调用
```

### 4.2 构造顺序

```
StateManager()
  1. connect(&m_battery ... batteryChanged)   // 先连信号
  2. connect(&m_signal  ... signalChanged)
  3. connect(&m_environment ... environmentChanged)
  4. connect(&m_gasConcentrations ... gasConcentrationsChanged)
  5. loadSnapshot()                            // 从盘上加载 → 触发 changed → 通知 QML
  6. bindUdpReceiver()                         // 最后绑 UDP
```

`loadSnapshot()` 在信号连接后调用，保证 QML 能收到初始值通知；在 UDP 绑定前调用，避免与下位机数据产生竞争。

### 4.3 QSettings group 路径

```
[state/battery]
level = 88
status = discharging

[state/signal]
type = wifi
wifiStrength = 3

[state/environment]
temperature = 25.0
humidity = 50.0
```

`loadSnapshot()` 用各 state 类构造时的出厂默认值作为 `QSettings::value()` 的第二参数，首次运行无盘上记录时沿用出厂值，行为与改造前一致。

### 4.4 statemanager.h 新增接口

```cpp
Q_INVOKABLE void clearPersistedState();   // 重置时清除盘上状态快照（内存值保持不变）

private:
    void loadSnapshot();
    void saveBatterySnapshot();
    void saveSignalSnapshot();
    void saveEnvironmentSnapshot();
```

`clearPersistedState()` 暴露给 QML，供 `SettingsDialog` "重置全部" 时同步清除状态快照，确保下次冷启动回到出厂值而非重置前的最后一帧。

---

## 5. 修改清单

| 文件 | 变更类型 | 变更内容 |
|------|----------|----------|
| `src/app/main.cpp` | 修改 | 设置 org/app name，配置 `QSettings` 路径指向 GasControl 目录 |
| `src/generalstyle/AppPreferences.qml` | 新建 | A 类偏好持久化单例 |
| `src/generalstyle/CMakeLists.txt` | 修改 | 注册 AppPreferences 为 QML singleton |
| `src/app/Main.qml` | 修改 | 启动 restore()、侧栏初始值/回写绑定、resetCompleted 监听 |
| `src/viewmodels/stateManageVM/statemanager.h` | 修改 | 声明快照方法、`clearPersistedState()`、`QSettings` 前向声明 |
| `src/viewmodels/stateManageVM/statemanager.cpp` | 修改 | 实现快照 load/save/clear，在 update* 方法中挂写盘调用 |

---

## 6. 关键约束与边界条件

**不改动 *Valid 标志及 QML 状态类的公共接口**  
B 类持久化通过已有的 `update*()` 方法灌入值，下位机断连时 topbar 显示的是"上次已知值"而不是 `--`，这是预期行为。

**气体浓度刻意不持久化**  
气体传感器在断电期间可能漂移，重启显示过期浓度值存在安全隐患，因此保留出厂默认值直到下位机首次发送数据。

**`_restoring` 标志防止写盘循环**  
A 类的 `restore()` 向 `FluTheme`/`UserSession` 赋值会触发它们的 `Changed` 信号，`Connections` 监听到后本应回写盘，`_restoring` 标志将这条回写路径短路。

**settings.ini 与 app.db 共目录，容器挂载点一致**  
Docker 部署时 `-v /opt/gas_control_data:/root/.local/share/GasControl` 同时覆盖两个文件，无需额外挂载点。