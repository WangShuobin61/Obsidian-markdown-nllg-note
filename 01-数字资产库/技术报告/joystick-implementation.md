# 虚拟摇杆（NavigationBall）实现技术报告

> 适用版本：gas_control_bringup / Qt 6.10.3 / FluentUI  
> 编写日期：2026-06-25

---

## 1. 背景与目标

上位机运行在触摸屏工控机上，需要在不接入物理手柄的情况下，直接通过触摸操控机器人底盘移动和云台转动。要求：

- 360° 全向，力度 0-100 分级
- 两路独立摇杆，分别控制底盘（chassis）和云台（gimbal）
- 支持多点触控（底盘和云台可同时操作）
- 浮动位置，用户可拖拽重新定位
- 通过设置面板统一控制显隐和尺寸
- 数据通过 UDP 实时输出给下位机

---

## 2. 整体架构

```
QML 层                         C++ 层                 网络层
─────────────────────────────  ─────────────────────  ──────────────────
Main.qml                       JoystickBridge          UDP 45455
  └─ NavigationBall (chassis)  (QML_SINGLETON)    →   {"chassis": {...},
  └─ NavigationBall (gimbal)   .update(side,           "gimbal":  {...}}
                                angle, mag, active)

UserSession.qml (pragma Singleton)
  ├─ chassisJoystickEnabled
  ├─ gimbalJoystickEnabled
  └─ joystickSize

SettingsDialog.qml
  └─ 摇杆设置卡片 → 绑定 UserSession 三个属性
```

**数据流向（单次拖拽事件）：**

```
用户触摸 ballArea
  → DragHandler.onCentroidChanged
    → 计算 rawX/rawY（相对大圆中心的偏移）
    → _clamp() 限制在半径范围内 → 更新 knobOffsetX/Y（旋钮位置）
    → 计算 angle（atan2，0°=正前顺时针）
    → 计算 magnitude（偏移/半径，四舍五入到 0-100）
    → JoystickBridge.update(side, angle, magnitude, true)
      → C++ 端更新对应摇杆状态
      → sendState() → UDP 发出合并 JSON
```

松手时：`knobOffsetX/Y` 归零，发送 `active=false, angle=0, magnitude=0`。

---

## 3. 文件清单


| 文件                                                | 层             | 职责                     |
| ------------------------------------------------- | ------------- | ---------------------- |
| `src/generalstyle/NavigationBall.qml`             | QML           | 摇杆组件本体，可复用             |
| `src/viewmodels/joystickVM/joystickbridge.h/.cpp` | C++           | QML 单例，UDP 出站桥         |
| `src/generalstyle/UserSession.qml`                | QML Singleton | 共享摇杆开关和尺寸状态            |
| `src/app/Main.qml`                                | QML           | 实例化两个摇杆，绑定 UserSession |
| `src/generalstyle/SettingsDialog.qml`             | QML           | 摇杆设置卡片（开关 + 尺寸滑块）      |
| `script/simulate_joystick.py`（可选测试）               | Python        | 接收 45455 端口 UDP，打印摇杆数据 |


---

## 4. NavigationBall.qml 组件设计

### 4.1 对外属性


| 属性          | 类型     | 默认值         | 说明                                             |
| ----------- | ------ | ----------- | ---------------------------------------------- |
| `side`      | string | `"chassis"` | 标识符，传给 JoystickBridge，`"chassis"` 或 `"gimbal"` |
| `label`     | string | `""`        | 显示在旋钮中心的文字（如「底盘」「云台」）                          |
| `sizeLevel` | int    | `50`        | 0-100，映射到球半径 44-84 px                          |


### 4.2 内部结构

```
Item (root)  width = ballRadius*2,  height = ballRadius*2 + 18
│
├─ Rectangle (moveHandle)          高度 6px 短横线，顶部居中
│   └─ DragHandler                 拖动整个 root 在父容器内移动
│
└─ Item (ballArea)                 y=12，宽高 = ballRadius*2
    ├─ Rectangle (外圈)            半透明填充 + primaryColor 边框
    ├─ Rectangle × 2 (十字线)      低透明度参考线
    └─ Rectangle (knob)            主色填充圆，随偏移量移动
        └─ FluText                 label 居中，白色
            └─ DragHandler         捕获摇杆拖拽，target=null 避免移动 knob
```

### 4.3 尺寸映射

```
ballRadius = 44 + sizeLevel / 100 * 40    // [44, 84] px
knobRadius = ballRadius * 0.3125          // 固定比例约 1/3
```

### 4.4 角度约定

- 0° = 正前方（向上）
- 顺时针递增：90° = 右，180° = 后，270° = 左
- 实现：`atan2(dx, -dy) * 180 / π`，负值加 360°

### 4.5 旋钮跟随与归中动画

拖拽活跃时（`joystickActive=true`）`Behavior on x/y` 被禁用，旋钮实时跟随手指；松手后 `joystickActive` 变为 false，`Behavior` 重新生效，`SmoothedAnimation { velocity: 350 }` 驱动旋钮平滑归位。

### 4.6 位置拖拽约束

`moveHandle` 上的 `DragHandler` 将 `root` 的可移动范围限制在父容器内：

```
xAxis.minimum: 0
xAxis.maximum: root.parent.width  - root.width
yAxis.minimum: 0
yAxis.maximum: root.parent.height - root.height
```

---

## 5. JoystickBridge（C++ UDP 桥）

### 5.1 注册方式

```cpp
QML_ELEMENT
QML_SINGLETON
```

在 QML 中通过 `import gascontrolbringup.viewmodels` 后直接用 `JoystickBridge.update(...)` 调用，无需手动注册。

### 5.2 状态模型

```
m_chassis : QJsonObject { angle, magnitude, active }
m_gimbal  : QJsonObject { angle, magnitude, active }
```

每次 `update()` 只更新对应摇杆，两路状态合并成一条 JSON 发出：

```json
{
  "chassis": { "angle": 45.0, "magnitude": 72, "active": true },
  "gimbal":  { "angle": 0.0,  "magnitude": 0,  "active": false }
}
```

### 5.3 端口规划


| 端口    | 用途                           |
| ----- | ---------------------------- |
| 45454 | 状态接收（StateManager，下位机→上位机）   |
| 45455 | 摇杆出站（JoystickBridge，上位机→下位机） |


---

## 6. UserSession 共享状态

`UserSession.qml`（`pragma Singleton`）新增三个属性作为全局单一数据源：

```
chassisJoystickEnabled : bool  = true
gimbalJoystickEnabled  : bool  = true
joystickSize           : int   = 50
```

- `Main.qml` 的两个 `NavigationBall` 实例的 `visible` 和 `sizeLevel` 直接绑定这三个属性
- `SettingsDialog.qml` 中的开关和滑块反向写入这三个属性
- 任意一处修改立即同步到另一处，无需额外信号/槽

---

## 7. 设置面板布局（SettingsDialog 摇杆设置卡片）

卡片内采用 3:3:4 比例横向分配三个区域：

```
[ RowLayout 占比3 ]  [ RowLayout 占比3 ]  [ RowLayout 占比4        ]
  FluToggleSwitch      FluToggleSwitch      FluText + FluSlider
  底盘摇杆              云台摇杆              摇杆大小  [────────]
  Item(fillWidth)      Item(fillWidth)
```

两个 `FluToggleSwitch` 各自放在一个 `RowLayout` 容器内，容器承担宽度比例，开关本身不设 `fillWidth`，保持自然宽度（方块紧贴文字），右侧用 `Item { Layout.fillWidth: true }` 填充剩余空间。这样解决了 `FluToggleSwitch` 被拉伸后文字与开关间隔过大的问题。

---

## 8. 关键设计决策

**为什么 DragHandler target=null？**  
如果 `target` 设为 `knob`，QML 会直接移动 `knob` 的位置，与 `knobOffsetX/Y` 绑定产生冲突；设为 `null` 后通过 `centroid.position` 手动计算偏移，完全受控。

**为什么两路摇杆合并成一条 UDP 报文？**  
下位机接收端只需绑定一个端口，无需区分来源，减少套接字开销，也便于原子性地读取两路状态快照。
