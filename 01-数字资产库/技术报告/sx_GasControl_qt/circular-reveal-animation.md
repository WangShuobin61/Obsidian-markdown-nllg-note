# CircularReveal 圆形涟漪主题切换动画技术报告

## 一、功能定位

`CircularReveal` 是 `gas_control_bringup` 项目主题切换场景下的过渡动画组件。当用户触发亮/暗主题切换时，它在旧主题画面上叠加一个从指定圆心扩散（或收缩）的圆形遮罩，将旧界面平滑"擦除"，露出底层已切换完毕的新主题。整个过程对用户呈现为流畅的涟漪扩散动效，视觉上比直接重绘更具仪式感。

---

## 二、核心设计思路

### 2.1 "截图-覆盖-擦除"三步模型

直接对 Qt Quick 场景树做动画存在一个根本矛盾：主题颜色是全局状态，一旦切换所有控件立刻刷新，没有中间状态可以插入动画。`CircularReveal` 绕开了这个矛盾，整体思路如下：

```
① 截图（grabToImage）
     ↓
② 覆盖：把截图贴满屏幕最上层，遮住真实场景
     ↓
③ 底层切换真实主题色（此时被截图挡住，用户看不见）
     ↓
④ 动画擦除截图：以圆形路径不断增大/减小，清除截图像素
     ↓
⑤ 截图全部擦除，新主题完整呈现
```

这样主题的实际切换发生在步骤 ③，完全是瞬间完成的；用户感知到的"渐变"只是步骤 ④ 的画面擦除过程。

### 2.2 `QQuickPaintedItem` + `CompositionMode_Clear`

组件继承自 `QQuickPaintedItem`，可以直接操控 `QPainter`。擦除算法的关键在于合成模式：

```cpp
painter->setCompositionMode(QPainter::CompositionMode_Clear);
```

`CompositionMode_Clear` 将目标区域的所有像素（含 Alpha 通道）清零，使其完全透明，从而"露出"下层的真实 QML 场景。这比用背景色填充更精确——无论新主题是深色还是浅色，底层的真实颜色都能正确透出。

### 2.3 双方向擦除逻辑

组件通过 `darkToLight` 属性区分两种切换方向：

| 方向 | 动画起止 | 擦除区域 |
|------|----------|----------|
| 亮→暗（`darkToLight = false`） | 半径 0 → 最大 | 圆形内部（逐步扩大挖洞） |
| 暗→亮（`darkToLight = true`） | 半径 最大 → 0 | 圆形外部（逐步填回，圆心处先透出） |

```cpp
if (m_darkToLight) {
    // 圆形路径区域清除，圆内先透出新主题
    painter->fillPath(path, Qt::white);
} else {
    // 矩形减去圆形 = 圆外区域，圆外先透出新主题
    QPainterPath outerRect;
    outerRect.addRect(0, 0, width(), height());
    outerRect = outerRect.subtracted(path);
    painter->fillPath(outerRect, Qt::black);
}
```

两种方向在视觉上产生完全对称的效果：一个是从内向外扩散，一个是从外向内收缩。

### 2.4 圆心跟随用户操作点

`start()` 方法接收 `center` 参数，圆形涟漪的圆心可以设置为用户点击切换按钮的位置。这让动画感知"从哪里触发"，大幅提升交互的空间一致性，是从主题切换按钮所在的屏幕坐标传入的。

### 2.5 打断重入处理

动画进行中再次触发切换时，会立即停止当前动画并以当前半径为起点重新启动：

```cpp
if (m_anim->state() == QAbstractAnimation::Running) {
    m_anim->stop();
    m_anim->setStartValue(m_radius);   // 从当前中间态开始
    m_anim->setEndValue(m_darkToLight ? 0 : radius);
}
```

避免了重入导致的画面跳变，同时也不会丢弃用户的第二次操作。

---

## 三、数据与状态流转

```
QML 触发切换按钮
        │
        ▼
start(w, h, center, radius)
        │
        ├─ m_target->grabToImage(QSize(w, h))   // 异步截图
        │
        └─→ handleGrabResult() [槽回调]
                │
                ├─ image.swap(m_source)          // 保存截图
                ├─ setVisible(true)              // 显示遮罩层
                ├─ Q_EMIT imageChanged()         // QML 侧可响应
                └─ m_anim->start()              // 启动半径动画
                        │
                        ▼ [每帧]
                    setRadius(n) → radiusChanged → update() → paint()
                        │
                        ▼ [完成]
                    setVisible(false) + Q_EMIT animationFinished()
```

截图是异步的（`grabToImage` 异步返回 `QSharedPointer<QQuickItemGrabResult>`），因此动画实际开始时间是截图完成后，而不是调用 `start()` 的瞬间。这段延迟通常不到一帧，实测不可感知。

---

## 四、工程细节

### 4.1 尺寸需传入 devicePixelRatio 倍数

`start()` 的 `w` 和 `h` 参数需要由调用方传入乘以 `devicePixelRatio` 之后的像素数，保证在高分屏下截图清晰度与实际分辨率一致。

### 4.2 动画曲线与时长

```cpp
m_anim->setDuration(333);                        // 333ms
m_anim->setEasingCurve(QEasingCurve::OutCubic); // 末端减速
```

`OutCubic` 使半径变化在前段快、末段慢，符合自然物理运动感知，避免动画结束时的生硬停顿。

### 4.3 内存安全

截图结果通过 `QSharedPointer<QQuickItemGrabResult>` 持有，确保在 `handleGrabResult` 回调触发前不会提前释放。`image.swap()` 后旧截图内存立即归还。

### 4.4 QML 侧注册

组件使用 Qt6 的声明式注册方式，仅需在头文件中添加 `QML_ELEMENT`，无需在 `main.cpp` 手动调用任何注册函数，由 `qt_add_qml_module` 在构建时自动处理类型注册。

---

## 五、小结

`CircularReveal` 的设计核心是一个工程上的"障眼法"：利用截图盖住瞬时的全局状态刷新，再用圆形遮罩的几何动画制造渐变假象。整个方案没有入侵 Qt Quick 场景树的绘制流程，对业务代码零侵入，可作为独立组件复用于任何需要过渡动效的主题切换场景。