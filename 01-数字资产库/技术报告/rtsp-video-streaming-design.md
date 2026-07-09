# RTSP 视频视图设计与调试报告

## 1. 背景与目标

`RtspVideoView.qml` 是全项目视频画面的基类，主/左/右三路视频 Tab 均复用它。
本轮工作在原有「主/子码流三态封装」基础上，完成以下需求：

1. 默认改为**子码流优先**（更轻、建链快），失败回退主码流。
2. 左下角新增**悬浮控制栏**（`FluGroupBox` 横排 `FluIconButton`）：清晰度、播放/暂停、刷新、截图、打开存储位置。
3. 截图落盘到与 `app.db` 同级的 `screenshot` 目录。
4. 修复断流重连缺陷（无退避、假成功重置计数）。
5. 应对 WSL 开发环境下 RTSP-over-UDP 不通导致的卡顿与画面横跳。

> 关键结论先行：**生产部署在真机 Linux，WSL 仅为开发环境**。
> 因此涉及 UDP 不通的症状（551 循环、`stop()` 卡顿）不为其污染生产架构，
> 只修复真机也存在的逻辑缺陷，并提供「默认手动播放」作为 WSL 下的务实规避。

## 2. 状态机设计

原实现把「码流选择」与「连接生命周期」揉进单一 `_phase`，
新增手动切换、暂停、刷新后暴露出复杂度扩散。故拆成正交维度：

| 维度 | 取值 | 职责 |
|------|------|------|
| `_preferred` | `sub` / `main` | 用户选定的清晰度（720p=sub / 4K=main） |
| `_stream` | `sub` / `main` | 本次实际尝试的码流（回退时可临时偏离 `_preferred`） |
| `_conn` | `idle`/`connecting`/`playing`/`reconnecting`/`stopped`/`failed` | 连接生命周期 |
| `_fallback` | bool | 本次失败是否自动改连另一路 |

**拆分 `_preferred` 与 `_stream` 的动机**：原来只有 `_stream`，回退时被覆写，
导致「下拉框显示的清晰度」和「重连起点」都被回退污染。分开后，
下拉框稳定反映用户意图，重连始终从 `_preferred` 起，职责清晰。

### 2.1 状态流转

```
                    ┌─────────────────────────── 用户点「播放」/「刷新」
                    ▼
  idle ──play()──► connecting ──首帧 Buffered──► playing
   ▲                  │  ▲                         │
   │ (开机默认)        │  │ 回退另一路(_fallback)     │ 中途断流
   │                  ▼  │                         ▼
 stopped ◄─暂停── 任意取流态          reconnecting ◄──┐ 指数退避
                                          │           │ base·2^(n-1) 封顶
                                          └─到点重连──┘ 超 maxReconnects → failed
```

- **首次连接**（开机/刷新/用户重连）失败：子→主都败且 `attempts==0` → 直接 `Error`，
  保留「用户手动重连」的既有 UX。
- **出画后断流**：进入 `reconnecting` 做退避，而非立即 `stop→play`（避免紧循环）。

## 3. 悬浮控制栏

### 3.1 两个关键约束（决定了实现方式）

1. **`FluStatusLayout` 默认子项容器 `visible: statusMode===Success`**：
   `VideoOutput` 与控制栏只在 Success 态可见。故 `idle`/`stopped` 也置 `Success`，
   让「空画面 + 控制栏」常显。
2. **`TabContentBase` 顶层 `preventStealing` MouseArea 会吞掉控制栏点击**：
   通过 `controlBarHovered` 别名暴露悬停态，由外层 Tab 在悬停时放行顶层拦截。

### 3.2 按钮清单

| 控件 | 类型 | 行为 |
|------|------|------|
| 清晰度 | `FluDropDownButton` | 720p→`selectStream("sub")`，4K→`selectStream("main")`；按钮文案跟随 `_preferred` |
| 播放/暂停 | `FluIconButton` + `FluTooltip` | 取流中显示「暂停」(`Pause`)，否则「播放」(`Play`)；悬浮提示当前连接态 |
| 刷新 | `FluIconButton` | `connectAuto()` |
| 截图 | `FluIconButton` | `grabToImage().saveToFile()` |
| 打开 | `FluIconButton` | `Qt.openUrlExternally(screenshotDirUrl())` |

- 清晰度：取流中选择立即切换（不回退，失败即报错）；idle/stopped 下仅记住选择，待下次播放生效。
- 悬浮提示参照 `TopBatteryStatus` 同款模式：`FluTooltip { visible: hovered; delay: 1000 }`。

## 4. 截图落盘：`MediaStorage` C++ 单例

新增 `src/viewmodels/mediaVM/mediastorage.{h,cpp}`，随 `viewmodels` 模块
以 `QML_ELEMENT + QML_SINGLETON` 暴露。职责边界清晰——**只负责「去哪存/叫什么名/怎么打开」，
不碰帧数据本身**（帧抓取与写文件交给 QML 侧 `grabToImage().saveToFile()`）。

```
~/.local/share/GasControl/           (GenericDataLocation)
├── app.db                           (AppDatabase)
└── screenshot/                      (MediaStorage，首次访问即创建)
    └── shot_20260709_153012_874.png
```

- `screenshotDir()` / `screenshotDirUrl()` / `newScreenshotPath()` 三个 `Q_INVOKABLE`。
- `grabToImage` 抓的是 `videoOutput` 渲染内容，控制栏是其兄弟项、不入画，截图干净。

## 5. 断流重连缺陷修复

### 5.1 缺陷一：无退避的紧循环（真机也存在）

原 `_onFailed()` 的 `playing` 分支直接 `connectAuto()` → 内部 `stop()+play()`，
无延迟、无上限。真机偶发断流即退化为「断流→stop→重连→再断流」紧循环。

**修法**：新增 `reconnecting` 态 + 指数退避（`base·2^(n-1)` 封顶 `reconnectMaxMs`），
连续失败超 `maxReconnects`（5）轮才置 `Error`。参数可配：
`reconnectBaseMs=2000`、`reconnectMaxMs=30000`、`maxReconnects=5`。

### 5.2 缺陷二：假成功重置计数（WSL 下暴露）

`_onSucceeded()` 一进 `BufferedMedia` 就 `_reconnectAttempts=0`。
WSL 下子码流会**短暂进入 Buffered**（日志可见解析出 `Input #0`），随后立刻 demux 失败：

```
断流 → _scheduleReconnect（attempts=1，显示 1/5）
  → 2s 后重连 → 子码流短暂 Buffered → _onSucceeded：计数清零 + Success（空画面闪现）
  → demux 立刻失败 → _scheduleReconnect（attempts 又从 1 开始）
  → 永远在「1/5」和「空画面」间横跳，5 次上限永远到不了
```

这正是用户观察到的三个现象：**空画面与重连争夺容器、一直显示 (1/5)、立刻切回空画面**。

**修法**：加**稳定门** `stableTimer`——出画后不立即清零计数，
需稳定播放 `stableResetMs`（5s）未再断流才认定健康并清零。
假成功不再重置退避，5 次上限得以真正累计生效。

## 6. WSL 环境根因（不为其改生产架构）

### 6.1 因果链（已证实）

| 现象 | 根因 |
|------|------|
| `method PAUSE failed: 551` | **非** `pause()`/`stop()` 所发，是 FFmpeg 在 UDP RTP 断流后试图 `seek(0)` 自救（RTSP 里 seek≈PAUSE），相机不支持 PAUSE 回 551 |
| 点「暂停」卡死 10+ 秒 | `MediaPlayer.stop()` 在 RTSP 会话上是**同步拆流**，卡在 `UDP timeout, retrying with TCP` 的 ~20s 超时等待上（GUI 线程） |
| 反复 `Demuxing failed` | WSL 的 NAT 网络收不到 LAN 相机的 UDP RTP 包 |

### 6.2 为什么不引入 GStreamer / 自建 FFmpeg 管线

- Qt 6.8/6.10 官方「Advanced FFmpeg Configuration」列出的全部 `QT_FFMPEG_*` 入口，
  **无一能设 `rtsp_transport=tcp`**；QML `MediaPlayer` 也无对应属性 → QML 方案无法强制 TCP。
- 强制 TCP 需切 C++ GStreamer 后端或自建 FFmpeg 管线，改动大。
- **生产是真机 Linux，UDP 大概率正常**：真机上不会断流自救、不会 551、`stop()` 也基本秒回。
  为 WSL 假象污染生产架构不划算。

### 6.3 务实规避：默认手动播放

开机进入 `idle`（空画面 + 控制栏可见），`statusMode` 初始 `Success`，
不在 `Component.onCompleted` 自动 `connectAuto`，需用户点「播放」才起流。
从根上避免相机不可达时的开机重连风暴。

## 7. 变更文件清单

| 文件 | 变更 |
|------|------|
| `src/mainwindow/video/RtspVideoView.qml` | 状态机重构、控制栏、退避重连、稳定门、默认手动播放 |
| `src/viewmodels/mediaVM/mediastorage.h/.cpp` | 新增截图存储 C++ 单例 |
| `src/mainwindow/tabs/{Main,Left,Right}VideoTab.qml` | `overlayInteractionEnabled` 联动控制栏 hover |

> 构建提示：新增 C++ 源目录后需删 `build/` 触发 CMake 重新 configure，
> 否则 `file(GLOB)` 缓存不识别新文件。纯 QML 改动则重启应用即生效（Debug 走源码重定向）。

## 8. 遗留与后续

- **强制 TCP**：若真机某些相机 UDP 端口被防火墙挡，仍需 TCP。届时再评估切 GStreamer 后端
  （`QT_MEDIA_BACKEND=gstreamer` + `rtspsrc protocols=tcp`），当前不做。
- **`stop()` 同步阻塞**：Qt FFmpeg 后端固有行为，QML 层无法异步化，真机网络抖动时仍可能短时卡顿，
  只能通过「尽量少调 `stop()`」规避。