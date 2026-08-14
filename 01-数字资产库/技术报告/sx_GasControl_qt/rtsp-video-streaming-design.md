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


| 维度           | 取值                                                              | 职责                               |
| ------------ | --------------------------------------------------------------- | -------------------------------- |
| `_preferred` | `sub` / `main`                                                  | 用户选定的清晰度（720p=sub / 4K=main）     |
| `_stream`    | `sub` / `main`                                                  | 本次实际尝试的码流（回退时可临时偏离 `_preferred`） |
| `_conn`      | `idle`/`connecting`/`playing`/`reconnecting`/`stopped`/`failed` | 连接生命周期                           |
| `_fallback`  | bool                                                            | 本次失败是否自动改连另一路                    |


**拆分** `_preferred` **与** `_stream` **的动机**：原来只有 `_stream`，回退时被覆写，
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

1. `FluStatusLayout` **默认子项容器** `visible: statusMode===Success`：
  `VideoOutput` 与控制栏只在 Success 态可见。故 `idle`/`stopped` 也置 `Success`，
   让「空画面 + 控制栏」常显。
2. `TabContentBase` **顶层** `preventStealing` **MouseArea 会吞掉控制栏点击**：
  通过 `controlBarHovered` 别名暴露悬停态，由外层 Tab 在悬停时放行顶层拦截。



### 3.2 按钮清单


| 控件    | 类型                             | 行为                                                                       |
| ----- | ------------------------------ | ------------------------------------------------------------------------ |
| 清晰度   | `FluDropDownButton`            | 720p→`selectStream("sub")`，4K→`selectStream("main")`；按钮文案跟随 `_preferred` |
| 播放/暂停 | `FluIconButton` + `FluTooltip` | 取流中显示「暂停」(`Pause`)，否则「播放」(`Play`)；悬浮提示当前连接态                              |
| 刷新    | `FluIconButton`                | `connectAuto()`                                                          |
| 截图    | `FluIconButton`                | `grabToImage().saveToFile()`                                             |
| 打开    | `FluIconButton`                | `Qt.openUrlExternally(screenshotDirUrl())`                               |


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


| 现象                         | 根因                                                                                                     |
| -------------------------- | ------------------------------------------------------------------------------------------------------ |
| `method PAUSE failed: 551` | **非** `pause()`/`stop()` 所发，是 FFmpeg 在 UDP RTP 断流后试图 `seek(0)` 自救（RTSP 里 seek≈PAUSE），相机不支持 PAUSE 回 551 |
| 点「暂停」卡死 10+ 秒              | `MediaPlayer.stop()` 在 RTSP 会话上是**同步拆流**，卡在 `UDP timeout, retrying with TCP` 的 ~20s 超时等待上（GUI 线程）      |
| 反复 `Demuxing failed`       | WSL 的 NAT 网络收不到 LAN 相机的 UDP RTP 包                                                                      |




### 6.2 为什么不引入 GStreamer / 自建 FFmpeg 管线

- Qt 6.8/6.10 官方「Advanced FFmpeg Configuration」列出的全部 `QT_FFMPEG_*` 入口，
**无一能设** `rtsp_transport=tcp`；QML `MediaPlayer` 也无对应属性 → QML 方案无法强制 TCP。
- 强制 TCP 需切 C++ GStreamer 后端或自建 FFmpeg 管线，改动大。
- **生产是真机 Linux，UDP 大概率正常**：真机上不会断流自救、不会 551、`stop()` 也基本秒回。
为 WSL 假象污染生产架构不划算。



### 6.3 务实规避：默认手动播放

开机进入 `idle`（空画面 + 控制栏可见），`statusMode` 初始 `Success`，
不在 `Component.onCompleted` 自动 `connectAuto`，需用户点「播放」才起流。
从根上避免相机不可达时的开机重连风暴。

## 7. 变更文件清单


| 文件                                                  | 变更                                      |
| --------------------------------------------------- | --------------------------------------- |
| `src/mainwindow/video/RtspVideoView.qml`            | 状态机重构、控制栏、退避重连、稳定门、默认手动播放               |
| `src/viewmodels/mediaVM/mediastorage.h/.cpp`        | 新增截图存储 C++ 单例                           |
| `src/mainwindow/tabs/{Main,Left,Right}VideoTab.qml` | `overlayInteractionEnabled` 联动控制栏 hover |


> 构建提示：新增 C++ 源目录后需删 `build/` 触发 CMake 重新 configure，
> 否则 `file(GLOB)` 缓存不识别新文件。纯 QML 改动则重启应用即生效（Debug 走源码重定向）。



## 8. 遗留与后续

- **强制 TCP**：若真机某些相机 UDP 端口被防火墙挡，仍需 TCP。届时再评估切 GStreamer 后端
（`QT_MEDIA_BACKEND=gstreamer` + `rtspsrc protocols=tcp`），当前不做。
- `stop()` **同步阻塞**：Qt FFmpeg 后端固有行为，QML 层无法异步化，真机网络抖动时仍可能短时卡顿，
只能通过「尽量少调 `stop()`」规避。

---



## 9. 真机调试补丁：RTSP 会话复用导致 PAUSE/seek 失败

> 本节记录真机上线调试时发现的第三处缺陷及修复方法。



### 9.1 现象

真机首次播放正常；点「暂停」再点「播放」后，控制台出现：

```
[rtsp @ 0x...] method PAUSE failed: 551 Option not supported
qt.multimedia.ffmpeg.demuxer: Failed to seek, pos 0
[rtsp @ 0x...] RTP: missed 503 packets
```

随后长时间黑屏（等 IDR 帧），数秒乃至数十秒后才出画，且出画后持续出现：

```
[rtsp @ 0x...] max delay reached. need to consume packet
[rtsp @ 0x...] RTP: missed 3281 packets
[h264 @ 0x...] reference picture missing during reorder
```



### 9.2 根因

`_startStream` 中 `player.source` 赋值时，若 URL 与上次相同，
**Qt6 FFmpeg 后端不会销毁旧 demuxer**，而是复用已有 RTSP 会话，随后向相机发送
`PAUSE`（seek 语义）再 `PLAY`。该相机不支持 PAUSE（回 551），
demuxer 降级为被动重同步，只能等下一个 IDR 关键帧才能解码，
导致画面延迟很长时间才出现，且期间积压大量乱序 RTP 包。

```
stop() → source = 相同URL → play()
  └→ FFmpeg 复用旧 RTSP 会话
       └→ 发 PAUSE → 551
       └→ seek(0) → Failed to seek
            └→ 被动重同步 → 等 IDR → 画面出现很晚
```



### 9.3 修复

在 `_startStream` 中 `player.stop()` 之后，先将 `source` 清空，
强制 Qt 完全释放旧 demuxer，再赋新 URL：

```qml
player.stop()
player.source = ""   // 强制释放旧 RTSP 会话，避免 FFmpeg 复用并发 PAUSE/seek
player.source = _buildUrl(stream)
player.play()
```

`source = ""` 触发 `mediaStatus → NoMedia`，在 `onMediaStatusChanged` 中已被
显式排除（注释：NoMedia 是 stop() 后的正常中间态，绝不能当失败），不会误触 `_onFailed`。

修复覆盖范围：`_startStream` 是唯一起流入口（`play` / `refresh` / `selectStream` /
退避重连 `_reconnectNow` 全部经过它），一处修改覆盖所有「再次播放/切流/刷新」路径。

---



## 10. OEM 相机 RTSP 路径调查与 ONVIF 探测方法



### 10.1 问题发现

真机调试时观察到：无论取 `Channels/101`、`Channels/102` 还是其他编号，
`ffprobe` 均返回 `3840x2160 h264`，子码流一直以 4K 输出。
而相机 web 控制台确认子码流已配置为 720p 且设置已生效。

验证命令（在具备局域网连通性的机器上执行）：

```bash
ffprobe -v quiet -select_streams v:0 \
  -show_entries stream=width,height,codec_name \
  -of default=noprint_wrappers=1 \
  "rtsp://admin:123456@192.168.1.123:554/Streaming/Channels/102"
# 输出: codec_name=h264 / width=3840 / height=2160  ← 应为子码流 720p，实为 4K
```



### 10.2 根因：相机不遵循海康 Channels/1xx 路径约定

这台 `icamra ipc`（OEM 相机）将所有 `/Streaming/Channels/*` 路径全部解析到同一路主码流。
`Channels/102`（代码中配置的子码流路径）在这台相机上无效，因此子码流从未被正确请求到。

### 10.3 ONVIF 探测方法

通过 ONVIF `GetStreamUri` 让相机自报真实的码流 URI，是最权威、不依赖路径格式猜测的方式。

**依赖安装：**

```bash
pip install onvif-zeep
```

**探测脚本**（已落地到 `tools/probe_onvif_streams.py`）：

```bash
python3 tools/probe_onvif_streams.py <相机IP> <用户名> <密码>
# 示例：
python3 tools/probe_onvif_streams.py 192.168.1.123 admin 123456
```

```py
#!/usr/bin/env python3
# 查询相机 ONVIF Media 服务，打印它注册的每条码流的分辨率与真实 RTSP URI。
# 用途：定位 OEM 相机真正的子码流(720p)地址，海康式 /Streaming/Channels/102 对这台相机无效。
#
# 依赖： pip install onvif-zeep
# 用法： python3 tools/probe_onvif_streams.py 192.168.1.123 admin 123456

import sys

try:
    from onvif import ONVIFCamera
except ImportError:
    sys.exit("缺少依赖，请先运行：pip install onvif-zeep")


def probe(host, user, pwd):
    # 不同 OEM 的 ONVIF 服务端口不一，逐个尝试直到连上。
    candidate_ports = [80, 8000, 2020, 8080, 8899, 8999]
    last_err = None
    for port in candidate_ports:
        try:
            cam = ONVIFCamera(host, port, user, pwd)
            media = cam.create_media_service()
            profiles = media.GetProfiles()
        except Exception as e:  # 端口不对/超时/认证失败都在此吞掉，换下一个端口
            last_err = e
            print(f"[端口 {port}] 连接失败：{e}")
            continue

        print(f"\n[端口 {port}] 连接成功，相机声明了 {len(profiles)} 条码流：\n")
        for p in profiles:
            enc = getattr(p, "VideoEncoderConfiguration", None)
            if enc is None:
                continue
            res = enc.Resolution
            uri = media.GetStreamUri({
                "StreamSetup": {"Stream": "RTP-Unicast",
                                "Transport": {"Protocol": "RTSP"}},
                "ProfileToken": p.token,
            }).Uri
            print(f"  名称={p.Name:<16} 分辨率={res.Width}x{res.Height} "
                  f"编码={enc.Encoding}")
            print(f"       RTSP URI = {uri}\n")
        print("提示：分辨率为 1280x720（或更小）的那条，其 RTSP URI 即为子码流地址。")
        print("      把该 URI 的路径部分填入 RtspVideoView 的 subPath 即可。")
        return

    sys.exit(f"\n所有候选端口都连不上，最后错误：{last_err}\n"
             "请确认相机 ONVIF 已开启，或从 web 后台查其 ONVIF 端口。")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit("用法： python3 tools/probe_onvif_streams.py <相机IP> <用户名> <密码>")
    probe(sys.argv[1], sys.argv[2], sys.argv[3])
```

脚本自动遍历常见 ONVIF 端口（80 / 8000 / 2020 / 8080 / 8899 / 8999），
连接成功后调用 `media.GetProfiles()` + `media.GetStreamUri()`，
打印每条码流的分辨率与 RTSP URI。

### 10.4 实测结果（192.168.1.123，icamra ipc）

```
[端口 80] 连接成功，相机声明了 2 条码流：

  名称=ProfileName_1    分辨率=3840x2160 编码=H264
       RTSP URI = rtsp://192.168.1.123:554/stream0?username=admin&password=<MD5>

  名称=ProfileName_2    分辨率=720x480   编码=H264
       RTSP URI = rtsp://192.168.1.123:554/stream1?username=admin&password=<MD5>
```

- 主码流路径：`/stream0`（3840×2160，4K H.264）
- 子码流路径：`/stream1`（720×480，标清 H.264）

注意：子码流实际为 `720×480`（4:3），非标准 `1280×720`（16:9 720p）。

### 10.5 代码修正

将 `RtspVideoView.qml` 默认路径从海康式路径改为实测路径：

```qml
// 旧（海康约定，此相机无效）
property string mainPath: "/Streaming/Channels/101"
property string subPath:  "/Streaming/Channels/102"

// 新（icamra ipc ONVIF 实测路径）
property string mainPath: "/stream0"
property string subPath:  "/stream1"
```

三个 Tab（`Main/Left/RightVideoTab.qml`）均未覆写这两个属性，
改默认值即对所有三路视频生效。换用不同品牌相机时，
按其 ONVIF `GetStreamUri` 声明的路径部分覆写对应 Tab 的 `mainPath` / `subPath` 即可。

---



## 11. 变更文件清单（补充）


| 文件                                       | 变更                                                                                           |
| ---------------------------------------- | -------------------------------------------------------------------------------------------- |
| `src/mainwindow/video/RtspVideoView.qml` | `_startStream` 加 `source=""` 修复会话复用；`mainPath`/`subPath` 改为 OEM 实测路径；截图/播放提示改用 `HoveringCue` |
| `tools/probe_onvif_streams.py`           | 新增：ONVIF 多端口探测脚本，打印各码流分辨率与 RTSP URI                                                          |


---



## 12. 端到端延迟优化：调查报告与结论



### 12.1 问题描述

真机上线后观察到无论主/子码流均存在约 **3 秒**的端到端延迟（从相机采集到屏幕出画）。

### 12.2 延迟来源分解

RTSP 端到端延迟由三层叠加：

```
相机编码延迟（等 IDR/GOP）
  + FFmpeg RTP jitter buffer（max_delay，默认 500ms）
  + Qt 解码/渲染缓冲（~200ms，固定）
```

初始 3 秒的主要来源是**相机 GOP 间隔过大**——出厂默认帧率 20fps，GOP=60+ 帧（约 3s），
播放器首帧和断流恢复都必须等下一个 IDR 关键帧，导致明显的启动/恢复延迟。

### 12.3 第一阶段：相机端优化（已完成，效果显著）

调整相机 web 后台编码参数：


| 参数        | 调整前             | 调整后                         |
| --------- | --------------- | --------------------------- |
| 主码流 I 帧间隔 | 大值（出厂默认，约 60 帧） | **20 帧**（@20fps → GOP=1s）   |
| 子码流 I 帧间隔 | 大值              | **20 帧**（@25fps → GOP=0.8s） |


> **注意**：参数单位须确认为「帧」而非「秒」（改后延迟明显下降即为帧，反之改为 1~2 即可）。

结果：延迟从 ~3s 降至 **~1s**，效果显著，无需修改代码。

### 12.4 第二阶段：播放器端优化（调查后结论为无法继续压缩）

剩余延迟来自 FFmpeg jitter buffer（默认 500ms）和 Qt 渲染缓冲。逐一核查可用入口：

**方案 A：Qt FFmpeg 环境变量**

查阅 Qt 6.10.3 官方《Advanced FFmpeg Configuration》文档，其 FFmpeg 后端环境变量
入口仅覆盖日志、实验编解码、协议白名单、硬件加速四类，
**不存在任何用于设置** `max_delay`**/**`fflags`**/RTSP 缓冲的变量**。
`QT_FFMPEG_AVFORMAT_MAX_DELAY` 此类变量名在 Qt 6.10 中不存在，实测无效。结论：**此路不通**。

**方案 B：RTSP URL 附加 FFmpeg avformat option（**`fflags=nobuffer`**）**

Qt 的 FFmpeg 后端将 `source` URL 整体传给 `avformat_open_input` 作为 filename，
**不解析 URL query 字符串成** `AVDictionary`。
`fflags`/`max_delay` 这类必须经 `AVDictionary` 传入的选项，Qt 没有任何暴露的注入入口。
在 URL 里附加 `?fflags=nobuffer` 只会作为 RTSP 路径的一部分发给相机，可能导致取流失败。
结论：**此路不通**。

**方案 C：切 GStreamer 后端**

GStreamer 的 `rtspsrc` 支持 `latency=0 drop-on-latency=true`，理论延迟可达 200ms 以内。
但存在两个障碍：

1. **运行时依赖缺失**：本机 GStreamer 装了基础框架和 `rtspsrc`（good），
  但 H.264 解码所需的 `gstreamer1.0-libav`（含 `avdec_h264`）尚未安装。
2. **Qt 后端插件缺失**：官方 Qt 6.10.3 桌面安装包默认**不编译 GStreamer 媒体后端插件**
  （官方仅为 Embedded Linux/Boot2Qt 维护 GStreamer 后端，桌面 Linux 默认 FFmpeg）。
   要在桌面 Linux 上用 GStreamer 后端，需从源码重编 Qt Multimedia，改动成本极高。

结论：**成本远超收益，不推荐**。

### 12.5 结论与现状评估（已更新）


| 优化阶段 | 手段                                         | 延迟                             |
| ---- | ------------------------------------------ | ------------------------------ |
| 初始状态 | 出厂大 GOP + RTSP 会话复用 bug                    | ~3s                            |
| 已完成① | 相机 GOP→1s + `source=""` 修复                 | **~1s**                        |
| 方案 A | Qt 环境变量                                    | Qt 6.10 无此入口，**无效**            |
| 方案 B | URL 附加 fflags                              | Qt 不透传 avformat option，**不可行** |
| 已完成② | C++ GStreamer pipeline + `QVideoSink`（§13） | **预期 ~150-300ms**              |


方案 C（GStreamer 后端）**已实施**，详见 §13。
`gstreamer1.0-libav` 安装后 `avdec_h265` 可用，不需重编 Qt Multimedia，§12.4 的"成本过高"结论已失效。

---



## 13. C++ GStreamer 管线 + QVideoSink 实现

> 状态：已实施。§12.4 方案 C 判定"需重编 Qt Multimedia"的前提被推翻，故重新立项落地。



### 13.1 §12.4 结论失效的原因

§12.4 把方案 C 否掉，依据是「桌面 Qt 不带 GStreamer **媒体后端插件**，要用得重编 Qt Multimedia」。
这个判断混淆了两条完全不同的路径：

```text
路径①（§12.4 评估的，确实成本高）：
    换掉 Qt Multimedia 的后端实现
    QML MediaPlayer → [GStreamer 后端插件] → 相机
    需要 Qt 编译期带 GStreamer 后端 → 桌面版没有 → 重编 Qt

路径②（本次采用，无需碰 Qt）：
    不用 MediaPlayer，自己当生产者
    C++ 直接 link libgstreamer → 解码帧 → QVideoSink → VideoOutput 渲染
    只用到 Qt Multimedia 的「帧接收」能力（QVideoSink/QVideoFrame），
    这部分桌面版本来就有，与后端插件无关
```

关键差别：路径②里 GStreamer **不是 Qt 的后端**，而是与 Qt 平级的独立管线，
两者只在 `QVideoSink` 这一个接口处对接。Qt 侧只负责「把交给它的帧画出来」。

### 13.2 数据流与所有权（本次最大的坑）

`QVideoSink` 归**渲染端**所有，生产者只是往里写。方向反了会直接起不来：

```text
✅ 正确
   VideoOutput ──拥有──▶ QVideoSink
        │                    ▲
        │                    │ setVideoFrame() 逐帧写入
        │ Component.onCompleted 交出引用
        ▼                    │
   gstSink.videoSink = videoOutput.videoSink
                             │
                    GStreamer 流线程

❌ 错误（首个实现版本）
   RtspGstSink ──new──▶ QVideoSink ──赋给──▶ VideoOutput.videoSink
                                              ↑ 只读属性，QML 引擎直接拒绝
```

写反的实际后果不是「画面不出」，而是**整棵组件树加载失败**、程序起不来：

```text
RtspVideoView.qml:331: Invalid property assignment:
                       "videoSink" is a read-only property
  └─ MainVideoTab.qml:13: Type RtspVideoView unavailable
       └─ MainWinFunction.qml:78: Type MainVideoTab unavailable
```

`VideoOutput.videoSink` 只读是有意设计：它对外暴露自己内部的 sink 供任意生产者写帧，
而不接受外部替换渲染目标。凡是「往 Qt 里灌帧」的场景，都是这个方向。

### 13.3 `RtspGstSink` 对 QML 的接口面

`src/viewmodels/mediaVM/rtspgstsink.{h,cpp}`，`QML_ELEMENT`（**非** Singleton——
主/左/右多个视频视图各自独立取流，pipeline 生命周期跟随各自的 Loader）。

```cpp
Q_PROPERTY(QVideoSink *videoSink READ videoSink WRITE setVideoSink NOTIFY videoSinkChanged)
Q_PROPERTY(bool        playing   READ playing                      NOTIFY playingChanged)

Q_INVOKABLE void play(const QString &url);   // 重复调用 = 切流（先拆旧再建新）
Q_INVOKABLE void stop();                     // 拆到 GST_STATE_NULL

signals:
    void errorOccurred(const QString &message);  // bus ERROR / EOS / 启动失败，统一出口
```

状态刻意收敛成 **首帧到达 →** `playing` 与 **失败 →** `errorOccurred` 两个事件，
不把 GStreamer 内部相位（NULL/READY/PAUSED/PLAYING）泄漏到 UI 层。
外层 `RtspVideoView` 既有的连接状态机（connecting/playing/reconnecting/failed、
看门狗、指数退避、稳定门）**逻辑一行未改**，只是信号来源从 `MediaPlayer` 换成了它。

### 13.4 低延迟 pipeline 参数

```text
rtspsrc location=<url> latency=0 protocols=4 drop-on-latency=true
  ! rtph265depay ! avdec_h265
  ! videoconvert ! video/x-raw,format=NV12
  ! appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false
```


| 参数                        | 作用                                                 |
| ------------------------- | -------------------------------------------------- |
| `latency=0`               | 关掉 jitter buffer——这正是 Qt FFmpeg 后端不给改的那部分          |
| `protocols=4`             | 仅 TCP（`GST_RTSP_LOWER_TRANS_TCP`），免掉 UDP 超时回退的额外等待 |
| `drop-on-latency=true`    | 缓冲超限直接丢帧，不让延迟累积                                    |
| `max-buffers=1 drop=true` | appsink 只保最新一帧                                     |
| `sync=false`              | 不按时间戳节流，帧到即取（实时流不需要回放同步）                           |


延迟下界由此变成「网络 RTT + 一到两帧解码时间」，而非固定的数百毫秒缓冲。

### 13.5 跨线程约定

GStreamer 回调在自己的流线程，Qt 对象在主线程，三处需要处理：


| 位置                                 | 处理                                                            |
| ---------------------------------- | ------------------------------------------------------------- |
| `m_videoSink`                      | `std::atomic<QVideoSink*>`——QML 线程写、流线程读                      |
| `setVideoFrame()`                  | Qt 6 内部有锁，可直接从流线程调；sink 未接上时早退丢帧，不崩                           |
| `errorOccurred` / `playingChanged` | `QMetaObject::invokeMethod(..., Qt::QueuedConnection)` 投递回主线程 |


帧拷贝按 `GST_VIDEO_INFO_PLANE_STRIDE/OFFSET` 逐行搬，不假设 GStreamer 与 Qt 的
行距一致（NV12 两个平面：Y 全高、UV 半高）。

### 13.6 构建依赖

```cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET
    gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

target_link_libraries(viewmodels PUBLIC Qt6::Multimedia PkgConfig::GST)
```

运行时需 `gstreamer1.0-libav`（提供 `avdec_h265`/`avdec_h264`）。
本机原先只有 `rtspsrc`/`rtph26xdepay`/`appsink`，缺解码器，这是唯一新增的系统包。

落地前先用 `gst-launch-1.0` 验证同一条 pipeline 能出画、延迟达标，
再动 C++——避免写完才发现相机编码格式与 depay/decoder 不匹配。

### 13.7 变更文件清单


| 文件                                       | 改动                                                                    |
| ---------------------------------------- | --------------------------------------------------------------------- |
| `src/viewmodels/mediaVM/rtspgstsink.h`   | 新增，接口面与跨线程成员声明                                                        |
| `src/viewmodels/mediaVM/rtspgstsink.cpp` | 新增，pipeline 构建/拆除、appsink 取帧、bus 错误转发                                 |
| `src/viewmodels/CMakeLists.txt`          | 加 `PkgConfig::GST` + `Qt6::Multimedia`                                |
| `src/mainwindow/video/RtspVideoView.qml` | `MediaPlayer` → `RtspGstSink`；`Component.onCompleted` 交接 sink；状态机逻辑不变 |


`grabToImage()` 截图不受影响——它抓的是 `VideoOutput` 的渲染结果，与帧来源无关。

### 13.8 待验证

- [ ] 实测延迟是否落在 150-300ms（对比 `ffplay` 低延迟模式基准）
- [ ] 4K 主码流下 NV12 逐行拷贝的 CPU 占用
- [ ] 断流自愈：拔网线后退避重连是否仍按预期收敛
- [ ] 若 `latency=0` 被相机侧协商覆盖，用 `GST_DEBUG=rtspsrc:4` 确认实际生效值