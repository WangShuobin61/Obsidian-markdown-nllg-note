# WSLg 视频不出画问题分析与交接（给 WSL 端 agent）

> 交接背景：该问题在 Ubuntu 真机开发环境（原生终端，agent 在此侧）分析并写下修复；
> 用户切回 Windows/WSL 后本对话不可见，故留此文档供 WSL 端 agent 继续验证。
> 分支：`feature/WSLg`。改动文件：`src/app/main.cpp`（+本文档）。

> **【WSL 端实测更正 · 结论已推翻原假设】** 
> WSL 端实测：`QT_QUICK_BACKEND=`（空，走默认 GL）能出画；直接运行（触发 software 注入）不能出画。
> 因此 §2/§3 的因果**反了**——真相是：**默认 GL 后端在 WSL 与真机都能出画，
> 而 `QT_QUICK_BACKEND=software` 注入才是空窗元凶**（FluentUI 的亚克力/模糊等 ShaderEffect 在软件后端不渲染）。
> 补充证据：①`qInfo` 那行本就正常打印，检测从未失效；②未登录时 `RtspVideoView` 不在渲染树，
> "开机即可见 VideoOutput" 不成立；③强制默认 GL 时场景图日志显示窗口已 expose、QRhi OpenGL 已建、渲染线程在跑。
> **最终修复：删除 `main.cpp` 中的 WSL→software 注入段，两个环境统一走 Qt 默认 GL，无需任何环境判断。**
> 下方 §2/§3 保留原文仅作历史记录，勿再据其调整"检测方式"。

## 1. 现象

- 环境：Windows + WSL（WSLg 图形），Qt 6.10.3 Debug 构建。
- 编译通过，运行后任务栏有窗口，但**窗口画面不出现**（只有标题栏/空窗）。
- 终端输出停在这一行之后就没有后续，程序界面也不显示：

  ```
  qt.multimedia.ffmpeg: Using Qt multimedia with FFmpeg version 7.1.3 LGPL version 2.1 or later
  ```

- 对照：真机 Ubuntu 同一份代码可正常出画，终端在上面那行之后还会多出两条**无害**警告：

  ```
  qt.qpa.theme.gnome: dbus reply error: [ "org.freedesktop.DBus.Error.ServiceUnknown" ]
    "The name org.freedesktop.portal.Desktop was not provided by any .service files"
  ```

  这两条只是 GNOME 主题探测走 DBus portal 失败后回退默认主题，与出画无关，真机照常显示。
  **真机的 dbus 警告与 WSL 的不出画是两个独立现象，无因果关系。**

## 2. 根因分析（基于 git 历史对比）

已知 good commit `e49051e`（WSL 能出画）与当前 `92af4fc`（WSL 不出画）之间，
`RtspVideoView.qml` 的**开机初始态**发生了关键变化：

| 维度 | `e49051e`（能出画） | 当前（不出画） |
|------|------|------|
| 初始 `statusMode` | `Loading` | `Success` |
| `Component.onCompleted` | `connectCamera()`（开机自动取流） | `_conn = "idle"`（默认手动播放） |
| 开机时 `VideoOutput` | 处于 Loading 占位，**未进入可见渲染树** | Success 态，**立即进入可见渲染树** |

推断的因果链：

```
开机即 Success 态
  -> VideoOutput 立即进入可见渲染树
    -> WSLg 的 GL/RHI 后端初始化视频渲染 surface 时阻塞
      -> QML 场景图首帧渲染卡住
        -> 窗口只有标题栏、画面不出，终端停在 multimedia.ffmpeg 之后
```

真机 Linux 的 GL 驱动完整，不触发此阻塞，故不受影响。
这是 **WSLg 的 GL 支持短板**，不是业务逻辑 bug。

## 3. 已实施的修复（本分支改动）

`src/app/main.cpp`：在 `QGuiApplication` 构造**之前**检测 WSL 环境，
命中则设 `QT_QUICK_BACKEND=software` 回退软件渲染：

```cpp
if ((qEnvironmentVariableIsSet("WSL_DISTRO_NAME") || qEnvironmentVariableIsSet("WSL_INTEROP"))
    && !qEnvironmentVariableIsSet("QT_QUICK_BACKEND")) {
    qputenv("QT_QUICK_BACKEND", "software");
    qInfo() << "检测到 WSL 环境，已回退 QtQuick 软件渲染后端 (QT_QUICK_BACKEND=software)";
}
```

设计要点：
- **仅 WSL 命中**：真机 Linux 无 `WSL_DISTRO_NAME`/`WSL_INTEROP`，零影响。
- **尊重用户显式设置**：已手动设 `QT_QUICK_BACKEND` 时不覆盖。
- **必须在 `QGuiApplication` 构造前**，否则渲染后端已初始化，设置无效。

## 4. 待 WSL 端 agent 验证的步骤

1. 在 `feature/WSLg` 分支重新构建：
   ```bash
   cmake --build --preset clang_debug
   ```
2. 直接运行，观察是否出画、终端是否打印「检测到 WSL 环境，已回退…」：
   ```bash
   ./build/<你的预设目录>/app_gas_control_bringup
   ```
3. **判定**：
   - 若画面正常出现 -> 根因确认为 WSLg GL/`VideoOutput` 渲染短板，本修复有效。
   - 若仍不出画 -> 本假设不成立，见下方「备选诊断」。

### 备选诊断（若软件后端仍不出画）

用更细的渲染日志定位卡点，同时手动强制软件后端复核：

```bash
QT_QUICK_BACKEND=software QSG_INFO=1 \
  QT_LOGGING_RULES="qt.rhi.*=true;qt.scenegraph.*=true" \
  ./build/<预设目录>/app_gas_control_bringup 2>&1 | head -80
```

- 若手动加 `QT_QUICK_BACKEND=software` 能出画、而仅靠 main.cpp 注入不行
  -> 说明 WSL 环境变量名不匹配（用 `env | grep -i wsl` 核对实际变量名，据此调整检测条件）。
- 若无论如何都卡在同一处 -> 卡点可能不在 VideoOutput，而在 MediaPlayer 或其它开机即加载组件；
  可临时把 `RtspVideoView.qml` 初始 `statusMode` 改回 `Loading` 验证是否 VideoOutput 触发。

## 5. 环境自证（确认是否在特殊 shell / 环境）

- 真机侧 agent 运行在 user namespace 沙箱（`cat /proc/self/uid_map` = `0 1000 1`，
  详见同目录 `README-uid映射说明.md`）。**沙箱内 git 写操作（建分支/commit）受限**，
  这不是属主损坏，勿执行任何 `chown`/`sudo git`。git 操作应在真机原生终端完成。
- WSL 端如需确认变量名：`env | grep -i wsl`，核对 `WSL_DISTRO_NAME` / `WSL_INTEROP` 是否存在。

---
*记录：Ubuntu 真机侧 agent。根因假设=开机即可见 VideoOutput 触发 WSLg GL 首帧渲染阻塞；修复=WSL 检测回退软件渲染，待 WSL 端实测确认。*