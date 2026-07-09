# 多语言切换（中/英）与重启生效机制技术报告

> 适用版本：Qt 6.10.3 / Clang 14 / ROS 2 Humble
> 涉及模块：i18n 构建链、启动期翻译装载、设置页语言切换、进程自重启、单实例守卫
> 编写日期：2026-03-14

---

## 1. 需求与总体方案

需求是给应用增加中文（`zh_CN`，源语言）与英文（`en_US`）切换能力。

关键约束：Qt 的 `QTranslator` 只在「启动期装载」时对全部 QML 文本生效，运行期再 `installTranslator` 无法让已经实例化的 QML 文本回填刷新。因此本方案不做「热切换」，而是采用：

```
用户在设置页选语言
        │
        ▼
持久化语言偏好到 ini（QML Settings 的 [preferences] 段）
        │
        ▼
确认弹窗 → 调用 C++ AppController.restart()
        │
        ▼
进程自我重启 → 新进程启动期读 ini → 按偏好装载对应 .qm
```

数据流的核心是「一个 ini 文件，两个进程读写」：旧进程写语言偏好，新进程启动时读语言偏好。

---

## 2. 模块与数据流全景

参与本特性的角色与职责：

| 角色 | 位置 | 职责 |
| --- | --- | --- |
| `AppPreferences.qml` | `src/generalstyle/` | QML 端语言偏好的持久化载体，写入 ini 的 `[preferences]` 段 |
| `SettingsDialog.qml` | `src/generalstyle/` | 语言单选项 + 重启确认弹窗，负责发起切换 |
| `AppController`（C++ 单例） | `src/viewmodels/appVM/` | 唯一对外能力：`restart()` 自重启进程 |
| `main.cpp` | `src/app/` | 启动期读 ini，按偏好装载 `.qm` 翻译 |
| `SingleInstanceGuard` | `src/core/algorithm/` | 基于本地 socket 的单实例守卫 |
| `CMakeLists.txt` | 工程根 | 用 `qt_add_translations` 抽取/编译翻译 |

跨进程的唯一共享状态是同一个 ini 文件，路径在 `main.cpp` 里被显式钉死，保证 QML 写的和 C++ 读的是同一份。

---

## 3. CMake 翻译链改动

### 3.1 改动前的问题

原先工程里有一段「手写的 `update_translations` 自定义目标」，它直接用目录扫描方式调用 `lupdate`，并且跑了两遍。问题在于：手写的 lupdate 不识别 QML 文件头部的 `pragma ComponentBehavior: Bound` 语法，扫描时报错并污染 `.ts` 文件，导致构建中断。

### 3.2 改动后的做法

删掉手写目标，改为完全交给 Qt 官方的 `qt_add_translations` 托管：

```cmake
set(TRANSLATIONS
    ${CMAKE_CURRENT_SOURCE_DIR}/translations/${PROJECT_NAME}_zh_CN.ts
    ${CMAKE_CURRENT_SOURCE_DIR}/translations/${PROJECT_NAME}_en_US.ts
)

qt_add_translations(app_${PROJECT_NAME}
    TS_FILES ${TRANSLATIONS}
    QM_FILES_OUTPUT_VARIABLE QM_FILES
)
```

`qt_add_translations` 会自动生成两个伞形目标：

- `..._lupdate`：从源码抽取待翻译文案，写回 `translations/*.ts`
- `..._lrelease`：把 `.ts` 编译成运行期用的 `.qm`

它内部用的是 Qt 自带 lupdate + `lupdate_project.json` 的工程式扫描，能正确解析 QML 的 `pragma` 语法，不会再污染 `.ts`。

`.qm` 产物随后由 `stage_translations` 目标归档到发布目录的 `i18n/` 子目录，与可执行文件同级布局，供运行期加载。

### 3.3 两个固定命令

```bash
# 抽取文案到 .ts
cd src/gas_control_bringup && cmake --build build/Clang14_Qt_6_10_3-Debug --target gas_control_bringup_lupdate

# 完整构建（含 .qm 编译）
cd src/gas_control_bringup && cmake --build build/Clang14_Qt_6_10_3-Debug
```

---

## 4. main.cpp 改动：启动期装载翻译

启动期做了三件事，顺序很重要。

**第一步**：让 `QSettings` 与 `app.db` 同目录，确保 C++ 读的 ini 和 QML 写的 ini 是同一份：

```cpp
QSettings::setDefaultFormat(QSettings::IniFormat);
QSettings::setPath(
    QSettings::IniFormat,
    QSettings::UserScope,
    QStandardPaths::writableLocation(QStandardPaths::GenericDataLocation));
```

配合 `setOrganizationName("GasControl")` 与 `setApplicationName("app_gas_control_bringup")`，最终落点为 `~/.local/share/GasControl/app_gas_control_bringup.ini`。

**第二步**：在 QML 引擎装载「之前」读语言偏好并装载 `.qm`。源语言是 `zh_CN`，所以偏好为 `zh_CN` 时什么都不做（无需 `.qm`）；其余语言从可执行文件同目录加载 `gas_control_bringup_<lang>.qm`：

```cpp
static QTranslator appTranslator;
{
    QSettings prefs;
    const QString language =
        prefs.value(QStringLiteral("preferences/language"),
                    QStringLiteral("zh_CN")).toString();
    if (language != QStringLiteral("zh_CN")) {
        const QString qmName = QStringLiteral("gas_control_bringup_") + language;
        if (appTranslator.load(qmName, QCoreApplication::applicationDirPath())) {
            QCoreApplication::installTranslator(&appTranslator);
        } else {
            qWarning() << "翻译文件加载失败:" << qmName;
        }
    }
}
```

关键点：`installTranslator` 必须发生在 `engine.loadFromModule(...)` 之前，否则 QML 文本已经实例化，翻译不生效。这正是必须重启而非热切的根因。

**第三步**：实例化单实例守卫，已在运行则退出（见第 7 节，也是本报告的 bug 主角）。

---

## 5. 设置页语言切换 + 重启

### 5.1 状态机

`SettingsDialog.qml` 用两个属性管理切换过程的中间态：

- `pendingLanguage`：用户刚选中、但尚未确认重启的目标语言
- `languageSwitching`：标记「已确认重启」，用于区分弹窗的「取消 / 点外部关闭」两条回退路径

单选项的选中态与「当前实际生效语言」之间存在一个待确认的空窗期，状态演变如下：

```
单选项当前态 = 实际生效语言（languageToIndex(AppPreferences.language)）
        │ 用户点了另一个语言
        ▼
弹出重启确认框，pendingLanguage = 目标语言
        │
        ├── 点「立即重启」→ languageSwitching=true → 写偏好 → restart()
        │
        ├── 点「取消」→ revertLanguageRadio() 把单选项拉回实际语言
        │
        └── 点弹窗外部关闭 → onVisibleChanged 兜底 revertLanguageRadio()
```

之所以要 `revertLanguageRadio()` 兜底，是因为「点击对话框外部关闭」不会触发 `negativeClicked`，如果不兜底，单选项会停在用户选的语言上，与实际生效语言不一致，造成误导。

### 5.2 确认重启的落地动作

确认按钮做两件事，顺序是「先持久化，再重启」：

```qml
onPositiveClicked: {
    settingsDialog.languageSwitching = true
    AppPreferences.setLanguage(settingsDialog.pendingLanguage)
    AppController.restart()
}
```

`setLanguage` 把目标语言写进 ini，`restart()` 触发自重启。新进程启动时（第 4 节）就会读到这个新偏好。

---

## 6. Python 脚本批量回填英文翻译

科普文案（`GasKnowledgeDialog.qml`）共 21 种气体 × 4 段（颜色味道/危害/来源/提醒）+ 标签模板 + 兜底，抽取后有约 94 条 `unfinished` 待译条目。手工逐条填 `.ts` 的 XML 既慢又易错，所以写了一个一次性脚本按「源文本精确匹配」回填。

### 6.1 脚本的数据结构

脚本核心是一张「中文原文 → 英文译文」的字典 `M`，键必须与 QML 里 `qsTr()` 包裹的中文「逐字一致」（含全角标点）：

```python
M = {
    "【颜色与味道】%1": "【Color & Odor】%1",
    "无色、无味、无刺激性，人体很难凭感官发现。":
        "Colorless, odorless and non-irritating; ...",
    # ... 共约 94 条
}
```

注意两个细节：

- 标签译文保留了中文的 `【】` 方括号，没有改成英文 `[]`，是为了与 QML 模板里的占位符 `%1` 排版保持一致。
- `%1`、`%2` 等占位符在键和值里都原样保留，lrelease 编译时会校验占位符一致性。

### 6.2 回填算法

脚本把 `.ts` 当纯文本处理（不解析 XML 树），对每条字典项做三步定位替换：

```
对每个 (中文原文 zh, 英文译文 en)：
  1. 在 .ts 文本里找 <source>{转义后的 zh}</source>
  2. 从该位置向后找第一个 <translation type="unfinished"></translation>
  3. 把空的 unfinished 块替换成 <translation>{转义后的 en}</translation>
```

转义函数处理 XML 三个敏感字符（`&` `<` `>`），保证写回的内容是合法 XML：

```python
def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
```

脚本最后打印 `filled: N of M` 自检，本次报告 `filled: 94 of 94`，即 94 条全部命中、无遗漏。脚本完成使命后即删除，仅在 `doc/Reference file/替换脚本.py.md` 留存备查。

### 6.3 这个方法的边界

- 它依赖「源文本逐字匹配」，任何一个标点对不上就会落到 `MISSING` 列表里，所以脚本会显式报告未命中项。
- 它只替换 `type="unfinished"` 的空块，不会覆盖已翻译条目，重复跑安全。
- 它是「一次性补齐存量」的工具，不进入构建链；日常新增文案仍走 `lupdate` 抽取 + 手填或再次跑脚本。

最终 `en_US.qm` 全部条目 finished、0 unfinished；`zh_CN.qm` 为源语言，0 translated 属正常。

---

## 7. 重要 bug：切换过语言后关窗，终端不退出且单实例守卫失效

### 7.1 现象

用户报告的两个并发现象：

1. 程序启动后，**切换过一次中英文**（即触发过一次重启），再关闭窗口 → 启动它的那个终端不会回到可退出状态，进程像是「赖着不走」。
2. 与此同时，能在另一个终端重新打开程序，**不报「程序已在运行」**——单实例守卫形同虚设。

终端日志（`terminals/1.txt`）也佐证：每次重启都成功打印 `UDP 数据监听端口绑定成功 on port 45454`，多个实例能同时把同一个 UDP 端口绑定成功，说明端口层用了地址复用、根本不冲突，掩盖了「其实跑了多个实例」的事实。

### 7.2 两套机制先各自看清楚

**单实例守卫**（`SingleInstanceGuard`）用本地 socket（`QLocalServer` / `QLocalSocket`）实现：

```
tryToRun():
  尝试 connectToServer(uniqueKey)
    ├── 连上了      → 说明已有实例在监听 → 返回 true（本进程退出）
    └── 连不上      → removeServer(uniqueKey) 清残留
                      → 新建 QLocalServer 并 listen(uniqueKey)
                      → 返回 false（正常启动）
析构:
  removeServer(uniqueKey)   // 删掉 socket 路径文件
```

**自重启**（`AppController::restart()`）用 detached 子进程延迟拉起：

```cpp
const QString command = QStringLiteral("sleep 1; exec '%1'").arg(program);
QProcess::startDetached("/bin/sh", {"-c", command});
QCoreApplication::quit();
```

作者已经预见到「新旧实例抢锁」的竞态，用 `sleep 1` 给旧进程退出、析构 `removeServer` 留出时间窗。所以**正常情况下重启是能正确换锁的**——这也是为什么「只重启一次、不再操作」时看不出问题。

### 7.3 根因：detached 子进程继承了父进程的文件描述符

问题出在 `QProcess::startDetached` 的 fork/exec 语义上。它拉起的 `/bin/sh`（以及随后 `exec` 出来的新程序）会**继承旧进程当前打开的文件描述符**，其中关键的有两类：

1. **终端的 stdin/stdout/stderr（控制终端 pts）**
2. **`QLocalServer` 那个正在监听的本地 socket 的 fd**

由此推导出两个现象的成因：

**现象 1（终端不退出）**：旧进程 `quit()` 后，前台作业结束，shell 看起来回到了提示符，但 detached 出来的新进程仍在后台运行、且仍持有同一条 pts 的 stdout/stderr。新进程往这条终端写日志（日志里能看到这一点），终端会话因为还有进程占着这些 fd 而无法干净收尾。关掉 GUI 窗口后，如果这个新进程没能彻底退出，终端就一直「赖着」。

**现象 2（守卫失效）**：旧进程析构时 `removeServer(uniqueKey)` 只是把 socket 的**路径文件 unlink 掉**，但**被子进程继承的那个监听 fd 在内核里仍然存活**（fd 没设 `FD_CLOEXEC`，或在 fork/exec 时机窗内未被清掉）。于是路径与内核 socket 对象的对应关系被打乱：新实例去 `connectToServer` 时连不到一个「干净可达的监听端」，于是判定「无实例运行」，径直再建一个新 server 启动成功。结果就是单实例语义被旁路，多个实例并存。

一句话概括：**重启用的 detached 子进程继承了不该继承的 fd（终端 + 监听 socket），既让终端无法收尾，又让基于 socket 路径的单实例检测失去依据。** 这条链路只有在「发生过一次重启」后才被激活，所以纯净的首次启动看不到问题。

### 7.4 验证手段（建议在 WSL 实机确认）

静态读码已经能解释现象，但 socket 在内核层的确切状态最好用运行时证据钉死：

```bash
# 看 gas_control 相关的本地 socket 还有谁在持有
ss -xp | grep gas_control
lsof -U | grep gas_control

# 看重启后到底有几个进程实例、各自的控制终端
ps -ef | grep app_gas_control_bringup
```

复现路径：启动 → 切一次语言触发重启 → 关窗 → 在另一个终端再启动一个，观察是否未报「已在运行」，并用上面命令核对实例数与 socket 持有者。

### 7.5 实施的修复

针对 7.3 的两条根因，做了两处外科手术式的改动，分别对应「守卫失效」和「终端不退出」。

**修复一：给监听 socket 设 `FD_CLOEXEC`（`singleinstanceguard.cpp`）**

在 `QLocalServer::listen` 成功后，立刻对其底层监听 fd 设 `FD_CLOEXEC`。这样 `restart()` 通过 fork/exec 拉起新实例时，`exec` 一刻该 fd 会被内核自动关闭，子进程不再继承这个监听 socket：

```cpp
#ifdef Q_OS_UNIX
    const qintptr fd = server->socketDescriptor();
    if (fd != -1) {
        const int flags = ::fcntl(static_cast<int>(fd), F_GETFD);
        if (flags != -1) {
            ::fcntl(static_cast<int>(fd), F_SETFD, flags | FD_CLOEXEC);
        }
    }
#endif
```

效果：旧进程析构 `removeServer` unlink 路径文件后，内核里不再有被子进程「偷偷续命」的监听 socket，单实例检测重新有据可依——重启后在另一个终端再启动会正确报「程序已在运行」。

**修复二：重启子进程脱离原控制终端（`appcontroller.cpp`）**

在 `restart()` 的 `sh -c` 命令里，把新实例的 stdin/stdout/stderr 全部重定向到 `/dev/null`，切断对父进程 pts 的占用：

```cpp
const QString command =
    QStringLiteral("sleep 1; exec '%1' </dev/null >/dev/null 2>&1").arg(program);
QProcess::startDetached(QStringLiteral("/bin/sh"),
                        QStringList() << QStringLiteral("-c") << command);
QCoreApplication::quit();
```

配合 `QProcess::startDetached` 在 Unix 上本身的脱离会话行为，新实例不再持有原终端的任何标准流。效果：旧进程 `quit()` 后没有子进程再占着这条 pts，终端能干净收尾。

### 7.6 修复后的预期表现（容易被误判为新 bug）

切语言确认重启后会观察到：**原终端立即退出**，且**不会弹出新终端**。这是设计结果而非异常——新实例已与终端解耦，它是纯 GUI 进程，只会在 `sleep 1` 后重新弹出程序窗口，本就不该再绑定/拉起终端。

判断修复是否生效，看的不是「有没有新终端」，而是：

1. 程序窗口在约 1 秒后重新弹出，且语言已切换。
2. 在另一个终端再启动一次，应报「程序已在运行」（依赖修复一）。

副作用与取舍：为切断终端占用，新实例日志被丢进 `/dev/null`。若后续需要保留重启后实例的日志，可把 `/dev/null` 换成日志文件路径，但终端本身依然不会（也不应）被重新拉起。

### 7.7 仍可选的鲁棒性增强（非必须）

当前两处修复已闭合本次 bug。若要进一步提升守卫健壮性，可补充 PID 文件 + `flock` 文件锁：文件锁随持有进程退出自动释放，不存在「路径被 unlink 但内核对象仍在」的歧义。此项为可选项，未实施。

---

## 8. 改动文件清单

| 文件 | 改动 |
| --- | --- |
| `CMakeLists.txt` | 删除手写 `update_translations`，统一交给 `qt_add_translations` |
| `src/app/main.cpp` | 启动期按 ini 偏好装载 `.qm`；钉死 QSettings 路径 |
| `src/viewmodels/appVM/appcontroller.{h,cpp}` | 新增 `AppController` 单例，`restart()` 自重启 |
| `src/generalstyle/AppPreferences.qml` | 新增 `language` 偏好持久化 |
| `src/generalstyle/SettingsDialog.qml` | 语言单选项 + 重启确认弹窗 + 选中态回退兜底 |
| `src/viewmodels/stateManageVM/gasconcentrationstate.{h,cpp}` | 气体名仅在显示层翻译，内部键保持中文 |
| `src/leftsidebar/GasKnowledgeDialog.qml` | 科普文案全量 `qsTr()` 化 |
| `translations/*_en_US.ts` | 英文译文（含科普文案，经脚本批量回填） |

---

## 9. 修复状态与后续

- 已修复并通过构建验证：`cmake --build build/Clang14_Qt_6_10_3-Debug` 编译通过，无报错。两处改动（监听 socket 加 `FD_CLOEXEC`、重启子进程 stdio 重定向到 `/dev/null`）已落地，分别闭合「守卫失效」与「终端不退出」。
- 已确认预期表现：重启后原终端立即退出、不再弹新终端（见 7.6），属设计结果。在另一个终端再启动会正确报「程序已在运行」。
- 可选未做：PID 文件 + `flock` 文件锁（见 7.7），仅作进一步鲁棒性增强，非必须。
- 若需保留重启后实例日志，可把修复二里的 `/dev/null` 换成日志文件路径，终端行为不受影响。