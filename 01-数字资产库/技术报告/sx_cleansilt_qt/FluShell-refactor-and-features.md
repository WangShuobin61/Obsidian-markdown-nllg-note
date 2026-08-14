# FluShell 重构与功能扩展说明

> 本文档记录一次对 FluShell 项目的完整结构重组与功能扩展，涵盖：目录结构迁移、CMake target 拆分、主窗口三标签页、底部日志/消息分栏、触摸屏悬浮球等内容。

---

## 一、重构动机

原始目录按**技术角色**横向分层（`auth/`、`viewmodels/`、`shell/`、`theme/`、`core/`），同一功能（如鉴权）的 QML 与 C++ 分布在多个顶层目录，修改时需要跨目录跳转，且 CMake target 的边界与功能边界不一致。

重构目标：**顶层按功能内聚，功能内部保留 MVVM 分层，CMake target 与功能边界对齐。**

---

## 二、重构优缺点分析

### 优势

**1. 功能内聚，修改路径短**

修改任何一个功能时，相关文件集中在同一目录下。以鉴权为例：

```
原来：src/auth/ + src/viewmodels/authVM/ + src/core/database/（至少3处）
现在：src/features/auth/（1处）
```

**2. CMake target 与功能边界一致**

每个功能是一个 CMake target，也是一个 QML module。删除一个功能 = 从 CMakeLists.txt 删一行 `add_subdirectory` + 删一个目录，不会遗留僵尸头文件或孤立 target。

**3. QML module URI 语义清晰**

`import flushell.auth 1.0` 的字面意思就是"导入鉴权模块"，而不是以前的 `import flushell.viewmodels`（实际包含了 Auth、SystemEvent、AppController、CircularReveal 四类毫无关联的东西）。

**4. 依赖方向可追踪**

重构后的依赖图是单向的：`app → features/* → shared`，`shared` 不反向依赖任何 feature。原来的 `viewmodels → core` 虽然也单向，但 `viewmodels` 本身过于宽泛，等于没有边界。

**5. 可裁剪性**

派生新项目时，不需要某个功能可以整体移除：

- 不需要系统事件：删 `features/systemevents/`，移除对应 `add_subdirectory` 和 `target_link_libraries`
- 不需要触摸悬浮球：删 `ui/components/`，一行 CMake
- 以前横向分层时，删一个功能需要在多个目录里分别找文件

**6. 测试边界自然形成**

每个 feature 目录天然是一个测试单元，未来加测试时可以对 `features/auth/` 独立建 `tests/auth/`，不需要 mock 其他模块。

---

### 局限与代价

**1. 目录层级增加**

`src/features/auth/authmanager.cpp` 比原来的 `src/viewmodels/authVM/authmanager.cpp` 更深，但也更语义化。对于小型项目，这层多余的目录有时感觉冗余。

**2. 跨功能共享逻辑的归属判断需要主动决策**

原来所有"不知道放哪里"的东西都丢进 `core/` 或 `viewmodels/`，现在需要主动判断"这个工具是只被一个功能用，还是真的跨功能共享"。误判会导致 `shared/` 越来越大，退化成新的 `core/`。

> 判断原则：已经被 **≥2 个 feature** 实际引用，才进入 `shared/`；只有一个 feature 用的工具，放在该 feature 内部。

**3. QML module 增多，import 行数增加**

原来一行 `import flushell.viewmodels` 解决所有 C++ 类型，现在需要分别写：

```qml
import flushell.auth 1.0
import flushell.systemevents 1.0
```

对于大量使用两个模块的文件（如 SettingsDialog），import 区会略长，但这是"显式优于隐式"的代价，是可接受的。

**4. Debug 热重载拦截器需要维护映射表**

`SourceUrlInterceptor` 中的 `kModuleMap` 需要与模块目录保持同步。新增模块如果目录名与 URI 段名不同，必须手动补一行，否则 Debug 下 QML 热编辑不生效（Release 构建不受影响）。

---

## 三、文件结构对比

### 重构前

```
src/
├── app/
│   ├── main.cpp
│   └── Main.qml
├── auth/                        # 仅包含 QML（View 层）
│   ├── ChangePasswordDialog.qml
│   ├── CustomUserManagerDialog.qml
│   ├── LogIn.qml
│   ├── RegisterUserDialog.qml
│   └── CMakeLists.txt           # flushell.auth（QML-only）
├── core/                        # 所有 C++ 基础设施混放
│   ├── algorithm/
│   │   ├── singleinstanceguard.cpp/.h
│   ├── database/
│   │   ├── appdatabase.cpp/.h
│   ├── io/
│   │   ├── xlsxwriter.cpp/.h
│   ├── paths/
│   │   ├── apppaths.cpp/.h
│   └── CMakeLists.txt           # target: core（静态库）
├── shell/                       # 外壳 QML
│   ├── DemoPanel.qml
│   ├── SettingsDialog.qml
│   ├── ShellLayout.qml
│   ├── ShellTitleBar.qml
│   ├── ShellUserStatus.qml
│   ├── ShellWindowControls.qml
│   ├── Tour.qml
│   └── CMakeLists.txt           # flushell.shell（QML-only）
├── theme/                       # 主题 QML
│   ├── AppPreferences.qml
│   ├── HoveringCue.qml
│   ├── TouchScrollColumn.qml
│   ├── UserSession.qml
│   └── CMakeLists.txt           # flushell.theme
└── viewmodels/                  # 所有 C++ 类混在一起
    ├── appVM/
    │   ├── appcontroller.cpp/.h
    ├── authVM/
    │   ├── authmanager.cpp/.h
    │   ├── authrepository.cpp/.h
    │   ├── passwordhasher.cpp/.h
    ├── componentsVM/
    │   ├── circularreveal.cpp/.h
    ├── systemEventVM/
    │   ├── systemeventmanager.cpp/.h
    │   ├── systemeventmodel.cpp/.h
    │   ├── systemeventrepository.cpp/.h
    └── CMakeLists.txt           # flushell.viewmodels（C++ 大杂烩）
```

**主要问题：**
- 鉴权功能分布在 `auth/`（QML）和 `viewmodels/authVM/`（C++）两处
- `viewmodels` 同时包含 ViewModel、Repository、密码工具、动画组件，职责混乱
- `core` 目录无边界约束，所有头文件都 `PUBLIC` export，任何地方都能包含任何东西
- QML import 全部依赖 `import flushell.viewmodels`，模块边界模糊

---

### 重构后

```
src/
├── app/                         # 唯一组合根：进程启动 + QML engine 装配
│   ├── main.cpp                 # 启动入口，Debug 模式含 SourceUrlInterceptor
│   ├── Main.qml                 # 根窗口，负责组合所有功能模块
│   ├── circularreveal.cpp/.h    # 主题切换圆形涟漪动画（仅 Main.qml 使用）
│
├── features/                    # 功能模块：每个子目录 = 一个独立功能边界
│   ├── auth/                    # 鉴权功能：QML + C++ 合并，URI: flushell.auth
│   │   ├── ChangePasswordDialog.qml   # 修改密码对话框
│   │   ├── CustomUserManagerDialog.qml # 自定义用户管理
│   │   ├── LogIn.qml                  # 登录页面
│   │   ├── RegisterUserDialog.qml     # 注册用户对话框
│   │   ├── authmanager.cpp/.h         # ViewModel：QML 接口层，QML_SINGLETON
│   │   ├── authrepository.cpp/.h      # Model/Data：SQLite 数据访问
│   │   ├── passwordhasher.cpp/.h      # 密码哈希/HMAC 工具
│   │   └── CMakeLists.txt             # qt_add_qml_module，混合 QML + C++
│   │
│   ├── systemevents/            # 系统事件（日志 + 消息），URI: flushell.systemevents
│   │   ├── systemeventmanager.cpp/.h   # QML_SINGLETON：logAction / postMessage
│   │   ├── systemeventmodel.cpp/.h     # QAbstractListModel，供 ListView 绑定
│   │   ├── systemeventrepository.cpp/.h # SQLite CRUD，查询/导出/删除
│   │   └── CMakeLists.txt
│   │
│   └── mainwindow/              # 主内容区功能，URI: flushell.mainwindow
│       ├── framework/
│       │   ├── TabContentBase.qml  # 标签页基类：背景 + 右键/长按开独立窗口
│       │   └── TabDetachWindow.qml # 独立窗口容器，按需 createObject 创建
│       ├── tabs/
│       │   ├── DemoTab1.qml         # 功能预留1：随机日志/消息测试按钮
│       │   ├── DemoTab2.qml         # 功能预留2：同上，文本内容不同
│       │   ├── DatabaseTab.qml      # 数据库标签页外壳（Loader 互斥加载视图）
│       │   ├── DatabaseViewBase.qml # 数据库视图底座：表格+分页+导出+二级删除
│       │   └── SystemEventDbView.qml # 系统事件查询视图（6列，支持筛选）
│       ├── MainWinFunction.qml      # 三标签页宿主（FluTabView）
│       ├── SystemLogPanel.qml       # 实时日志滚动面板（绑定 logModel）
│       ├── SystemMessagePanel.qml   # 实时消息滚动面板（绑定 messageModel）
│       ├── BottomDockPanel.qml      # 底部停靠区：左日志/右消息的水平 SplitView
│       └── CMakeLists.txt
│
├── ui/                          # 通用 UI 层：与具体业务无关的框架组件
│   ├── shell/                   # 应用外壳框架，URI: flushell.shell
│   │   ├── appcontroller.cpp/.h   # 进程级动作：restart()（语言切换重启）
│   │   ├── DemoPanel.qml          # 停靠区占位面板（派生项目替换后可删除）
│   │   ├── SettingsDialog.qml     # 系统设置窗口
│   │   ├── ShellLayout.qml        # 主布局：四区可折叠 SplitView
│   │   ├── ShellTitleBar.qml      # 自绘标题栏（拖拽/用户状态/扩展插槽）
│   │   ├── ShellUserStatus.qml    # 标题栏左侧用户身份显示
│   │   ├── ShellWindowControls.qml # 最小化/最大化/关闭/折叠/固定
│   │   ├── Tour.qml               # 新手导览
│   │   └── CMakeLists.txt
│   │
│   ├── theme/                   # 主题与持久化，URI: flushell.theme
│   │   ├── AppPreferences.qml     # INI 持久化中心（主题/语言/布局/悬浮球）
│   │   ├── HoveringCue.qml        # 浮动提示条
│   │   ├── TouchScrollColumn.qml  # 支持触摸惯性滚动的竖向布局
│   │   ├── UserSession.qml        # 会话身份态（isLoggedIn / currentIdentity）
│   │   └── CMakeLists.txt
│   │
│   └── components/              # 通用 UI 组件，URI: flushell.ui.components
│       ├── FloatingBall.qml       # 触摸屏悬浮球（DragHandler，位置持久化）
│       └── CMakeLists.txt
│
└── shared/                      # 跨功能共享基础设施（静态库 target: shared）
    ├── database/
    │   ├── appdatabase.cpp/.h   # 全局 SQLite 连接提供者（WAL 模式）
    ├── export/
    │   ├── xlsxwriter.cpp/.h    # 轻量级 xlsx 导出（纯 Qt + zlib）
    ├── platform/
    │   ├── apppaths.cpp/.h      # 平台数据目录路径工具
    │   └── singleinstanceguard.cpp/.h # 单实例守卫（QLocalServer）
    └── CMakeLists.txt
```

---

## 三、QML 模块 URI 变更

| 模块内容 | 重构前 URI | 重构后 URI |
|---|---|---|
| 认证 C++ 类型（AuthManager 等） | `flushell.viewmodels` | `flushell.auth` |
| 认证 QML（LogIn 等） | `flushell.auth` | `flushell.auth`（不变，C++ 合入） |
| 系统事件（SystemEventManager 等） | `flushell.viewmodels` | `flushell.systemevents` |
| 进程控制（AppController） | `flushell.viewmodels` | `flushell.shell`（同模块，无需 import） |
| 主题圆形涟漪（CircularReveal） | `flushell.viewmodels` | `flushell.app`（同模块，无需 import） |
| 外壳 QML | `flushell.shell` | `flushell.shell`（不变，目录迁移） |
| 主题 QML | `flushell.theme` | `flushell.theme`（不变，目录迁移） |
| 主窗口功能 | ——（新增） | `flushell.mainwindow` |
| 触摸屏组件 | ——（新增） | `flushell.ui.components` |
| `flushell.viewmodels` | 存在 | **已消除** |

---

## 四、新增功能说明

### 4.1 三标签页主窗口（centerPane）

`Main.qml` 的 `centerPane` 挂载 `MainWinFunction`，包含三个等宽标签页：

| 标签 | 组件 | 说明 |
|---|---|---|
| 功能预留 1 | `DemoTab1.qml` | 随机写入日志/消息的测试按钮，派生项目在此添加业务内容 |
| 功能预留 2 | `DemoTab2.qml` | 同上，独立文本集合，便于区分来源 |
| 数据库 | `DatabaseTab.qml` | 系统事件表格查询：等级/关键字/时间范围筛选，分页，导出 xlsx，二级确认删除 |

### 4.2 底部停靠区左右分割（bottomPane）

`BottomDockPanel` 使用 `FluSplitLayout`（`Qt.Horizontal`）将底部区域对半分：

- **左**：`SystemLogPanel`——实时滚动日志（`logModel`，含时间+等级徽章+内容换行）
- **右**：`SystemMessagePanel`——实时滚动消息（`messageModel`，同格式）

### 4.3 标签页在独立窗口中打开（触摸屏长按）

`TabContentBase` 实现统一机制：
- **右键** → 弹出上下文菜单
- **左键长按 0.7 秒** → 弹出上下文菜单（触摸屏场景）
- 菜单选「在独立窗口中打开」→ 延迟 `createObject` 创建 `TabDetachWindow`（独立 `Window`），同路径只创建一次
- 使用 `TapHandler` 替代 `MouseArea`，不阻断子控件（按钮等）的点击事件

### 4.4 触摸屏悬浮球 + INI 持久化

**`FloatingBall.qml`**：
- `DragHandler` 实现全屏自由拖拽
- 位置以比值（`xRatio` / `yRatio`）落盘，随窗口缩放正确恢复
- 父容器尺寸变化时自动夹回边界

**`AppPreferences.qml` 新增字段**：

```
floatingBallEnabled  bool   默认 false（设置面板开关）
floatingBallSize     int    0~100，映射到球的实际像素大小
floatingBallXRatio   real   0~1，相对父容器宽度的位置比
floatingBallYRatio   real   0~1，相对父容器高度的位置比
```

`SettingsDialog` 新增「触摸屏悬浮球」开关卡片，控制 `AppPreferences.store.floatingBallEnabled`。

---

## 五、Debug 模式 QML 热重载适配

`main.cpp` 中的 `SourceUrlInterceptor` 将 `qrc:/qt/qml/flushell/<module>/...` 重定向到源码目录，实现 QML 文件保存后无需重新编译即可生效。重构后模块名与源码目录名不再一一对应，因此拦截器改为查表：

```
URI 段          →  src/ 下实际路径
-----------        ------------------
app             →  app
auth            →  features/auth
systemevents    →  features/systemevents
mainwindow      →  features/mainwindow
shell           →  ui/shell
theme           →  ui/theme
ui              →  ui               （处理 ui/components）
其他            →  与 URI 段同名（兼容将来新增模块）
```

**后续新增模块时**，若源码目录与 URI 段名不同，在 `main.cpp` 的 `kModuleMap` 数组中补一行即可。

---

## 六、参考项目符号链接

```
FluShell/
└── gas_control_bringup_ref  →  ../sx_GasControl_qt/src/gas_control_bringup
```

已加入 `.gitignore`，仅用于本地开发对照，不纳入版本控制。

---

## 七、依赖关系（向下不可反向）

```
app
 ├── features/auth
 ├── features/systemevents
 ├── features/mainwindow
 ├── ui/shell
 ├── ui/theme
 ├── ui/components
 └── shared          ← features/* 和 ui/shell 均依赖

shared              ← 不依赖任何上层模块
```

`shared` 是唯一被多个功能模块共享的基础库，其他模块不得反向依赖具体 feature。