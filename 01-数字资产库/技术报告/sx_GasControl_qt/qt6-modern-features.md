# Qt6 新技术应用实践报告

## 一、报告目标

本报告聚焦 `gas_control_bringup` 项目中已经落地的 Qt6 新机制与现代化实践，重点不是泛泛介绍 Qt6 功能清单，而是总结**项目里真实使用了什么、解决了什么问题、相对于旧做法有什么收益**。

结合当前代码与 `doc/Reference file` 中的参考材料，项目的 Qt6 新技术应用主要集中在四个方面：

1. 新的 QML 类型注册机制
2. QML 模块化构建与 URI 组织
3. 新式信号槽与 lambda 连接方式
4. Debug 场景下的 QML 源码级热重载与模块重定向

---

## 二、Qt6 新的 QML 注册机制

### 2.1 从命令式注册，转向声明式注册

传统 Qt5 / 早期 Qt Quick 项目里，常见做法是在 `main.cpp` 中手动调用：

```cpp
qmlRegisterType<T>()
qmlRegisterSingletonType<T>()
engine.rootContext()->setContextProperty(...)
```

这种方式的问题是：

- 注册逻辑集中在 `main.cpp`，入口文件越来越臃肿
- 类本身看不出自己是否打算暴露给 QML
- 模块边界弱，类型归属不清晰
- 重构时容易出现"类改了但 main.cpp 忘记同步"的问题

本项目采用 Qt6 推荐的新机制：**在类定义处直接用宏声明类型能力，由构建系统自动完成 QML 注册。**

### 2.2 项目中的实际用法

#### `AuthManager` 与 `StateManager`：QML 单例

它们在头文件里使用：

```cpp
Q_OBJECT
Q_DISABLE_COPY_MOVE(AuthManager)
QML_ELEMENT
QML_SINGLETON
```

含义可以直接理解为：

- `QML_ELEMENT`：这个类要进入 QML 类型系统
- `QML_SINGLETON`：这个类在 QML 世界里只保留一个实例

对应效果是，QML 侧可以直接：

```qml
import gascontrolbringup.viewmodels

AuthManager.verifyLogin(...)
StateManager.batteryLevel
```

而不需要 `main.cpp` 手动创建对象并注入上下文。

#### `CircularReveal`：普通 QML 元素

`CircularReveal` 只用了：

```cpp
QML_ELEMENT
```

说明它是一个可实例化的普通视觉组件，不是单例。

### 2.3 这一机制的工程收益

这种注册机制最大的价值不是"少写几行代码"，而是**类型归属关系变清晰了**：

- 类的声明处就能看出它是普通类型还是单例
- 类型能力和类定义绑定，不再散落在入口文件
- `main.cpp` 从"全局注册中心"退化为纯启动器
- 模块之间的边界更稳定，便于拆分和重构

在本项目里，`main.cpp` 现在只保留真正需要上下文注入的纯值：

```cpp
engine.rootContext()->setContextProperty(
    "appDataPath",
    QStandardPaths::writableLocation(QStandardPaths::AppDataLocation));
```

也就是说，**对象类全部交给 Qt6 模块机制管理，入口只保留简单配置值。**

---

## 三、Qt6 的 QML 模块化构建机制

### 3.1 `qt_add_qml_module` 成为一等公民

本项目每个 UI 或 ViewModel 子系统都不是简单地"把 QML 文件塞进资源"，而是明确声明为 QML 模块，例如：

```cmake
qt_add_qml_module(viewmodels
    URI gascontrolbringup.viewmodels
    VERSION 1.0
    SOURCES
        ${VIEWMODEL_SOURCES}
        ${VIEWMODEL_HEADERS}
    OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/gascontrolbringup/viewmodels
)
```

类似地：

- `gascontrolbringup.app`
- `gascontrolbringup.generalstyle`
- `gascontrolbringup.topbar`
- `gascontrolbringup.mainwindow`
- `gascontrolbringup.leftsidebar`
- `gascontrolbringup.rightsidebar`
- `gascontrolbringup.viewmodels`

全部是独立模块。

### 3.2 URI 命名空间的意义

采用 `gascontrolbringup.xxx` 这样的 URI，不只是命名好看，而是建立了一层**逻辑命名空间**：

| URI | 代表的职责边界 |
|-----|----------------|
| `gascontrolbringup.app` | 应用入口与根窗口 |
| `gascontrolbringup.generalstyle` | 通用界面组件 |
| `gascontrolbringup.viewmodels` | C++ 业务对象与状态对象 |
| `gascontrolbringup.topbar` | 顶栏功能区 |

这样做之后，QML 的 `import` 本身就变成了架构说明文档。

### 3.3 与旧式资源打包方式的区别

旧方式通常是：

- 所有 QML 资源一股脑进入同一个 `.qrc`
- 类型发现依赖运行期搜索
- 模块之间没有清晰物理边界

Qt6 模块方式则把问题前移到构建期：

- 每个模块有清晰 URI
- 自动生成 `qmldir` / 类型元数据
- 模块边界可检查、可追踪
- IDE 与 QML 语言服务能更准确理解类型来源

这对大型项目尤其重要，因为文件一多之后，"所有 QML 都在一个资源包里"会迅速退化成不可维护状态。

---

## 四、新式信号槽：类型安全 + lambda 风格

### 4.1 项目中的典型写法

例如 `CircularReveal` 构造函数中：

```cpp
connect(m_anim, &QPropertyAnimation::finished, this, [this]() {
    update();
    setVisible(false);
    Q_EMIT animationFinished();
});

connect(this, &CircularReveal::radiusChanged, this, [this]() { update(); });
```

这已经完全是 Qt5/Qt6 的现代信号槽风格：

- 使用函数指针而不是字符串宏
- 直接绑定 lambda
- 编译期检查参数与类型匹配

### 4.2 相比旧语法的实际收益

旧式写法：

```cpp
connect(obj, SIGNAL(finished()), this, SLOT(onFinished()));
```

问题是很多错误只能运行时暴露。

项目当前采用的新写法有三类明显收益：

1. **类型安全**：信号与槽不匹配会在编译时报错
2. **局部性更强**：简单响应逻辑可直接写在连接位置，不必额外造槽函数
3. **重构更稳定**：IDE 能跟踪符号引用，改名不会留下字符串黑洞

对于本项目这种 UI 驱动型程序来说，大量状态联动都很短小，lambda 连接非常适合，能减少为了"只做一件小事"而额外创建一堆成员槽函数的样板代码。

---

## 五、Debug 模式下的 QML 热重载思路

### 5.1 项目里的实际问题

Qt6 使用 `qt_add_qml_module` 后，运行期默认会从模块资源路径加载 QML，例如：

```text
qrc:/qt/qml/gascontrolbringup/topbar/TopFunction.qml
```

这对 Release 很好，但对 Debug 开发有一个痛点：

- 你修改了 `src/topbar/TopFunction.qml`
- 运行时如果还是读打包资源，就看不到源码的即时变化

### 5.2 当前项目的解决方式：URL 拦截器

`main.cpp` 在 `QT_DEBUG` 下引入了 `QQmlAbstractUrlInterceptor`：

```cpp
class SourceUrlInterceptor : public QQmlAbstractUrlInterceptor
{
public:
    QUrl intercept(const QUrl &url, DataType type) override
    {
        if (type != QmlFile && type != JavaScriptFile)
            return url;

        if (url.scheme() != QLatin1String("qrc"))
            return url;

        static const QString kPrefix = QStringLiteral("/qt/qml/gascontrolbringup/");
        const QString path = url.path();
        if (!path.startsWith(kPrefix))
            return url;

        const int slash = path.indexOf(QLatin1Char('/'), kPrefix.size());
        if (slash < 0)
            return url;

        const QString seg = path.mid(kPrefix.size(), slash - kPrefix.size());
        const QString tail = path.mid(slash);
        return QUrl::fromLocalFile(QStringLiteral(QML_SOURCE_DIR)
                                   + QLatin1Char('/') + seg + tail);
    }
};
```

它的作用可以用一句话概括：

> Release 仍走模块资源；Debug 时把模块资源路径自动映射回源码目录。

### 5.3 这个方案的巧思

这不是简单的"再加一个 import path"，而是保留模块 URI 不变，只在加载阶段把目标文件重定向回源码路径。

因此同时满足了三件事：

1. **架构不变**：模块 URI 仍然是正式结构，不为了热更新而破坏命名空间
2. **开发效率高**：Debug 模式直接读取 `src/` 下真实文件
3. **发布一致性好**：Release 仍然走标准模块加载路径

这是典型的 Qt6 工程化思路——不是为了调试方便而放弃模块系统，而是在模块系统之上增加一层开发期适配。

---

## 六、Qt6 下的 QML 单例与状态驱动 UI

### 6.1 单例型 ViewModel 的落地

项目里最典型的两个单例型对象是：

- `AuthManager`
- `StateManager`

它们对 QML 来说不是"页面创建时顺便塞进来的对象"，而是**模块内天然存在的全局状态入口**。

这带来两个直接好处：

#### 好处 1：去上下文注入化

QML 文件不依赖 `main.cpp` 的 `setContextProperty` 次序，也不担心某个页面忘记引入上下文。

#### 好处 2：跨页面共享状态自然成立

例如顶栏多个组件都能直接读取：

- `StateManager.batteryLevel`
- `StateManager.signalType`
- `StateManager.temperature`

登录与权限页面直接读取：

- `AuthManager.isAuthenticated`
- `AuthManager.currentRole`

这种方式非常适合嵌入式控制面板：应用天然只有一套全局设备状态与一套全局用户会话。

### 6.2 Q_PROPERTY 驱动声明式刷新

例如 `StateManager` 暴露：

- `batteryLevel`
- `batteryCharging`
- `signalType`
- `wifiStrength`
- `temperature`
- `humidity`

QML 顶栏组件只关心"当前值是什么"，而不关心"谁什么时候来通知我更新"。这正是 Qt6 + QML 最合适的使用方式：**C++ 负责产出状态，QML 负责声明状态如何映射到界面。**

---

## 七、资源与构建侧的现代化细节

### 7.1 `QT_RESOURCE_ALIAS` 统一资源别名

项目对入口 `Main.qml` 使用了：

```cmake
set_source_files_properties(src/app/Main.qml PROPERTIES
    QT_RESOURCE_ALIAS Main.qml
)
```

这样资源里的注册名与模块结构保持一致，避免出现源码路径和资源路径语义不对齐的问题。

### 7.2 `OPTIONAL_IMPORTS` 与模块依赖声明

例如 `generalstyle` 模块声明：

```cmake
OPTIONAL_IMPORTS
    FluentUI
    gascontrolbringup.viewmodels
```

这使模块依赖关系在构建期就是显式的，而不是运行时遇到 `import` 才知道缺什么。

### 7.3 `QML_IMPORT_PATH` 与 IDE 体验

顶层 CMake 里还设置了：

```cmake
set(QML_IMPORT_PATH
    ${CMAKE_BINARY_DIR}
    CACHE STRING "IDE QML import path" FORCE
)
```

这类配置虽然不是 Qt6 新 API，但与 Qt6 模块机制结合后，能显著改善 IDE 对模块类型的识别能力，减少编辑器里的假报错与跳转失败。

---

## 八、与旧项目风格的演进对比

结合 `doc/src_old` 可以看出，项目技术路线已经发生明显转型：

| 维度 | 旧风格 | 当前 Qt6 风格 |
|------|--------|---------------|
| UI 组织 | Widgets + UI 文件 | QML 模块化 |
| 对象暴露 | `setContextProperty` 为主 | `QML_ELEMENT` / `QML_SINGLETON` |
| 信号槽 | 容易走向传统槽函数堆积 | lambda + 新式 connect |
| 资源加载 | 文件/资源混合，边界不清 | `qt_add_qml_module` + URI |
| 调试方式 | 更依赖手工路径控制 | URL 拦截器配合模块机制 |

这不是单纯的"从 Qt5 换到 Qt6"，而是从**命令式拼装型工程**转向**声明式模块型工程**。

---

## 九、项目层面的实际价值

从结果看，Qt6 新机制在本项目中带来的价值不是抽象的，而是非常具体的：

### 9.1 降低入口复杂度

`main.cpp` 不再承担对象注册中心职责，只负责：

- 应用初始化
- 单实例守护
- Debug 重定向
- 加载根模块

### 9.2 提升模块可替换性

`topbar`、`generalstyle`、`viewmodels` 各自独立，未来重做某个 UI 区域时，不需要重写整个系统。

### 9.3 让 QML 与 C++ 协作边界更稳定

QML 只消费属性与动作；C++ 只产出状态与服务，不需要感知具体页面结构。

### 9.4 为后续扩展留下空间

当前已经形成一种可复制模式：

- 新增业务域 → 在 `viewmodels/xxxVM` 下增加子模块类
- 新增界面区 → 新建一个 `qt_add_qml_module`
- 需要全局状态 → `QML_SINGLETON`
- 需要可视组件 → `QML_ELEMENT`

这意味着项目后续增加新页面、新状态类型、新交互组件时，都有明确模板可循。

---

## 十、结论

`gas_control_bringup` 在 Qt6 新技术上的应用，核心不是炫技，而是围绕工程目标做了四个正确决策：

1. **用 `QML_ELEMENT` / `QML_SINGLETON` 取代手工注册**，让类型归属回到类定义本身
2. **用 `qt_add_qml_module` 组织所有 QML/C++ 模块**，让架构边界前移到构建期
3. **用新式 connect + lambda 管理对象联动**，减少样板代码并提升类型安全
4. **用 Debug URL 拦截器解决模块化与热重载之间的矛盾**，兼顾开发效率与发布一致性

从工程视角看，这些实践共同说明：项目已经不再把 Qt6 当作"更高版本的 Qt5"来用，而是开始真正采用 Qt6 推荐的模块化、声明式、构建驱动的现代开发方式。