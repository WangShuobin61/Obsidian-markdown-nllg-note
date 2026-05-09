模块这里都是这样写的
```cmake
qt_add_qml_module(app${PROJECT_NAME}
    URI WheelScanning.App
    VERSION 1.0
    QML_FILES
        src/app/Main.qml
    RESOURCES
        res/resource.qrc
    OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/WheelScanning/App
)
```
```cmake
qt_add_qml_module(modeStyle
    URI WheelScanning.ModeStyle
    VERSION 1.0
    ...
    OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/WheelScanning/ModeStyle
)
```
所以导致了 @main.cpp 在debug构建下热重载必须在src路径下设置
```bash
nllg@LegionR7000:~/my_project/wheel_scanning/src$ ls
app  core  modeStyle  moduleA  moduleB  qmldir  viewmodels  WheelScanning
nllg@LegionR7000:~/my_project/wheel_scanning/src$ cd WheelScanning/
nllg@LegionR7000:~/my_project/wheel_scanning/src/WheelScanning$ ll
总计 8
drwxrwxr-x 2 nllg nllg 4096  4月 27 10:19 ./
drwxrwxr-x 9 nllg nllg 4096  4月 27 10:19 ../
lrwxrwxrwx 1 nllg nllg    6  4月 27 10:19 App -> ../app/
lrwxrwxrwx 1 nllg nllg    7  4月 27 10:19 Core -> ../core/
lrwxrwxrwx 1 nllg nllg   12  4月 27 10:19 ModeStyle -> ../modeStyle/
lrwxrwxrwx 1 nllg nllg   10  4月 27 10:19 ModuleA -> ../moduleA/
lrwxrwxrwx 1 nllg nllg   10  4月 27 10:19 ModuleB -> ../moduleB/
lrwxrwxrwx 1 nllg nllg   13  4月 27 10:19 ViewModels -> ../viewmodels/
```
目录连接所有src文件夹，才能找到qml文件实现热更新，
我发现build目录下也有 /home/wangshuobin/project/wheel_scanning/build/Clang14_Qt_6_10_2-Debug/src 文件夹内容
```bash
nllg@LegionR7000:~/my_project/wheel_scanning/build/Clang14_Qt_6_10_2-Debug/src$ ll
总计 28
drwxrwxr-x  7 nllg nllg 4096  4月  1 13:01 ./
drwxrwxr-x 11 nllg nllg 4096  5月  9 09:56 ../
drwxrwxr-x  4 nllg nllg 4096  5月  9 09:56 core/
drwxrwxr-x  9 nllg nllg 4096  5月  9 09:56 modeStyle/
drwxrwxr-x  9 nllg nllg 4096  5月  9 09:56 moduleA/
drwxrwxr-x  9 nllg nllg 4096  5月  9 09:56 moduleB/
drwxrwxr-x  9 nllg nllg 4096  5月  9 09:56 viewmodels/
```
和/home/wangshuobin/project/wheel_scanning/build/Clang14_Qt_6_10_2-Debug/WheelScanning
```bash
nllg@LegionR7000:~/my_project/wheel_scanning/build/Clang14_Qt_6_10_2-Debug/WheelScanning$ ll
总计 28
drwxrwxr-x  7 nllg nllg 4096  4月  1 13:01 ./
drwxrwxr-x 11 nllg nllg 4096  5月  9 09:56 ../
drwxrwxr-x  4 nllg nllg 4096  5月  9 09:56 App/
drwxrwxr-x  2 nllg nllg 4096  5月  9 09:56 ModeStyle/
drwxrwxr-x  2 nllg nllg 4096  5月  9 09:56 ModuleA/
drwxrwxr-x  2 nllg nllg 4096  5月  9 09:56 ModuleB/
drwxrwxr-x  2 nllg nllg 4096  5月  9 09:56 ViewModels/
```
基本相同，是否可以修改模块URI的写法，不在需要 src路径下设置/home/wangshuobin/project/wheel_scanning/src/WheelScanning，目录连接所有src文件夹，才能找到qml文件也可实现热更新

## 为什么现在必须要 `src/WheelScanning/` 这一串符号链接

QML 模块的解析规则是：导入路径 + URI 按 `.` 拆成子目录，必须能找到 `qmldir` 文件。

你现在的设置：
```cmake
qt_add_qml_module(app${PROJECT_NAME}
    URI WheelScanning.App
    VERSION 1.0
    QML_FILES
        src/app/Main.qml
    RESOURCES
        res/resource.qrc
    OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/WheelScanning/App
)

qt_add_qml_module(modeStyle
    URI WheelScanning.ModeStyle
    VERSION 1.0
    ...
    OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/WheelScanning/ModeStyle
)
```
```cpp
#ifdef QT_DEBUG
    QString qmlRoot = QStringLiteral(QML_SOURCE_DIR);  // = ${CMAKE_SOURCE_DIR}/src
    ...
    importPaths.prepend(qmlRoot);
    engine.setImportPathList(importPaths);
    ...
    engine.load(QUrl::fromLocalFile(qmlRoot + "/app/Main.qml"));
```

Debug 时 import path = `src/`，URI = `WheelScanning.ModuleA`，所以 Qt 必须找到 `src/WheelScanning/ModuleA/qmldir`——这正是你那堆符号链接的来源。而真正自动生成的 `qmldir` 在 `build/.../WheelScanning/ModuleA/qmldir`（你看到的另一份），并不在 `src/` 下。

至于你提到的 `build/.../src/`——那只是各 CMake 子目录的构建产物（`*_autogen`、`*_qmltyperegistrations.cpp`、`meta_types/`、`qmltypes/`），没有 `qmldir`、也没有 QML 文件，跟 `build/.../WheelScanning/` 内容并不相同，不能直接当 import path 用。

另外我刚验证过，`build/.../WheelScanning/ModuleA/Area1.qml` 是真实拷贝（同 md5、独立 inode、非软链），所以"把 import path 改到 `${CMAKE_BINARY_DIR}`"无法实现热重载——文件每次构建才会被复制过来。

---

## 可行的去符号链接方案（推荐二选一）

### 方案 A：修改 URI 让其与目录名一致（直接回答你的问题）

把多段 URI 改成单段，且与现有 `src/` 下文件夹名严格一致；同时把 `qmldir` 直接写到源码目录，让 import path = `src/` 时天然就能命中。

具体改动：

|模块|原 URI|新 URI|源码目录|
|---|---|---|---|
|根模块|`WheelScanning.App`|`App`（或把 `src/app` 改名为 `src/App`，二者择一以匹配大小写）|`src/app`|
|modeStyle|`WheelScanning.ModeStyle`|`modeStyle`|`src/modeStyle`|
|moduleA|`WheelScanning.ModuleA`|`moduleA`|`src/moduleA`|
|moduleB|`WheelScanning.ModuleB`|`moduleB`|`src/moduleB`|
|viewmodels|`WheelScanning.ViewModels`|`viewmodels`|`src/viewmodels`|

每个 `qt_add_qml_module(...)` 改成：

```cmake
qt_add_qml_module(moduleA
    URI moduleA
    VERSION 1.0
    QML_FILES ${MODULEA_QML_FILES}
    OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}   # 关键：qmldir 写到源码目录
)
```

随之要做：

1. 删除 `src/WheelScanning/` 符号链接目录、删除 `src/qmldir`。
2. 全局搜索替换 QML 里的 `import WheelScanning.ModuleA` → `import moduleA`，其它模块同理。`src/app/Main.qml` 第 1-3 行就要改。
3. `main.cpp` Release 分支的 `engine.loadFromModule("WheelScanning.App", "Main")` 改成 `loadFromModule("App", "Main")`。
4. `.gitignore` 增加：

```gitignore
src/*/qmldir
src/*/*.qmltypes
```

效果：debug 模式下 import path 仍是 `src/`，Qt 直接读 `src/moduleA/qmldir` 和源 QML 文件 → 真正的零拷贝热重载，不需要任何符号链接。Release 模式 qrc 路径变成 `:/qt/qml/moduleA/` 一并自动适配。

代价：

- URI 失去 `WheelScanning.` 命名空间（如果项目里没有同名冲突就无所谓）。
- 自动生成的 `qmldir`/`*.qmltypes` 落到源码树里（必须 gitignore）。
- 需要批量改 QML import 语句（不多，主要在 `Main.qml` 和模块间互相引用处）。

---

### 方案 B：保留现有 URI，用 URL 拦截器实现热重载（推荐，改动最小）

这是 Qt 官方处理 "release 用 qrc、debug 用源文件" 的标准玩法，无需改 URI、无需改 CMake、不需要任何符号链接。

原理：`build/.../WheelScanning/ModuleA/qmldir` 里有这一行：

prefer :/qt/qml/WheelScanning/ModuleA/

Qt 解析每个 QML 类型时都会把它前置成 `qrc:/qt/qml/WheelScanning/ModuleA/Area1.qml`。我们在 debug 模式装一个 `QQmlAbstractUrlInterceptor`，把这种 qrc URL 重定向到源码路径即可。

`main.cpp` 改成下面这样（示意）：
```cpp
#include <QQmlAbstractUrlInterceptor>

#ifdef QT_DEBUG
class SourceUrlInterceptor : public QQmlAbstractUrlInterceptor {
public:
    QUrl intercept(const QUrl &url, DataType) override {
        if (url.scheme() != "qrc") return url;
        const QString p = url.path();
        // 只拦截我们自己的模块前缀
        static const QString prefix = QStringLiteral("/qt/qml/WheelScanning/");
        if (!p.startsWith(prefix)) return url;
        // /qt/qml/WheelScanning/ModuleA/Area1.qml -> ModuleA/Area1.qml
        QString tail = p.mid(prefix.size());
        // URI 段 -> 源码目录段
        static const QHash<QString, QString> map = {
            {"App", "app"}, {"ModeStyle", "modeStyle"},
            {"ModuleA", "moduleA"}, {"ModuleB", "moduleB"},
            {"ViewModels", "viewmodels"},
        };
        const int slash = tail.indexOf('/');
        if (slash < 0) return url;
        const QString seg = tail.left(slash);
        auto it = map.find(seg);
        if (it == map.end()) return url;
        return QUrl::fromLocalFile(QStringLiteral(QML_SOURCE_DIR) + "/" + *it + tail.mid(slash));
    }
};
#endif

int main(int argc, char *argv[]) {
    QGuiApplication app(argc, argv);
    ...
    QQmlApplicationEngine engine;
#ifdef QT_DEBUG
    qputenv("QML_DISABLE_DISK_CACHE", "1");
    static SourceUrlInterceptor interceptor;
    engine.addUrlInterceptor(&interceptor);
#endif
    engine.loadFromModule("WheelScanning.App", "Main");   // debug/release 用同一行
    return app.exec();
}
```
随之删除 `src/WheelScanning/` 符号链接目录与 `src/qmldir` 即可。

优点：

- URI、CMake、QML 源文件统统不动；
- debug/release 加载路径统一为 `loadFromModule`，再无两套代码；
- 完整热重载（重启进程即生效，因为读源文件）。

代价：

- 多一个拦截器类（约 20 行代码），需要维护一份 `URI 段 → 源目录段` 的映射表（5 个条目）。

---

## 我的建议

如果你接受改 URI 命名，方案 A 更"干净"——build/部署一致性最好，源码层级与模块系统对齐，缺点是改的文件比较多。

如果你想改动最小、保持现有命名空间感，选方案 B——只改 `main.cpp` 一处，删一组符号链接就完事，是 Qt 项目里最通行的热重载做法。

你倾向哪一种？我可以直接给你完整改动 patch。