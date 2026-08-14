[[Qt6 QML 宏机制]]

注册方式：在 C++ 头文件中直接用 `QML_ELEMENT` + `QML_SINGLETON` 宏声明，不需要 `main.cpp` 中手动调用任何注册函数。

CMake 侧：通过 `qt_add_qml_module(viewmodels URI WheelScanning.ViewModels SOURCES ...)` 把所有 ViewModel 源文件打进一个 QML 模块，Qt 的 CMake 宏自动处理类型注册。

QML 侧：`import WheelScanning.ViewModels` 后直接用类型名 `SharedMemoryManager.xxx` 访问单例。

`main.cpp` 中唯一残留的 `setContextProperty` 只有 `appDataPath`（一个纯字符串值），没有任何 C++ 对象通过 context property 注入。

对比你当前的 `gas_control_bringup`，差异在于：

||`wheel_scanning`|`gas_control_bringup`|
|---|---|---|
|单例注册|`QML_ELEMENT` + `QML_SINGLETON` 宏（声明式）|`setContextProperty`（命令式）|
|实例创建者|Qt QML 引擎自动创建|`main()` 栈上手动创建|
|构造控制|需在类中提供 `static create()` 工厂方法或默认构造|完全手动控制|

|文件|改动|
|---|---|
|`authmanager.h`|+`QML_ELEMENT` +`QML_SINGLETON` +`Q_DISABLE_COPY_MOVE` +`<QQmlEngine>`|
|`viewmodels/CMakeLists.txt`|SOURCES 加入 `${VIEWMODEL_HEADERS}`|
|`generalstyle/CMakeLists.txt`|OPTIONAL_IMPORTS 加入 `gascontrolbringup.viewmodels`|
|`main.cpp`|删除 `AuthManager` 实例和 `setContextProperty` 注册|
|`ChangePasswordDialog.qml`|`import gascontrolbringup.viewmodels`，`authManager` → `AuthManager`|
|`LogIn.qml`|同上|
|`SettingsDialog.qml`|同上|