# 本文档说明如何将 FluentUI 动态组件库编译并安装到 Qt 的 qml 模块目录。

## 环境要求

- Qt 6.10.3 (安装在 `/opt/Qt/6.10.3/gcc_64`)
- Clang 14 或 GCC
- CMake 3.20+
- Ninja 构建系统

## 构建步骤

### 1. 配置构建

使用 CMake Presets 配置，**关键是指定 FLUENTUI_QML_PLUGIN_DIRECTORY 到本地构建目录**，避免在 configure 阶段尝试写入系统目录导致权限错误。

```bash
cmake --preset clang_rel \
  -DFLUENTUI_QML_PLUGIN_DIRECTORY=${PWD}/build/Clang14_Qt_6_10_3-Release/FluentUI
```

**重要提示：**
- ❌ 不要使用 `sudo` 执行 configure 命令
- ❌ 不要在 configure 时设置 `-DCMAKE_INSTALL_PREFIX`
- ✅ 必须指定 `FLUENTUI_QML_PLUGIN_DIRECTORY` 到构建目录

### 2. 编译

```bash
cmake --build --preset clang_rel
```

编译过程约 2 分钟（取决于 CPU 性能），会生成：
- 动态库：`build/Clang14_Qt_6_10_3-Release/FluentUI/libfluentuiplugin.so`
- QML 文件：`build/Clang14_Qt_6_10_3-Release/FluentUI/Controls/*.qml`
- 资源文件：字体、JavaScript 等

**重要提示：**
- ❌ 不要使用 `sudo` 执行 build 命令

### 3. 安装到 Qt 目录

将构建产物复制到 Qt 的 qml 目录（**此步骤需要 sudo 权限**）：

```bash
sudo cp -r build/Clang14_Qt_6_10_3-Release/FluentUI /opt/Qt/6.10.3/gcc_64/qml/
```

### 4. 验证安装

检查安装是否成功：

```bash
ls -lh /opt/Qt/6.10.3/gcc_64/qml/FluentUI/
```

应该能看到：
- `libfluentuiplugin.so` - 动态库
- `qmldir` - QML 模块描述文件
- `Controls/` - QML 控件目录
- `Font/` - 字体资源
- `JS/` - JavaScript 资源

## 使用 FluentUI

在你的 QML 项目中导入：

```qml
import FluentUI 1.0

FluWindow {
    // 你的应用代码
}
```

## 常见问题

### Q1: Configure 阶段报错 "Could not open file for write"

**原因：** CMake 在 configure 阶段尝试直接写入 Qt 系统目录，但没有权限。

**解决：** 必须指定 `-DFLUENTUI_QML_PLUGIN_DIRECTORY` 到构建目录（见步骤 1）。

### Q2: Configure 阶段找不到 Qt5X11Extras

**原因：** 使用 `sudo` 执行 cmake 导致环境变量被清理，CMake 找到了系统的 Qt5 而不是预设中的 Qt6。

**解决：** 不要使用 `sudo` 执行 configure 和 build 命令。

### Q3: 构建目录权限错误 "Unable to create pkgRedirects directory"

**原因：** 之前使用 `sudo` 创建的构建目录属于 root 用户。

**解决：** 清理构建目录后重新构建：
```bash
sudo rm -rf build/Clang14_Qt_6_10_3-Release
cmake --preset clang_rel -DFLUENTUI_QML_PLUGIN_DIRECTORY=${PWD}/build/Clang14_Qt_6_10_3-Release/FluentUI
cmake --build --preset clang_rel
```

## 权限总结

| 操作阶段 | 是否使用 sudo | 原因 |
|---------|--------------|------|
| Configure | ❌ 否 | 避免环境变量问题和权限混乱 |
| Build | ❌ 否 | 构建在用户目录，不需要特权 |
| Install | ✅ 是 | 需要写入系统目录 `/opt/Qt/` |

## 清理

如果需要重新构建：

```bash
# 清理构建目录
rm -rf build/Clang14_Qt_6_10_3-Release

# 如果需要卸载
sudo rm -rf /opt/Qt/6.10.3/gcc_64/qml/FluentUI
```

## CMake Presets 说明

项目使用的 CMake Presets 配置（`CMakePresets.json`）：

- `clang_rel` - Clang 编译器，Release 模式（推荐）
- `clang_debug` - Clang 编译器，Debug 模式
- `gcc_rel` - GCC 编译器，Release 模式
- `gcc_debug` - GCC 编译器，Debug 模式

Presets 已预配置 Qt 路径为 `/opt/Qt/6.10.3/gcc_64`。

## 参考

- FluentUI 项目：https://github.com/zhuzichu520/FluentUI
- Qt6 QML 模块文档：https://doc.qt.io/qt-6/qtqml-modules-topic.html