项目地址：https://github.com/zhuzichu520/FluentUI?tab=readme-ov-file

```bash
sudo cmake -S .. -B . \
-DCMAKE_PREFIX_PATH=/opt/Qt/6.10.2/gcc_64 \
-DCMAKE_BUILD_TYPE=Debug \
-DFLUENTUI_QML_PLUGIN_DIRECTORY=/opt/Qt/6.10.2/gcc_64/qml/FluentUI
# 编译并安装
sudo cmake --build . --target install
```

  
文件：fluent_ui/src/CMakeLists.txt:41-43 安装到Qt组件目录下
```cmake
#设置QML插件输出目录，可以通过外部设置，如果外部没有设置就默认到<QT_SDK_DIR_PATH>\qml\FluentUI目录下
set(FLUENTUI_QML_PLUGIN_DIRECTORY "" CACHE PATH "Path to FluentUI plugin")
if (NOT FLUENTUI_QML_PLUGIN_DIRECTORY)
    set(FLUENTUI_QML_PLUGIN_DIRECTORY ${QT_SDK_DIR}/qml/FluentUI)
endif ()
```
