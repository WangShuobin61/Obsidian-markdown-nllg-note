以下是从 CMake Release 构建到 AppImage 打包完成的完整命令流程总结。这个过程将您的源代码转化为了一个可在任何 Linux 系统上直接运行的独立程序包。

### 阶段一：构建与安装 (Build & Install)
这一步将源代码编译成二进制文件，并按照标准的 Linux 目录结构（bin, lib, plugins 等）安装到一个临时文件夹中，为打包做准备。

1. CMake 配置 (Configure)
   
   ```sh
   sudo mv /opt/Qt/6.10.1/gcc_64/plugins/sqldrivers_backup/libqsqlmimer.so /opt/Qt/6.10.1/gcc_64/plugins/sqldrivers/ && 
   sudo mv /opt/Qt/6.10.1/gcc_64/plugins/sqldrivers_backup/libqsqloci.so /opt/Qt/6.10.1/gcc_64/plugins/sqldrivers/ && 
   sudo rmdir /opt/Qt/6.10.1/gcc_64/plugins/sqldrivers_backup && 
   sudo mv /opt/Qt/6.10.1/gcc_64/plugins/position_backup/libqtposition_nmea.so /opt/Qt/6.10.1/gcc_64/plugins/position/ && 
   sudo rmdir /opt/Qt/6.10.1/gcc_64/plugins/position_backup
   ```
   
   ```sh
   cmake --preset clang_rel
   ```
   - 作用 : 根据 CMakePresets.json 中的配置（指定了编译器 Clang 14 和 Qt 6.10.1 环境），生成构建系统文件（Makefile 或 Ninja 文件）。
   
2. CMake 构建 (Build)
   
   ```sh
   cmake --build --preset clang_rel
   ```
   - 作用 : 执行编译链接过程，生成可执行文件 appwheel_scanning 和各个模块的动态库（如 libviewmodels.so , libcore.a 等）。
   
3. CMake 安装 (Install)
   
   ```sh
   cmake --install build/Clang14_Qt_6_10_1-Release --prefix release_output
   ```
   - 作用 : 将编译好的程序、库文件、资源文件复制到 release_output 目录。这是打包的“原材料”目录。我们之前修改了多个 CMakeLists.txt 添加 install 指令就是为了这一步能正确收集所有文件。
### 阶段二：环境与工具准备 (Preparation)
这一步确保打包工具就绪，并解决系统依赖问题。

4. 安装系统依赖
   
   ```
   sudo apt-get install -y libfbclient2
   ```
   - 作用 : 解决 linuxdeployqt 运行时报错找不到 libfbclient.so 的问题。这是 Qt SQL 驱动的一个依赖项。
5. 下载打包工具
   
   ```sh
   wget https://github.com/probonopd/linuxdeployqt/releases/download/
   continuous/linuxdeployqt-continuous-x86_64.AppImage
   wget https://github.com/AppImage/appimagetool/releases/download/continuous/
   appimagetool-x86_64.AppImage
   wget https://github.com/AppImage/type2-runtime/releases/download/
   continuous/runtime-x86_64
   chmod +x *.AppImage runtime-x86_64
   ```
   - 作用 : 下载并赋予执行权限。
     - linuxdeployqt : 用于自动收集 Qt 依赖。
     - appimagetool : 用于生成最终的 AppImage 文件。
     - runtime-x86_64 : AppImage 的运行环境（因为自动下载失败，我们需要手动下载备用）。
6. 临时隔离有问题的插件 (可选但重要)
   
   ```sh
   sudo mkdir -p /opt/Qt/6.10.1/gcc_64/plugins/sqldrivers_backup && 
   sudo mv /opt/Qt/6.10.1/gcc_64/plugins/sqldrivers/libqsqlmimer.so /opt/Qt/6.10.1/gcc_64/plugins/sqldrivers_backup/ && 
   sudo mv /opt/Qt/6.10.1/gcc_64/plugins/sqldrivers/libqsqloci.so /opt/Qt/6.10.1/gcc_64/plugins/sqldrivers_backup/ && 
   sudo mkdir -p /opt/Qt/6.10.1/gcc_64/plugins/position_backup && 
   sudo mv /opt/Qt/6.10.1/gcc_64/plugins/position/libqtposition_nmea.so /opt/Qt/6.10.1/gcc_64/plugins/position_backup/
   ```
   - 作用 : 暂时移走 Mimer 和 Oracle OCI 等数据库插件。因为这些插件依赖特定的商业库，如果系统里没有安装， linuxdeployqt 扫描时会报错并中断打包。打包完后我们已将它们恢复。
### 阶段三：依赖部署与打包 (Deploy & Package)
这是核心步骤，将程序变成“自带干粮”的独立包。

7. 生成 Desktop 文件
   
   - 操作 : 在 release_output 目录下创建 default.desktop 文件。
   - 作用 : 告诉 Linux 系统这个程序的名称、图标和启动命令，是 AppImage 的元数据。
   
8. 部署 Qt 依赖 (Deploy)
   
   ```sh
   ./linuxdeployqt-continuous-x86_64.AppImage release_output/default.desktop -qmldir=src -qmake=/opt/Qt/6.10.1/gcc_64/bin/qmake -bundle-non-qt-libs -verbose=2
   
   export VERSION=1.0-beta
   ./linuxdeployqt-continuous-x86_64.AppImage release_output/default.desktop -appimage -qmldir=src -qmake=/opt/Qt/6.10.1/gcc_64/bin/qmake
   
   export VERSION=1.0-beta
   ./linuxdeployqt-continuous-x86_64.AppImage release_output/default.desktop \
     -qmldir=src \
     -qmake=/opt/Qt/6.10.1/gcc_64/bin/qmake \
     -appimage
     -bundle-non-qt-libs
     -verbose=2
     
   export VERSION=1.0-beta
   cd dist && \
     ../linuxdeployqt-continuous-x86_64.AppImage ../release_output/default.desktop -qmldir=../src -qmake=/opt/Qt/6.10.1/gcc_64/bin/qmake -appimage -bundle-non-qt-libs -verbose=2
     
     ../linuxdeployqt-continuous-x86_64.AppImage AppDir/default.desktop -qmldir=../src -qmake=/opt/Qt/6.10.1/gcc_64/bin/qmake -bundle-non-qt-libs -verbose=2 -appimage
   ```
   - 作用 :
     - -qmldir=src : 扫描源代码中的 QML 导入，自动复制用到的 QML 模块。
     - -bundle-non-qt-libs : 不仅复制 Qt 库，还复制程序用到的其他非系统核心的 .so 库。
     - 该命令会自动修改 release_output 中的文件结构，创建一个自包含的 AppDir 。
     - ***注意：使用 -appimage 参数时，linuxdeployqt 会自动调用内置的工具来生成文件，这种方式通常兼容性最好。***
   
9. 生成 AppImage (Package)
   
   ```sh
   // ARCH=x86_64 ./appimagetool-x86_64.AppImage --runtime-file runtime-x86_64 release_output
   ARCH=x86_64 ./appimagetool-x86_64.AppImage release_output /release_test/Wheel_Scanning_v1.0_Bate.AppImage
   ```
   - 作用 : 将处理好的 release_output 目录压缩成一个单一的文件 Wheel_Scanning-x86_64.AppImage 。
   - --runtime-file : 指定使用我们手动下载的运行环境，解决自动下载失败的问题。

​		9.2. 指定生成的完整路径

如果你想把它生成到特定的文件夹中（例如 deploy 目录）：

```
mkdir -p release_test
ARCH=x86_64 ./appimagetool-x86_64.AppImage release_output ./release_test/Wheel_Scanning_v1.0.AppImage
```
​		9.3. 如果不指定会怎样？

如果你不写最后一个参数（即你之前用的命令）：

```
ARCH=x86_64 ./appimagetool-x86_64.AppImage release_output
```
appimagetool 会根据 release_output 目录下的 .desktop 文件中的 Name 字段， 自动生成 一个文件名，通常格式为 [Name]-x86_64.AppImage 。

### 阶段四：清理与恢复 (Cleanup)
打包完成后，恢复环境以避免影响后续开发。

10. 恢复 Qt 插件
    
    ```sh
    sudo mv /opt/Qt/6.10.1/gcc_64/plugins/sqldrivers_backup/libqsqlmimer.so /opt/Qt/6.10.1/gcc_64/plugins/sqldrivers/ && 
    sudo mv /opt/Qt/6.10.1/gcc_64/plugins/sqldrivers_backup/libqsqloci.so /opt/Qt/6.10.1/gcc_64/plugins/sqldrivers/ && 
    sudo rmdir /opt/Qt/6.10.1/gcc_64/plugins/sqldrivers_backup && 
    sudo mv /opt/Qt/6.10.1/gcc_64/plugins/position_backup/libqtposition_nmea.so /opt/Qt/6.10.1/gcc_64/plugins/position/ && 
    sudo rmdir /opt/Qt/6.10.1/gcc_64/plugins/position_backup
    ```
    - 作用 : 将之前临时移走的插件放回原位，保证开发环境的完整性。
    
11. (可选) 创建 Tar 包
    
    ```sh
    tar -czf Wheel_Scanning-x86_64.tar.gz release_output
    ```
    - 作用 : 除了 AppImage，额外打一个传统的压缩包作为备份发布格式。



### ./linuxdeployqt-continuous-x86_64.AppImage 命令执行完

是的， 原则上是可以的 ，但这种“到处运行”的方式是 文件夹形式 的便携版，而不是单文件形式。

在您执行完 linuxdeployqt 这条命令后， release_output 目录就已经变成了一个 自包含的运行环境 （通常被称为 AppDir）。

### 此时 release_output 目录的状态：
1. 依赖齐全 : 所有的 Qt 库（ libQt6Core.so 等）、插件（platforms, sqldrivers 等）以及非系统标准库都已经被拷贝到了目录内部（通常在 lib/ 和 plugins/ 子目录下）。
2. 路径修正 : linuxdeployqt 已经自动修改了二进制文件的 RPATH ，让程序优先去当前目录下的 lib 找库，而不是去系统的 /usr/lib 找。
3. AppRun 脚本 : 目录下会生成一个 AppRun 脚本（或者是指向可执行文件的软链接）。
### 如何运行程序？
您可以直接把整个 release_output 文件夹拷贝到另一台（同样是 Linux x86_64 架构的）电脑上，然后运行目录下的 AppRun 文件：

```
./release_output/AppRun
```
程序应该就能正常启动，不需要那台电脑安装 Qt。

### 为什么还需要下一步（appimagetool）？

ARCH=x86_64 ./appimagetool-x86_64.AppImage --runtime-file runtime-x86_64 release_output

虽然文件夹形式已经可以运行，但它有几个缺点，所以通常建议进行最后一步打包：

1. 文件散乱 : 文件夹里包含成百上千个小文件（库、资源、插件），传输和管理很不方便（容易丢文件）。
2. 权限问题 : 拷贝过程中可能会丢失可执行权限。
3. 用户体验 : 给用户一个单独的 .AppImage 文件（双击即用），比给一个文件夹让他们“进去找 AppRun 脚本运行”体验要好得多。
总结 ：执行完 linuxdeployqt 后，您其实已经得到了一个**“绿色版/免安装版”的文件夹**。如果您把它压缩成 .tar.gz （就像我们刚才做的），那就是一个标准的绿色软件发布包。而 appimagetool 只是把这个绿色文件夹进一步封装成了一个更高级的“单文件可执行包”。



**怎样把最终 AppImage 和中间文件分离？**

### 方法 1：在 dist 目录中执行打包（最简单）

```
mkdir -p dist
export VERSION=1.0-beta
(cd dist && \
  ../linuxdeployqt-continuous-x86_64.AppImage ../default.desktop -appdir=../release_output -qmldir=../src -qmake=/opt/Qt/6.10.1/gcc_64/bin/qmake -appimage
```

**效果**：

- 中间文件都在 release_output/
- 最终 AppImage 在 dist/
- 项目根目录不会再出现 AppRun / default.png / .DirIcon / default.desktop

------

### 方法 2：固定 AppDir，只改变输出目录

linuxdeployqt 的 AppImage 输出位置**由当前工作目录决定**。
所以只要在 dist/ 目录执行，就能分离输出。

------

**结论**

- **最终 AppImage 文件可独立运行**。
- **推荐把中间 AppDir 和最终输出分开**（release_output + dist）。
- **分离方法**就是在 dist/ 目录执行 -appimage 打包，或者显式指定 -appdir=release_output 并切换工作目录。



### 第一条命令：打包与清理

```sh
chmod +x release_output/AppRun && 
tar -czf WheelScanning_Linux_x86_64.tar.gz release_output && 
```
- chmod +x release_output/AppRun :
  - 作用 : 确保启动脚本 AppRun 具有“可执行”权限。如果没有这个权限，用户下载后双击是没反应的。
- tar -czf WheelScanning_Linux_x86_64.tar.gz release_output :
  - 作用 : 将整个 release_output 文件夹（也就是那个“绿色版”文件夹）压缩成一个 .tar.gz 文件。这就是我们常见的“便携版”安装包。

### 第二条命令：模拟安装与验证 

```sh
mkdir -p test_release && tar -xzf WheelScanning_Linux_x86_64.tar.gz -C 
test_release && 
cd test_release/release_output && 
./AppRun --version
```
这条命令是在模拟一个完全没有源代码的“用户电脑”环境，验证包是否真的能跑起来：

- mkdir -p test_release :
  - 作用 : 创建一个全新的、干净的目录 test_release 。
- tar -xzf WheelScanning_Linux_x86_64.tar.gz -C test_release :
  - 作用 : 将刚刚打好的压缩包解压到这个新目录里。这模拟了用户下载并解压的过程。
- cd test_release/release_output && ./AppRun --version :
  - 作用 : 进入解压后的目录，尝试运行程序并查看版本号。
  - 核心目的 : 如果程序能打印出版本号（或者报错信息，只要不是提示“找不到某某 .so 库”），就说明我们的依赖打包（Deploy）是成功的，程序已经具备了独立运行的能力。
  总结 ：这两步操作确保了你发给用户的压缩包是 完整且可用 的，而不是一个“在我的电脑上能跑，发给别人就报错”的半成品。