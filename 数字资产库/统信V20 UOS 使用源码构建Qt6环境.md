1、Qt for Linux/X11 - 从源代码构建 https://doc.qt.io/qt-6/zh/linux-building.html

2、构建Qt6源代码 https://doc.qt.io/qt-6/zh/build-sources.html

3、从Git仓库中获取Qt6源代码 https://doc.qt.io/qt-6/zh/getting-sources-from-git.html

4、Clang14 https://github.com/llvm/llvm-project/releases?q=14&expanded=true

## 1️⃣ 下载

在浏览器里点：

```
clang+llvm-14.0.6-x86_64-linux-gnu-ubuntu-18.04.tar.xz
```

或终端：

```
wget https://github.com/llvm/llvm-project/releases/download/llvmorg-14.0.6/clang+llvm-14.0.6-x86_64-linux-gnu-ubuntu-18.04.tar.xz
```

------

## 2️⃣ 解压到 /opt（推荐做法）

```
cd ~/Desktop
sudo mkdir -p /opt/llvm-14
sudo tar -xf clang+llvm-14.0.6-x86_64-linux-gnu-ubuntu-18.04.tar.xz -C /opt/llvm-14 --strip-components=1
```

------

## 3️⃣ 配环境变量

```
nano ~/.bashrc
```

末尾加：

```
export PATH=/opt/llvm-14/bin:$PATH
export LD_LIBRARY_PATH=/opt/llvm-14/lib:$LD_LIBRARY_PATH
```

然后：

```
source ~/.bashrc
```

------

## 4️⃣ 验证（这一步很关键）

```
clang --version
```

你应该看到类似：

```
clang version 14.0.6
```

👉 **看到 14.x = 成功**
👉 **不是 7.x = 说明你真正切换成功了**



# 三、你现在应该用的**正确 configure 命令**

## 第一步：确认编译器（必须是 Clang14）

```
export CC=/opt/llvm-14/bin/clang
export CXX=/opt/llvm-14/bin/clang++

export LD_LIBRARY_PATH=/opt/llvm-14/lib:$LD_LIBRARY_PATH
export CXXFLAGS="-stdlib=libc++"
export LDFLAGS="-stdlib=libc++ -lc++abi"

cd ~/Desktop/qt/qt-build
rm -rf *   
```

------

## 第二步：清空 build 目录

```
cd ~/Desktop/qt/qt-build
rm -rf *
```

------

## 第三步：**用 Qt6 正确参数重新 configure**

👉 **这一条是为 UOS + 老 CPU + 老 libc 定制的安全组合：**

```
../qt-sources/configure \
  -prefix /opt/qt6 \
  -opensource -confirm-license \
  -release \
  -nomake examples -nomake tests \
  -skip qtwebengine
```

👉 **注意：**

- 不要 `-clang`

- 不要 `-no-feature-rdrand`

- 不要 `-no-feature-x86intrin`

- Qt 会自动根据 CPU 能力关闭不支持的指令

  仍报错，那么我们走**终极稳妥方案**：

  ```
  ../qt-sources/configure \
    -prefix /opt/qt6 \
    -opensource -confirm-license \
    -release \
    -nomake examples -nomake tests \
    -skip qtwebengine \
    -DQT_FEATURE_x86intrin=OFF
  ```

  👉 这个是 **CMake 级别**关闭，不是 Qt feature 名字猜测，所以一定生效。

------

## ④ 编译

```
cmake --build . --parallel $(nproc)
```

------

## ⑤ 安装

```
sudo cmake --install .
```



## ✅ 可能可行方案 A：**全程 Clang + libc++ 编译 Qt6**

你已经做对一半了：

```
clang 14.0.6 installed in /opt/llvm-14
```

这是**正确方向** 👍

### 关键点：

不要再让它链接 **系统的 libstdc++ (GCC 8)**
要强制使用 **LLVM 自带 libc++**

否则就会出现你现在这种：

```
std::filesystem undefined reference
```

### 正确环境变量设置：

```
export CC=/opt/llvm-14/bin/clang
export CXX=/opt/llvm-14/bin/clang++
export LD_LIBRARY_PATH=/opt/llvm-14/lib:$LD_LIBRARY_PATH
export CXXFLAGS="-stdlib=libc++"
export LDFLAGS="-stdlib=libc++ -lc++abi"

cd ~/Desktop/qt/qt-build
rm -rf *   
```

然后重新：

```
../qt-sources/configure \
  -prefix /opt/qt6 \
  -opensource -confirm-license \
  -release \
  -nomake examples -nomake tests \
  -skip qtwebengine
```

接着：

```
cmake --build . --parallel $(nproc)
```

👉 这是**目前在 UOS 上最稳的一条路**

