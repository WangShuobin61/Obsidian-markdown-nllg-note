# 完整命令

```
docker run -it --rm \
 -e DISPLAY=$DISPLAY \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 -v $HOME/.local/share/GasControl:/root/.local/share/GasControl \
 gas_control
```

其中 `\` 表示 **Linux 的续行符**，意思是：

```
docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/.local/share/GasControl:/root/.local/share/GasControl gas_control
```

和写成一行效果完全一样。

---

# ① docker run

```
docker run
```

作用：

> 创建并启动一个新的容器。

它的大致语法是：

```
docker run [OPTIONS] IMAGE [COMMAND] [ARG...]
```

例如：

```
docker run ubuntu
```

就是启动 Ubuntu 镜像。

你的：

```
docker run gas_control
```

就是启动

```
gas_control:latest
```

这个镜像。

---

# ② -it

实际上：

```
-it
```

等于：

```
-i-t
```

两个参数。

---

## -i

```
-i
```

全称：

```
--interactive
```

作用：

保持 STDIN （标准输入）打开。

例如：

没有 `-i`

```
docker run ubuntu bash
```

bash 会立刻退出。

有：

```
docker run -i ubuntu bash
```

bash 可以等待输入。

---

## -t

```
-t
```

全称：

```
--tty
```

作用：

分配一个终端。

例如：

没有：

```
ls
```

输出可能没有颜色。

有：

```
ls
```

会和彩色终端一致。

通常：

```
-it
```

都是一起使用。

---

# ③ --rm

```
--rm
```

作用：

> 容器退出以后自动删除。

例如：

运行：

```
docker run --rm ubuntu
```

退出后：

```
docker ps -a
```

看不到这个容器。

如果没有：

```
docker run ubuntu
```

退出后：

```
docker ps -a
```

还能看到：

```
Exited
```

状态。

开发调试建议：

```
--rm
```

生产环境：

一般不用。

---

# ④ -e DISPLAY=$DISPLAY

```
-e DISPLAY=$DISPLAY
```

实际上：

```
--env
```

设置环境变量。

等于：

```
--env DISPLAY=:0
```

假设：

宿主机：

```
echo $DISPLAY
```

输出：

```
:0
```

Docker 中就会得到：

```
DISPLAY=:0
```

Qt 就知道：

去连接：

```
X11 Server
```

否则：

Qt 会报：

```
could not connect to display
```

---

## 常见 -e

例如：

```
-e QT_DEBUG_PLUGINS=1
```

开启 Qt 插件调试。

例如：

```
-e LANG=zh_CN.UTF-8
```

设置中文。

例如：

```
-e TZ=Asia/Shanghai
```

设置时区。

---

# ⑤ -v /tmp/.X11-unix:/tmp/.X11-unix

这是：

```
-v 主机目录:容器目录
```

也叫：

```
Bind Mount
```

作用：

共享目录。

---

为什么共享：

```
/tmp/.X11-unix
```

因为：

这里面有：

```
X Socket
```

例如：

```
X0
```

Qt 就是通过：

```
/tmp/.X11-unix/X0
```

去连接宿主 X Server。

没有这一句：

Qt 会报：

```
could not connect to display
```

---

# ⑥

```
-v $HOME/.local/share/GasControl:/root/.local/share/GasControl
```

作用：

保存程序数据。

假设：

程序会写：

```
配置数据库日志历史记录
```

写到：

```
/root/.local/share/GasControl
```

如果不用挂载：

容器删了：

数据全部没了。

用了：

```
-v
```

真正写到：

```
宿主机~/.local/share/GasControl
```

所以：

重新启动：

数据仍然存在。

---

# ⑦ gascontrol

最后：

```
gas_control
```

表示：

镜像名字。

实际上：

Docker 会自动补：

```
gas_control:latest
```

等价：

```
docker run gas_control:latest
```

---

# Docker 数据流

可以理解成：

```
                 Host
         +----------------+
 DISPLAY=:0              |
 /tmp/.X11-unix          |
 ~/.local/share/...      |
         |               |
         |  映射(-v)      |
         v               |
+-----------------------------+
|         Docker              |
|                             |
| DISPLAY=:0                  |
| Qt                          |
|                             |
| /tmp/.X11-unix              |
|                             |
| /root/.local/share/...      |
+-----------------------------+
```

---

# 常见参数扩展

下面这些参数在实际开发中也非常常见。

## 1. 指定容器名称

```
--name gas_control
```

例如：

```
docker run --name gas_control ...
```

以后可以：

```
docker stop gas_control
```

而不用记容器 ID。

---

## 2. 后台运行

```
-d
```

例如：

```
docker run -d nginx
```

后台运行。

查看：

```
docker ps
```

---

## 3. 端口映射

```
-p 主机端口:容器端口
```

例如：

```
-p 8080:80
```

宿主：

```
8080
```

映射：

容器：

```
80
```

---

## 4. 挂载目录

除了 `-v` 外，还有推荐的新语法：

```
--mount type=bind,source=$HOME/data,target=/data
```

相比 `-v`，可读性更好，也更容易扩展。

---

## 5. 设置工作目录

```
-w /app
```

启动后默认就在：

```
/app
```

目录。

---

## 6. 指定用户

```
-u $(id -u):$(id -g)
```

让容器里的程序以当前宿主用户身份运行，避免生成 root 权限文件。

---

## 7. 映射设备

机器人开发中很常见：

```
--device=/dev/ttyUSB0
```

映射串口。

例如：

```
--device=/dev/video0
```

映射摄像头。

例如：

```
--device=/dev/input/js0
```

映射手柄。

---

## 8. Host 网络

```
--network host
```

容器直接使用宿主网络。

ROS1 开发经常使用。

---

## 9. 共享 IPC

```
--ipc host
```

深度学习、ROS2、共享内存通信时经常用。

---

## 10. GPU

NVIDIA：

```
--gpus all
```

让容器访问所有 GPU。

---

## 11. 自动重启

```
--restart unless-stopped
```

服务器部署时很常用。

---

## 12. 挂载整个工程目录

开发 Qt/ROS 时：

```
-v $(pwd):/workspace
```

容器直接看到当前源码。

---

## 13. 进入交互式 Shell

覆盖镜像默认启动命令：

```
docker run -it gas_control /bin/bash
```

适合调试镜像内容。

---

## 14. 指定启动命令

例如：

```
docker run ubuntu ls /
```

这里的 `ls /` 会替代镜像中的默认命令（`CMD`）。

---

## 对于你的 Qt + Docker 项目，比较推荐的启动方式

如果后续需要接入 ROS、激光雷达、串口等硬件，可以在现有命令基础上增加：

```
docker run -it --rm \
 --name gas_control \
 --network host \
 -e DISPLAY=$DISPLAY \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 -v $HOME/.local/share/GasControl:/root/.local/share/GasControl \
 --device=/dev/ttyUSB0 \
 gas_control
```

如果后续需要使用摄像头、CAN、USB 设备，还可以继续添加对应的 `--device` 或挂载 `/dev` 下的相关节点。这种启动方式也是工业机器人和 ROS 容器化部署中最常见的基础模板。