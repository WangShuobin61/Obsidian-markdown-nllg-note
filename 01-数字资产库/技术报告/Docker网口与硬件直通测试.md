# 网口与硬件测试

## 一、测试环境说明

- **宿主机**：统信 UOS 工控机
- **运行载体**：Docker Ubuntu 容器（程序编译、运行均在容器内部，宿主机仅提供硬件、网络底层支撑）
- **测试目的**：验证容器程序与外部设备 UDP 网口通信通路正常，仅提供连通现象证明

---

## 二、基础网络信息确认（宿主机 UOS 操作）

### 2.1 获取工控机本机 IP

```bash
ip addr
```

记录以太网 IPv4 地址（示例：`192.168.1.36`），外部测试设备需配置同网段地址才可通信。

### 2.2 防火墙开放业务 UDP 端口 45454

查看当前防火墙放行规则：

```bash
sudo ufw status
```

若无 `45454/udp` 放行规则，执行端口放行命令：

```bash
sudo ufw allow 45454/udp
```

运维远程 SSH 端口放行（现场调试使用）：

```bash
sudo ufw allow 22/tcp
```

### 2.3 基础链路连通校验

在外部测试设备执行 ping 命令，确认网络链路通畅：

```bash
ping 192.168.1.36
```

数据包无丢包代表物理网络、网段配置正常。

---

## 三、启动容器（共享宿主机完整网络栈）

宿主机终端执行命令，拉起交互式容器终端：

```bash
docker run -it --rm \
  --name gas_control_shell \
  --network host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/input:/dev/input \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /dev/dri:/dev/dri \
  -v /opt/gas_control_data:/root/.local/share/GasControl \
  --entrypoint /bin/bash gas_control
```

**关键参数说明**：

- `--network host`：容器复用宿主机全部网卡、IP 地址、端口资源，容器内程序可直接监听宿主机 UDP 45454 端口。

---

## 四、Docker 容器启动验证

容器启动后，在容器终端内执行以下验证步骤，确认环境正常。

### 4.1 检查容器网络配置

确认容器复用了宿主机网络栈，IP 地址与宿主机一致：

```bash
ip addr / ifconfig
```

**预期结果**：容器内显示的网卡信息（如 `enp2s0`、`enp3s0`）和 IP 地址与宿主机完全相同。

### 4.2 检查 UDP 端口监听状态

确认业务端口 `45454` 处于监听状态：

```bash
ss -ulnp | grep 45454
```

或：

```bash
netstat -ulnp | grep 45454
```

**预期结果**：显示 `udp UNCONN 0.0.0.0:45454` 或类似信息，表示端口已开放。

### 4.3 检查容器内 USB 设备访问权限

确认容器内可正常访问宿主机 USB 设备：

```bash
lsusb
```

**预期结果**：列出所有 USB 设备，与宿主机执行 `lsusb` 输出一致。

### 4.4 检查显示环境变量

确认 X11 转发环境正常（如需在容器内运行 GUI 程序）：

```bash
echo $DISPLAY
```

**预期结果**：输出类似 `:0` 或 `:1` 的值，表示 DISPLAY 环境变量已正确设置。

---

## 五、UDP 网口通信验证判定标准

1. 外包侧测试设备配置通信参数：UDP 协议、端口 45454、工控机对应 IP
2. 两端建立 UDP 数据交互后，程序实时接收外部数据并产生数据数值变化
3. **现象结论**：Docker 容器内程序可正常收发外部 UDP 45454 端口数据，网络通路无拦截、通信功能正常

---

## 六、USB 设备检测

### 6.1 监控 USB 插拔事件

在宿主机或容器终端执行：

```bash
dmesg -w | grep -i usb
```

> 注：双方在宿主机和容器内执行输出内容一致，root 用户下部分关键字显示为红色字体。

**USB 插拔示例输出**：

```
[14001.883698] usb 1-2.3: USB disconnect, device number 12
```

> `12` 为 USB 连接的接口号，插拔时会变化。

**再次插入后的识别日志**：

```
[14358.764724] usb 1-2.3: new full-speed USB device number 13 using xhci_hcd
[14358.882946] usb 1-2.3: New USB device found, idVendor=046d, idProduct=c07f, bcdDevice=91.00
[14358.882950] usb 1-2.3: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[14358.882952] usb 1-2.3: Product: Gaming Mouse G302
[14358.882954] usb 1-2.3: Manufacturer: Logitech
[14358.882955] usb 1-2.3: SerialNumber: 158539663836
[14358.973734] input: Logitech Gaming Mouse G302 as /devices/pci0000:00/0000:00:07.1/0000:04:00.3/usb1/1-2/1-2.3/1-2.3:1.0/0003:046D:C07F.000C/input/input32
[14359.032792] hid-generic 0003:046D:C07F.000C: input,hidraw1: USB HID v1.11 Mouse [Logitech Gaming Mouse G302] on usb-0000:04:00.3-2.3/input0
[14359.035605] input: Logitech Gaming Mouse G302 Keyboard as /devices/pci0000:00/0000:00:07.1/0000:04:00.3/usb1/1-2/1-2.3/1-2.3:1.1/0003:046D:C07F.000D/input/input33
[14359.092838] input: Logitech Gaming Mouse G302 Consumer Control as /devices/pci0000:00/0000:00:07.1/0000:04:00.3/usb1/1-2/1-2.3/1-2.3:1.1/0003:046D:C07F.000D/input/input34
[14359.092909] input: Logitech Gaming Mouse G302 System Control as /devices/pci0000:00/0000:00:07.1/0000:04:00.3/usb1/1-2/1-2.3/1-2.3:1.1/0003:046D:C07F.000D/input/input35
[14359.093045] hid-generic 0003:046D:C07F.000D: input,hiddev0,hidraw2: USB HID v1.11 Keyboard [Logitech Gaming Mouse G302] on usb-0000:04:00.3-2.3/input1
```

**结论**：出现上述日志即表示 USB 设备被正常识别。

---

## 七、网口状态查看

### 7.1 查看网卡信息

```bash
ip addr
```

**输出示例**：

```
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host
       valid_lft forever preferred_lft forever

2: enp2s0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc pfifo_fast state UP group default qlen 1000
    link/ether 90:e2:fc:b7:9a:e9 brd ff:ff:ff:ff:ff:ff
    inet 192.168.1.36/24 brd 192.168.1.255 scope global dynamic noprefixroute enp2s0
       valid_lft 86371sec preferred_lft 86371sec
    inet6 2408:8207:18df:b540::1/128 scope global dynamic noprefixroute
       valid_lft 259171sec preferred_lft 172771sec
    inet6 2408:8207:18df:b540:601b:a2a6:1d8f:eb72/64 scope global dynamic noprefixroute
       valid_lft 259151sec preferred_lft 172751sec
    inet6 fe80::7fd:6a48:87db:80af/64 scope link noprefixroute
       valid_lft forever preferred_lft forever

3: enp3s0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc pfifo_fast state DOWN group default qlen 1000
    link/ether 90:e2:fc:b7:c3:e9 brd ff:ff:ff:ff:ff:ff
```

**状态说明**：

| 网卡 | 状态 | 说明 |
|------|------|------|
| `enp2s0` | `UP,LOWER_UP` | 连接正常，IPv4 地址 `192.168.1.36/24`（SSH 连接使用） |
| `enp3s0` | `DOWN`（`NO-CARRIER`） | 未连接网线 |

---

## 八、显示输出检测

### 8.1 查看显示接口状态

```bash
xrandr
```

**输出示例**：

```
Screen 0: minimum 320 x 200, current 1920 x 1080, maximum 16384 x 16384
eDP-1 connected primary 1920x1080+0+0 (normal left inverted right x axis y axis) 0mm x 0mm
   1920x1080     60.00*
   1680x1050     60.00
   1600x900      60.00
   1280x1024     60.00
   1440x900      60.00
   1280x960      60.00
   1280x800      60.00
   1280x720      60.00
   1024x768      60.00
   800x600       60.00
   720x480       60.00
   640x480       60.00
HDMI-1 disconnected (normal left inverted right x axis y axis)
HDMI-2 disconnected (normal left inverted right x axis y axis)
VGA-1 disconnected (normal left inverted right x axis y axis)
```

**状态说明**：

| 接口 | 状态 | 说明 |
|------|------|------|
| `eDP-1` | `connected` | 内置显示屏，分辨率 1920x1080 |
| `HDMI-1` | `disconnected` | 未连接 |
| `HDMI-2` | `disconnected` | 未连接 |
| `VGA-1` | `disconnected` | 未连接 |

> 注：工控机仅有 1 个 HDMI 接口，若 `HDMI-1` 或 `HDMI-2` 显示 `connected`，表示外接显示器已连接。
