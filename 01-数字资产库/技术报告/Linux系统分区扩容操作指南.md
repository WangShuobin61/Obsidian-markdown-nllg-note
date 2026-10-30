# Linux 系统分区扩容操作指南

> 适用场景：将一块较小的系统盘完整克隆到更大的硬盘后，新增容量显示为“未分配”，需要在保留原系统、启动分区和数据的前提下扩容现有 Linux 分区。
>
> 本文以一台安装统信 UOS 的工控机为实例：原 128 GB 系统盘克隆到 256 GB SSD，使用 GParted Live 将 `Roota`、`Rootb` 分别扩展至 70 GiB，并将剩余空间全部分配给 `_dde_data`。

## 1. 最终结果

实例硬盘为 `/dev/sda`，扩容完成后的主要分区如下：

| 分区 | 文件系统 | 卷标 | 最终容量 | 用途 |
| --- | --- | --- | ---: | --- |
| `/dev/sda1` | FAT32 | EFI | 300 MiB | UEFI 启动分区，不调整 |
| `/dev/sda2` | EXT4 | Boot | 1.50 GiB | Linux 启动文件，不调整 |
| `/dev/sda3` | Linux swap | SWAP | 10.72 GiB | 交换分区，不调整 |
| `/dev/sda4` | EXT4 | Backup | 13.38 GiB | UOS 备份/恢复分区，不调整 |
| `/dev/sda5` | EXT4 | Roota | 70.00 GiB | UOS A 系统分区 |
| `/dev/sda6` | EXT4 | Rootb | 70.00 GiB | UOS B 系统分区 |
| `/dev/sda7` | EXT4 | `_dde_data` | 72.58 GiB | 用户数据分区 |

扩容前后三个目标分区的变化：

```text
扩容前：Roota 20 GiB | Rootb 20 GiB | _dde_data 53.35 GiB | 未分配 119.23 GiB
扩容后：Roota 70 GiB | Rootb 70 GiB | _dde_data 72.58 GiB
```

## 2. 重要风险提示

移动或调整分区属于高风险磁盘操作。即使软件标注为“无损”，断电、硬盘故障、文件系统损坏或选错磁盘仍可能造成数据丢失。

操作前必须满足以下条件：

1. 原来的小容量硬盘暂时不要覆盖、格式化或重复利用，应将其作为完整回退备份。
2. 另行备份扩容后新产生的重要数据。原硬盘不能代替最近数据的备份。
3. 工控机必须供电稳定，移动分区时不能关机、重启或拔掉硬盘。
4. 仔细确认目标硬盘容量、型号和设备名。本例目标是 238.47 GiB 的 `/dev/sda`，启动 U 盘则是 `/dev/sdb`。
5. 不要点击“新建分区表”“快速分区”“格式化”“删除分区”等命令。
6. `e2fsck` 只能对未挂载的 EXT2/EXT3/EXT4 文件系统执行修复。
7. 建议每次只提交一个移动或扩容操作，完成并验证后再进行下一项。

## 3. 为什么要使用 GParted Live

正在运行的 Linux 根分区通常无法离线移动，UOS 专业版还可能因未激活而无法使用 `sudo`。从 GParted Live 启动后：

- 内置硬盘上的 UOS 不处于运行状态；
- 系统分区通常没有被挂载；
- 可以使用 Linux 原生的 `e2fsck`、`resize2fs` 和 GParted；
- 不依赖原 UOS 的开发者模式或 `sudo` 权限。

复杂 EXT4 分区操作建议优先使用最新版 GParted Live。Windows PE 中较旧的第三方分区工具即使能识别 EXT4，也可能在检查、移动或扩容时失败。

## 4. 准备工具

需要准备：

- 一台可正常使用的 Windows 电脑；
- 一个至少 1 GB 的 U 盘；
- GParted Live `amd64` ISO 镜像；
- Rufus 启动盘制作工具；
- 原系统盘或其他有效备份；
- 稳定电源，条件允许时使用 UPS。

官方下载地址：

- GParted Live：<https://gparted.org/download.php>
- Rufus：<https://rufus.ie/>

## 5. 制作 GParted Live 启动盘

制作启动盘会清空目标 U 盘。若准备复用现有 PE 启动盘，先把其中的重要文件复制到其他磁盘。

1. 下载最新版 `gparted-live-*-amd64.iso`。
2. 插入 U 盘并打开 Rufus。
3. 在“设备”中选择目标 U 盘，务必核对容量，避免误选数据盘。
4. “引导类型选择”设置为“磁盘或 ISO 镜像”。
5. 选择下载的 GParted Live ISO。
6. 对 UEFI 设备可采用以下设置：

   ```text
   分区类型：GPT
   目标系统：UEFI（非 CSM）
   文件系统：FAT32
   簇大小：默认
   ```

7. 点击“开始”。如果 Rufus 询问写入模式，优先选择“以 ISO 镜像模式写入（推荐）”。
8. 等待状态显示“准备就绪”，然后安全弹出 U 盘。

## 6. 从 GParted Live 启动

1. 将 GParted U 盘插入待扩容的工控机。
2. 开机时连续按启动菜单键，常见按键为 `F7`、`F11`、`F12` 或 `Esc`。
3. 选择带有 `UEFI:` 前缀的 U 盘启动项。
4. 在 GRUB 菜单中选择：

   ```text
   GParted Live (Default settings)
   ```

5. 键盘布局选择 `Don't touch keymap`。
6. 语言可直接按回车使用默认值。
7. 如果询问图形界面启动模式，输入 `0` 并按回车。

如果出现 Secure Boot 验证失败，可进入固件设置临时关闭 Secure Boot；能够正常启动时不需要修改。

## 7. 修复克隆后 GPT 未覆盖整盘的问题

从小硬盘逐扇区克隆到大硬盘后，GPT 备份分区表可能仍停留在旧硬盘容量的末端。GParted 启动时可能提示：

```text
Not all of the space available to /dev/sda appears to be used...
```

此时选择“修复（Fix）”。它会将 GPT 备份表移动到新硬盘真正的末尾，使新增空间可以正常使用。该操作不会格式化已有分区。

## 8. 识别硬盘与分区

打开终端，执行：

```bash
lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINTS
```

实例输出的关键部分为：

```text
sda      238.5G
├─sda1     300M vfat  EFI
├─sda2     1.5G ext4  Boot
├─sda3    10.7G swap  SWAP
├─sda4    13.4G ext4  Backup
├─sda5      20G ext4  Roota
├─sda6      20G ext4  Rootb
└─sda7    53.3G ext4  _dde_data
sdb       58.6G
└─sdb1    58.6G vfat  GPARTED-LIVE  /run/live/medium
```

由此可确定：

- `/dev/sda` 是内置 256 GB SSD；
- `/dev/sdb` 是 GParted 启动 U 盘；
- `_dde_data` 对应 `/dev/sda7`；
- `MOUNTPOINTS` 为空表示目标分区当前未挂载。

不同电脑的设备名可能是 `/dev/nvme0n1`、`/dev/mmcblk0` 等，不能机械照抄 `/dev/sda`。

## 9. 扩容前检查并修复 EXT4

如果文件系统存在 inode 位图、Block Bitmap、空闲块计数等错误，GParted 或其他工具可能拒绝调整容量。应先使用 Linux 原生 `e2fsck` 修复。

在 GParted Live 中可以选择“Terminal with root privileges”，直接执行：

```bash
e2fsck -f -v /dev/sda5
```

```bash
e2fsck -f -v /dev/sda6
```

```bash
e2fsck -f -v /dev/sda7
```

如果使用普通终端，则在命令前加 `sudo`：

```bash
sudo e2fsck -f -v /dev/sda7
```

检查过程中若出现：

```text
Fix<y>?
```

输入 `y` 并按回车。修复结束后再次执行同一条命令，直到不再询问修复。

正常结果具有以下特征：

- `Pass 1` 至 `Pass 5` 全部完成；
- 没有再次出现 `Fix<y>?`；
- 显示 `0 bad blocks`；
- 最后正常返回命令提示符。

以下内容只是统计信息，并不代表故障：

```text
non-contiguous files
large files
symbolic links
sockets
```

刚执行完 `e2fsck` 时还可使用以下命令查看退出码：

```bash
echo $?
```

输出 `0` 代表检查正常完成。

## 10. 规划容量和移动距离

本例扩容前：

```text
Roota 20 GiB
Rootb 20 GiB
_dde_data 53.35 GiB
未分配 119.23 GiB
```

目标是：

```text
Roota 70 GiB
Rootb 70 GiB
其余空间全部给 _dde_data
```

因此：

```text
Roota 需要增加 50 GiB
Rootb 需要增加 50 GiB
共需在 sda7 前方腾出 100 GiB
数据盘可获得剩余约 19.23 GiB
数据盘最终约为 53.35 + 19.23 = 72.58 GiB
```

GParted 的输入单位通常是 MiB：

```text
1 GiB = 1024 MiB
50 GiB = 51200 MiB
70 GiB = 71680 MiB
100 GiB = 102400 MiB
```

## 11. 第一步：将数据分区 sda7 向右移动 100 GiB

目的：在 `sda6` 和 `sda7` 之间创建 100 GiB 连续未分配空间，同时保持 `sda7` 当前容量不变。

1. 在 GParted 中右键 `/dev/sda7`。
2. 选择“调整大小/移动”。
3. 填写：

   ```text
   前部空闲空间：102400 MiB
   新大小：保持当前值不变，约 54630 MiB
   后部空闲空间：自动变为约 19691 MiB
   对齐到：MiB
   ```

4. 如果 GParted 因对齐将数值调整 1～2 MiB，属于正常现象。
5. 点击“调整大小/移动”。
6. GParted 可能警告“移动分区可能导致操作系统无法启动”。`sda7` 是数据分区，此提示属于通用风险提示，可确认继续。
7. 回到主界面后，预览应为：

   ```text
   sda6 | 未分配 100 GiB | sda7 53.35 GiB | 未分配约 19.23 GiB
   ```

8. 确认底部只有一项待处理操作，再点击绿色对勾应用。
9. 等待操作完成。即使进度长时间没有明显变化，也不能强制关机。
10. 完成后再次执行：

    ```bash
    e2fsck -f -v /dev/sda7
    ```

## 12. 第二步：将 Rootb 向右移动 50 GiB

目的：把刚产生的 100 GiB 空间分成 Roota 前后各需要的 50 GiB。

1. 右键 `/dev/sda6`，选择“调整大小/移动”。
2. 填写：

   ```text
   前部空闲空间：51200 MiB
   新大小：20480 MiB，保持不变
   后部空闲空间：约 51200 MiB
   对齐到：MiB
   ```

3. 目标预览为：

   ```text
   sda5 | 未分配 50 GiB | sda6 20 GiB | 未分配 50 GiB | sda7
   ```

4. 确认只有一项待处理操作，然后点击绿色对勾应用。
5. 移动完成后执行：

   ```bash
   e2fsck -f -v /dev/sda6
   ```

Rootb 属于备用系统分区，移动时出现启动风险警告属于预期行为。移动操作应保留分区编号、文件系统 UUID 和 GPT 分区 GUID；EFI 与 Boot 分区没有移动。

## 13. 第三步：将 Roota 扩容到 70 GiB

此时 `sda5` 右侧紧邻 50 GiB 未分配空间，可以直接扩展：

1. 右键 `/dev/sda5`。
2. 选择“调整大小/移动”。
3. 将右边缘拖到紧靠 `/dev/sda6`，或者填写：

   ```text
   前部空闲空间：0 MiB
   新大小：71680 MiB
   后部空闲空间：0 MiB
   对齐到：MiB
   ```

4. 确认预览中 Roota 为 70 GiB。
5. 点击绿色对勾应用。

该操作只扩大分区右边界，不移动 Roota 的起始位置，通常比移动分区更快。

## 14. 第四步：将 Rootb 扩容到 70 GiB

此时 `sda6` 右侧紧邻另外 50 GiB 未分配空间：

1. 右键 `/dev/sda6`。
2. 选择“调整大小/移动”。
3. 将右边缘拖到紧靠 `/dev/sda7`，或者填写：

   ```text
   前部空闲空间：0 MiB
   新大小：71680 MiB
   后部空闲空间：0 MiB
   对齐到：MiB
   ```

4. 确认预览中 Rootb 为 70 GiB。
5. 点击绿色对勾应用。

UOS 使用 A/B 系统分区时，建议 Roota 与 Rootb 保持相同容量，避免系统更新、A/B 切换或回退时出现容量不足。

## 15. 第五步：将剩余空间全部分配给数据盘

1. 右键 `/dev/sda7`。
2. 选择“调整大小/移动”。
3. 将右边缘拖到磁盘最右端，或设置：

   ```text
   前部空闲空间：0 MiB
   后部空闲空间：0 MiB
   新大小：自动达到最大，约 72.58 GiB
   ```

4. 点击“调整大小/移动”。
5. 确认只有一项待处理操作后，点击绿色对勾应用。

最终 GParted 应显示：

```text
/dev/sda5  70.00 GiB
/dev/sda6  70.00 GiB
/dev/sda7  72.58 GiB
```

并且：

- 磁盘末尾不再有大块未分配空间；
- 底部显示“0 项操作待处理”；
- EFI、Boot、Swap 和 Backup 分区保持原状。

## 16. 最终文件系统检查

在 root 终端依次执行：

```bash
e2fsck -f -v /dev/sda5
```

```bash
e2fsck -f -v /dev/sda6
```

```bash
e2fsck -f -v /dev/sda7
```

然后查看最终布局：

```bash
lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINTS
```

预期关键结果：

```text
sda5   70G    ext4   Roota
sda6   70G    ext4   Rootb
sda7   72.6G  ext4   _dde_data
```

如果三次检查均完成五个阶段、没有修复询问、显示 `0 bad blocks` 并正常返回命令提示符，就可以尝试启动系统。

## 17. 关机并从硬盘启动

不建议直接在 Live 环境中强制重启。可以在 root 终端执行：

```bash
poweroff
```

待设备彻底关机后：

1. 拔掉 GParted 启动 U 盘；
2. 重新开机；
3. 让设备从内置 SSD 启动 UOS。

本例没有移动或修改 EFI、Boot 分区，也没有改变分区编号，因此正常情况下不需要执行 `grub-install` 或 `update-grub`。

## 18. 进入系统后的验证

UOS 正常进入桌面后，打开终端执行：

```bash
lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINTS
```

检查当前根分区容量：

```bash
df -hT /
```

检查数据分区容量：

```bash
df -hT /data
```

还可以执行：

```bash
findmnt /
```

```bash
findmnt /data
```

验证要点：

- UOS 能够正常进入桌面；
- `/` 挂载到 Roota 或当前活动根分区；
- `/data` 正常挂载到 `_dde_data`；
- 系统盘显示约 70 GiB；
- 数据盘显示约 72.6 GiB；
- 原有文件、用户配置和应用可以正常访问；
- 重启一次后依然能够正常进入系统。

`df` 显示的可用容量通常略小于 GParted 中的分区容量，这是 EXT4 元数据、日志和预留块造成的，属于正常现象。

## 19. 常见问题处理

### 19.1 GParted 看不到新硬盘的全部空间

如果提示 GPT 没有使用全部可用空间，选择“修复（Fix）”，让备份 GPT 移动到新硬盘末尾。

### 19.2 调整分区时提示 inode 或 Block Bitmap 错误

停止调整，不要反复重试。进入 GParted Live 的 root 终端，对相应 EXT4 分区执行：

```bash
e2fsck -f -v /dev/对应分区
```

修复后再次检查，确认干净再继续。

### 19.3 无法扩展某个分区

分区只能直接扩展到与它相邻的未分配空间。如果未分配空间与目标分区之间隔着其他分区，必须先移动中间分区。

### 19.4 GParted 提示“移动分区可能导致系统无法启动”

这是移动分区起始位置时的通用警告。必须确认：

- 目标分区选对；
- EFI 和 Boot 分区未被误移动；
- 已经做好备份；
- 操作期间不会断电。

对于通过 UUID 挂载的 Linux 分区，移动通常不会改变文件系统 UUID，但仍然存在启动风险。

### 19.5 扩容后无法启动

不要立即重装系统或重新分区：

1. 拍摄完整报错或 GRUB 画面；
2. 保留原硬盘，不要覆盖；
3. 重新从 GParted Live 启动；
4. 使用 `lsblk -f`、`blkid` 和 `e2fsck` 检查分区；
5. 确认 EFI、Boot、Roota、Rootb 的分区编号和 UUID；
6. 只有确认引导配置损坏后，才考虑修复 GRUB。

### 19.6 `df` 容量小于 GParted 显示值

这是单位换算、文件系统元数据和 EXT4 预留空间共同造成的，一般不是扩容失败。

## 20. 通用计算方法

假设：

- 根分区当前容量为 `R`；
- 希望根分区扩到 `T`；
- UOS 有两个等大的 A/B 根分区；
- 新增未分配空间为 `F`。

两个根分区需要的总新增空间为：

```text
2 × (T - R)
```

数据盘最终能够增加的空间为：

```text
F - 2 × (T - R)
```

本例：

```text
R = 20 GiB
T = 70 GiB
F = 119.23 GiB

两个根分区需要：2 × (70 - 20) = 100 GiB
数据盘可增加：119.23 - 100 = 19.23 GiB
数据盘最终容量：53.35 + 19.23 = 72.58 GiB
```

若计算结果小于 0，说明新增空间不足，必须降低根分区目标容量或另行迁移数据。

## 21. 操作检查清单

### 扩容前

- [ ] 原系统盘或完整镜像备份仍可用
- [ ] 新增数据已经单独备份
- [ ] 确认目标硬盘型号和容量
- [ ] 制作最新版 GParted Live 启动盘
- [ ] 保证供电稳定

### 扩容中

- [ ] 修复克隆后 GPT 末端位置
- [ ] 使用 `lsblk` 确认分区编号
- [ ] 使用 `e2fsck` 检查所有待移动/扩容的 EXT4 分区
- [ ] 每次只提交一个操作
- [ ] 每次移动后再次检查文件系统
- [ ] 不移动 EFI、Boot、Swap、Backup 分区
- [ ] 不在执行期间断电或拔盘

### 扩容后

- [ ] Roota 和 Rootb 容量一致
- [ ] 数据盘已获得剩余空间
- [ ] GParted 显示 0 项待处理操作
- [ ] `e2fsck` 最终检查无错误
- [ ] 正常关机后拔掉启动 U 盘
- [ ] UOS 正常启动
- [ ] `lsblk`、`df -hT` 和 `findmnt` 验证通过
- [ ] 原有数据与应用可以正常访问

## 22. 参考资料

- GParted 官网：<https://gparted.org/>
- GParted Live 下载：<https://gparted.org/download.php>
- GParted Live USB：<https://gparted.org/liveusb.php>
- GParted 分区间移动空间说明：<https://gparted.org/display-doc.php?name=moving-space-between-partitions>
- `e2fsck(8)` 手册：<https://man7.org/linux/man-pages/man8/e2fsck.8.html>
- `resize2fs(8)` 手册：<https://man7.org/linux/man-pages/man8/resize2fs.8.html>

---

文档版本：1.0  
整理日期：2026-09-22  
实例结果：UOS 扩容成功，Roota 70 GiB、Rootb 70 GiB、`_dde_data` 72.58 GiB，系统正常启动。
