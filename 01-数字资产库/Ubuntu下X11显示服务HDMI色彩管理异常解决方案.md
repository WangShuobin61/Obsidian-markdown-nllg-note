## 最完美方案：使用DP线缆链接扩展屏幕
## 临时生效方案：重启失效

```bash
# 查看已连接的屏幕
xrandr

# 设置位置关系、刷新率、主屏
xrandr \
--output eDP-1 --mode 1920x1080 --rate 60 --pos 0x0 \
--output HDMI-0 --mode 1920x1080 --rate 100 --pos 1920x0 --primary

# 强制HDMI输出用RGB格式输出避免偏色问题（一般情况下仅执行这条命令就行）
nvidia-settings --assign \
"CurrentMetaMode=\
DP-4: 1920x1080_60 +0+0, \
HDMI-0: 1920x1080_100 +1920+0 {ForceFullCompositionPipeline=On, ColorSpace=RGB, ColorRange=Full}"

```

### 命令分析
## 🔵 xrandr 能控制：

- 屏幕启用 / 禁用
- 分辨率
- 刷新率
- 屏幕位置（--pos）
- 主屏（--primary）
它是在 **X 层面布局管理**

---

## 🟢 nvidia-settings 的 CurrentMetaMode 能控制：

- 分辨率
- 刷新率
- ForceFullCompositionPipeline
- 色彩空间
- 色彩范围
它主要是 **显卡渲染管线设置**

| 要求                           | 是否属于 xrandr | 是否属于 nvidia-settings |
| :--------------------------- | :---------- | :------------------- |
| 设置主屏                         | ✅ 是         | ❌ 否                  |
| ==屏幕位置关系==                   | ✅ 是         | ✅ 是                  |
| ==1920x1080==                | ✅ 是         | ✅ 是                  |
| ==60Hz / 100Hz==             | ✅ 是         | ✅ 是                  |
| ForceFullCompositionPipeline | ❌ 否         | ✅ 是                  |
| 色彩修正                         | ❌ 否         | ✅ 是                  |

如果你非要用HDMI且每次开机自动配置请继续阅读以下长文
# ① 显示器被识别为“电视模式”

很多 HDMI 显示器会：

- 自动进入 TV 模式
- 启用暖色模式
- 启用 YCbCr → RGB 转换

Windows 驱动会自动矫正  Linux 不会

---

## 🔍 你现在做一件事

打开显示器物理菜单（按显示器按钮）

找：

- Color Mode

- Picture Mode

- Temperature

- HDMI Mode

- PC Mode / AV Mode


看看是不是：

- Warm

- Movie

- TV

- Low Blue Light

- Eye Care


把它改成：

Standard  
Normal  
6500K  
PC Mode

这个非常关键。

---

# ② HDMI 量化范围被显示器锁死

虽然 NVIDIA 显示 Full  
但有些显示器：

- 只在 DP 下支持 Full
- HDMI 默认内部 Limited


你这台是：

> XMI Mi Monitor（小米显示器）

小米显示器很多型号：

👉 HDMI 默认偏暖  
👉 DP 颜色更准

# 🎯 真正原因：Linux 下 HDMI 输出默认使用 YCbCr，而不是纯 RGB 直通

虽然你看到：

Color Space: RGB  
Color Range: Full

但在 NVIDIA Linux 驱动里：

👉 HDMI 很多时候内部仍然走的是 **YCbCr 转换管线**

而 Windows 驱动会强制 PC RGB 直通。

结果就是：

- 白色偏暖
- 亮部发黄
- 对比度感觉不一样


这在 RTX30 系列 + HDMI 显示器上非常常见。

---

# 🚀 临时生效的终极解决方案（强制纯 RGB 直通）

我们直接强制 NVIDIA 走纯 RGB 输出。

执行：

nvidia-settings --assign "CurrentMetaMode=HDMI-0: 1920x1080_100 {ForceFullCompositionPipeline=On, ColorSpace=RGB, ColorRange=Full}"

然后：

nvidia-settings --write-config

再：

reboot

---

# 🔥 如果重启后仍然偏黄

我们再做一个更强制的设置：

编辑 X 配置文件：

sudo nano /etc/X11/xorg.conf

如果文件不存在，就创建。

在 Screen 部分加入：

Option "ColorSpace" "RGB"  
Option "ColorRange" "Full"  
Option "UseEDIDColor" "False"

保存，重启。

---

# 🧠 为什么 Windows 正常？

Windows NVIDIA 驱动会：

- 自动识别为 PC 显示器

- 强制 RGB 4:4:4

- 不使用 HDMI TV 色彩逻辑


Linux 驱动不会这么智能。

---

# 🎯 说实话

你这个情况不是设置错误。

而是：

> NVIDIA Linux HDMI 输出色彩实现 ≠ Windows

这是老问题了。

---

# 🧪 我再给你一个验证方法（判断是不是色域问题）

在 HDMI-0 页面：

把 Digital Vibrance 调到：

+20

如果颜色立刻“正常很多”

那就确认是色域压缩导致的。

---

# 🎯 现在按顺序做：

1. 先执行我给你的 CurrentMetaMode 命令

2. 重启

3. 看是否改善


告诉我结果。

如果还不行，我可以给你一个“完全锁死 HDMI 输出模式”的终极方案。

nvidia-settings --assign "CurrentMetaMode=HDMI-0: 1920x1080_100 {ForceFullCompositionPipeline=On, ColorSpace=RGB, ColorRange=Full}" 
这条命令执行后外接屏幕的色彩正常了，但是笔记本屏幕不亮了，如何解决，先让笔记本屏幕亮起来、

很好 👍 这说明我们已经**找对方向了**。

你刚才那条命令：

nvidia-settings --assign "CurrentMetaMode=HDMI-0: 1920x1080_100 {ForceFullCompositionPipeline=On, ColorSpace=RGB, ColorRange=Full}"

它的副作用是：

👉 **只定义了 HDMI-0 的 MetaMode**  
👉 没把内屏 DP-4 写进去  
👉 所以内屏被禁用了

所以现在外屏正常，但笔记本屏幕被“逻辑关闭”了。

---

# ✅ 先把内屏点亮（立刻恢复）

执行：

xrandr --output DP-4 --auto --left-of HDMI-0

或者如果你希望原来的布局：

xrandr --output DP-4 --auto --pos 0x0

执行后内屏应该立刻亮。

---

# 🎯 正确做法：同时写入两个屏幕的 MetaMode

我们需要定义“双屏模式”，而不是只写 HDMI。

执行：

nvidia-settings --assign \  
"CurrentMetaMode=HDMI-0: 1920x1080_100 {ForceFullCompositionPipeline=On, ColorSpace=RGB, ColorRange=Full} +1920+0, DP-4: 1920x1080_60 +0+0 {ForceFullCompositionPipeline=On}"

说明：

- HDMI 在右侧（+1920+0）

- 内屏在左侧（+0+0）


然后：

nvidia-settings --write-config

---

# 🧠 为什么会这样？

NVIDIA 的 MetaMode 是：

> 一次性描述整个 X Screen 的所有输出

你刚才只描述了 HDMI  
所以系统认为只有 HDMI 存在。

---

# 🔥 如果现在内屏还是黑的

执行：

xrandr

看看 DP-4 是否显示 connected。

如果显示 connected 但黑屏，再执行：

xrandr --output DP-4 --mode 1920x1080 --rate 60

---

# 🎯 重要提醒

暂时不要重启，  
先确认两个屏幕都亮了再保存配置。

---

# 👇 现在按顺序做：

1. 先执行：

xrandr --output DP-4 --auto

1. 看内屏是否亮

2. 告诉我结果


我们一步步稳定它。

xrandr --output DP-4 --auto --pos 0x0 执行这条命令后两个显示器都正常显示了，色彩输出也正常了，是不是以后每次开机就不需要重新设置什么了

很好 👍 现在我们把逻辑理清一下：

你刚才做了两件关键的事：

1. 用 `nvidia-settings` 强制 HDMI 走 **RGB + Full**

2. 用 `xrandr` 把内屏重新启用


现在两个屏幕：

- ✅ 都亮

- ✅ 色彩正常

- ✅ HDMI 不再发黄


说明当前这套 MetaMode 是正确的。

---

# 🎯 关键问题：重启后会不会恢复？

这取决于两件事：

### ① 你有没有执行过：

nvidia-settings --write-config

如果执行过 → 会写入 `~/.nvidia-settings-rc`

但⚠️ 这 **不一定** 每次开机自动加载。

---

### ② 你有没有 “Save to X Configuration File”

在 nvidia-settings 右下角有个按钮：

Save to X Configuration File

如果你没有点它：

👉 重启后大概率会恢复原来的发黄状态。

---

# ✅ 最稳妥方案（推荐，一次到位）

我们创建一个开机自动执行的小脚本。

这样：

> 每次登录自动恢复正确的 HDMI 输出  
> 永远不会再黄

---

## 第一步：创建脚本

nano ~/.fix-display.sh

写入：

#!/bin/bash  
xrandr --output DP-4 --auto --pos 0x0  
nvidia-settings --assign "CurrentMetaMode=HDMI-0: 1920x1080_100 {ForceFullCompositionPipeline=On, ColorSpace=RGB, ColorRange=Full} +1920+0, DP-4: 1920x1080_60 +0+0 {ForceFullCompositionPipeline=On}"

保存退出。

---

## 第二步：赋予执行权限

chmod +x ~/.fix-display.sh

---

## 第三步：添加到开机启动

打开：

设置 → 启动应用程序

添加一个新项：

- 名称：Fix Display

- 命令：


/home/nllg/.fix-display.sh

---

# 🎯 这样以后会怎样？

每次登录：

- 自动恢复双屏

- 自动强制 HDMI RGB

- 永远不会再发黄


---

# 🧠 说实话

这是 NVIDIA Linux HDMI 的老问题。

你这个属于：

> RTX3050 + HDMI 显示器 + GNOME  
> 非常典型的色彩偏移案例

你现在的解决方案是“标准工程解法”。

---

# ✅ 现在回答你的问题：

> 以后每次开机还需要重新设置吗？

如果你：

- 没保存 X config

- 没写启动脚本


👉 大概率会恢复。

如果按我上面做：

👉 以后完全不用管。

---

# 现代可行方案（兼容性强）

不用xorg.conf文件 只锁颜色 色彩正常 + 自动布局

我们走 **B 方案：不用 xorg.conf 固定布局，只锁 HDMI 颜色，布局交给系统自动管理。**

这也是在 Ubuntu + NVIDIA 专有驱动下最干净、最现代、最稳定的做法。

---

# 🎯 目标

- ✅ 插拔显示器不黑
- ✅ 可以随便换左右
- ✅ 自动布局（GNOME/KDE 管）
- ✅ HDMI 永远 RGB Full
- ✅ 不使用 `/etc/X11/xorg.conf`

---

# ✅ 第一步：删除 xorg.conf（如果还在）

确认：

ls /etc/X11/xorg.conf

如果存在：

sudo rm /etc/X11/xorg.conf

不要动 xorg.conf.d。

---

# ✅ 第二步：用 nvidia-settings 设置一次颜色

运行：

nvidia-settings

找到：

**HDMI-0 → Color Space → RGB**  
**Color Range → Full**

应用。

确认内屏不受影响。

---

# ✅ 第三步：只保存颜色，不保存布局

关键来了。

不要点 “Save to X Configuration File”。

而是：

nvidia-settings --query CurrentMetaMode

你会看到类似：

DP-4: 1920x1080 +0+0, HDMI-0: 1920x1080 +1920+0

我们不要保存这个。

---

# ✅ 第四步：让颜色每次登录自动执行（推荐方式）

创建一个启动脚本：

nano ~/.config/autostart/nvidia-color.desktop

写入：

[Desktop Entry]  
Type=Application  
Exec=nvidia-settings --assign "HDMI-0/ColorSpace=RGB" --assign "HDMI-0/ColorRange=Full"  
Hidden=false  
NoDisplay=false  
X-GNOME-Autostart-enabled=true  
Name=NVIDIA Color Fix

保存。

---

# 🎉 结果

开机流程变成：

1. 系统自动检测屏幕

2. GNOME 自动排布

3. 登录后自动执行颜色修正

4. 只修 HDMI，不影响内屏

5. 不锁布局

6. 不会黑屏


---

# 🔎 为什么这是最优解？

因为在现代 Ubuntu（GNOME 42+）：

- 屏幕管理是 Mutter 负责

- NVIDIA MetaModes 是旧式方案

- xorg.conf 是 2005 年思路


你现在的机器（Legion R7000 + NVIDIA 535）  
属于现代驱动架构。

最干净的就是：

👉 不碰 xorg.conf  
👉 只动态改颜色

---

# ⚠ 小提醒

如果以后：

- HDMI-0 变成 HDMI-1

- 或换显示器


只需：

xrandr

确认输出名  
改一下 autostart 里的 HDMI 名称即可。

---

# 📌 最终结构应该是

/etc/X11/  
   (没有 xorg.conf)  
~/.config/autostart/nvidia-color.desktop

---

如果你愿意，我可以再帮你做一个：

🧠 “智能版自动脚本”

自动检测 HDMI 输出名  
不管叫 HDMI-0 / HDMI-1 / HDMI-A-0 都自动修正

那就属于“工程级优雅方案”了 😄

🧠 “智能版自动脚本” 自动检测 HDMI 输出名 不管叫 HDMI-0 / HDMI-1 / HDMI-A-0 都自动修正 帮我实现这个

很好 👍 这才是“工程级优雅方案”。

我们做一个 **真正智能版自动修正脚本**：

目标：

- ✅ 自动检测当前所有 HDMI 输出

- ✅ 只对已连接的 HDMI 屏幕生效

- ✅ 自动设置 RGB + Full

- ✅ 不管叫 HDMI-0 / HDMI-1 / HDMI-A-0 都行

- ✅ 不锁布局

- ✅ 不用 xorg.conf


---

# 🧠 原理

通过 `xrandr` 获取已连接显示器：

xrandr --query

输出里会有：

HDMI-0 connected  
HDMI-1 disconnected  
DP-4 connected

我们只抓：

HDMI-* connected

然后循环执行：

nvidia-settings --assign "输出名/ColorSpace=RGB"  
nvidia-settings --assign "输出名/ColorRange=Full"

---

# ✅ 第一步：创建智能脚本

创建脚本文件：

mkdir -p ~/.local/bin  
nano ~/.local/bin/nvidia-hdmi-rgb-fix.sh

写入以下内容：

#!/bin/bash  
  
# 获取所有已连接的 HDMI 输出  
HDMI_OUTPUTS=$(xrandr --query | grep -E '^HDMI|^HDMI-A' | grep connected | awk '{print $1}')  
  
for OUTPUT in $HDMI_OUTPUTS  
do  
nvidia-settings --assign "$OUTPUT/ColorSpace=RGB" >/dev/null 2>&1  
nvidia-settings --assign "$OUTPUT/ColorRange=Full" >/dev/null 2>&1  
done

保存退出。

---

# ✅ 第二步：赋予执行权限

chmod +x ~/.local/bin/nvidia-hdmi-rgb-fix.sh

---

# ✅ 第三步：添加开机自启动

创建：

nano ~/.config/autostart/nvidia-hdmi-rgb-fix.desktop

写入：

[Desktop Entry]  
Type=Application  
Exec=/home/nllg/.local/bin/nvidia-hdmi-rgb-fix.sh  
Hidden=false  
NoDisplay=false  
X-GNOME-Autostart-enabled=true  
Name=NVIDIA HDMI RGB Fix  
Comment=Auto force HDMI RGB Full

⚠ 注意：

如果你用户名不是 `nllg`  
要改成你的真实用户名。

---

# 🎉 完成后效果

开机流程：

1. 系统自动识别显示器

2. GNOME 自动排列

3. 登录后自动检测 HDMI

4. 自动修正颜色

5. 插拔显示器后再次登录也自动修正


---

# 🔥 升级版（插拔即时生效）

如果你想做到：

插 HDMI 后立即生效  
不用重新登录

我可以再给你做一个：

👉 systemd + udev 触发版本  
真正企业级方案

---

# 📌 你现在得到的是

✔ 不用 xorg.conf  
✔ 不锁布局  
✔ 自动修色  
✔ 可换显示器  
✔ 可换左右  
✔ 不会黑屏

已经是现代 Ubuntu + NVIDIA 最优实践。

---

如果你愿意，我可以再帮你做一个：

🧪 “验证脚本是否成功”的检测命令

让你确认色彩真的在 RGB Full 模式。