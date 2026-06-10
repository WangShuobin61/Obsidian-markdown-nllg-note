# Tailscale 1.98.4 安装失败问题解决报告

## 1. 问题概述

本次处理的问题是：在 Windows 10 x64 环境中安装 `D:\Apps\tailscale\tailscale-setup-1.98.4.exe` 时，安装程序报错：

```text
0x80070643 - 安装时发生严重错误（Fatal error during installation）
```

安装失败后，目录中保留了以下原始日志：

```text
D:\Apps\tailscale\Tailscale_20260610100111.log
D:\Apps\tailscale\Tailscale_20260610100111_000_MsiAMD64.log
D:\Apps\tailscale\Tailscale_20260610100113.elevated.log
```

最终处理结果：Tailscale 已成功安装，版本为 `1.98.4`，服务 `Tailscale` 已创建并处于运行状态。

---

## 2. 关键结论

外层错误码 `0x80070643` 不是根因，只是 Windows Installer 安装失败后的包装错误。

真正失败点在 MSI 子日志 `Tailscale_20260610100111_000_MsiAMD64.log` 中：

```text
Doing action: CheckServiceDependencies
Action start 10:01:17: CheckServiceDependencies.
Custom Action CheckServiceDependencies ... Error: Obtaining Config for "Dnscache": The system cannot find the file specified.
CustomAction CheckServiceDependencies returned actual error code 1603
Action ended 10:01:17: CheckServiceDependencies. Return value 3.
Action ended 10:01:17: INSTALL. Return value 3.
```

也就是说：

- 外层安装器下载并调用了 `tailscale-setup-1.98.4-amd64.msi`。
- MSI 在真正安装前执行了 `CheckServiceDependencies`。
- 该检查尝试读取系统服务 `Dnscache` 配置。
- 读取失败后，MSI 返回 `1603`。
- 外层 Burn 安装器把它包装成 `0x80070643`。

Tailscale MSI 内置的依赖服务列表是：

```text
Dnscache;iphlpsvc;netprofm;WinHttpAutoProxySvc
```

后续检查确认，这些服务在系统中实际存在，并且 Tailscale 最终安装后也正常依赖这些服务。因此这不是“服务真的缺失”，而是 MSI 的前置依赖检查动作在当前系统环境中读取服务配置失败。

---

## 3. 原始失败链路

### 3.1 外层安装器行为

原始日志 `Tailscale_20260610100111.log` 显示，外层 EXE 安装器识别当前系统为 x64，并选择安装 AMD64 MSI：

```text
Planned package: MsiAMD64, state: Absent, default requested: Present, execute: Install
Acquiring package: MsiAMD64, payload: MsiAMD64, download from: https://pkgs.tailscale.com/stable/tailscale-setup-1.98.4-amd64.msi
Verified acquired payload: MsiAMD64 ... tailscale-setup-1.98.4-amd64.msi
Applying execute package: MsiAMD64, action: Install
```

随后失败：

```text
Error 0x80070643: Failed to install MSI package.
Error 0x80070643: Failed to execute MSI package.
Error 0x80070643: Failed to configure per-machine MSI package.
Apply complete, result: 0x80070643
Exit code: 0x643
```

### 3.2 MSI 真实失败点

MSI 详细日志显示，失败发生在 `CheckServiceDependencies`：

```text
Custom Action CheckServiceDependencies ... Error: Obtaining Config for "Dnscache": The system cannot find the file specified.
CustomAction CheckServiceDependencies returned actual error code 1603
Action ended 10:01:17: CheckServiceDependencies. Return value 3.
```

同时日志中可以看到 Tailscale 的服务依赖配置：

```text
Property(S): TS_SERVICE_DEPENDENCIES = Dnscache;iphlpsvc;netprofm;WinHttpAutoProxySvc
```

### 3.3 系统待重启状态

原始外层日志末尾还记录了：

```text
Variable: RebootPending = 1
```

后续检查也发现系统存在 `PendingFileRenameOperations`。这说明系统当时存在待重启状态，但本次最终失败的直接触发点仍然是 MSI 的 `CheckServiceDependencies` 动作。

---

## 4. 已做的检查

### 4.1 检查依赖服务是否真实存在

检查了以下服务：

```text
Dnscache
iphlpsvc
netprofm
WinHttpAutoProxySvc
```

检查结果：

```text
Dnscache Status=Running StartType=Automatic
iphlpsvc Status=Running StartType=Automatic
netprofm Status=Running StartType=Manual
WinHttpAutoProxySvc Status=Running StartType=Manual
```

对应注册表服务项也存在：

```text
HKLM:\SYSTEM\CurrentControlSet\Services\Dnscache
HKLM:\SYSTEM\CurrentControlSet\Services\iphlpsvc
HKLM:\SYSTEM\CurrentControlSet\Services\netprofm
HKLM:\SYSTEM\CurrentControlSet\Services\WinHttpAutoProxySvc
```

结论：依赖服务没有真实缺失。

### 4.2 检查底层服务配置

使用 `sc.exe qc` 检查服务配置，服务均可读取。

例如 `Dnscache`：

```text
SERVICE_NAME: Dnscache
TYPE               : 10  WIN32_OWN_PROCESS
START_TYPE         : 2   AUTO_START
BINARY_PATH_NAME   : C:\WINDOWS\system32\svchost.exe -k NetworkService -p
DEPENDENCIES       : nsi
                   : Afd
SERVICE_START_NAME : NT AUTHORITY\NetworkService
```

这进一步说明，系统服务配置本身不是完全损坏状态。

---

## 5. 尝试过的安装路径

### 5.1 直接重试原 EXE 安装

使用原始安装器重试安装，并输出新日志：

```powershell
D:\Apps\tailscale\tailscale-setup-1.98.4.exe /quiet /norestart /log D:\Apps\tailscale\tailscale-install-retry.log
```

结果：失败，返回 `1603`。

新生成的 MSI 子日志：

```text
D:\Apps\tailscale\tailscale-install-retry_000_MsiAMD64.log
```

失败点仍然是：

```text
CheckServiceDependencies
Obtaining Config for "Dnscache": The system cannot find the file specified.
```

结论：单纯重试不能解决。

### 5.2 下载官方 AMD64 MSI

下载了官方 MSI：

```text
D:\Apps\tailscale\tailscale-setup-1.98.4-amd64.msi
```

来源：

```text
https://pkgs.tailscale.com/stable/tailscale-setup-1.98.4-amd64.msi
```

### 5.3 尝试覆盖依赖列表

尝试通过 MSI 属性覆盖依赖检查列表，先去掉 `Dnscache`：

```powershell
msiexec.exe /i "D:\Apps\tailscale\tailscale-setup-1.98.4-amd64.msi" /qn /norestart INSTALLDIR="C:\Program Files\Tailscale" TS_SERVICE_DEPENDENCIES="iphlpsvc;netprofm;WinHttpAutoProxySvc" /L*V "D:\Apps\tailscale\tailscale-msi-override-install.log"
```

结果：失败。

失败点从 `Dnscache` 移到 `iphlpsvc`：

```text
Error: Obtaining Config for "iphlpsvc": The system cannot find the file specified.
```

结论：不是 `Dnscache` 单个服务异常，而是这个 MSI 自定义检查动作读取服务配置时整体异常。

### 5.4 尝试清空依赖列表

尝试设置：

```text
TS_SERVICE_DEPENDENCIES=""
```

结果：失败。

失败原因：MSI 尝试读取空服务名：

```text
OpenServiceForRead(""): The filename, directory name, or volume label syntax is incorrect.
```

结论：不能通过清空依赖列表绕过。

### 5.5 尝试使用基础服务占位

尝试将依赖列表设置为：

```text
TS_SERVICE_DEPENDENCIES="RpcSs"
```

结果：失败。

失败原因：

```text
OpenServiceForRead("RpcSs"): Access is denied.
```

结论：继续调整依赖列表不是稳定方案。

### 5.6 直接修改 MSI 跳过检查

曾复制一份 MSI，并删除 `InstallExecuteSequence` 中的 `CheckServiceDependencies` 行：

```text
D:\Apps\tailscale\tailscale-setup-1.98.4-amd64-skip-service-check.msi
```

这能绕过原始失败点，但直接修改 MSI 文件本体后，后续安装阶段出现：

```text
Error 2717: Bad action condition or error calling custom action 'InstallExecute'.
```

结论：直接改 MSI 本体不可用，会破坏安装执行脚本一致性。

---

## 6. 最终解决方案

最终采用的方式是：

- 保留官方原始 MSI 不变。
- 生成一个 MST transform。
- MST 只做一件事：从 `InstallExecuteSequence` 中移除 `CheckServiceDependencies` 这一项。
- 使用原始 MSI + MST 安装。

生成的 transform 文件：

```text
D:\Apps\tailscale\tailscale-skip-service-check.mst
```

最终安装命令等价于：

```powershell
msiexec.exe /i "D:\Apps\tailscale\tailscale-setup-1.98.4-amd64.msi" TRANSFORMS="D:\Apps\tailscale\tailscale-skip-service-check.mst" /qn /norestart INSTALLDIR="C:\Program Files\Tailscale" /L*V "D:\Apps\tailscale\tailscale-msi-transform-admin-install.log"
```

注意：此命令需要管理员权限运行。

最终安装日志：

```text
D:\Apps\tailscale\tailscale-msi-transform-admin-install.log
```

成功证据：

```text
TRANSFORMS = D:\Apps\tailscale\tailscale-skip-service-check.mst
Product: Tailscale -- Installation completed successfully.
Windows Installer 已安装产品。产品名称: Tailscale。产品版本: 1.98.4。产品语言: 1033。制造商: Tailscale Inc.。安装成功或错误状态: 0。
MainEngineThread is returning 0
```

---

## 7. 最终验证结果

### 7.1 安装目录

安装目录存在：

```text
C:\Program Files\Tailscale
```

目录中包含核心文件：

```text
tailscale-ipn.exe
tailscale.exe
tailscaled.exe
wintun.dll
```

### 7.2 Windows 服务

服务已创建并运行：

```text
SERVICE_NAME: Tailscale
STATE              : 4  RUNNING
START_TYPE         : 2  AUTO_START
BINARY_PATH_NAME   : "C:\Program Files\Tailscale\tailscaled.exe"
SERVICE_START_NAME : LocalSystem
```

服务依赖关系正常：

```text
DEPENDENCIES       : Dnscache
                   : iphlpsvc
                   : netprofm
                   : WinHttpAutoProxySvc
```

### 7.3 版本验证

执行：

```powershell
"C:\Program Files\Tailscale\tailscale.exe" version
```

输出版本：

```text
1.98.4
long version: 1.98.4-t9e69045b2-ged3a62f14
```

---

## 8. 当前目录中的相关产物

原始安装器：

```text
D:\Apps\tailscale\tailscale-setup-1.98.4.exe
```

官方 MSI：

```text
D:\Apps\tailscale\tailscale-setup-1.98.4-amd64.msi
```

最终成功使用的 transform：

```text
D:\Apps\tailscale\tailscale-skip-service-check.mst
```

最终成功日志：

```text
D:\Apps\tailscale\tailscale-msi-transform-admin-install.log
```

中间尝试日志：

```text
D:\Apps\tailscale\tailscale-install-retry.log
D:\Apps\tailscale\tailscale-install-retry_000_MsiAMD64.log
D:\Apps\tailscale\tailscale-msi-override-install.log
D:\Apps\tailscale\tailscale-msi-no-dependency-check-install.log
D:\Apps\tailscale\tailscale-msi-rpcss-dependency-install.log
D:\Apps\tailscale\tailscale-msi-skip-service-check-install.log
D:\Apps\tailscale\tailscale-msi-skip-service-check-admin-install.log
```

中间生成但最终未采用的 MSI 副本：

```text
D:\Apps\tailscale\tailscale-setup-1.98.4-amd64-skip-service-check.msi
D:\Apps\tailscale\tailscale-setup-1.98.4-amd64-transform-source.msi
```

---

## 9. 复现和再次安装建议

如果之后需要在同一台机器上重新安装，优先使用最终成功路径：

```powershell
Start-Process -FilePath "msiexec.exe" -Verb RunAs -Wait -ArgumentList '/i "D:\Apps\tailscale\tailscale-setup-1.98.4-amd64.msi" TRANSFORMS="D:\Apps\tailscale\tailscale-skip-service-check.mst" /qn /norestart INSTALLDIR="C:\Program Files\Tailscale" /L*V "D:\Apps\tailscale\tailscale-msi-transform-admin-install.log"'
```

如果要使用图形界面登录 Tailscale，可直接运行：

```text
C:\Program Files\Tailscale\tailscale-ipn.exe
```

如果要使用命令行登录：

```powershell
"C:\Program Files\Tailscale\tailscale.exe" up
```

---

## 10. 风险和注意事项

1. 本次最终安装方式没有修改官方 MSI 主体文件，而是通过 MST transform 跳过了异常的前置检查动作。
2. 跳过的是安装前检查，不是跳过服务依赖配置。最终服务依赖关系仍然是 Tailscale 官方配置的四个依赖服务。
3. 原始系统存在待重启标记，建议在方便时重启一次 Windows，以清理 `PendingFileRenameOperations` 等待处理状态。
4. 如果未来升级 Tailscale 时再次遇到同类错误，可以优先检查新版 MSI 是否仍有 `CheckServiceDependencies` 前置检查问题，再决定是否生成新版对应的 MST。

---

## 11. 最终状态

本次问题已解决。

```text
Tailscale 版本：1.98.4
安装目录：C:\Program Files\Tailscale
Windows 服务：Tailscale
服务状态：RUNNING
安装结果：成功
```