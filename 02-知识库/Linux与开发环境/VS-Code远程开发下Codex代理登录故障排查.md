# VS Code 中 Codex 代理、登录与联网故障通用排查指南（WSL2 / Remote-SSH）

> 更新日期：2026-10-30  
> 适用环境：Windows + WSL2、VS Code Remote-SSH，以及 Linux 本机运行 Codex 的类似代理场景。  
> 原则：先确认进程运行位置，再检查代理链路、进程环境和登录回调。已恢复的环境先核验，不必重新登录或切换网络模式。  
> 本文保留原文件名以兼容已有引用，示例中的主机地址均使用角色占位符。

## 1. 先理解：浏览器联网与 Codex 联网是两条链路

Codex 登录通常涉及以下阶段：

```text
浏览器 ──→ 登录/授权页面
   │
   └──→ localhost 回调 ──→ Codex 进程 ──→ 认证服务交换令牌
                                       └──→ 后续模型请求、流式连接
```

浏览器、VS Code、远程扩展宿主、Codex 子进程、终端可能使用不同的代理来源。浏览器授权成功，只能说明浏览器这一段可用；终端执行 `curl` 成功，也不能直接证明 Codex 进程采用了相同配置。

Windows“系统代理”、FlClash“允许局域网连接”和 Linux 的 `HTTP_PROXY` 是三个不同层面的设置：分别影响应用代理发现、代理端口可访问范围、进程出网方式。TUN 模式又是另一种网络接管方式，不能与环境变量代理混为一谈。

### 1.1 先确认 Codex 到底在哪里运行

在 VS Code 中查看左下角的连接类型、扩展安装位置，以及 `Developer: Show Running Extensions`。如果有疑问，以实际 Codex 进程所在系统为准。

| 使用方式 | 需要重点检查的环境 | 代理地址的含义 |
| --- | --- | --- |
| Windows 本机运行 Codex | Windows 进程及其配置 | `127.0.0.1` 指 Windows 本机 |
| VS Code 连接 WSL，Codex 在 WSL 中运行 | WSL Linux 的扩展宿主和 Codex 进程 | 代理在 Windows 宿主机时区分 NAT / mirrored；代理在其他 Ubuntu 主机时测试到该主机的网络 |
| VS Code Remote-SSH，Codex 在远程 Linux 上运行 | SSH 目标主机 | `127.0.0.1` 指远程 Linux 自己 |
| Linux 本机运行 Codex | 本机进程 | 本机有代理监听时可用 `127.0.0.1` |

也可能出现 Windows 界面调用 WSL 后端的组合，应继续按后端的实际运行位置排查。本文的 `server-env-setup` 是 **VS Code WSL Server** 的配置入口，不应直接套用到独立 Codex 桌面应用。

## 2. 最短排查路径

按下面的顺序定位，不要一开始同时修改多个层面的设置：

1. 确认 Codex 进程位于 Windows、WSL 还是 SSH 目标主机。
2. 在该环境内确认代理地址、端口可连接。
3. 用显式代理测试 HTTP CONNECT、TLS 和目标域名。
4. 核对当前终端与实际 Codex 进程的代理变量是否一致。
5. 只在环境变量未生效时，修正启动配置并重建相关进程。
6. 联网已正常、但浏览器回调失败时，再处理 localhost、端口转发或设备码登录。
7. 用实际 Codex 登录状态和一次请求验收。

如果 Codex 已经正常工作，完成第 1～4 项检查即可先保留现状；重新登录不是常规健康检查。

## 3. 选择正确的代理地址

### 3.1 地址占位符与主机角色

代理主机可以是 Windows，也可以是 Ubuntu；它不一定是打开 VS Code 窗口的那台电脑。本文统一使用以下占位符：

| 占位符 | 含义 |
| --- | --- |
| `[ 代理主机 ip ]` | 从 Codex 所在环境能够访问的代理服务地址；代理主机为 Windows 或 Ubuntu 均可 |
| `[ 本机 ip ]` | 运行 Codex 后端的环境所使用的地址：Remote-SSH 场景指远程 Linux，WSL 场景指 WSL 环境，不默认指 Windows 桌面电脑 |
| `[ Windows 宿主机 ip ]` | 承载 WSL 的 Windows 主机地址；仅当 Windows 上运行代理时，才可能用作代理地址 |
| `[ WSL 默认网关 ip ]` | WSL 默认路由的网关地址，只在相应拓扑中作为候选 Windows 地址，需实测 |
| `[ 历史直连目标 ip ]` | 旧配置中保留的绕过代理目标，需确认是否仍然需要 |

**所有占位符都必须替换成实际值，包括方括号及其中空格，再执行命令。** 例如下面的代理 URL 中，应完整替换 `[ 代理主机 ip ]`：

```bash
PROXY_URL='http://[ 代理主机 ip ]:7890'
```

`7890` 只是示例，应使用代理软件中的 HTTP 或 mixed-port。HTTP 代理地址不能直接指向一个只支持 SOCKS 的端口。文中的 `127.0.0.1`、`::1`、`0.0.0.0`、`::` 是回环或通配监听地址，具有固定网络含义，不是某台设备的实际地址，不能一律替换成主机 IP。

### 3.2 先区分代理主机是 Windows 还是 Ubuntu

| Codex 所在环境 | 代理主机 | 应使用的地址与检查方向 |
| --- | --- | --- |
| WSL2 | 承载该 WSL 的 Windows | NAT 下测试 `[ Windows 宿主机 ip ]`；mirrored 下可测试回环地址；具体见下一节 |
| WSL2 | 局域网内另一台 Ubuntu | 使用该 Ubuntu 的 `[ 代理主机 ip ]`，在 Ubuntu 上检查监听与防火墙；不能拿 WSL 网关代替它 |
| SSH 远程 Linux | Windows 或 Ubuntu | 使用远程 Linux 实际可达的 `[ 代理主机 ip ]`；按代理主机系统选择第 4 节对应步骤 |
| 本机 Linux / WSL | 代理运行在同一个 Linux 环境内 | 测试本地代理监听；不能仅凭安装了代理软件就认定服务正在运行 |

Ubuntu 如果只是 WSL 内的发行版，并不意味着它就是代理主机；代理运行在哪里，就检查哪里的监听和防火墙。

### 3.3 WSL2 访问 Windows 宿主机代理：先区分 NAT 与 mirrored

在 WSL 中执行：

```bash
uname -r
wslinfo --networking-mode
ip -4 route show default
```

旧版 WSL 可能没有 `wslinfo`。此时结合 Windows PowerShell 中的 `wsl --version`、`wsl -l -v` 以及 `%UserProfile%\.wslconfig` 判断；配置文件表达的是设置意图，仍需连接测试确认实际行为。

| 网络模式 | 从 WSL 访问 Windows 代理 | 注意事项 |
| --- | --- | --- |
| NAT | Windows 的 WSL 虚拟网卡地址，或实测可达的 Windows 局域网地址 | WSL 内的 `127.0.0.1` 通常不能访问 Windows 代理 |
| mirrored | 可测试 `http://127.0.0.1:7890` | 依赖 Windows / WSL 版本和实际网络行为；优先使用明确的 IPv4 回环地址 |
| 代理就在 WSL 内 | 使用 WSL 本地监听地址 | 应先确认 WSL 内确实存在代理监听 |

微软文档说明了 NAT 与 mirrored 的访问差异；不能笼统断言“WSL 一律不能用 localhost”或“WSL 与 Windows 永远共用 localhost”。[微软 WSL 网络说明](https://learn.microsoft.com/en-us/windows/wsl/networking)

NAT 下可先获取默认网关作为候选 Windows 地址：

```bash
ip -4 route show default | awk '{print $3; exit}'
```

**候选地址不等于可用代理地址。** 代理监听范围、Windows 防火墙、VPN 和虚拟网卡策略都可能影响结果。必须在 WSL 内实际测试该地址与代理端口；不要把自动获取网关的代码直接加入启动文件后就认定配置完成。

不要默认从 `/etc/resolv.conf` 的 `nameserver` 推导 Windows 代理地址；启用 DNS tunneling 等机制时，那里可能是专用 DNS 地址。

如果代理实际运行在独立 Ubuntu 主机上，应直接测试到 `[ 代理主机 ip ]` 的连接；上述 Windows 宿主机地址选择规则不适用于该 Ubuntu 主机。

### 3.4 Remote-SSH：远程主机不会自动复用本地代理

```text
Windows 或 Ubuntu 代理主机              运行 Codex 的远程 Linux
[ 代理主机 ip ]:7890   ←── HTTP 代理 ──  [ 本机 ip ]
                                       VS Code 扩展宿主 + Codex
```

如果代理位于另一台机器，远程 Linux 必须能路由到它；本地 VS Code 能访问该代理，不代表远程主机也能访问。跨网络时需要可达的企业代理、VPN 或经过批准的隧道方案。

在远程 Linux 上填写 `127.0.0.1:7890`，只有远程本机确有代理监听或已建立相应隧道时才成立。SSH 登录自身使用的 `ProxyCommand` 也不等同于远程 Codex 的 HTTPS 代理。

## 4. 检查代理监听与防火墙

只在代理需要接受其他网络端点连接时，开启 FlClash 等软件的“允许局域网连接 / Allow LAN”。WSL NAT 使用宿主机 IP 访问代理属于这一类；mirrored 模式下使用 Windows 回环代理通常不必因此开放整个局域网。

### 4.1 代理主机为 Windows

在运行 FlClash 的 Windows 主机上，确认代理软件的 HTTP / mixed-port；如果客户端需要通过主机 IP 连接，开启“允许局域网连接”，并检查监听地址设置是否已应用到代理核心。

在该 Windows 主机的 PowerShell 中查看网卡地址与监听：

```powershell
Get-NetIPAddress -AddressFamily IPv4 |
    Select-Object InterfaceAlias, IPAddress
Get-NetTCPConnection -State Listen -LocalPort 7890 |
    Select-Object LocalAddress, LocalPort, OwningProcess
```

根据客户端所在网络选择 `[ 代理主机 ip ]`；WSL 使用 Windows 宿主机代理时，同时参考第 3.3 节。通过 IP 连接仍失败时，检查 Windows Defender Firewall 的端口、来源及网络配置文件；涉及 WSL 网络策略时再检查相关 Hyper-V 防火墙规则。

### 4.2 代理主机为 Ubuntu

在运行 FlClash 等代理软件的 Ubuntu 主机上，确认 HTTP / mixed-port，并在需要跨主机访问时开启“允许局域网连接”。在该 Ubuntu 主机的终端中查看网卡地址与监听：

```bash
ip -4 address show
ss -lntp 'sport = :7890'
```

选择客户端可达的 Ubuntu 网卡地址作为 `[ 代理主机 ip ]`。Ubuntu 桌面的“系统代理”不会自动把代理服务开放给其他设备，客户端仍需指向这个地址与正确端口。

如果该 Ubuntu 主机启用了 UFW，可检查状态，并按实际客户端来源设置限定规则：

```bash
sudo ufw status
# 先将占位符完整替换成代理主机实际看到的客户端来源 IP，再执行：
sudo ufw allow from '[ 本机 ip ]' to any port 7890 proto tcp
```

如果未使用 UFW，应检查实际生效的防火墙工具，避免为了套用示例而另行启用一套防火墙。

### 4.3 两种代理主机都要检查监听范围与访问来源

监听结果的含义：

- `127.0.0.1:7890`：仅限所在系统的 IPv4 回环访问；WSL mirrored 是需要单独验证的情形。
- 指定网卡 IP：只监听该接口，优先在软件支持时缩小监听范围。
- `0.0.0.0:7890`：监听所有 IPv4 接口，应配合防火墙限制来源。
- `[::]:7890` / `:::7890`：IPv6 通配监听，是否同时接受 IPv4 取决于实现，不能只凭这行判断；继续实测。

开启 Allow LAN 后如仍无法连接，检查代理核心是否已应用设置、`[ 代理主机 ip ]` 是否正确，并按代理主机的系统检查相应防火墙。

只放行实际需要的端口、来源和网络配置文件。WSL NAT 的访问来源可能是 WSL 虚拟网段，不能机械套用 SSH 案例里的客户端局域网 IP。不要为了排查全局关闭防火墙，也不要默认将代理端口转发到公网。

`[ 本机 ip ]` 用于防火墙来源规则时，应以**代理主机实际看到的连接来源**为准。WSL NAT 或其他地址转换可能改变来源地址；Windows 代理与独立 Ubuntu 代理看到的来源也可能不同，不能仅复制本机某块网卡的地址就认定规则正确。

## 5. 分层测试：端口、代理隧道、TLS、应用响应

### 5.1 测试端口是否可连接

在 **Codex 实际运行的 Linux 环境**执行，例如 WSL 终端或远程 SSH 终端：

```bash
python3 - <<'PY'
import socket
proxy_host = '[ 代理主机 ip ]'  # 完整替换占位符，Windows / Ubuntu 代理均适用
proxy_port = 7890
with socket.create_connection((proxy_host, proxy_port), timeout=5):
    print('代理 TCP 端口可连接')
PY
```

`Connection refused` 通常表示该地址、端口没有接受连接的服务，或被主动拒绝；超时可能涉及防火墙、路由、地址错误或服务无响应。TCP 成功只代表端口可连接，尚未证明代理协议和外部联网正常。

### 5.2 显式代理测试，避免旧变量影响结果

```bash
PROXY_URL='http://[ 代理主机 ip ]:7890'  # 完整替换占位符，Windows / Ubuntu 代理均适用
curl --silent --show-error \
  --connect-timeout 10 --max-time 25 \
  --proxy "$PROXY_URL" --noproxy '' \
  --output /dev/null \
  --write-out 'http=%{http_code} connect=%{http_connect} tls_verify=%{ssl_verify_result} seconds=%{time_total}\n' \
  https://chatgpt.com/cdn-cgi/trace
```

这里的 `--noproxy ''` 只对本次测试清空绕过列表，确保目标请求经过显式代理。它不用于日常登录配置。

对 HTTPS 目标通过 HTTP 代理访问时：

- `connect=200` 表示代理接受了 CONNECT 隧道，**不是**目标网站已经返回 200。
- 命令退出码为 0、完成 TLS 握手且 `tls_verify=0`，才支持本次证书校验成功的判断；仅看一个字段不足以下结论。
- `http=200` 表示该测试 URL 返回成功，仍不能证明令牌交换、账号权限或流式请求一定正常。
- `http=000` 配合非零退出码表示尚未取得有效目标 HTTP 响应，应结合错误文本定位。

如果需要查看更详细的阶段信息，可在本机使用 `curl -v`；分享前清除代理认证信息、Cookie、令牌和出口 IP。

### 5.3 分别测试实际涉及的域名

将上面命令最后的 URL 按需替换为：

```text
https://auth.openai.com/
https://api.openai.com/v1/models
```

这些请求不附带账号凭据。未认证请求可能返回 `401`，认证站点根路径也可能返回 `403`；它们可用于区分“没有建立连接”和“已经收到 HTTP 响应”，不能拿来验证登录是否可用，也不能直接判定地区问题。API 域名测试不能替代 ChatGPT 登录或 Codex 后端的实际验收。

Cloudflare trace 只是辅助诊断 URL，不是服务健康检查接口。单个域名一次超时，应有界复查并核对代理日志、域名规则和出口稳定性，避免立即重装扩展或覆盖配置。

## 6. 检查环境变量：终端与进程要分别核验

### 6.1 查看当前终端

```bash
env | sort | grep -iE '^(http|https|all|no)_proxy='
```

如果代理 URL 含用户名、密码，输出包含凭据，请只在本机查看。重点检查：

- 大小写版本是否一致，是否有空值覆盖有效值。
- `ALL_PROXY` / `all_proxy` 是否残留旧 SOCKS 地址。
- `NO_PROXY` 是否错误包含目标服务域名或 `*`。
- 同一变量是否被 `.profile`、`.bashrc`、WSL 自动代理和其他脚本多次覆盖。

### 6.2 查看实际 Codex 和扩展宿主的代理环境

下面的只读脚本仅显示相关进程的 PID、角色和代理变量，不打印完整命令行或其他环境变量；它会尽量遮盖代理 URL 中的用户名和密码。

```bash
python3 - <<'PY'
import os
import re
from pathlib import Path

keys = {'http_proxy', 'https_proxy', 'all_proxy', 'no_proxy'}
for proc in sorted(Path('/proc').iterdir(), key=lambda p: int(p.name) if p.name.isdigit() else -1):
    if not proc.name.isdigit():
        continue
    try:
        if proc.stat().st_uid != os.getuid():
            continue
        args = (proc / 'cmdline').read_bytes().split(b'\0')
        exe = Path(args[0].decode(errors='replace')).name
        role = None
        if exe == 'codex':
            role = 'Codex'
        elif exe == 'node':
            if any(a == b'--type=extensionHost' for a in args):
                role = 'VS Code extension host'
            elif any(a.endswith(b'/server-main.js') for a in args):
                role = 'VS Code Server'
        if role is None:
            continue
        values = []
        for item in (proc / 'environ').read_bytes().split(b'\0'):
            if b'=' not in item:
                continue
            key, value = item.decode(errors='replace').split('=', 1)
            if key.lower() in keys:
                value = re.sub(r'(://)[^/\s]*@', r'\1<redacted>@', value)
                values.append(f'  {key}={value}')
        print(f'PID={proc.name} {role}')
        print('\n'.join(sorted(values)) if values else '  未发现代理环境变量')
    except (OSError, IndexError):
        continue
PY
```

若受容器、沙箱或 `/proc` 权限限制，脚本可能看不到实际进程；应在普通 WSL / SSH 终端执行。不要把“未显示进程”当成“进程没有代理”。本脚本不列出某些包装器或不同命名的后端进程。

`/proc/<pid>/environ` 主要反映进程启动时的环境，不能显示运行后所有内部网络设置。它是核对继承关系的证据，最终仍以应用行为为准。VS Code Server、扩展宿主和 Codex 的环境也可能不同；不能仅因 Server 没变量就断定 Codex 没变量。

## 7. 持久化代理配置

### 7.1 建议使用一个共享配置文件

以下是一个供 Shell 与 VS Code WSL 启动脚本共同加载的示例。**已有配置先备份、合并，不要直接覆盖；文件中只放环境变量，不运行网络测试或交互命令。**

创建目录后编辑 `~/.config/proxy/env.sh`：

```bash
mkdir -p ~/.config/proxy
```

文件内容：

```sh
# 使用前替换地址；选择经过连接测试的 HTTP / mixed 代理端口。
export HTTP_PROXY='http://[ 代理主机 ip ]:7890'
export HTTPS_PROXY="$HTTP_PROXY"
export http_proxy="$HTTP_PROXY"
export https_proxy="$HTTPS_PROXY"

# 最小回环绕过列表；按实际需求增加内网目标 IP 或域名。
export NO_PROXY='localhost,127.0.0.1,::1'
export no_proxy="$NO_PROXY"

# 如有历史 ALL_PROXY，先确认用途，再决定删除旧配置或统一设置。
```

`HTTPS_PROXY=http://...` 是合理写法：这里的 `http` 指连接 HTTP 代理的协议，再由 CONNECT 隧道承载目标 HTTPS，不表示目标通信被改成明文。

大小写变量同时设置可兼容更多客户端，但必须保持一致。不要因配置 HTTP 代理就盲目删除其他工具仍需要的 SOCKS 代理。

### 7.2 正确设置 NO_PROXY

- 至少包含 `localhost,127.0.0.1,::1`，让遵循这些变量的客户端绕过本地回调地址；这不能代替端口转发或修复浏览器自身的代理配置。
- 不完整的 IP 前缀不能作为通用的内网绕过规则；不要假定只写地址前几位就代表整个私网网段。
- 优先添加实际需要直连的完整 IP、主机名或经客户端验证的域名规则。
- CIDR 并非所有客户端都支持。例如 curl 从 7.86.0 起支持 CIDR，其他网络库仍需单独验证。
- 不要写 `NO_PROXY='*'`，否则遵循它的请求会整体绕过代理。
- 列表匹配的是**目标请求地址**，不是代理服务器地址；一般不必只因代理位于某 IP 就把该 IP 加进去。只有应用也需直接访问该 IP 的服务时才有必要。

这些规则的客户端差异可参见 [curl 官方手册](https://curl.se/docs/manpage.html)。

### 7.3 Bash / Remote-SSH：加载实际读取的启动文件

先检查 Shell 和启动文件：

```bash
getent passwd "$USER" | cut -d: -f7
ls -l ~/.bash_profile ~/.bash_login ~/.profile ~/.bashrc 2>/dev/null
```

对于 Bash 登录 Shell，`~/.bash_profile`、`~/.bash_login`、`~/.profile` 按优先级读取第一个存在的文件。若实际使用 `.profile`，可将下面一行加入其中：

```sh
[ ! -r "$HOME/.config/proxy/env.sh" ] || . "$HOME/.config/proxy/env.sh"
```

不同 SSH / VS Code 启动路径可能涉及 `.bashrc`。确实需要非交互 Bash 读取时，可将同一加载语句放在 `.bashrc` 的“非交互时 return”判断之前。保留 Bash 原有逻辑，并避免 `.profile` 与 `.bashrc` 相互递归加载。Zsh 等 Shell 应使用对应文件，不能照搬 Bash 文件名。

将代理只写入 VS Code 的 `terminal.integrated.env.linux`，只保证终端配置，不能保证扩展宿主和 Codex 后端继承；VS Code 的 `http.proxy` 也不能代替对独立后端网络行为的核验。[VS Code 远程排障说明](https://code.visualstudio.com/docs/remote/troubleshooting)

### 7.4 WSL：保留已验证的 .profile 方案，必要时增加专用入口

VS Code WSL 的扩展宿主通过默认 Shell 的交互登录模式探测环境；Bash 会按启动文件优先级读取配置，因此 `.profile` 可以生效。本文当前 WSL 案例也已验证这一点。需要区分的是，WSL Server 本身启动时不执行这些 Shell 配置文件，不能把 `.profile` 视为所有进程、所有启动方式的通用入口。[VS Code 对 Server 与扩展宿主环境的说明](https://code.visualstudio.com/docs/remote/troubleshooting#_configure-the-environment-for-the-remote-extension-host)

VS Code 官方提供 WSL 专用文件：

```text
稳定版：~/.vscode-server/server-env-setup
Insiders：~/.vscode-server-insiders/server-env-setup
```

需要更明确地配置 WSL Server 启动环境时，可在对应文件中加入：

```sh
#!/bin/sh
[ ! -r "$HOME/.config/proxy/env.sh" ] || . "$HOME/.config/proxy/env.sh"
```

该脚本使用 Bourne Shell 语法；不要直接加载包含 Bash 专用语法、ROS 初始化、交互提示等内容的整个 `.bashrc`。脚本语法错误可能阻止 VS Code Server 启动，编辑后先检查：

```bash
sh -n ~/.config/proxy/env.sh
sh -n ~/.vscode-server/server-env-setup
```

如果因此无法连接，用普通 WSL 终端将新脚本移走或恢复备份，再重连。原文将 `server-env-setup` 视为可随意删除的残留文件不够通用：在 WSL 场景，它是官方支持的配置入口。[VS Code WSL 环境设置文档](https://code.visualstudio.com/docs/remote/wsl#_advanced-environment-setup-script)

### 7.5 可选：WSL 自动代理与 mirrored

在满足版本要求的 Windows / WSL 环境中，可评估 Windows 用户目录下的 `%UserProfile%\.wslconfig`：

```ini
[wsl2]
autoProxy=true
# 只有经过评估需要更换网络模式时，才添加下面这一项：
# networkingMode=mirrored
```

`autoProxy` 用于从 Windows 获取 HTTP 代理信息；它不能替代对端口可达性、客户端支持和实际进程环境的检查。已有 `.profile` 手动设置时，也应避免两种来源互相覆盖。

mirrored 是可选网络架构调整，当前 NAT 已稳定工作时无需仅为代理切换。修改 `.wslconfig` 后需完整停止并重启相关 WSL 环境才能生效；`wsl --shutdown` 会停止所有发行版及相关工作负载，应先保存工作，再自行安排执行。[微软 WSL 高级配置](https://learn.microsoft.com/en-us/windows/wsl/wsl-config)

## 8. 让新配置生效：按运行方式重建进程

环境变量由新进程继承。`source ~/.profile` 只能改变当前 Shell 及其后续子进程，不能反向更新已经运行的扩展宿主或 Codex。

### Remote-SSH

保存工作后，在命令面板执行 `Remote-SSH: Kill VS Code Server on Host...`，选择目标主机后重新连接。重连后重新核验扩展宿主和 Codex 的变量。

### WSL

先保存工作并关闭连接该发行版的 VS Code 窗口，再重新打开。若仍在复用旧环境，或已经修改 WSL 网络配置，在 Windows PowerShell 中安排停止目标发行版后重连：

```powershell
wsl -l -v
# 将名称替换为上一步实际显示的发行版；会终止该发行版内的进程。
wsl --terminate Ubuntu-22.04-LTS
```

确实需要停止全部 WSL 环境时再使用 `wsl --shutdown`。不要在仍需保持的任务或当前代理排查会话中直接终止它们。

重建后重复第 6 节的进程核验；终端有变量但 Codex 没变量时，继续修复加载入口。仅重载 Codex 侧边栏不能保证后端已更换。

## 9. 网络已通后，单独处理登录与 localhost 回调

先根据报错确定阶段：

| 现象 | 优先排查 | 不能直接得出的结论 |
| --- | --- | --- |
| 代理地址 `Connection refused` | 地址、端口、监听范围、主动拒绝规则 | 不能说明账号有问题 |
| 连接或 TLS 超时 | 路由、防火墙、代理核心、域名规则、出口稳定性 | 不能单凭一次超时判断地区限制 |
| 证书校验失败 | 时间、CA 信任链、企业 TLS 检查 | 不应通过关闭证书校验长期解决 |
| 浏览器授权后，localhost 无法访问 | 回调监听、端口占用、浏览器绕过代理、转发 | 与远端 token 端点返回 403 不是同一阶段 |
| 明确提示 `Country, region, or territory not supported` | 实际请求出口与服务可用性要求 | 不应将所有 403 都归为这种错误 |
| 普通 URL 返回 403 | 具体响应、请求方式、访问策略或挑战页面 | 不足以证明 token 交换失败 |
| 登录后反复重连或流式中断 | 实际后端连接、长连接、代理日志和稳定性 | 简单 GET 成功不足以完成验收 |

确认使用地区与网络符合服务要求及组织政策；代理配置用于修复合法网络访问，不改变服务本身的可用性要求。

### 9.1 回调端口检查

Codex CLI 官方文档描述的默认回调端口是 `1455`；实际登录 URL 和监听端口优先，不能把端口固定值套用到所有版本或宿主。

在发生问题的登录尝试期间，用以下命令检查实际监听（只读）：

```bash
ss -lntp 'sport = :1455'
```

WSL 中启动的回调需要 Windows 浏览器能够访问；Remote-SSH 场景则需要 VS Code 端口转发或其他合适通道。回调只应向本机开放，不应为此暴露到公网。不要重复使用过期的授权链接或手动分享含 `code` / `state` 的完整回调 URL。

### 9.2 CLI 可选设备码登录

如果安装的 Codex CLI 和账号策略支持，可使用：

```bash
codex login --device-auth
```

该方式减少对 localhost 浏览器回调的依赖，但仍要求 CLI 能访问认证服务，无法解决代理不通或服务地区限制。使用前核对当前版本和账号 / 工作区设置。

SSH 场景也可按官方说明将回调通过 `ssh -L 1455:localhost:1455 user@remote` 转发。不要为了排查优先复制、打印或覆盖 `auth.json`；CLI 与扩展共享凭据还取决于用户、`CODEX_HOME` 和凭据存储配置。[OpenAI 官方认证文档](https://learn.chatgpt.com/docs/auth)

企业 TLS 代理环境应使用组织提供的 CA，并按当前 Codex 官方说明配置 `CODEX_CA_CERTIFICATE` 或相应信任链；不要使用 `curl -k`、`NODE_TLS_REJECT_UNAUTHORIZED=0` 或关闭 VS Code 证书校验作为长期修复。[OpenAI 自定义 CA 说明](https://learn.chatgpt.com/docs/auth#custom-ca-bundles)

## 10. 两种代理主机的典型案例

以下案例保留已验证的排查结论，实际主机地址统一使用占位符。

### 10.1 案例 A：Ubuntu 代理主机 → 远程 Linux → Remote-SSH

原案例记录日期：2026-09-14，本次未重新连接该设备复测。

- 代理运行在 Ubuntu 主机，服务地址为 `http://[ 代理主机 ip ]:7890`；远程开发机地址用 `[ 本机 ip ]` 表示。
- FlClash 最初仅监听回环地址；开启局域网访问后，远程设备可访问代理。
- 远程进程未继承代理变量；配置实际读取的 `.profile` 并重建 VS Code Server 后，Codex 登录恢复。

这是“跨机器代理可达性 + 远程扩展环境继承”两个问题叠加的案例。代理主机侧检查 Ubuntu 的监听与防火墙；开发机侧检查代理变量及扩展宿主环境。

### 10.2 案例 B：Windows FlClash → WSL2 Ubuntu → VS Code Codex

本次检查日期：2026-10-30。

**用户确认的恢复过程：** WSL 中原先也无法登录；修改 WSL 用户的 `.profile`，开启 Windows FlClash 的局域网代理，重启 VS Code 与 WSL 后恢复。

**本次只读核验：**

| 检查项 | 结果 |
| --- | --- |
| 系统 | Ubuntu 22.04.5 LTS，WSL2，实际网络模式为 NAT |
| 代理位置 | Windows 宿主机运行代理；本案例中 `[ 代理主机 ip ]` 是经过测试可达的 `[ Windows 宿主机 ip ]` |
| Windows 监听 | 检测到 `::` 通配地址监听；WSL 到其 IPv4 局域网地址实测成功 |
| WSL 到 `[ 代理主机 ip ]:7890` | TCP 可连接 |
| WSL 到 `127.0.0.1:7890` | 连接被拒绝；当前不能改用该地址 |
| WSL 到 `[ WSL 默认网关 ip ]:7890` | 测试超时；不能直接改成网关地址 |
| 扩展宿主与 Codex 进程 | 均继承一致的大小写 HTTP / HTTPS 代理变量 |
| VS Code Server 初始环境 | 未见代理变量；不能据此否定下游进程已取得变量 |
| `NO_PROXY` | 包含回环地址，也包含不能表示网段的不完整前缀及 `[ 历史直连目标 ip ]`，需清理或确认用途 |
| ChatGPT trace 测试 | 首次发生一次 TLS 连接超时；之后默认协议与 HTTP/1.1 两次复查均返回 200，证书校验成功 |
| 认证站点根路径 / API models | 分别返回 403 / 401，均完成 TLS 并通过证书校验；未测试携带凭据的 token 交换 |
| 近期扩展日志抽查 | 未匹配到所检查的令牌交换、连接拒绝、证书校验和流式中断错误关键词；不等于完整日志审计 |

**后续优化点：**

1. 将 `NO_PROXY` 中不能表示网段的不完整前缀改为实际需要直连的目标；确认 `[ 历史直连目标 ip ]` 是否仍需保留。
2. 当前 `.profile` 方案已经生效，可保留；如希望明确覆盖 WSL Server 的启动阶段，可按第 7 节合并为共享配置并添加 WSL 专用入口，再在方便时重建验证。
3. 使用 Windows 局域网 IP 会依赖 DHCP、网卡和所接网络；可考虑 DHCP 地址保留。不要用未经测试的 WSL 网关或 `127.0.0.1` 自动替换。
4. Allow LAN 扩大了监听范围，应确认 Windows 防火墙限制了必要来源。通配监听本身不能证明公网或全部局域网设备可访问，本次没有审计完整防火墙规则。
5. 若 TLS 超时再次出现，优先记录发生时间并检查 FlClash 的域名规则、连接与出口稳定性；本次复查通过，尚无依据认定为持续故障。

本次没有修改运行配置、重新登录、切换 WSL 网络模式或重启服务；上述修改建议应在保留现有可用配置的基础上逐项实施。

## 11. 验收清单

```text
[ ] 已明确 Codex 后端实际运行在 Windows、WSL 还是 SSH 目标主机
[ ] 已按网络模式选择地址，并在后端所在环境测试代理端口
[ ] HTTP CONNECT 与 TLS 检查通过，未用关闭证书校验掩盖问题
[ ] 未将任意 401 / 403 或一次 trace 测试当成完整登录诊断
[ ] HTTP_PROXY / HTTPS_PROXY 及小写版本一致，无冲突的旧 ALL_PROXY
[ ] NO_PROXY 包含回环地址，内网绕过规则符合实际客户端支持
[ ] 实际扩展宿主和 Codex 进程继承了预期代理配置
[ ] 修改启动配置后已在合适时机重建相关进程
[ ] 已登录的环境保持可用；未登录环境完成一次实际登录
[ ] Codex 能完成一次实际请求，流式输出正常
[ ] 代理监听与防火墙限制符合需要，宿主机地址变化风险已记录
```

## 12. 回滚与维护

修改前备份实际涉及的文件，例如 `.profile`、`.bashrc`、共享代理文件、WSL Server 启动脚本；已有文件采用合并方式，不要覆盖其他工具的初始化配置。

不再需要代理或出现回归时：

1. 恢复备份，或移除本次新增的加载语句与变量；仅对本次新建且不再使用的文件进行删除或移走。
2. 当前终端可执行下面的 `unset`，但它不影响已经运行的进程，也不清除持久化配置。
3. 保存工作后重建相应 VS Code / WSL 进程，并重新核验实际环境。
4. 若曾更改 Windows 系统代理、`.wslconfig` 或防火墙，仅回滚本次添加的具体设置。

```bash
unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy
# 仅在确认这些变量也是本次设置、且不再需要时一并清除：
unset ALL_PROXY all_proxy NO_PROXY no_proxy
```

排查资料应保留错误阶段、时间、版本、网络模式、脱敏后的地址与结果；不要记录完整环境、VS Code 连接令牌、OAuth 回调参数或 `auth.json`。公开文档统一使用角色占位符，实际执行命令时再替换为当前环境的地址。

## 13. 官方参考资料

- [OpenAI：Authentication](https://learn.chatgpt.com/docs/auth)
- [Microsoft：WSL networking](https://learn.microsoft.com/en-us/windows/wsl/networking)
- [Microsoft：WSL advanced settings](https://learn.microsoft.com/en-us/windows/wsl/wsl-config)
- [VS Code：Developing in WSL](https://code.visualstudio.com/docs/remote/wsl)
- [VS Code：Remote Development using SSH](https://code.visualstudio.com/docs/remote/ssh)
- [VS Code：Remote Development Tips and Tricks](https://code.visualstudio.com/docs/remote/troubleshooting)
- [curl：命令行手册](https://curl.se/docs/manpage.html)

本文把官方支持的配置入口、原案例记录和本次实测结果分别标明；版本更新后，以对应官方说明、实际进程环境和连接测试为准。
