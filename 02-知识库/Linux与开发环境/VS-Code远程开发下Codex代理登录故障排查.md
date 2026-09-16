# VS Code Remote-SSH 环境下 Codex 代理登录故障排查

> 状态：已在 Ubuntu + FlClash + RK3588 麒麟系统 + VS Code Remote-SSH 环境中验证  
> 最后验证日期：2026-09-14

## 1. 适用场景

本文适用于以下环境：

- 一台局域网内的 Ubuntu 电脑能够正常访问外网，并运行 FlClash。
- FlClash 开启混合代理端口，供局域网其他设备使用。
- 另一台 Linux 开发设备通过 Ubuntu 的代理访问网络。
- 本地 VS Code 使用 Remote-SSH 连接远程 Linux，并将 Codex 扩展安装在远程主机上。
- 远程系统浏览器可以访问网络，但 Codex 登录失败，常见错误为：

```text
Token exchange failed: token endpoint returned status 403 Forbidden:
Country, region, or territory not supported
```

本文示例拓扑：

```text
Ubuntu 代理主机                    RK3588 远程开发机
192.168.1.41                      192.168.1.39
FlClash mixed-port: 7890          VS Code Server + Codex 扩展
        │                                  │
        └──────── 局域网 HTTP 代理 ────────┘
```

替换本文中的 IP 地址和端口时，应以实际网络配置为准。

## 2. 核心结论

这类故障通常包含两个彼此独立的问题：

1. **代理端口只监听回环地址**：FlClash 只监听 `127.0.0.1:7890`，导致局域网设备连接时被拒绝。
2. **远程扩展没有继承代理配置**：麒麟桌面的“系统代理”只保证部分桌面应用走代理，不代表通过 SSH 运行的 VS Code Server 和 Codex 扩展也会走代理。

需要同时满足以下条件：

- Ubuntu 上的代理端口监听局域网地址。
- RK3588 能连接 Ubuntu 的代理端口。
- RK3588 的登录 Shell 含有正确的代理环境变量。
- 重启远程 VS Code Server，使 Codex 扩展继承新的环境变量。
- `localhost` 和 `127.0.0.1` 必须通过 `NO_PROXY` 排除，避免 OAuth 本地回调被送入代理。

## 3. 第一步：检查 FlClash 是否允许局域网连接

在 Ubuntu 代理主机执行：

```bash
sudo ss -lntp | grep ':7890'
```

### 异常结果

```text
LISTEN ... 127.0.0.1:7890 ... FlClashCore
```

`127.0.0.1` 只允许 Ubuntu 本机访问。RK3588 连接该端口时，浏览器通常显示“代理服务器拒绝连接”。

### 处理方法

在 FlClash 中：

1. 开启“允许局域网连接”“Allow LAN”或名称相近的选项。
2. 如果存在“监听地址 / Bind Address”，将其设置为 `0.0.0.0`。
3. 重启 FlClash 核心。

再次执行检查命令，正常结果应包含以下形式之一：

```text
0.0.0.0:7890
*:7890
192.168.1.41:7890
```

### 可选：检查 Ubuntu 防火墙

如果端口已经监听局域网地址，但远程设备仍无法连接，可以检查防火墙：

```bash
sudo ufw status
```

如果 UFW 已启用，建议只允许指定开发机访问代理端口：

```bash
sudo ufw allow from 192.168.1.39 to any port 7890 proto tcp
```

不要将代理端口暴露到公网。

## 4. 第二步：确认远程主机没有自动继承本地代理

在连接 RK3588 的 VS Code 远程终端中执行：

```bash
env | grep -iE '^(http|https|all|no)_proxy='
```

如果没有输出，说明当前远程环境没有代理变量。即使远程系统中的 Firefox 能正常上网，Codex 扩展仍可能直接出网。

这是因为 Remote-SSH 的 VS Code Server 和多数工作区扩展运行在远程主机上；本地 VS Code 或远程桌面的代理设置不会自动等价地应用到这些进程。

## 5. 第三步：验证远程主机能够使用局域网代理

如果系统安装了 `wget`，执行：

```bash
http_proxy=http://192.168.1.41:7890 \
https_proxy=http://192.168.1.41:7890 \
wget -S -O /dev/null --timeout=15 https://chatgpt.com/cdn-cgi/trace
```

成功时应看到：

```text
正在连接 192.168.1.41:7890... 已连接。
HTTP/1.1 200 OK
```

如需查看代理出口信息，可以执行：

```bash
http_proxy=http://192.168.1.41:7890 \
https_proxy=http://192.168.1.41:7890 \
wget -qO- --timeout=15 https://chatgpt.com/cdn-cgi/trace \
  | grep -E '^(ip|loc)='
```

如果已经安装 `curl`，也可以使用：

```bash
curl -sS --max-time 15 \
  -x http://192.168.1.41:7890 \
  https://chatgpt.com/cdn-cgi/trace \
  | grep -E '^(ip|loc)='
```

若此步骤失败，应先处理代理主机监听、防火墙、局域网隔离或 FlClash 规则问题，不要继续修改 Codex 配置。

## 6. 第四步：将代理变量写入远程登录环境

先确认远程用户的默认 Shell：

```bash
getent passwd "$USER" | cut -d: -f7
```

如果输出为 `/bin/bash`，继续检查 Bash 登录文件：

```bash
ls -l ~/.bash_profile ~/.bash_login ~/.profile ~/.bashrc 2>/dev/null
```

Bash 按优先级读取 `~/.bash_profile`、`~/.bash_login`、`~/.profile` 中第一个存在的登录配置文件。本文验证环境中只有 `~/.profile`，因此编辑：

```bash
nano ~/.profile
```

在文件末尾添加：

```bash
# Proxy for VS Code Remote extensions and Codex
export HTTP_PROXY="http://192.168.1.41:7890"
export HTTPS_PROXY="http://192.168.1.41:7890"
export http_proxy="$HTTP_PROXY"
export https_proxy="$HTTPS_PROXY"

# Keep OAuth localhost callback and LAN traffic direct
export NO_PROXY="localhost,127.0.0.1,::1,192.168.1.39,192.168.1.41"
export no_proxy="$NO_PROXY"
```

注意事项：

- 同时设置大写和小写变量，以兼容不同网络库。
- `HTTPS_PROXY` 的值仍然可以是 `http://...`，它表示通过 HTTP 代理的 `CONNECT` 隧道访问 HTTPS。
- 不要把 RK3588 上的代理地址写成 `127.0.0.1:7890`。在 RK3588 上，`127.0.0.1` 指向 RK3588 自己，而不是 Ubuntu。
- `NO_PROXY` 中必须包含 `localhost`、`127.0.0.1` 和 `::1`，否则 Codex 的本地 OAuth 回调可能被错误地转发到代理。
- 如果存在 `~/.bash_profile` 或 `~/.bash_login`，应修改实际被 Bash 优先读取的文件，或由该文件显式加载 `~/.profile`。

## 7. 第五步：彻底重启远程 VS Code Server

仅关闭终端或重载 Codex 面板通常不够。旧的远程扩展进程仍保留启动时的环境变量。

在 VS Code 中执行：

1. 按 `Ctrl+Shift+P` 打开命令面板。
2. 运行 `Remote-SSH: Kill VS Code Server on Host...`。
3. 选择对应的 RK3588 主机。
4. 等连接断开后，重新通过 Remote-SSH 连接该主机。

重新连接后，在远程终端验证：

```bash
env | grep -iE '^(http|https|no)_proxy='
```

预期至少能看到：

```text
HTTP_PROXY=http://192.168.1.41:7890
HTTPS_PROXY=http://192.168.1.41:7890
NO_PROXY=localhost,127.0.0.1,::1,192.168.1.39,192.168.1.41
```

随后打开 Codex 侧边栏，重新发起登录并完成浏览器授权。

## 8. 本次验证结果

本次实测依次确认了以下状态：

1. FlClash 最初只监听 `127.0.0.1:7890`，开启局域网访问后，RK3588 浏览器恢复联网。
2. RK3588 的代理环境变量最初为空。
3. RK3588 使用显式代理访问 `chatgpt.com` 时返回 `HTTP/1.1 200 OK`，证明局域网代理链路正常。
4. 将代理变量写入 `~/.profile` 并终止、重建远程 VS Code Server 后，远程环境成功读取变量。
5. Codex 扩展登录成功，403 错误消失。

## 9. 常见误区

### 误区 1：远程浏览器能上网，所以 Codex 一定能上网

不成立。桌面系统代理、终端环境、VS Code Server 和远程扩展可能使用不同的网络配置来源。

### 误区 2：本地 VS Code 已经配置代理，Remote-SSH 扩展会自动复用

不成立。安装在远程主机上的工作区扩展通常在远程扩展宿主中运行，需要远程主机自身具备正确的代理配置。

### 误区 3：远程机器也使用 `127.0.0.1:7890`

只有在远程机器本身运行代理，或已经建立 SSH 端口转发时才成立。本例的代理实际位于 `192.168.1.41`，因此 RK3588 必须使用 `192.168.1.41:7890`。

### 误区 4：所有 403 都是 OAuth 回调失败

错误信息必须区分处理：

- `Connection refused`、`localhost` 无法访问：更像是回调端口或端口转发问题。
- `Country, region, or territory not supported`：说明 token 请求已经到达服务端，但请求出口不符合服务可用性要求，常见原因是远程进程没有走预期的合规网络出口。

### 误区 5：只写 `HTTPS_PROXY`，不设置 `NO_PROXY`

这可能导致 `localhost` OAuth 回调也尝试经过代理。设置代理变量时应同时设置 `NO_PROXY`。

### 误区 6：修改配置后只重启 Codex 面板

环境变量由进程启动时继承。应终止并重新创建远程 VS Code Server，而不是只重载网页或侧边栏。

## 10. 安全与运维建议

- 代理服务应只监听可信局域网，不要在路由器上将 `7890` 映射到公网。
- 使用主机防火墙限制可访问代理端口的客户端 IP。
- 在访客 Wi-Fi、公共网络或多人共享网段中，建议启用代理认证或使用 SSH 隧道。
- 如果代理需要用户名和密码，不要将明文凭据提交到代码仓库或公司知识库；应使用组织批准的凭据管理方式。
- 将代理主机 IP 固定为 DHCP 静态租约或静态地址，避免重启路由器后地址变化。
- 确保代理出口、账号使用方式和服务所在地区符合公司安全政策及服务条款。
- 定期确认 FlClash 的“允许局域网连接”设置在升级或切换配置后没有被重置。

## 11. 快速检查清单

```text
[ ] Ubuntu 的 7890 监听在 0.0.0.0、* 或局域网 IP，而不是 127.0.0.1
[ ] RK3588 能连接 192.168.1.41:7890
[ ] 通过该代理访问 chatgpt.com 能得到 HTTP 200
[ ] 远程登录配置中存在 HTTP_PROXY 和 HTTPS_PROXY
[ ] NO_PROXY 包含 localhost、127.0.0.1 和 ::1
[ ] 已执行 Kill VS Code Server on Host 并重新连接
[ ] 新远程终端能看到代理环境变量
[ ] Codex 已重新发起登录
```

## 12. 回滚方法

如果不再需要远程代理：

1. 从实际使用的 Bash 登录文件中删除本文添加的代理变量。
2. 再次执行 `Remote-SSH: Kill VS Code Server on Host...`。
3. 重新连接远程主机。
4. 用以下命令确认变量已清除：

```bash
env | grep -iE '^(http|https|no)_proxy='
```

如果此前创建过但环境未使用 `~/.vscode-server/server-env-setup`，也可以删除该文件，避免以后版本或其他工具意外加载旧配置。

## 13. 参考资料

- [OpenAI Docs：Codex IDE extension](https://learn.chatgpt.com/docs/codex/ide)
- [OpenAI Docs：Developer settings](https://learn.chatgpt.com/docs/developer-settings?surface=ide)
- [Visual Studio Code：Remote Development using SSH](https://code.visualstudio.com/docs/remote/ssh)
- [Visual Studio Code：Remote Development Tips and Tricks](https://code.visualstudio.com/docs/remote/troubleshooting)
- [补充参考：CSDN 本地 Codex 403 处理文章](https://blog.csdn.net/m0_66677850/article/details/154015100)

> 第三方文章可用于理解代理变量和 `NO_PROXY` 的作用，但 Remote-SSH 场景必须区分“本地主机”和“远程主机”的 `127.0.0.1`，不能直接照搬本机代理地址。
