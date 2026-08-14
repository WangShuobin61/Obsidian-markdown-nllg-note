# 重要说明：本机 AI Agent 沙箱的 uid 映射陷阱

> 放置位置：`~/.local/share/applications/`
> 目的：防止以后（尤其是 AI 编程助手）再次误判"家目录属主变成 root"，从而做出不必要甚至危险的 `chown` 操作。

## 一句话结论

在 Cursor / AI Agent 的 **user namespace 沙箱** shell 里：

```
沙箱内的 root(uid 0)  ==  宿主机上的 nllg(uid 1000)
```

**所以在 agent 终端里看到的 "root 属主" 文件，映射到真实系统上恰恰就是 nllg 本人。家目录属主一直是正常的，从未损坏。**

## 如何一眼确认自己处在这种沙箱里

在可疑的 shell 中执行：

```bash
cat /proc/self/uid_map
```

若输出形如（三列：容器内起始ID / 宿主起始ID / 数量）：

```
         0       1000          1
```

即代表：**当前 shell 的 root 就是宿主的 uid 1000（nllg）**，只映射这一个用户。

旁证：

```bash
ls -l /etc/shadow        # 真正属于宿主 root 的文件，在这里会显示为 nobody/nogroup
readlink /proc/self/ns/user   # 会显示 user:[..........]，说明在独立 user namespace
```

## 会产生的"假象"与正确解读

| 在沙箱里观察到的现象 | 错误解读（❌） | 正确解读（✅） |
|---|---|---|
| `ls -ld ~` 显示属主 `root` | 家目录被 chown 成 root 了 | 那个 root 就是 nllg，属主正常 |
| `find ~ -user nllg` 返回 0 | 我对自己家目录零所有权 | 命名空间视角错觉，宿主上仍属 nllg |
| `find ~ -user root` 命中一切 | 全被 root 占了 | 全部是 nllg 的正常文件 |
| `id -un` 显示 `root` | 我在用 root 干活 | 沙箱内身份，等于宿主 nllg |

## 绝对不要做的事

- **不要**因为"看到属主是 root"就执行 `sudo chown -R nllg:nllg ~` 之类的批量改属主。
  - 在此沙箱内 `chown` 到 uid 1000 会全部返回 `EINVAL（无效的参数）`，因为 uid 1000 在沙箱内没有被映射——即"看起来危险，实则空操作，不会改动系统"，但这纯属白费力气且会误导判断。
  - 如果哪天沙箱映射范围变了，这类批量 `chown` 才可能真正造成破坏。

## 判断属主问题的正确流程

1. 先 `cat /proc/self/uid_map` 确认是否在映射沙箱里。
2. 若在沙箱里，看到的"root"要按上表翻译成宿主真实用户后再判断。
3. 只有在**宿主原生终端**（非 agent 沙箱）里看到属主异常，才考虑是否真的需要修属主。
4. 家目录内有跨文件系统挂载时（如本机 `~/data` 是 NTFS，挂载参数 `uid=0` 固定属主），`chown` 对它无效且不该碰；批量操作应使用 `find ~ -xdev` 限定在家目录本体文件系统内。

## 关联：Cursor 助手启动器

同目录下的 `cursor-assistant.desktop` 指向 `~/.local/bin/launch-cursor-assistant.sh`：

- 点击后用 `zenity` 弹窗询问「打开终端」/「静默启动」。
- 「静默启动」用 `setsid` 让 AppImage 完全脱离终端后台运行，不占用终端。
- desktop 与脚本分别位于 `~/.local/share/applications/` 和 `~/.local/bin/`，符合 XDG 规范。

## 遗留清理：删除误建的旧副本

本文档最初被写到了 `~/.local/share/applications/`，后来才挪到当前的 `doc/Reference file/`。

由于 agent 沙箱对 `~/.local/share/applications/`（目录权限 `drwxr-xr-x`）没有写权限，无法在沙箱内删除源文件，因此那里残留了一份旧副本。请在**真机原生终端**（非 agent 沙箱）里执行以下命令清理：

```bash
rm "$HOME/.local/share/applications/README-uid映射说明.md"
```

> 注：这与本文主题同源——沙箱内"看似有权限、实则受映射与目录权限限制"，删源失败正是这种环境特性的又一次体现，并非命令写错。

---
*记录时间：2026-07-07。根因：agent shell 运行于 user namespace，uid_map = `0 1000 1`。*