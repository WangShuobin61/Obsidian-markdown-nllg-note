要**真正释放本地和远程仓库空间**，关键不是简单 `git rm`，而是：

> **从 Git 历史中彻底删除这个大文件，并强制重写远程仓库。**

下面给你按**正确且完整流程**讲清楚。

---

# 一、为什么普通删除没用

如果你只是：

`git rm "UNIX环境高级编程第二版.pdf" git commit -m "remove pdf" git push`

**文件仍然占空间**，因为：

- Git 会在**历史提交**中永久保存该文件
- 本地 `.git` 和远程仓库空间都不会减少

---

# 二、正确做法：彻底从历史删除

## 方法一（推荐）：使用 `git filter-repo`

这是目前**官方推荐**方式，比 `filter-branch` 快很多。

### 1️⃣ 安装

`pip install git-filter-repo`
`python --version`
`python -m pip install git-filter-repo`


---

### 2️⃣ 从历史彻底删除该文件

在仓库根目录执行：

`git filter-repo --path "UNIX环境高级编程第二版.pdf" --invert-paths`
`python -m git_filter_repo --path "UNIX环境高级编程第二版.pdf" --invert-paths --force`

含义：

- `--path` 指定要删除的文件
- `--invert-paths` 表示**删除它，其余保留**
- 会**重写整个 Git 历史**

---

### 3️⃣ 强制推送到远程

⚠️ 这一步会改写远程历史，所有协作者都需要重新 clone。

`git push origin --force --all git push origin --force --tags`


---

# 三、释放本地磁盘空间（重要）

重写历史后，本地旧对象还在，需要清理：

`git reflog expire --expire=now --all git gc --prune=now --aggressive`

这样：

- `.git` 目录大小会明显下降
- 才算真正释放空间

---

# 四、让远程仓库也真正变小


### 成功后你还必须做的两步（真正释放 50MB）

### ① 清理引用日志

`git reflog expire --expire=now --all`

### ② 强力垃圾回收

`git gc --prune=now --aggressive`

---

# 然后验证仓库体积

`git count-objects -vH`

强推后：

- GitHub / GitLab **不会立刻释放空间**
- 需要等待服务器 GC（通常几分钟～几小时）

如果仓库超限：

- GitHub 可在 **Settings → Repository → Cleanup**
- 或联系平台支持触发 GC

---

# 五、防止以后再误提交大文件

## 1️⃣ 加入 `.gitignore`

`*.pdf`

## 2️⃣ 或使用 Git LFS（更专业）

适合需要保留 PDF 的情况：

`git lfs track "*.pdf"`

---

# 六、最简单总结

如果你的目标是：

> **删除 PDF + 释放本地 + 释放远程空间**

只需执行三步：

```bash
git filter-repo --path "UNIX环境高级编程第二版.pdf" --invert-paths
git push origin --force --all 
git gc --prune = now --aggressive
```

---

分析仓库体积情况

```bash
git count-objects -vH
```

