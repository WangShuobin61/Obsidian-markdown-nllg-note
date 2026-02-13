[[git使用技巧-仓库空间释放]]
[[git使用技巧-规范提交]]


很好，这是**非常正确的时间点**来把分支结构设计好 👍
下面我**按“你现在只有 main 分支”这个前提**，一步一步给你一套**两人 Qt 项目的标准 Git 分支结构 + 每条命令的含义**。

------

## 一、推荐的分支结构（多人团队 · 简洁实用版）

```
main        ← 稳定可发布（随时可打包）
│
└── develop ← 日常集成分支（你和同事都往这里合）
     │
     ├── feature/xxx   ← 你开发某个功能
     └── feature/yyy   ← 同事开发某个功能
```

### 各分支职责一句话版

| 分支           | 作用            |
| ------------ | ------------- |
| **main**     | 永远保持“可运行、可交付” |
| **develop**  | 日常开发汇总、测试     |
| **feature/** | 每个人的具体功能开发    |

👉 **规则**：

- ❌ 不允许直接在 `main` 上开发
- ❌ 不允许 feature 直接合并到 `main`
- ✅ feature → develop → main

------

## 二、你现在要做的第一步（关键）

### 1️⃣ 从 main 创建 develop 分支（你来做）

```
git checkout -b develop
```

**含义：**

- `checkout -b develop`
  - 创建一个新分支 `develop`
  - 并立刻切换到它
  - 初始内容 = 当前 main

------

### 2️⃣ 把 develop 推送到远程（非常重要）

```
git push -u origin develop
```

**含义逐条解释：**

- `push`：推送到远程
- `origin`：默认远程仓库
- `develop`：要推送的分支
- `-u`（--set-upstream）：
  - 建立 **本地 develop ↔ 远程 develop** 的跟踪关系
  - 以后只用 `git push` 就行

📌 **这一步做完，你的同事才能看到 develop 分支**

------

## 三、同事加入后，他/她应该怎么做

### 1️⃣ 克隆项目（同事做）

```bash
git clone <仓库地址>
cd 项目目录
```

### 2️⃣ 切换到 develop（不是 main）

```bash
git checkout develop
```

如果本地没有 develop：

```bash
git fetch
git checkout develop
```

------

## 四、你和同事日常开发的标准流程（重点）

### ✨ 每个人开发功能 = 一个 feature 分支

#### 你要开发“雷达显示功能”

```bash
git checkout develop
git pull
git checkout -b feature/radar-view
```

#### 同事开发“通信模块”

```
git checkout develop
git pull
git checkout -b feature/comm-module
```

------

### ✍ 开发 + 提交（各自在自己的 feature 上）

```bash
git status     //查看当前git状态
git add .
git commit -m "feat: 实现雷达视图基础显示"
```

**提交规范建议（很重要）**

- `feat:` 新功能
- `fix:` 修 bug
- `refactor:` 重构
- `ui:` 纯 UI 改动
- `chore:` 杂项

------

### ⬆ 推送 feature 分支到远程

```bash
git push -u origin feature/radar-view

/* 使用 -u(或 --set-upstream) 参数，Git会记录下本地 feature/radar-view分支与远程 origin/feature/radar-view分支的关联关系。这个跟踪关系一旦建立，会带来直接的便利：
简化后续命令：之后在该分支（feature/radar-view）上进行操作时，你只需简单地使用 git push或 git pull，而无需再指定远程仓库和分支名。Git会自动知道要将代码推送到或从哪里拉取代码。 */
```

------

## 五、功能完成后，如何合并（你来做或共同约定）

### 1️⃣ 切回 develop

```bash
git checkout develop
git pull
```

### 2️⃣ 合并 feature

```bash
git merge feature/radar-view
```

- 有冲突 → 解决 → `git add` → `git commit`

### 3️⃣ 推送 develop

```
git push
```

------

## 六、什么时候动 main 分支？

### ✔ 只有在这些时候：

- 阶段性里程碑
- 要发版本
- 要给客户 / 测试 / 交付

### 合并方式：

```
git checkout main
git pull
git merge develop
git push
```

📌 **main 永远干净、稳定**

------

## 七、你现在立刻该做的三条命令（总结）

```
git checkout -b develop
git push -u origin develop
git branch
```

看到：

```
* develop
  main
```

✅ 分支结构准备完成
✅ 同事下周直接开干
✅ 不会乱、不怕回滚、不怕冲突


 现有如下使用场景，我和我的同事使用git版本管理工具共同开发一个项目，分支结构为master->develop-feature，我和我的同事分别在feature/modeA和feature/modeB上代码向develop合并，请给我梳理一个日常开发流程，包括git命令，和命令及其命令参数的详细解释

你们采用的 `master -> develop -> feature`分支结构是一种非常经典且高效的协作模式。下面我为你梳理一个清晰的日常开发流程，包含详细的Git命令解释和关键协作要点。

### 📁 初始准备：同步远程仓库

在开始任何新功能开发前，确保你的本地仓库与远程仓库保持同步，这是避免后续冲突的第一步。

```bash
# 1. 获取远程仓库所有分支的最新信息（不会自动合并）
git fetch origin

# 2. 切换到主分支（通常是 main 或 master）
git checkout main

# 3. 拉取远程主分支的最新代码到本地
git pull origin main

# 4. 切换到开发分支 develop
git checkout developgit

# 5. 同样，拉取远程 develop 分支的最新代码
git pull origin develop
```


  **命令解释**：`git fetch`会下载远程的更新，但让你自己决定何时合并。`git pull`= `git fetch`+ `git merge`，它会自动尝试合并。先同步 `main`再同步 `develop`是为了确保 `develop`分支是基于最新的稳定代码。

### 🔁 日常开发流程  

日常开发的核心循环是：**基于最新开发分支创建功能分支 -> 开发与提交 -> 推送前同步 -> 发起合并请求**。

#### 1. 创建功能分支

始终基于最新的 `develop`分支创建你的功能分支。

```bash
# 创建并切换到新功能分支 feature/modeA
git checkout -b feature/modeA develop
```

- 

  **命令解释**：`checkout -b`是 `branch`（创建分支）和 `checkout`（切换分支）两个操作的结合。这里的 `develop`参数指定了新分支的“源头”，确保分支起点正确。

#### 2. 进行开发与本地提交

在功能分支上独立开发，并养成频繁提交的习惯。

```bash
# 1. 查看当前修改了哪些文件
git status

# 2. 将修改的文件添加到暂存区（准备打包这次提交）
git add <file_name>  # 添加特定文件，或使用 git add . 添加所有修改

# 3. 将暂存区的更改正式提交到本地仓库，并附上清晰的提交信息
git commit -m "feat(modeA): 描述具体完成的功能"
```

- 

  **命令解释**：`git add`将工作区的变化暂存起来。`git commit`则将暂存区的内容永久记录到本地版本库。提交信息（commit message）推荐使用类型化格式（如 `feat:`, `fix:`, `docs:`），便于追溯历史。

#### 3. 推送功能分支

当完成一个阶段性开发或需要备份代码时，将分支推送到远程仓库。

```bash
# 首次推送功能分支，并建立本地分支与远程分支的追踪关系
git push -u origin feature/modeA
```

- 

  **命令解释**：`-u`（或 `--set-upstream`）参数至关重要。它建立了本地分支与远程分支的跟踪关系，之后在此分支上直接使用 `git push`或 `git pull`即可，无需再指定远程仓库和分支名。

#### 4. 推送前同步develop分支（关键步骤）

在准备将功能分支合并回 `develop`之前，必须先将 `develop`分支的最新改动同步到你的功能分支上，这能最大程度减少合并冲突。

```bash
# 1. 拉取远程 develop 分支的最新代码（确保本地develop是最新的）
git fetch origin
git checkout develop
git pull origin develop

# 2. 切换回你的功能分支
git checkout feature/modeA

# 3. 将 develop 分支的更新合并到当前功能分支
git merge develop
```

- 

  **命令解释**：这个过程叫做 **“变基（rebase）或合并（merge）上游更改”**。通过在功能分支上解决可能出现的冲突，可以保证向 `develop`分支的合并是快速且干净的。如果选择变基，可以使用 `git rebase develop`，它能获得更清晰的历史线，但操作相对复杂。

#### 5. 发起Pull Request (PR) 或 Merge Request (MR)

代码同步无误后，在代码托管平台（如 GitHub, GitLab, Gitee）上发起一个从 `feature/modeA`到 `develop`的合并请求。**这是代码合并的唯一方式，禁止直接向 `develop`分支推送代码。** 在PR/MR中清晰描述你的修改，并邀请同事进行代码审查（Code Review）。

#### 6. 代码合并与分支清理

审查通过后，由具有权限的成员（可能是你或同事）在平台上完成合并。合并后，及时清理分支以避免冗余。

```bash
# 1. 删除远程的 feature/modeA 分支（可选，建议清理）
git push origin --delete feature/modeA

# 2. 切换回 develop 分支并拉取最新代码（包含你刚合并的功能）
git checkout develop
git pull origin develop

# 3. 删除本地的 feature/modeA 分支
git branch -d feature/modeA
```

### ⚠️ 核心协作要点

要保证协作顺畅，请特别注意以下几点：

1. 

   **主分支保护**：`master`和 `develop`分支应设置为**保护分支**。任何代码都不能直接推送（push）到这两个分支，必须通过 PR/MR 流程进行合并，并需要至少一名其他同事的审核（Review）。

2. 

   **冲突解决**：遇到代码冲突时不要慌张。冲突标记 `<<<<<<< HEAD`，`=======`，`>>>>>>> branch-name`会明确指出不同修改的位置。与相关同事沟通后，手动编辑文件，保留正确的代码，删除标记，然后执行 `git add`和 `git commit`来完成合并。

3. 

   **提交规范**：每次提交（commit）的信息应清晰明了，说明本次修改的内容，这有助于代码审查和日后维护。

### 💡 如何选择合并方式：Merge vs. Rebase

在同步 `develop`分支的更新到你的功能分支时，有两个主要选择：

| 方式               | 命令                 | 优点                           | 缺点                                 | 适用场景                                  |
| ------------------ | -------------------- | ------------------------------ | ------------------------------------ | ----------------------------------------- |
| **Merge（合并）**  | `git merge develop`  | 保留完整的提交历史，操作安全   | 会产生额外的合并提交，历史线可能复杂 | **团队协作推荐**，尤其适合不熟悉Git的团队 |
| **Rebase（变基）** | `git rebase develop` | 生成一条线性的、整洁的提交历史 | 重写历史，不适用于已推送的共享分支   | 整理本地提交，追求清晰历史线              |

**团队协作建议**：在功能分支**尚未推送到远程**时，可以使用 `rebase`来整理本地提交。在功能分支**已推送并与他人协作**后，为避免历史冲突，建议使用 `merge`来同步 `develop`的更新。

### 💎 总结

这个流程的核心在于 **“基于最新代码开发、频繁提交、推送前同步、PR合并”**。只要团队中的每位成员都遵守这个规范，并充分利用 PR 进行代码审查，就能极大提升协作效率，减少代码冲突。

希望这个详细的流程能帮助你们团队顺利进行协作！如果对某个具体命令或场景有更深入的疑问，可以随时再问我。pu
