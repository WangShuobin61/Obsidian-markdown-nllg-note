# 登录认证系统安全设计技术报告

## 一、功能定位

`authManagerVM` 是 `gas_control_bringup` 项目的登录认证子系统，负责用户身份验证、密码管理、角色授权和用户注册等业务。系统的核心安全目标是：**数据库被直接读取时，攻击者无法伪造合法用户身份**，也无法在不知晓密钥的前提下修改任何用户记录而不被察觉。

---

## 二、模块结构

```
authManagerVM/
├── authmanager.h / .cpp      ── 业务门面，QML 可调用的公共接口
├── authrepository.h / .cpp   ── 数据访问层，SQLite 读写封装
└── passwordhasher.h / .cpp   ── 密码哈希与 HMAC 工具类（纯静态）
```

三层职责划分清晰：QML 只与 `AuthManager` 交互，`AuthManager` 只与 `AuthRepository` 交互，`PasswordHasher` 无状态纯工具。

---

## 三、数据库结构

数据库存储在 `~/.local/share/GasControl/app.db`（`QStandardPaths::GenericDataLocation`），单表结构如下：

```
users
├── id        INTEGER  PRIMARY KEY AUTOINCREMENT
├── username  TEXT     UNIQUE NOT NULL
├── role      TEXT     NOT NULL
├── salt      BLOB     NOT NULL     ← 16 字节随机值
├── hash      BLOB     NOT NULL     ← SHA-256(password_utf8 + salt)
└── hmac      BLOB     NOT NULL     ← HMAC-SHA256(username + role + salt + hash, 硬编码密钥)
```

每条用户记录存储三个字段的组合：随机盐、密码哈希、以及覆盖全部字段的完整性签名。

---

## 四、核心安全机制

### 4.1 加盐哈希（防彩虹表）

密码不以明文存储，也不以固定盐存储，而是对每位用户生成 **16 字节密码学随机盐**：

```cpp
QByteArray PasswordHasher::generateSalt() {
    QByteArray salt;
    salt.resize(16);
    for (int i = 0; i < 16; ++i)
        salt[i] = static_cast<char>(QRandomGenerator::global()->bounded(256));
    return salt;
}
```

最终存储的是：

```
hash = SHA-256(password.toUtf8() + salt)
```

同一密码对应的哈希值因盐不同而完全不同，彩虹表攻击失效。

### 4.2 HMAC-SHA256 完整性校验（防数据库篡改）

这是本系统最具特色的设计。数据库中每条记录额外存储一个 **HMAC 签名**，签名覆盖了该用户的全部关键字段：

```
HMAC 输入 = username.toUtf8() + role.toUtf8() + salt + hash
HMAC 密钥 = 硬编码的 32 字节密钥（见下节）
```

验证时（登录、改密、重置密码前均会验证）：

```cpp
QByteArray data = username.toUtf8() + role.toUtf8() + storedSalt + storedHash;
if (!PasswordHasher::verifyHMAC(data, storedHMAC, hmacKey)) {
    emit loginFailed("用户数据校验失败");
    return false;
}
```

若攻击者直接修改数据库中的 `role`、`hash` 等任意字段，下次登录时 HMAC 校验必然失败，系统拒绝该用户，**修改行为被立即检测到**。

### 4.3 role 值参与签名（同密异钥保障）

HMAC 输入中特意包含了 `role` 字段，这是设计上的关键选择：

- 用户 A（`visitor` 角色）和用户 B（`admin` 角色）即便设置了相同密码，其 HMAC 也完全不同，因为 `role` 不同导致 HMAC 输入不同。
- 攻击者无法通过仅修改数据库中的 `role` 字段来提权——修改 `role` 后 HMAC 校验失败，该账号变为不可登录状态。
- **无法通过拷贝一个高权限用户的 `hash + salt` 覆盖另一个低权限用户来伪造提权**，因为 HMAC 中锁定了用户名和角色的组合。

这意味着：**同一个密码，在不同用户名或不同角色下，HMAC 完全不同**，是一种"同密异钥"的隐式效果。

### 4.4 硬编码 HMAC 密钥（示例）

```cpp
QByteArray PasswordHasher::getHMACKey() {
    return QByteArray::fromHex(
        "54e6wf65we4f67tew8ff6b84t8h7hj9tr8u84wr9ef4w8d7ewf9ds64c64ew897f98we" 
    );
}
```

密钥以 32 字节十六进制常量硬编码在二进制中。这是一种**带保护程序代码保密性前提下的方案**——攻击者需要反编译二进制才能提取密钥。其防御目标不是密码学上的"密钥不可见"，而是对数据库文件的直接篡改攻击：即使攻击者拿到了 `app.db`，在不知道密钥的情况下，无法构造一条通过 HMAC 校验的伪造记录。

---

## 五、角色体系

系统内置三个默认用户，对应三个权限级别：


| 用户名                  | 角色          | 权限描述       |
| -------------------- | ----------- | ---------- |
| `Visitors`           | `visitor`   | 只读访问       |
| `Administrator`      | `admin`     | 管理操作       |
| `SuperAdministrator` | `developer` | 最高权限，含用户管理 |


自定义用户只能由 `SuperAdministrator` 授权注册，注册时必须提供超级管理员密码验证，并选择 `visitor` 或 `admin` 角色之一，不能注册 `developer` 角色。

```cpp
// registerUser 中的角色限制
if (normalizedRole != "visitor" && normalizedRole != "admin") {
    emit registerResult(false, "请选择有效用户身份");
    return false;
}
```

---

## 六、完整登录验证流程

```
verifyLogin(username, password)
        │
        ├─ 查询用户 → 不存在：拒绝
        │
        ├─ 验证 HMAC(username + role + salt + hash) → 失败：拒绝
        │      ↑ 检测数据库篡改
        │
        ├─ 计算 SHA256(password + salt) → 不匹配：拒绝
        │      ↑ 验证密码正确性
        │
        └─ 通过：设置 isAuthenticated / currentUser / currentRole
                 emit authenticationChanged()
```

三层校验串联：存在性 → 完整性 → 密码正确性，缺一不可。

---

## 七、密码修改的原子性保障

修改密码时，系统重新生成 salt，同时重新计算 HMAC：

```
新 salt = generateSalt()
新 hash = SHA256(newPassword + 新salt)
新 HMAC = HMAC(username + role + 新salt + 新hash)
→ 三者原子更新到数据库同一行
```

旧密码验证通过后，新三元组一次性写入，不会出现 `hash` 与 `HMAC` 不一致的中间状态。

---

## 八、已知局限与改进方向


| 局限             | 原因             | 改进方向                         |
| -------------- | -------------- | ---------------------------- |
| HMAC 密钥硬编码在二进制 | 嵌入式场景无密钥管理基础设施 | 生产环境可从安全存储（TPM / 密钥文件）读取     |
| SHA-256 单次哈希   | 工程简洁性优先        | 可换用 PBKDF2 / bcrypt 增大暴力破解成本 |
| SQLite 文件无加密   | 本地存储场景         | 可引入 SQLCipher 对数据库文件整体加密     |


当前方案适合**嵌入式工控场景**：设备物理安全有保障，主要威胁是误操作或内部用户越权，HMAC 完整性校验对此已有效防御。