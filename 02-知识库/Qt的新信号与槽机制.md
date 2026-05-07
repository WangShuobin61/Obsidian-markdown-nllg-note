Qt 的“新信号槽（New Signal-Slot Syntax）”，是相对于 Qt4 时代“旧信号槽”而言的。

它本质上是：

> Qt5 开始引入的一种更安全、更现代的 connect 写法。

[[编译与反编译]]
[[编译时&运行时]]

------

# 一、先说信号槽是什么

Qt 里最核心机制之一：

- 一个对象发出“信号（Signal）”
- 另一个对象执行“槽函数（Slot）”

类似：

| 场景       | Qt机制            |
| ---------- | ----------------- |
| 按钮被点击 | 发出 clicked 信号 |
| 页面响应   | 执行槽函数        |

------

# 二、旧信号槽写法（Qt4风格）

以前这样写：

```
connect(button,
        SIGNAL(clicked()),
        this,
        SLOT(onButtonClicked()));
```

这里：

- SIGNAL
- SLOT

本质是：

> 字符串宏

展开后类似：

```
"clicked()"
"onButtonClicked()"
```

------

# 三、旧写法的问题

这是 Qt 当年非常经典但也很“古老”的设计。

问题很多。

------

## 1. 编译器无法检查

比如：

```
SLOT(onButtonCliecked())
```

拼错了。

编译：

✅ 不报错

因为：

它只是字符串。

------

程序运行后：

```
QObject::connect: No such slot
```

才发现问题。

这就是：

> 运行时错误

------

## 2. 参数检查弱

比如：

信号：

```
void valueChanged(int);
```

槽：

```
void onValueChanged(QString);
```

类型不匹配。

旧语法：

很多情况编译还能过。

运行才报问题。

------

## 3. 重构困难

IDE 无法很好跟踪字符串。

你改函数名：

```
onClicked -> onBtnClicked
```

字符串不会自动更新。

容易埋雷。

------

# 四、新信号槽（Qt5+）

Qt5 引入：

```
connect(button,
        &QPushButton::clicked,
        this,
        &MainWindow::onButtonClicked);
```

看起来：

像“函数指针”。

本质上：

> 使用 C++ 类型系统。

------

# 五、新语法为什么好

------

## 1. 编译时检查

如果函数不存在：

```
&MainWindow::onButtonCliecked
```

直接：

❌ 编译错误

而不是运行时报错。

------

## 2. 参数类型检查

如果：

```
clicked(int)
```

连接到：

```
slot(QString)
```

编译器直接拒绝。

------

## 3. IDE支持更好

因为是真实函数：

- 可跳转
- 可重构
- 可自动补全

------

## 4. 更现代C++

符合：

- lambda
- function pointer
- template

风格。

------

# 六、还能连接lambda

这是新语法特别强的地方。

------

## 例子

```
connect(button,
        &QPushButton::clicked,
        this,
        [](){
            qDebug() << "clicked";
        });
```

直接写匿名函数。

非常方便。

------

# 七、Qt项目里现在怎么选

现在基本：

| 写法           | 建议     |
| -------------- | -------- |
| 旧 SIGNAL/SLOT | 不推荐   |
| 新函数指针写法 | 主流     |
| lambda connect | 非常常见 |

------

# 八、你做Qt面试时经常会问

------

## 面试官常问：

### “新旧信号槽区别”

核心答：

1. 旧语法基于字符串宏
2. 新语法基于函数指针
3. 新语法支持编译时类型检查
4. 新语法支持 lambda
5. IDE重构支持更好

------

# 九、底层本质（稍深入）

Qt 的 signal/slot：

其实还是依赖：

- moc（Meta Object Compiler）
- 元对象系统

即使是新语法：

底层依然不是纯 C++。

只是：

> connect 的“接口层”现代化了。

------

# 十、一句话理解

你可以这么记：

| 旧信号槽   | 新信号槽    |
| ---------- | ----------- |
| 字符串匹配 | C++类型系统 |
| 运行时报错 | 编译时报错  |
| 不安全     | 类型安全    |
| Qt4时代    | Qt5主流     |

或者一句话：

> Qt 新信号槽，本质上是“把很多运行时错误提前到了编译时”。