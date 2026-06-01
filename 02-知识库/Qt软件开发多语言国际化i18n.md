```xml
<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="en_US" sourcelanguage="en">
<context>
    <name>Main</name>
    <message>
        <location filename="../src/app/Main.qml" line="10"/>
        <location filename="../src/app/Main.qml" line="46"/>
        <source>气体调控机器人综合控制平台</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../src/app/Main.qml" line="80"/>
        <source>切换到亮色模式</source>
        <translation type="unfinished"></translation>
    </message>
```


解释这段代码

这是一段 **Qt 国际化（i18n）翻译文件 TS（Translation Source）** 的内容，通常由 Qt Linguist 工具维护，用于将程序界面中的文字翻译成不同语言。

文件格式是 XML，扩展名一般为 `.ts`。

---

## 整体结构

```
<?xml version="1.0" encoding="utf-8"?><!DOCTYPE TS><TS version="2.1" language="en_US" sourcelanguage="en">
```

含义：

|属性|说明|
|---|---|
|version="2.1"|TS文件格式版本|
|language="en_US"|目标翻译语言是美式英语|
|sourcelanguage="en"|源语言|

---

## context 节点

```
<context>    <name>Main</name>
```

`context` 对应 Qt 中的翻译上下文。

这里表示：

```
Main
```

或

```
Main
```

模块中的文本。

通常：

```
tr("xxx")
```

或者

```
qsTr("xxx")
```

都会被归到某个 Context 下。

---

## 第一个 message

```
<message>    <location filename="../src/app/Main.qml" line="10"/>    <location filename="../src/app/Main.qml" line="46"/>    <source>气体调控机器人综合控制平台</source>    <translation type="unfinished"></translation></message>
```

### location

表示这段文字出现的位置：

```
../src/app/Main.qml
```

第：

```
10 行46 行
```

都出现了这段文字。

---

### source

原始字符串：

```
气体调控机器人综合控制平台
```

对应源码可能是：

```
Text {    text: qsTr("气体调控机器人综合控制平台")}
```

---

### translation

```
<translation type="unfinished"></translation>
```

表示：

> 还没有完成翻译。

Qt Linguist 会把它标记成未完成状态。

例如：

```
<translation>Gas Control Robot Integrated Control Platform</translation>
```

翻译完成后就会变成：

```
<translation>    Gas Control Robot Integrated Control Platform</translation>
```

并去掉：

```
type="unfinished"
```

---

## 第二个 message

```
<message>    <location filename="../src/app/Main.qml" line="80"/>    <source>切换到亮色模式</source>    <translation type="unfinished"></translation></message>
```

对应源码：

```
Button {    text: qsTr("切换到亮色模式")}
```

原文：

```
切换到亮色模式
```

建议英文翻译：

```
<translation>Switch to Light Mode</translation>
```

---

## 为什么 source 是中文？

一般国际化有两种做法：

### 方式1（Qt官方推荐）

源码写英文：

```
text: qsTr("Switch to Light Mode")
```

TS文件：

```
<source>Switch to Light Mode</source><translation>切换到亮色模式</translation>
```

优点：

- 英文是国际通用开发语言
- 翻译语言可以无限扩展

例如：

```
en_USzh_CNja_JPfr_FR
```

---

### 方式2（你项目当前采用）

源码直接写中文：

```
text: qsTr("切换到亮色模式")
```

TS文件：

```
<source>切换到亮色模式</source><translation>Switch to Light Mode</translation>
```

优点：

- 国内项目开发方便

缺点：

- 后续维护多语言时不太规范
- 修改中文会导致翻译失效

---

## 这段 TS 文件最终作用

假设源码：

```
Text {    text: qsTr("切换到亮色模式")}
```

当前加载：

```
translator.load("en_US.qm");
```

程序界面会显示：

```
Switch to Light Mode
```

如果没有加载翻译文件：

```
切换到亮色模式
```

---

## 你的翻译文件补全后大概应该这样

```
<context>    <name>Main</name>    <message>        <location filename="../src/app/Main.qml" line="10"/>        <location filename="../src/app/Main.qml" line="46"/>        <source>气体调控机器人综合控制平台</source>        <translation>Gas Control Robot Integrated Control Platform</translation>    </message>    <message>        <location filename="../src/app/Main.qml" line="80"/>        <source>切换到亮色模式</source>        <translation>Switch to Light Mode</translation>    </message></context>
```

整个流程通常是：

```
源码(QML/C++)       ↓qsTr()/tr()       ↓lupdate       ↓.ts翻译文件       ↓Qt Linguist翻译       ↓lrelease       ↓.qm二进制翻译文件       ↓QTranslator加载       ↓界面动态切换语言
```

所以这段代码本质上是 **Qt/QML 项目的英文语言包模板文件**，目前里面的两条中文字符串已经被提取出来，但英文翻译还没有填写，因此都显示为 `unfinished`。

那使用Qt Widget可以方便的进行国际化吗？

可以，而且 **Qt Widgets 的国际化支持非常成熟**，实际上 Qt 的整套国际化机制最早就是为 Widgets 应用设计的，QML 后来也是沿用了同一套体系。

## Qt Widgets 国际化基本流程

### 1. 代码中使用 tr()

Widgets 中通常使用：

```
label->setText(tr("Hello"));button->setText(tr("Exit"));
```

或者：

```
QMessageBox::information(    this,    tr("Warning"),    tr("File not found"));
```

不要直接写：

```
button->setText("Exit");    // 无法翻译
```

而应该：

```
button->setText(tr("Exit")); // 可翻译
```

---

### 2. 提取翻译文本

执行：

```
lupdate project.pro
```

或者：

```
lupdate project.pro -ts translations/zh_CN.ts
```

Qt 会扫描：

```
tr("xxx")
```

生成：

```
<source>Hello</source><translation></translation>
```

---

### 3. 使用 Qt Linguist 翻译

打开：

```
linguist zh_CN.ts
```

界面类似：

```
Source:HelloTranslation:你好
```

翻译完成后保存。

---

### 4. 编译成 qm

```
lrelease zh_CN.ts
```

生成：

```
zh_CN.qm
```

---

### 5. 程序加载翻译

```
QTranslator translator;translator.load(":/translations/zh_CN.qm");qApp->installTranslator(&translator);
```

安装后：

```
tr("Hello")
```

自动变成：

```
你好
```

---

## Qt Designer 设计的界面也支持

假设你有：

```
mainwindow.ui
```

里面：

```
PushButtonText = Open
```

Qt 的 `lupdate` 会自动提取：

```
<source>Open</source>
```

无需手工处理。

这是 Widgets 的一个优势。

很多控件文字：

```
QPushButtonQLabelQCheckBoxQGroupBoxQMenuQAction
```

都会自动进入翻译文件。

---

## 动态切换语言

Widgets 还支持运行时切换语言。

例如：

```
qApp->installTranslator(&translator);
```

然后：

```
ui->retranslateUi(this);
```

Designer 自动生成的代码中本来就有：

```
void Ui_MainWindow::retranslateUi(QMainWindow *MainWindow){    pushButton->setText(        QCoreApplication::translate(            "MainWindow",            "Open",            nullptr));}
```

重新调用即可刷新界面。

---

## 与 QML 对比

|项目|Qt Widgets|Qt Quick/QML|
|---|---|---|
|翻译函数|tr()|qsTr()|
|提取工具|lupdate|lupdate|
|翻译文件|.ts|.ts|
|编译文件|.qm|.qm|
|Qt Linguist|支持|支持|
|动态切换语言|成熟|成熟|
|Designer自动提取|支持|无Designer|
|大型工业软件|非常多|越来越多|

底层其实是一套机制：

```
QTranslator↓.ts↓.qm↓lupdate↓lrelease
```

完全通用。

---

## 工业项目中的常见做法

像：

- Qt Group Creator
- Wireshark Foundation Wireshark
- Autodesk 部分桌面工具
- 各类工控、医疗设备上位机

大多数 Qt Widgets 项目都是：

```
tr("打开")tr("保存")tr("设备连接")tr("报警信息")
```

然后维护：

```
zh_CN.tsen_US.tsja_JP.ts
```

即可实现中英日等多语言切换。

---

对于你面试中经常接触的 Qt 开发岗位来说，如果面试官问：

> Qt Widgets 如何实现国际化？

一个比较完整的回答是：

1. 所有界面文本使用 `tr()` 包裹。
2. 使用 `lupdate` 提取翻译文本生成 `.ts` 文件。
3. 使用 Qt Linguist 完成翻译。
4. 使用 `lrelease` 编译生成 `.qm` 文件。
5. 程序运行时通过 `QTranslator` 加载 `.qm`。
6. 动态切换语言时重新安装 Translator，并调用 `retranslateUi()` 刷新界面。

这基本就是标准答案。