# SolidWorks 清淤机器人模型导出为 ROS1 URDF 完整流程

## 1. 目标与整体流程

本文说明如何把 SolidWorks 清淤机器人总装转换为 ROS1 Noetic 可使用的 URDF，并在 Docker 容器中的 RViz 显示。本项目不依赖旧版 SW2URDF 插件，而是采用可复现的 STL 分件处理流程：

```text
SolidWorks 总装
  → 分件导出二进制 STL
  → 统计每个零件的坐标和包围盒
  → 从装运车中筛选内部清淤小车
  → 合并小车网格
  → SolidWorks 坐标转 ROS 坐标
  → 创建 URDF/Xacro、TF、Launch 和 RViz 配置
```

## 2. 项目路径

```text
SolidWorks模型：E:\清淤机器人模型
ROS1工作区：   E:\Projects\sx_catkin_cleansilt_ws
描述包：       E:\Projects\sx_catkin_cleansilt_ws\cleansilt_description
```

最终目录：

```text
cleansilt_description
├── CMakeLists.txt
├── package.xml
├── config
│   ├── MODEL_BREAKDOWN.md
│   ├── stl_components.csv
│   └── small_robot_parts.csv
├── launch/display.launch
├── meshes
│   ├── cleansilt_robot_body.stl
│   └── cleansilt_robot_full.stl
├── rviz/display.rviz
└── urdf
    ├── cleansilt_robot_full.urdf
    └── cleansilt_robot_full.urdf.xacro
```

- `cleansilt_robot_full.stl`：包含装运车的完整总装，仅作备份。
- `cleansilt_robot_body.stl`：只包含内部清淤小车，URDF 实际引用该文件。

## 3. SolidWorks 模型准备

打开完整总装，等待零部件解析完成。确认小车在装运车中的装配位置正确，不要移动零件或修改总装坐标系，否则后续无法根据装配坐标筛选。

如果提示“可能缺少模板或模板已损坏”，进入：

```text
工具 → 选项 → 系统选项 → 文件位置 → 文件模板
```

添加：

```text
C:\ProgramData\SOLIDWORKS\SOLIDWORKS 2024\templates
```

默认模板可选择：

```text
零件：   gb_part.prtdot
装配体： gb_assembly.asmdot
```

## 4. 从 SolidWorks 分件导出 STL

### 4.1 为什么不能只导出一个 STL

完整总装保存成单一 STL 后，会丢失零件名称、装配层级和部件开关状态。URDF 无法从单一网格中区分装运车、小车、车轮、传感器和执行机构。因此必须保留一套分件 STL。

### 4.2 导出步骤

选择：

```text
文件 → 另存为 → STL (*.stl) → 选项
```

设置：

```text
输出格式：二进制
单位：毫米
分辨率：粗糙或自定义
将装配体所有零部件保存在单一文件中：不勾选
```

保存到：

```text
E:\清淤机器人模型\零部件
```

本项目导出了656个 STL，总大小约513.4 MB。

如需完整总装备份，可再次导出，并勾选“将装配体所有零部件保存在单一文件中”，保存为 `cleansilt_robot_full.stl`。该文件只能做静态外观显示。

## 5. 分析 STL 的坐标和尺寸

分析脚本：

```text
tools\analyze_stl_components.ps1
```

Windows PowerShell 执行：

```powershell
cd E:\Projects\sx_catkin_cleansilt_ws
& .\tools\analyze_stl_components.ps1 `
  -InputDirectory 'E:\清淤机器人模型\零部件'
```

脚本读取每个二进制 STL，统计：

- 文件名、文件大小、三角面数；
- X/Y/Z 最小值和最大值；
- 零件几何中心；
- 零件包围盒尺寸。

结果写入：

```text
cleansilt_description\config\stl_components.csv
```

脚本只读取 STL，不修改 SolidWorks 源模型。

## 6. 从装运车中识别内部清淤小车

### 6.1 用驱动轮定位

两只驱动轮文件名包含：

```text
AGV200X46ZALNR080103 75A-63NR
```

其 SolidWorks 装配坐标约为：

```text
第一轮中心：(2119.9, 1924.8, 1295.8) mm
第二轮中心：(2119.9, 1924.8, 1627.6) mm
轮径：约200 mm
轮宽：约46 mm
轮中心距：约331.8 mm
```

由此确定 SolidWorks 坐标含义：

```text
X：小车前后
Y：竖直向上
Z：小车左右
```

### 6.2 严格包围盒筛选

内部小车候选范围：

```text
X：1680 ～ 2780 mm
Y：1800 ～ 2920 mm
Z：1250 ～ 1675 mm
```

必须判断整个零件是否位于范围中，不能只判断零件中心：

```text
MinX >= 1680，MaxX <= 2780
MinY >= 1800，MaxY <= 2920
MinZ >= 1250，MaxZ <= 1675
```

原因是装运车平台的中心也靠近小车，但平台宽度超过1 m；只按中心筛选会错误保留平台。

### 6.3 实际筛选结果

```text
总装零件：656个
内部小车：184个
排除外部：472个
```

内部小车 SolidWorks 边界：

```text
X：1774.4349 ～ 2503.3110 mm
Y：1824.7977 ～ 2908.9473 mm
Z：1267.3733 ～ 1658.6733 mm
```

尺寸：

```text
前后：728.9 mm
高度：1084.1 mm
左右：391.3 mm
```

保留的主要部件：

- 两只200 mm AGV驱动轮；
- 左右600 W驱动机构；
- 主体车架与外壳；
- `YK-IMU300` IMU；
- 电源、IO及驱动器；
- `FMIE-50-60-1610 / PJ-M400` 推杆或执行机构；
- 上部清淤机构及安装件。

排除的主要部件：

- 外部装运车平台与框架；
- 四个 `TCF35-1.2T` 装运车轮组；
- 吊装、输送和运输辅助机构；
- 人体及场景模型；
- 外部泵、储罐和电气柜。

筛选结果保存在：

```text
cleansilt_description\config\small_robot_parts.csv
```

## 7. 合并内部小车网格

合并脚本：

```text
tools\merge_selected_stl.ps1
```

运行：

```powershell
cd E:\Projects\sx_catkin_cleansilt_ws
& .\tools\merge_selected_stl.ps1
```

脚本根据 `small_robot_parts.csv` 合并184个 STL，输出：

```text
cleansilt_description\meshes\cleansilt_robot_body.stl
```

结果：

```text
三角面：5,388,872
大小：约256.96 MB
二进制STL结构：验证通过
```

原始分件不会被删除或修改。

## 8. SolidWorks 坐标转 ROS 坐标

ROS REP-103 使用：

```text
X：机器人前方
Y：机器人左方
Z：机器人上方
```

SolidWorks Y轴向上，因此绕 X轴正向旋转90°：

```text
roll  = π/2 = 1.5707963
pitch = 0
yaw   = 0
```

映射关系：

```text
ROS X = SolidWorks X
ROS Y = -SolidWorks Z
ROS Z = SolidWorks Y
```

根据小车包围盒中心和最低点，CAD到ROS的固定变换为：

```text
xyz = -2.1388730  1.4630233  -1.8247977
rpy =  1.5707963  0           0
```

效果：小车平面中心位于 `x=0,y=0`，最低点位于 `z=0`，模型由Y-up变为Z-up。

## 9. URDF 和 TF 树

当前 TF：

```text
base_footprint
└── base_link
    └── cad_link
```

- `base_footprint`：机器人在地面的二维投影；
- `base_link`：底盘主坐标系；
- `cad_link`：转换后的 SolidWorks 模型坐标系。

关键 URDF：

```xml
<joint name="base_link_to_cad_link" type="fixed">
  <parent link="base_link"/>
  <child link="cad_link"/>
  <origin xyz="-2.1388730 1.4630233 -1.8247977"
          rpy="1.5707963 0 0"/>
</joint>

<link name="cad_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://cleansilt_description/meshes/cleansilt_robot_body.stl"
            scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>
</link>
```

SolidWorks STL 使用毫米，ROS 使用米，所以必须设置：

```xml
scale="0.001 0.001 0.001"
```

坐标补偿应写在固定关节中，不应只写在 `<visual><origin>`，否则 TF 树不能表达该相对关系。

## 10. ROS 包配置

`package.xml` 主要依赖：

```xml
<buildtool_depend>catkin</buildtool_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>joint_state_publisher</exec_depend>
<exec_depend>rviz</exec_depend>
<exec_depend>xacro</exec_depend>
```

维护者邮箱必须是合法格式：

```xml
<maintainer email="john@example.com">john</maintainer>
```

使用 `john@localhost` 会触发 `Invalid email` 错误。

## 11. Docker/ROS1 运行流程

### 11.1 WSL2终端启动容器

```bash
cd /mnt/e/Projects/sx_catkin_cleansilt_ws
docker compose up -d
docker compose ps
docker compose exec ros1 bash
```

若 WSL 找不到 Docker，在 Docker Desktop 中打开：

```text
Settings → Resources → WSL Integration → Ubuntu-22.04
```

再在 Windows PowerShell 执行 `wsl --shutdown`，重开 WSL 后验证 `docker version`。

### 11.2 容器内设置环境并启动

```bash
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=/ws/src:$ROS_PACKAGE_PATH
rospack find cleansilt_description
roslaunch cleansilt_description display.launch
```

正常情况下 `rospack find` 输出：

```text
/ws/src/cleansilt_description
```

启动文件会展开 Xacro、设置 `robot_description`、启动状态发布器和 RViz，并显示 RobotModel 与 TF。

## 12. 调整位置和角度

可通过 Launch 参数临时调整：

```bash
roslaunch cleansilt_description display.launch \
  cad_x:=-2.1388730 \
  cad_y:=1.4630233 \
  cad_z:=-1.8247977 \
  cad_roll:=1.5707963 \
  cad_pitch:=0 \
  cad_yaw:=0
```

```text
90°  = 1.5707963
-90° = -1.5707963
180° = 3.1415926
```

- 模型侧躺：修改 `cad_roll` 或 `cad_pitch`；
- 水平朝向错误：修改 `cad_yaw`；
- 没有落地：修改 `cad_z`；
- 偏离中心：修改 `cad_x`、`cad_y`。

## 13. 验证 URDF、Xacro 和 TF

若没有 `check_urdf`：

```bash
apt-get update
apt-get install -y liburdfdom-tools
```

检查：

```bash
check_urdf /ws/src/cleansilt_description/urdf/cleansilt_robot_full.urdf

xacro /ws/src/cleansilt_description/urdf/cleansilt_robot_full.urdf.xacro \
  > /tmp/cleansilt_robot.urdf
check_urdf /tmp/cleansilt_robot.urdf

rosrun tf tf_echo base_link cad_link
```

## 14. 当前工作区编译问题

原项目的 `robot_driver` 缺少：

```text
robot_driver/src/can2eth.cpp
```

整个工作区执行 `catkin_make` 会报告 `Cannot find source file` 和 `No SOURCES given to target`。这与描述包无关。`cleansilt_description` 没有需要编译的程序，设置 `ROS_PACKAGE_PATH` 后可直接 `roslaunch`。

## 15. 性能优化

当前内部小车网格约539万三角面、257 MB，可用于 RViz 外观检查，但不适合作为 Gazebo 碰撞模型。建议后续建立：

```text
meshes/visual/      降面后的视觉模型
meshes/collision/   盒体、圆柱或低面数凸包
```

建议视觉模型降到50 MB以下；单个运动 link 尽量低于10万三角面；螺栓、线缆和内部电气件不进入碰撞模型。

## 16. 制作可运动 URDF 的后续步骤

当前是内部小车的静态整体模型。差速运动结构应进一步拆成：

```text
base_footprint
└── base_link
    ├── left_drive_wheel_link   continuous
    ├── right_drive_wheel_link  continuous
    ├── imu_link                fixed
    ├── laser_link              fixed
    └── cleaning_actuator_link  prismatic/revolute
```

后续需要：

1. 从184个零件中分离左右驱动轮；
2. 分别合并 `base_link`、左轮和右轮网格；
3. 按轮中心建立两个 `continuous` 关节；
4. 根据实际机构添加推杆/清淤关节；
5. 从 SolidWorks 质量属性获取质量、质心和惯量；
6. 添加差速驱动与 Gazebo 配置。

完成后，模型才能从 RViz 静态展示升级为 ROS/Gazebo 可运动机器人模型。
