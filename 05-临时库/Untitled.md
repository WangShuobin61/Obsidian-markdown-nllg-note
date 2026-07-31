有一个问题需要你帮我出一份文档，具体是我们公司A有两个关于特种作业机器人的项目，其中一个项目外包给了公司B,一个我们自己做就是现在的气体调控机器人，外包给公司B的是一个叫做“电器盘柜搬运机器人”的项目，外包给公司B给我们是闭源交付，不提供源代码，以Docker封装的形式部署到国产工控机的国产操作系统上，这两个项目都有一个共同的导航（建图、导航、遥控控制）功能，我的老板想要复用功能公司B电器盘柜上的代码，但是公司B的代码不开源，老板不是程序员出身，他觉得只需要和公司B简单协商沟通一下，让公司B给我们通过API或调用这些功能的接口就行，可以节省我们很多工作量，但我觉得这个想法几乎很难实现。
第一、公司B需要给我们提供的是一个“SDK二次开发包”要把导航功能涉及到的接口全部给我们开放出来，这会极大的增加公司B的工作量，且这一点没有在与B公司签订的合同上体现，公司B大概不会给我们提供这个sdk开发包，下面我简单罗列了一些我们大概率需要的接口，可以帮我补充一下我没想到的，另外帮我拟一个接口名和可能需要的参数
导航功能涉及接口：
1、建图启动
2、保存地图，存图路径设置，获取存图路径，地图参数设置
3、结束建图
4、运行\终止导航节点，导航节点参数设置，选择地图
5、运行\终止键盘控制节点，键盘控制节点参数设置
6、PID调参获取、设置、调用
第二、公司B给我们交付是一个完整项目的docker封装镜像+电器盘柜搬运机器人一台，也就是软件部分是一个整体、是专为这个电器盘柜搬运机器人定制的软件，虽然和我们的气体调控机器人有相同的功能点（导航部分），但是他的导航相关业务逻辑可以也是和其他一些机器本身的功能强耦合的，帮不是说给我们暴露出上面那些接口就能直接使用，可能也需要公司B去拆分对应模块，独立出单独的docker镜像包我们去部署到我们的气体调控的机器人上，我理解的对吗，以及我该如何和老板解释清楚这件事实现难度很大


1. 背景
公司 A 当前有两个特种作业机器人项目：

电器盘柜搬运机器人

由公司 B 外包开发；
以完整 Docker 镜像形式闭源交付；
部署在国产工控机和国产操作系统上；
公司 A 不掌握源代码，也未获得二次开发 SDK。
气体调控机器人

由公司 A 自主开发；
同样需要建图、定位、导航和遥控等移动机器人基础能力。
两个项目在功能名称上存在重叠，因此产生了复用 B 公司导航代码、减少重复开发的设想。

2. 核心结论
2.1 该设想可以向 B 公司提出，但不能理解成“增加几个 API 就能直接复用”
如果 B 公司现有软件是专门为电器盘柜搬运机器人开发的完整系统，那么公司 A 真正需要的不是几个简单接口，而是：

由 B 公司把现有项目中的导航能力重新整理为一个能够脱离电器盘柜搬运机器人独立运行、能够适配其他机器人硬件、具有稳定接口和配套文档的软件组件。

这项工作更接近于：

模块拆分；
硬件解耦；
接口产品化；
独立容器化；
参数体系整理；
运行环境适配；
联调和验收。
它不是简单地给原程序增加几个函数，也不只是写一份 API 文档。

2.2 “Docker 镜像交付”不等于“软件已经模块化”
Docker 解决的是软件的打包和运行环境问题，但不能证明软件内部已经解耦。

一个 Docker 镜像内部仍然可能包含：

底盘驱动；
激光雷达驱动；
里程计；
建图；
定位；
导航；
机械臂或搬运机构控制；
电器盘柜业务流程；
状态机；
UI 后台；
数据库；
设备认证和授权逻辑。
这些模块可能通过内部 ROS Topic、Service、Action、共享文件、数据库或者私有协议互相依赖。

因此，B 公司即使愿意暴露“启动导航”接口，也可能出现以下情况：

导航程序启动了，但找不到气体调控机器人的雷达数据；
TF 坐标系不匹配；
底盘速度指令格式不同；
里程计方向、单位或精度不同；
机器人尺寸和运动学模型不同；
原程序依赖搬运机器人的业务状态机；
原程序检查特定硬件编号或授权信息；
原镜像依赖专用驱动、GPU、串口、CAN 总线或设备路径；
导航启动后仍然无法正常定位或控制机器人运动。
所以，您的理解基本正确：

如果现有导航代码与电器盘柜搬运机器人强耦合，B 公司必须先进行模块拆分和适配，最好形成独立的导航 Docker 镜像，才能部署到气体调控机器人上。

3. “API”“SDK”和“独立导航组件”的区别
这三个概念需要向管理层区分清楚。

3.1 API
API 只是调用约定，例如：

开始建图
保存地图
加载地图
开始导航
发送目标点
停止导航
API 解决的是“公司 A 的程序怎么发命令给 B 公司的程序”。

它不解决：

B 公司的程序能不能在另一台机器人上运行；
是否兼容新的底盘、雷达和里程计；
导航模块是否依赖原项目其他模块；
出现故障后如何排查；
参数是否可以针对新机器人调整。
3.2 SDK 二次开发包
SDK 通常至少包含：

API 定义；
客户端调用库；
数据结构；
错误码；
示例程序；
配置说明；
接口版本说明；
开发和调试工具。
如果公司 A 只需要通过网络调用 B 公司的独立导航服务，B 公司不一定必须提供传统意义上的链接库 SDK，也可以提供 REST、gRPC 或 ROS 2 接口包。

但无论采用哪种形式，都必须形成一套完整的接口交付包，不能只有几条口头说明或几条命令。

3.3 独立导航组件
公司 A 真正希望复用的交付物应当是：

独立导航 Docker 镜像
+ 标准接口定义
+ 机器人硬件适配规范
+ 参数配置文件
+ 调用示例
+ 部署文档
+ 测试工具
+ 验收标准
+ 后续维护和版本升级约定
因此，更准确的采购或协商名称可以是：

机器人通用导航组件及二次集成接口包

而不应仅表述为“开放几个导航 API”。

4. 建议的目标架构
建议要求 B 公司将导航系统拆成以下边界：

REST/gRPC/ROS 2接口
公司A上位机程序
导航服务接口层
通用导航核心
建图与定位
路径规划与运动控制
地图管理
机器人适配层
激光雷达
里程计/IMU
气体调控机器人底盘
电器盘柜业务模块
推荐至少拆分为：

导航核心镜像

建图；
定位；
路径规划；
目标点导航；
地图管理；
导航状态管理。
机器人适配层

底盘速度接口；
里程计接口；
激光雷达接口；
IMU 接口；
TF 坐标系；
机器人尺寸和运动学参数。
接口定义包

ROS 2 的 .msg/.srv/.action，或者 REST/gRPC Schema；
客户端示例；
错误码；
状态机说明。
这样才能把“电器盘柜搬运机器人专用导航软件”变成“可在其他机器人上二次集成的导航组件”。

5. 建议补充的接口清单
以下接口名是逻辑接口名。实际可以由 B 公司实现为：

ROS 2 Service；
ROS 2 Action；
ROS 2 Topic；
REST API；
gRPC API。
对于启动、查询、参数设置等短操作，可采用 Service 或 REST；对于目标点导航这种长时间任务，推荐采用 ROS 2 Action 或带异步任务 ID 的 API。

5.1 公共数据和返回字段
所有接口建议包含以下公共请求字段：

request_id        请求唯一编号
robot_id          机器人编号
session_id        当前控制会话编号
timestamp         请求时间
api_version       接口版本
所有接口建议返回：

success           是否成功
code              错误码
message           错误描述
request_id        对应请求编号
current_state     当前运行状态
timestamp         返回时间
异步任务还应返回：

task_id           建图、导航等任务编号
progress          当前进度
task_state        pending/running/paused/succeeded/failed/canceled
5.2 系统能力与生命周期接口
这些接口容易被遗漏，但对闭源软件集成非常重要。

GetCapabilities
查询当前组件支持哪些能力。

参数：

无，或 robot_id
返回：

supports_mapping
supports_localization
supports_navigation
supports_multi_goal
supports_teleop
supports_dynamic_parameter
supported_map_formats
supported_sensor_types
GetComponentVersion
参数：

无
返回：

software_version
api_version
image_version
image_digest
ros_distribution
build_time
supported_architecture
HealthCheck
参数：

detail_level      basic/full
返回：

service_state
sensor_state
localization_state
controller_state
map_state
dependency_state
error_list
GetSystemState
返回整个导航组件的当前状态：

IDLE
MAPPING
MAP_SAVING
LOCALIZING
NAVIGATING
TELEOPERATING
PAUSED
FAULT
STOPPING
StartNavigationService
启动导航服务内部节点，而不是直接开始一次目标点导航。

参数：

config_profile
ros_namespace
domain_id
log_level
StopNavigationService
参数：

force             是否强制停止
timeout_ms
RestartNavigationService
参数：

restart_scope     all/localization/planner/controller/map_server
SubscribeSystemEvents
订阅以下事件：

节点启动失败
节点异常退出
传感器断开
定位丢失
路径规划失败
底盘控制失败
参数加载失败
地图加载失败
任务完成
5.3 建图接口
StartMapping
参数建议：

task_name
map_name
slam_mode                 2d_lidar/3d_lidar/visual
resolution                 地图分辨率
frame_id                   通常为 map
base_frame_id
odom_frame_id
sensor_profile
mapping_config_profile
initial_pose               可选
use_sim_time
返回：

task_id
mapping_state
PauseMapping
参数：

task_id
ResumeMapping
参数：

task_id
GetMappingStatus
参数：

task_id
返回：

mapping_state
elapsed_time
map_bounds
estimated_coverage
loop_closure_count
warning_list
StopMapping
参数：

task_id
save_before_stop
map_name
建议区分“停止建图”和“保存地图”。停止建图不应隐含保存，以免接口语义不清。

5.4 地图管理接口
地图不应只支持设置一个文件路径，还需要完整的生命周期管理。

SaveMap
参数建议：

task_id
map_name
storage_id                 推荐使用逻辑存储位置，不直接开放任意系统路径
relative_path
format                     yaml_pgm/yaml_png/其他
resolution
free_threshold
occupied_threshold
map_mode                   trinary/scale/raw
overwrite
description
返回：

map_id
map_name
yaml_path
image_path
created_at
checksum
GetMapStorageConfig
返回：

storage_root
available_space
allowed_formats
path_policy
SetMapStorageConfig
参数：

storage_root
max_storage_size
overwrite_policy
出于安全性考虑，不建议允许上位机任意指定宿主机绝对路径，建议由 B 公司提供受控地图目录。

ListMaps
参数：

page
page_size
keyword
返回每张地图的：

map_id
map_name
format
resolution
width
height
created_at
updated_at
checksum
description
GetMapInfo
参数：

map_id
LoadMap
参数：

map_id
localization_config_profile
GetCurrentMap
返回当前已经加载的地图信息。

DeleteMap
参数：

map_id
force
RenameMap
参数：

map_id
new_name
ImportMap
参数：

map_package
map_name
format
overwrite
ExportMap
参数：

map_id
export_format
ValidateMap
验证地图文件是否完整，检查 YAML、图片文件、分辨率和原点等参数是否一致。

参数：

map_id
5.5 定位接口
“加载地图”和“机器人已经成功定位”是两件不同的事情，因此定位接口不能省略。

StartLocalization
参数：

map_id
localization_profile
initial_pose               可选
StopLocalization
参数：

force
SetInitialPose
参数：

x
y
yaw
frame_id
covariance
timestamp
GetRobotPose
返回：

x
y
yaw
frame_id
timestamp
pose_covariance
GetLocalizationStatus
返回：

state                      uninitialized/localizing/localized/lost
confidence
pose_covariance
map_id
warning_list
Relocalize
参数：

mode                       local/global
initial_pose               可选
timeout_ms
5.6 导航接口
StartNavigation
用于启动导航所需的节点和配置。

参数：

map_id
navigation_profile
planner_profile
controller_profile
behavior_tree_profile
localization_profile
返回：

service_state
loaded_map_id
StopNavigation
参数：

force
timeout_ms
SendNavigationGoal
这是实际发送单个导航目标点的接口。

参数：

goal_id
x
y
yaw
frame_id
position_tolerance
yaw_tolerance
max_duration
behavior_tree
allow_replanning
返回：

task_id
goal_state
SendWaypointMission
多目标点导航。

参数：

mission_id
waypoints[]:
  - waypoint_id
  - x
  - y
  - yaw
  - stay_time
  - action
loop_count
failure_policy
GetNavigationStatus
参数：

task_id
返回：

goal_state
current_pose
remaining_distance
estimated_remaining_time
current_waypoint
failure_reason
PauseNavigation
参数：

task_id
stop_behavior
ResumeNavigation
参数：

task_id
CancelNavigation
参数：

task_id
cancel_mode                safe/immediate
GetGlobalPath
参数：

start_pose                 可选，省略时使用机器人当前位置
goal_pose
planner_id
返回：

path
path_length
planning_time
ClearCostmap
参数：

scope                      global/local/all
ExecuteRecovery
参数：

recovery_type              spin/back_up/wait/relocalize/custom
timeout_ms
5.7 遥控接口
“启动键盘控制节点”只适合开发调试，不适合作为正式二次开发接口。

正式接口应当控制机器人的速度，而不是把键盘按键传给 B 公司的终端程序。

AcquireMotionControl
获取运动控制权，避免导航和遥控同时向底盘发送速度指令。

参数：

controller_id
control_mode               navigation/teleop/maintenance
lease_timeout_ms
priority
返回：

control_token
expires_at
ReleaseMotionControl
参数：

control_token
StartTeleoperation
参数：

linear_speed_limit
angular_speed_limit
acceleration_limit
command_timeout_ms
deadman_switch_required
SendVelocityCommand
参数：

control_token
linear_x
linear_y                   差速底盘通常为 0
angular_z
duration_ms
timestamp
StopMotion
参数：

control_token
stop_mode                  normal/emergency
StopTeleoperation
参数：

control_token
兼容性接口：StartKeyboardTeleopNode
如果确实需要保留键盘节点，可以提供：

terminal_mode
linear_speed
angular_speed
speed_step
turn_step
但该接口应被定义为调试接口，而不是正式控制接口。

5.8 参数与调参接口
“PID 调参”范围过窄。

导航系统一般还需要调整：

底盘速度控制器参数；
最大线速度和角速度；
加速度和减速度；
局部规划器；
全局规划器；
障碍物膨胀半径；
机器人 footprint；
激光雷达过滤；
AMCL 定位参数；
SLAM 参数；
行为树；
到点容差；
恢复行为；
传感器超时；
TF 容差。
并不是所有导航算法都使用传统 PID，因此接口不应命名为单一的 SetPID。

ListParameters
参数：

component                  controller/planner/localization/slam/costmap
GetParameter
参数：

component
parameter_name
SetParameter
参数：

component
parameter_name
value
value_type
apply_immediately
SetParameters
批量设置：

component
parameters[]
atomic                     是否要求全部成功或全部回滚
ValidateParameters
在应用前检查参数范围和相互约束。

SaveParameterProfile
参数：

profile_name
components[]
description
LoadParameterProfile
参数：

profile_name
restart_if_required
ListParameterProfiles
查询已有参数模板。

PID 专用接口
如果底盘控制器确实使用 PID，可额外提供：

GetPIDParameters
controller_id
axis                       linear/angular/left_wheel/right_wheel
SetPIDParameters
controller_id
axis
kp
ki
kd
integral_limit
output_limit
derivative_filter
ApplyPIDParameters
controller_id
persist
restart_controller
5.9 机器人硬件适配接口
这是决定 B 公司导航模块能否在气体调控机器人上运行的关键部分。

GetRobotModel
返回：

drive_type                 differential/ackermann/omnidirectional
wheel_base
wheel_radius
track_width
max_linear_speed
max_angular_speed
footprint
base_frame_id
odom_frame_id
SetRobotKinematics
参数：

drive_type
wheel_base
wheel_radius
track_width
velocity_limits
acceleration_limits
SetRobotFootprint
参数：

footprint_points[]
padding
GetSensorConfiguration
返回：

lidar_topic
scan_frame
odom_topic
imu_topic
pointcloud_topic
cmd_vel_topic
sensor_frequency
SetSensorConfiguration
参数：

lidar_topic
scan_frame
odom_topic
imu_topic
cmd_vel_topic
qos_profile
timeout_ms
GetTFConfiguration
返回要求的坐标系关系：

map
odom
base_link
base_footprint
laser
imu_link
ValidateRobotIntegration
自动检查：

雷达是否有数据
里程计是否有数据
TF是否连通
时间戳是否正常
速度接口是否可用
坐标方向是否符合约定
传感器频率是否满足要求
这个接口对闭源交付非常有价值，因为公司 A 无法进入内部排查问题。

5.10 安全控制接口
导航和遥控涉及机器人运动，以下接口不可缺少。

EmergencyStop
参数：

reason
source
ResetEmergencyStop
参数：

operator_id
confirmation_code
SetSpeedLimit
参数：

max_linear_speed
max_angular_speed
valid_duration
reason
SetSafetyZone
参数：

zone_profile
front_distance
rear_distance
side_distance
GetSafetyState
返回：

emergency_stop_active
collision_stop_active
obstacle_detected
speed_limited
control_owner
safety_error_list
5.11 诊断、日志和运维接口
闭源交付意味着公司 A 无法通过源代码排查故障，因此诊断接口比普通开源项目更加重要。

建议提供：

GetDiagnostics
包括：

节点运行状态
CPU和内存占用
传感器频率
TF状态
定位置信度
规划器状态
控制器状态
最近错误
GetRecentErrors
参数：

start_time
end_time
error_level
component
SetLogLevel
参数：

component
log_level
ExportDiagnosticPackage
导出：

日志
参数快照
组件版本
节点状态
地图信息
最近任务记录
GetRuntimeMetrics
返回：

planning_latency
control_frequency
localization_frequency
sensor_delay
navigation_success_rate
recovery_count
6. B 公司需要提供的交付内容
如果双方决定推进复用，建议通过补充合同明确以下交付物。

6.1 软件交付
独立的导航 Docker 镜像；
镜像版本号和不可变 Digest；
支持的 CPU 架构，例如 x86_64、aarch64；
支持的国产操作系统版本；
ROS 版本和 DDS 实现；
启动、停止和升级脚本；
配置文件模板；
健康检查机制；
日志和诊断工具；
离线部署方式；
授权机制及授权失效处理方式。
6.2 接口交付
完整 API 文档；
ROS 2 的 .msg/.srv/.action 接口包，或 OpenAPI/gRPC Schema；
状态机说明；
错误码说明；
调用时序说明；
超时和重试机制；
接口版本兼容策略；
示例程序；
接口测试工具；
最小可运行 Demo。
6.3 硬件适配资料
底盘接口要求；
cmd_vel、里程计和 IMU 数据要求；
激光雷达型号和数据格式要求；
TF 坐标系要求；
时间同步要求；
机器人 footprint 和运动学参数要求；
串口、CAN、网口和设备权限要求；
Docker 网络模式；
共享内存要求；
GPU、驱动和内核依赖；
实时性要求。
6.4 运维与知识产权
第三方软件许可证清单；
SBOM 软件物料清单；
安全漏洞升级责任；
缺陷修复响应时间；
接口升级兼容期；
镜像维护年限；
授权是否允许部署到气体调控机器人；
是否允许复制到多台设备；
是否按机器人数量收费；
B 公司停止维护时的处置方案；
必要时约定源代码托管或退出机制。
特别需要注意：

原合同允许公司 A 使用“电器盘柜搬运机器人完整软件”，不一定代表允许把其中的导航能力拆出来部署到另一种机器人或其他项目中。

这不仅是技术问题，也涉及软件授权范围和知识产权，需要商务、法务同步确认。

7. 为什么即使接口齐全，也不一定能直接使用
7.1 两台机器人的底盘可能不同
例如：

差速底盘与阿克曼底盘不同；
轮距和轮径不同；
最大速度不同；
速度指令单位或正负方向不同；
底盘响应延迟不同；
是否支持横向移动不同。
导航控制参数必须针对气体调控机器人重新适配。

7.2 传感器可能不同
例如：

激光雷达型号不同；
扫描频率不同；
扫描角度不同；
安装高度和方向不同；
是否有 IMU 不同；
里程计精度不同；
时间戳来源不同。
这些差异会直接影响建图、定位和避障。

7.3 ROS 接口和坐标系可能不同
需要确认：

Topic 名称；
Message 类型；
ROS Domain ID；
Namespace；
QoS；
map → odom → base_link → laser 的 TF；
时间同步；
服务和 Action 名称。
只要其中一项不一致，导航程序就可能无法工作。

7.4 业务逻辑可能强耦合
B 公司导航程序可能默认依赖：

搬运任务状态；
机械臂是否收回；
盘柜是否夹紧；
安全门状态；
特定 PLC 信号；
电池状态；
特定数据库记录；
原项目账号和权限系统。
这种情况下，即使暴露了 StartNavigation，内部状态机也可能因为缺少搬运机器人状态而拒绝执行。

7.5 闭源导致公司 A 缺少问题处置能力
如果出现：

节点异常退出；
导航偶发失败；
CPU 占用过高；
新雷达不兼容；
操作系统升级后无法运行；
Docker 镜像存在安全漏洞；
公司 A 无法自行修改，只能依赖 B 公司处理。

因此，复用 B 公司模块虽然可能减少前期编码，但会引入长期的供应商依赖和维护成本。

8. 如何向老板解释
建议不要直接说“这个方案做不了”，而是这样表达：

B 公司的导航能力可以评估复用，但这不是简单开放几个 API 的问题。API 只能解决我们如何发送命令，不能解决软件是否能脱离原来的电器盘柜机器人运行。

如果 B 公司的导航代码已经独立模块化，并且采用标准 ROS 接口，那么复用成本可能较低；如果它与底盘驱动、传感器、机械臂和搬运业务状态机耦合，B 公司就需要先做模块拆分、硬件适配和独立容器化。这实际上属于一项新的二次开发工作，需要重新明确费用、交付内容、验收标准和维护责任。

因此不建议直接判断“复用一定能节省工作量”。应先让 B 公司证明其导航模块能够独立部署，并完成一次在气体调控机器人硬件上的最小验证，再比较外购复用和自主开发的总成本。

可以再用一个非技术类比：

API 相当于给机器增加几个操作按钮。现在的问题不是有没有按钮，而是按钮后面的整套机器能不能从电器盘柜机器人中拆下来，装到气体调控机器人上，并且与新的底盘、雷达和控制系统正确连接。

如果内部本来就是独立模块，加按钮比较容易；如果内部是焊接在整机上的，就必须重新拆分和改造。

9. 建议的推进方式
第一阶段：只做能力调查，不立即承诺复用
要求 B 公司书面回答：

导航模块是否为独立 ROS 2 Package；
是否可以脱离搬运业务模块启动；
是否可以作为独立 Docker 镜像交付；
是否使用标准 ROS 2 接口；
支持哪些底盘类型；
支持哪些激光雷达和 IMU；
是否允许修改 Topic、TF 和 Namespace；
是否支持动态调整导航参数；
是否存在硬件加密或设备绑定；
当前合同是否允许在另一个机器人项目中使用；
能否提供接口包、文档和 Demo；
是否愿意承担气体调控机器人的适配工作；
适配后的软件由谁维护；
如何收费以及授权设备数量。
第二阶段：做付费或约定范围的可行性验证
不要先要求 B 公司完成所有接口，而是先验证一条最小链路：

气体调控机器人雷达、里程计和底盘
        ↓
B公司独立导航镜像
        ↓
成功建图
        ↓
成功保存并加载地图
        ↓
成功定位
        ↓
发送一个目标点
        ↓
机器人安全到达目标点
最小验证至少应证明：

镜像能在目标国产工控机上运行；
能接收气体调控机器人的雷达和里程计；
能向气体调控机器人底盘发送速度指令；
TF 正确；
可以完成建图和定位；
可以完成单目标点导航；
紧急停止有效；
B 公司原有搬运业务模块不再是必要依赖。
第三阶段：验证成功后再补充正式合同
合同中应明确：

是交付“几个接口”，还是交付“可独立运行的通用导航组件”；
接口清单；
独立镜像；
硬件适配范围；
验收场景；
软件授权范围；
缺陷修复责任；
后续升级费用；
B 公司停止维护时的处置方案。
10. 建议的决策标准
适合复用
满足以下大部分条件时，复用价值较高：

B 公司导航模块已经独立；
使用标准 ROS 2 Topic、Service 和 Action；
底盘和传感器接口可配置；
没有依赖电器盘柜业务状态机；
B 公司愿意提供独立镜像和接口包；
授权允许用于气体调控机器人；
价格明显低于自主开发和长期维护成本；
B 公司能够持续提供技术支持。
不适合直接复用
出现以下情况时，应谨慎甚至放弃：

只能交付原项目完整镜像；
无法关闭搬运业务相关依赖；
Topic、TF、传感器型号被写死；
镜像绑定特定设备或授权；
不提供诊断接口和日志；
不提供接口版本兼容承诺；
不允许在其他机器人上部署；
适配费用接近重新开发；
关键故障必须完全依赖 B 公司处理。
11. 最终建议
建议公司 A 将此事定义为：

评估采购 B 公司“可独立部署的通用导航组件及二次集成接口包”，而不是要求 B 公司简单开放几个 API。

技术上可以争取复用，但必须先验证三个前提：

可拆分：导航功能能够脱离电器盘柜业务独立运行；
可适配：能够接入气体调控机器人的底盘、雷达、里程计和 TF；
可维护：公司 A 能够通过文档、接口、日志和诊断工具完成日常使用，重大问题有明确的 B 公司维护责任。
在 B 公司完成最小可行性验证之前，不能把“复用后能够节省大量开发工作”作为确定结论。更准确的表述应当是：

复用可能减少导航算法和基础功能的重复开发，但会增加模块拆分、硬件适配、闭源集成、商务授权以及长期供应商依赖成本。是否划算，必须通过技术调查、最小验证和正式报价后才能判断。

/home/nllg/my_project/sx_GasControl_qt/src/gas_control_bringup/doc/Reference file

整理成两份md输出到这个路径，一份接口清单 一份论证

当前处于 Ask mode，我不能创建或修改工作区文件。

请切换到 Agent mode 后重新发送该请求。我会在以下目录直接生成两份 Markdown 文档：

/home/nllg/my_project/sx_GasControl_qt/src/gas_control_bringup/doc/Reference file/

拟定文件名：

B公司导航能力复用-接口清单.md
系统生命周期接口
建图与地图管理接口
定位与导航接口
遥控及控制权接口