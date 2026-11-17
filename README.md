# Stage 1：UAV / UGV LIO-SAM 独立验收

需要先绕开漂移、查看实际 Gazebo 扫描的融合效果时，使用
[独立算法实验入口](docs/STAGE3_GAZEBO_ALGORITHM.md)：
`roslaunch start_up stage3_gazebo_algorithm.launch`。它用真值位姿累计子图，EHPR/GICP 自主检索配准，
不启动 LIO、不代表漂移修复；正式 LIO 入口保持。

最新扫描约束与 32 线对照见 [继续排查报告](../../results/stability/20261009_150010_mapping_replay/REPORT.md)。
32 线实际悬停位置 RMSE 改善到 0.141 m，但最长连续退化 4.114 秒，悬停整项及融合仍未通过。
新增独立 mapping 阶段回放工具和运行目录内的 32 线候选生成；正式默认配置保持。

2026-10-09 UAV 漂移处理计划与可执行指令见 [uav_drift_plan.md](docs/uav_drift_plan.md)。
已准备可选 `uav_sensor_profile:=wide`，仅将向下视场从 -15° 扩大到 -45°；默认 stable 和原安装外参保持。
stable/wide 实际仿真起飞、61 秒悬停和降落对照已完成；两轮 UAV 精度均未通过，wide 不升为默认配置。
记录与诊断图见 [实际运行报告](../../results/stability/20261009_112158_fov_trials/REPORT.md)。运动和融合验收尚未完成。

继续排查发现：wide 在起飞稳定后首次初始化 LIO，单次 hover 数值检查通过（RMSE 0.17775 m），
但仍有最大 0.325 m 误差，且未验证起飞段、运动或融合。
[后续初始化诊断与复现命令](../../results/stability/20261009_141204_takeoff_diagnosis/REPORT.md)。正式默认入口未切换到该诊断流程。

本包复用 `/home/yang/wzw/ct_ws` 中的 LIO-SAM、UAV bringup 和机器人模型。
不包含 SubmapBuilder、EHPR、GICP 或跨机器人地图融合。仿真验收通过与实机验收通过分别记录。
旧的 `ugv_uav_lio_sam.launch` 保留原有功能；验收使用新的独立入口。

## 当前开发阶段

2026-10-08：依据已有自动检查以及用户反馈“人工运行无明显问题、未录 bag”，
Stage 1 已记录为 **accepted_for_stage2**（工程放行）。记录见
`results/stage1/development_release.json`；未量化的运动/悬停精度仍保留，严格验收矩阵未伪造为通过。

Stage 2 已接入官方 EHPR 子图代码，双机启动：

```bash
source /home/yang/wzw/multi_robot_ws/src/start_up/scripts/setup_stage1.bash
roslaunch start_up stage2_sim.launch
```

Stage 2 默认 both，只启动一个 Gazebo，关闭 LIO 内部回环。详细接口、PCD 保存与尾段导出见
[ehpr_bridge/README.md](../ehpr_bridge/README.md)。停止已有仿真后再启动此入口。
Stage 2 的实际双机 PCD、RViz 截图及自动检查汇总见
[results/stage2/REPORT.md](../../results/stage2/REPORT.md)。

## 构建

```bash
source /home/yang/wzw/ct_ws/devel/setup.bash
cd /home/yang/wzw/multi_robot_ws
catkin_make
source devel/setup.bash
```

每个运行/控制/录制终端再执行以下命令，恢复 catkin setup 未保留的 PX4 和 Gazebo 搜索路径：

```bash
source /home/yang/wzw/multi_robot_ws/src/start_up/scripts/setup_stage1.bash
```

换机器时可通过 `STAGE1_WORKSPACE`、`STAGE1_LIO_WORKSPACE`、`STAGE1_PX4_DIR` 覆盖三个目录。

LIO 的 TF 修复位于外部工作区 `LIO-SAM/src/imuPreintegration.cpp`。若重新下载或修改 LIO，需要构建：

```bash
cd /home/yang/wzw/ct_ws
catkin build lio_sam --no-deps -j2 -p1
```

该修复在传感器 TF 未出现时重试，并禁止使用未初始化的变换；IMU 队列被清空时直接等待下一条消息。

## 启动与手动驾驶

每轮测试停止上一轮 launch，重新初始化地图。三种入口均只启动一个 Gazebo：

```bash
roslaunch start_up stage1_sim.launch robot:=ugv
roslaunch start_up stage1_sim.launch robot:=uav
roslaunch start_up stage1_sim.launch robot:=both
```

一次只执行其中一条。可追加 `gui:=false rviz:=false` 做无人值守的静止检查。
此入口不启动导航、自主探索、Octomap、Navsat 或自动 UAV 航点任务。

先查看 `rostopic echo -n 1 /stage1/health`，等到 JSON 中 `status` 为 `ready`。
检查工具会主动订阅注册点云，避免 LIO 的按订阅者发布逻辑导致点云没有输出。

UGV 在另一终端启动键盘输入：

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py \
  cmd_vel:=/stage1/ugv/cmd_vel_raw _speed:=0.3 _turn:=0.3 \
  _repeat_rate:=10.0 _key_timeout:=0.3
```

launch 中的保护节点将输入限幅为 `0.3 m/s`、`0.3 rad/s`，输入中断超过 0.5 秒输出零速度。
即使键盘工具提高速度，实际发送的命令仍有上限。默认不直接向 `/cmd_vel` 启动其他控制器。

UAV 在交互终端运行：

```bash
rosrun start_up stage1_uav_teleop.py
```

- `t`：显式进入 OFFBOARD、解锁、以 0.3 m/s 爬升到初始位置上方 1.5 米。
- `i/k`：本地 ENU X 正/负；`j/l`：Y 正/负；`w/s`：Z 正/负；`a/d`：偏航正/负。
- 按住/重复移动键，松开后保持目标位置；空格保持当前位置。
- `g`：请求 AUTO.LAND；Ctrl-C 请求降落并退出。模式请求失败会打印原因，不自动重新解锁。

位置目标由单独的发布线程以 20 Hz 持续发送，终端输入不会阻塞 OFFBOARD 流。
这两个控制入口检查 Gazebo 和仿真模式，不能用于真实车辆或飞行器。
终端退出后的 PX4 模式仍应在仿真界面中确认。

## 坐标与数据接口

| 机器人 | 里程计 | 注册特征云 | 注册完整云 | 参考系 |
|---|---|---|---|---|
| UGV | `/lio_sam/mapping/odometry` | `/lio_sam/mapping/cloud_registered` | `/lio_sam/mapping/cloud_registered_raw` | `odom` |
| UAV | `/uav/lio_sam/mapping/odometry` | `/uav/lio_sam/mapping/cloud_registered` | `/uav/lio_sam/mapping/cloud_registered_raw` | `uav_lio_odom` |

里程计位姿对应雷达原点，`odom_mapping` / `uav_odom_mapping` 是当前代码使用的 child 名称。
注册云已转换到表中的参考系；**不得再次乘里程计位姿**。工具使用同时间戳的 deskewed cloud
验证 `P_registered = T_reference_lidar × P_deskewed`。

UGV TF：`map → odom → base_link → velodyne`，同时 `base_link → base_footprint`、`base_link → imu`。
UGV 雷达相对 base_link 为 `[0.20, 0, 0.1677]`，相对模型原点 base_footprint 为 `[0.20, 0, 0.3477]`。
没有 `map → odom_combined`，也不同时发布 URDF 的 `base_footprint → base_link`。

当前世界是平地，UGV 仿真配置额外使用 `z_tollerance=0.10 m`、
`rotation_tollerance=0.05 rad` 和 `imuRPYWeight=0.1`，限制接触振动叠加瞬时退化造成的
垂直漂移和错误倾斜。高度限制相对本次 LIO 初始雷达位置。这是平地 UGV 假设，
不适用于坡道/三维运动；实机模板保留未约束配置，需按真实场景验证。

UAV TF：`uav_lio_map → uav_lio_odom → uav/base_link → uav/laser_3d`。
原始 UAV 传感器消息仍使用 `laser_3d` / `imu_link_stereo`，配置明确记录其对应的带前缀 TF 名称。
两端的 `viz_world` 固定连接只用于 RViz 摆放，不是跨机器人定位结果，也不用于误差评估。

Gazebo 点云的 `time` 全零表示当前插件输出瞬时扫描。报告会注明这个限制；不把它当作真实转动雷达的去畸变验证。

双雷达运行时 GPU 插件可能重复发布同一时间戳。原始话题仍完整录制，仿真输入节点
只将首帧转发到 `/stage1/ugv/points`、`/stage1/uav/points`，不会人为改写时间戳。
丢弃计数发布到 `/stage1/<机器人>/lidar_input_stats` 并随 bag 保存。
原始仿真数据的重复时间戳列为警告，倒退时间戳及 LIO 输出中的重复仍属于失败。

## 记录与检查报告

稳定后开始静止/悬停记录；不要把起飞过程算作悬停：

```bash
rosrun start_up stage1_record.py --scenario static --duration 80
```

时长使用 ROS 时间；默认前 15 秒不计入验收，因此留下约 65 秒有效数据。
启动仿真 master 前须先 source 当前工作区。记录工具默认匹配正在运行的机器人选择。

运动/高度/双机交替测试可手动结束：

```bash
rosrun start_up stage1_record.py --scenario motion --duration 0 --warmup 5
rosrun start_up stage1_record.py --scenario height --duration 0 --warmup 5
rosrun start_up stage1_record.py --scenario coexist --duration 0 --warmup 5
```

一条命令对应一轮记录，Ctrl-C 结束后会关闭 bag 并生成报告。
每轮 `results/stage1/<上海时间>_<机器人>_<场景>/` 保存：

- `data.bag`：传感器、注册云、deskewed cloud、位姿、路径、退化诊断、TF、时钟、真值、健康状态和 ROS 日志消息。
- `parameters.yaml`、`contract.yaml`、`session.json`：冻结的实际参数、接口和记录条件。
- `tf_snapshot.json`、`ros_logs/`：TF 发布者/父子关系和本轮 ROS 日志。
- `report.json`、`report.md`：逐话题统计、误差、退化、失败项和人工检查状态。

Gazebo ModelStates 没有消息头，真值以收到消息时的 `/clock` 时间记录，最多 50 Hz；
这是当前仿真真值时间对齐的限制。先把模型位姿换算到雷达原点，再与 LIO 比较。
仅根据第一对匹配位姿作 SE(3) 对齐；不作尺度校正或整条轨迹最佳拟合。
默认匹配时间间隔最大 50 ms，匹配覆盖不足 80% 不能通过。

报告不会根据文件存在就宣称通过。输出中断、无效四元数、TF 冲突、预积分重置、
注册云变换错误、测试时长或运动不足都会列为失败。

## 人工验收矩阵

| 模式 | 必做场景 | 操作 |
|---|---|---|
| ugv | static、motion | 静止 60 秒；低速累计行驶至少 20 米，完成至少四次 90° 转向 |
| uav | static、motion、height | 稳定悬停 60 秒；低速平移累计至少 20 米并转向；高度变化 0.5 米后回到起始悬停高度 |
| both | static、coexist | 两端保持 60 秒；UGV 运动/UAV 保持至少 10 秒，再交换角色至少 10 秒 |

路线由操作者在当前场景中选择无碰撞区域，可反复沿短路段往返累计距离。
运动报告按真值累计距离和总转角计算；四次 90° 转向对应累计至少约 360°。
height 记录应在起飞稳定后开始，以免将起飞误认为高度变化测试。
当前平地仿真的 UAV static/height 记录起始雷达高度至少为 1.0 米，避免把地面静止误算为悬停验收。
coexist 中运动速度需大于 0.05 m/s，保持端速度小于 0.03 m/s。

初始门槛见 `config/stage1_contract.yaml`：

- UGV 静止 60 秒位移误差不超过 0.10 米；UAV 悬停位置 RMSE 不超过 0.20 米。
- 运动位置 RMSE：UGV ≤0.20 米、UAV ≤0.30 米；姿态 RMSE ≤5°。
- 必需输出不得中断超过 1 秒；连续退化超过 2 秒标为待排查。
- 人工检查墙面/地面没有明显弯曲、双层墙面或持续重影；观察注册云及地图，不用点云显示密度代替质量判断。

数值通过时默认仍标为 `numeric_pass_map_review_pending`。查看 RViz 或回放 bag 完成人工检查后重算：

```bash
rosrun start_up stage1_evaluate.py /完整路径/data.bag \
  --contract /完整路径/contract.yaml --robot ugv --scenario static \
  --warmup 15 --map-review pass
```

按 `session.json` 填写该轮的 robot、scenario、warmup。失败的人工检查用 `--map-review fail`。
离线分析直接读取 bag，不需要启动 ROS master；仅查看 bag 可按需用 `rosbag play --clock`，
同时设置 `/use_sim_time=true`，避免同一 master 上的实时仿真话题混入。

汇总全部验收记录：

```bash
rosrun start_up stage1_suite.py /home/yang/wzw/multi_robot_ws/results/stage1
```

只有矩阵中的七个场景均有 `passed` 记录才标记 `stage1_sim_accepted`。
`smoke` 记录只证明链路启动和数据检查，永远不能替代手动运动和地图验收。

## 实机准备（设备尚未确定）

`stage1_ugv_real.template.yaml`、`stage1_uav_real.template.yaml`、`stage1_real_contract.template.yaml`
默认不允许启动。复制到自己的实机配置文件中，填完标定后才设置对应的 `configured` / `hardware_configured`。

```bash
roslaunch start_up stage1_real.launch robot:=both \
  ugv_config:=/标定配置/ugv.yaml uav_config:=/标定配置/uav.yaml \
  contract:=/标定配置/contract.yaml
```

入口只运行 LIO、测量后的传感器 TF 和检查工具；`use_sim_time=false`。
不启动硬件驱动、Gazebo、PX4、解锁或运动控制。先独立启动设备驱动。

实机标定顺序：

1. 核对话题、消息帧、点云 XYZ/intensity/ring/time、线数、水平分辨率和逐点时间单位。
2. 核对加速度、角速度、IMU 姿态四元数与 ENU 约定。当前 LIO 需要有效的姿态输入；不能用全零四元数蒙混通过。
3. 测量雷达/IMU 相对 base 的刚体变换。模板的 `lidar_to_base_xyz/rpy` 表示 **T_base_lidar**，`imu_to_base_xyz/rpy` 表示 **T_base_imu**。
4. 按本地 LIO 的 imuConverter 约定填 `extrinsicRot`、`extrinsicRPY`；填雷达坐标中 IMU 原点位置 `extrinsicTrans`。噪声参数根据设备确定。
5. 验证时间同步，将 Velodyne 格式的逐点时间转换为 FLOAT32 秒。非该格式的雷达需单独调整适配器和检查器，不能直接套用仿真配置。
6. 静止采集 → 低速运动 → rosbag 重复回放。真实转动雷达的逐点时间全零属于失败。

实机未提供独立真值时，仅输出数据质量检查 `hardware_data_checked`，不宣称达到仿真定位精度门槛。
实际设备型号、外参、同步方式和实机定位验收仍待确定。

## 开发检查

```bash
PYTHONPATH=src/start_up/python python3 -m unittest discover -s src/start_up/tests -v
```

测试覆盖初始坐标对齐保留后续漂移、姿态插值、真值时间不匹配、点云行填充/字节序、
不完整实机标定拒绝启动，以及 TF 多父坐标系和缺失数据无法通过验收。
同时检查非有限四元数、地面 UAV 数据不能替代悬停，以及重复扫描过滤不改写时间戳。

## Stage 3–4 空地相对定位入口

Stage 2 已依据自动检查和用户实测反馈释放为 `accepted_for_stage3`；Stage 1 未量化的运动项目继续保留。
正式启动双方 LIO、官方子图和 EHPR/GICP 匹配，使用一个 Gazebo，关闭 LIO 内部回环：

```bash
source src/start_up/scripts/setup_stage1.bash
roslaunch start_up stage3_sim.launch
```

RViz 启动时显示双方实时观察云：**UGV observed cloud**（绿色）和 **UAV observed cloud**（紫色）。
`/fused_submap` 在第一次成功匹配前没有消息；**Fused matched submaps** 显示项已启用。
在线 Fixed Frame 为 `viz_world`，仅用于共同显示，算法不使用该显示锚点。

另开已 source 的交互终端，运行 `rosrun start_up stage1_uav_teleop.py`：
按 `t` 起飞，用 `i/k`、`j/l`、`w/s` 调整本地 ENU X/Y/Z，空格保持，`g` 降落。
手动飞入 UGV 已观察区域且公共结构足够时，通过 EHPR/GICP 的子图自动融合并发布 `/fused_submap`。
关闭两项 observed cloud 可单独查看融合结果。融合云位于 UGV `odom`，
绿色表示仅 UGV 点参与的体素，紫色表示仅 UAV，黄色表示双方。

为避免静止 UGV 不达到 20 米而无法建库，Stage 3 使用专用定时预览配置：
预热 15 秒同步扫描、至少 20 帧后，UGV 每 10 秒提供 partial 子图；
UAV 完成相同预热和帧数检查后，每 3 秒提供 partial 查询；不要求先移动。
这些是仿真时间；预览不重置原有 20 米分段，不需要手动调用 flush。
匹配不通过时查看 `/stage3/status` 的拒绝原因；后续成功结果替换上次融合子图对。
旧 `/stage3/fused_submap` 仍发布相同结果，原 matched/aligned 显示项默认关闭。
`/stage3/match` 给出 **匹配关键帧**之间的六自由度变换，尚未生成完整历史合并地图。
操作、离线回放、默认质量门槛和已有检查见 [ehpr_bridge/README.md](../ehpr_bridge/README.md)。

## 双机稳定性修复与手动融合验收（2026-10-08）

已加入扫描状态保护和仿真对照配置；完整悬停、驾驶及融合验收需要新的手动运行记录。
现有失败地图只用于诊断，请结束旧运行后重新启动，关闭旧键盘控制终端，避免重复控制发布者。

每个终端先执行：

```bash
cd /home/yang/wzw/multi_robot_ws
source src/start_up/scripts/setup_stage1.bash
```

启动正式双机入口（默认启用完整录制，不会自动起飞）：

```bash
roslaunch start_up stage3_sim.launch sim_profile:=stable
```

`stable`：包内世界步长 0.001 s / 1000 Hz，UAV 雷达 16×1800 / 10 Hz；安装位置、外参和视场不变。
`legacy`：原世界步长 0.004 s、原 UAV 雷达 16×512 / 15 Hz。Stage 1/2 原入口仍默认 legacy；Stage 3 默认 stable。
可分别对照：`sim_profile:=stable uav_sensor_profile:=legacy`（只改物理步长），
`sim_profile:=legacy uav_sensor_profile:=stable`（只改雷达）。不要混用 512 点云和 1800 的配置契约。

UAV imuRPYWeight 默认仍为 0.01；0.1 候选通过
`uav_config:=$(rospack find start_up)/config/stage1_uav_stable_rpy01.yaml` 指定。
仅在完整悬停/运动数据改善独立真值误差后采用，不因静止地面检查通过就更换。
Gazebo 逐点 time 仍为零，本测试不等于实机旋转雷达去畸变验证。

等待两端观察云正常、录制器就绪：

```bash
rostopic echo -n 1 /stage3/recording
rostopic echo -n 1 /stage1/health
```

使用原键盘入口手动驾驶、起飞。以下命令只标记阶段，不控制机器人；开始每个阶段时运行对应命令：

```bash
# UAV 起飞到约 1.5 m，稳定后开始；UGV 静止，记录至少 61 s
rosrun start_up stage3_phase.py hover
# UAV 保持悬停，UGV 靠近公共区域，完成至少 20 m 和四次转向
rosrun start_up stage3_phase.py ugv_motion
# UGV 静止，UAV 完成至少 20 m 和四次转向
rosrun start_up stage3_phase.py uav_motion
# UAV 上下变化 0.5 m，再回到原高度
rosrun start_up stage3_phase.py height
rosrun start_up stage3_phase.py end
```

保持低速 0.3 m/s、0.3 rad/s；两端运动阶段应包含一端运动、另一端保持的公共区域测试。
操作完成后正常 Ctrl-C 关闭 launch，让 bag 收尾。
`/stage3/recording` 给出本次 `results/stability/<上海时间>/` 目录。随后执行：

```bash
rosrun start_up stage3_evaluate.py /绝对路径/本次录制目录
# 实际检查公共结构重合、无明显重影后，保存 RViz 截图，再填写人工结论
rosrun start_up stage3_evaluate.py /绝对路径/本次录制目录 --map-review pass --screenshot /绝对路径/截图.png
```

报告对所有阶段沿用整次运行最初的同一刚体对齐，不按阶段重新拟合。
阶段标记缺失、数据不足、持续退化、无通过匹配或真值误差超标都会失败；无人工地图检查/截图只标数值通过。
录制使用 LZ4、512 MB 分包，记录原始/门控雷达、IMU、注册云、诊断、子图、匹配、TF、时钟、真值及控制话题。
不记录巨大历史 map_global；需要观察时启用 RViz 的独立诊断显示项。
剩余磁盘小于 5 GiB 时明确报错停止本次链路，不删除历史记录。
`resources.jsonl` 每秒保存整机负载、可用内存及 ROS/墙钟进度；整机负载包含其他程序，不能当作单节点 CPU 占用。

RViz 默认两端观察云启用，融合显示订阅 `/fused_submap`、RGB8。
预热 15 s 且至少 20 帧后，两端静止也可输出 partial；UGV 10 s、UAV 3 s 间隔。
首次可靠匹配并落盘前融合话题无数据；绿色仅 UGV、紫色仅 UAV、黄色两端贡献。
它是最新匹配子图对，不是完整历史全局融合地图。共同显示锚点 viz_world 不参与匹配。

匹配默认高度邻域 3。`config:=$(rospack find ehpr_bridge)/config/stage3_height_radius1.yaml`
仅用于高度邻域 1 的对照，其他门槛相同；两者均通过保留 3，只有 1 通过完整正负例、手动运动和真值检查时才采用 1。
已畸变的云即使获得高度解，也必须通过 GICP/重叠/残差门槛，不能强制融合。

LIO 修复实际应用于 `/home/yang/wzw/ct_ws/src/LIO-SAM`，补丁及原/新文件 hash 保存在 start_up/patches。
复现到同一源码基线时先执行 `python3 src/start_up/scripts/apply_lio_scan_patch.py`，再在 ct_ws 执行
`catkin build lio_sam -j1 --no-status`，最后在本工作区执行 `catkin_make -j2`。构建与 Gazebo 测试分开进行，避免编译负载干扰频率对照。未知源码修改不会被补丁工具覆盖。

被动对照工具：

```bash
python3 src/start_up/tests/run_stability_probe.py --physics stable --sensor stable --duration 45
```

工具创建独立 ROS/Gazebo 实例，只读取实际两端传感器，不起飞、不驾驶；报告明确为地面静止探测，不计作悬停验收。

### 本轮验证结论与质量联锁

本轮并未通过最终验收：stable 配置改善了仿真输入，但 UAV 在部分场景仍会持续退化、漂移。
同房间地面测试中，UAV 在暖机后约 25 秒采样窗口内位置 RMSE 为 0.70 m；此结果不满足 0.20 m 门槛，且不是飞行验收。
扩大近距离过滤到 0.5 m 的候选没有改善；它不作为默认配置。
同一份原始数据重启 LIO 回放，姿态权重 0.01、0.1 均失败，因此保留默认 0.01。
完整结果见 results/stability/REPORT.md，不能据此宣称已解决 UAV 漂移或完成地图融合。

此外，过滤候选的一次 GICP 成功结果被独立真值检查判定错误（1.46 m / 20.3°）。
因此正式 Stage 3 和 bridge-only launch 默认启用 `require_lio_quality:=true`：
当前扫描及匹配关键帧的诊断必须完成优化、无退化、特征值有限；诊断过期超过 1 秒会拒绝。
暖机后连续退化/未优化达到 2 秒，匹配节点清空数据库、待查询和已显示融合结果，锁定本轮输入。
后续偶然一帧健康不会解除锁定；需正常关闭整个 Stage 3，重新初始化 LIO、子图和匹配节点。
这是因为已有 LIO 历史地图可能受污染，仅清空子图不能修正底层状态。
该联锁使用 LIO 自身诊断，真值仍只进入独立验收工具。
算法离线回放/合成测试可以明确禁用联锁，但不能用这种结果宣称在线输入健康。

当前世界中，(-1,0) 与 (1,0) 之间存在 Wall_6：大致 x=-0.335，y=-6.86～0.39，高 2.5 m。
近距离隔墙测试按真值对齐后的完整云覆盖率仅约 18%，因此应保留拒绝。
这些位置不能作为“靠近就应该融合”的正例；必须观察同一片可见结构并保持健康 LIO。
同房间检查可以用 `uav_x:=-5 uav_y:=-8.5` 与默认 UGV 起点配对，但当前地面数据仍发现 UAV 退化，尚不能作为通过示例。
