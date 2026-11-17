# EHPR ROS 桥接：Stage 2 子图与 Stage 3–4 空地相对定位

## 配准安全更新

`partial` / `partial_contour` 现使用双向真实单帧自由空间验证和独立观测确认：首次需要两组，纠错需要三组。可靠冲突会清空融合显示、保留两端自有地图并自动重新定位。静止重复扫描不计确认票；详见[确认与自动恢复说明](../start_up/docs/STAGE3_ALIGNMENT_SAFETY.md)。此流程替代下文旧记录中的“首个子图立即融合”和“冲突保留旧关系”。

真实错配回归、两种移动场景的隔离仿真及工作区构建结果见 [2026-10-15 验收报告](../../results/stage3_rendezvous/20261014_220720_alignment_safety/REPORT.md)。


直接包含 `third_party/EHPR/include/VoxelMap.hpp`，封装官方
`test/multisession_video.cpp:369-413` 的 `generateSubmap()`。官方仓库固定于
`b4b7137f7e675974e025fc3c8e0ab891d353739a`，下载与头文件 SHA256 见 `third_party/EHPR_SOURCE.json`。
第三方源文件未修改；桥接层持有每机器人独立状态，并在释放时调用官方 `Clear()`。
集成方式和桥接层差异见 [UPSTREAM.md](UPSTREAM.md)。
Stage 2 入口只生成子图；Stage 3–4 入口增加官方 EHPR/HBST/RANSAC 和 PCL GICP。
当前输出匹配子图间的相对位姿及显示云；两种 partial 模式还维护双方完整历史地图，在关系确认后投影融合。全局图优化和协同导航留到后续阶段。

若 LIO 漂移妨碍观察算法效果，可以使用独立的
[Gazebo 真实扫描＋理想位姿前端实验](../start_up/docs/STAGE3_GAZEBO_ALGORITHM.md)：
`roslaunch start_up stage3_gazebo_algorithm.launch`。该入口绕开 LIO，仍让 EHPR/GICP 自己
检索配准，通过后发布 `/fused_submap`；它不代表 LIO 漂移已修复。

“初始扫描不重叠，UAV 地面静止，UGV 手动接近”的入口：
`roslaunch start_up stage3_gazebo_rendezvous.launch`。
[预热含义、路线和沿途点云的保留规则](../start_up/docs/STAGE3_RENDEZVOUS.md)见专门说明；
两种 partial 模式确认后发布完整历史累计融合图；选中子图的对比显示另行保留。
针对 UGV 静止、UAV 低空靠近导致的观测不均衡，新增 `matching_profile:=partial`，
保留 EHPR 检索并增加重力约束的鲁棒精配准、部分重叠验证和自由空间检查。
参数、拒绝原因与回放方法见 [部分重叠配准说明](../start_up/docs/STAGE3_PARTIAL_MATCHING.md)。
会合实验使用 UGV 关键帧附近 8 米局部云检索/配准、0.5 米 GICP 降采样、最多 10 个候选回退，
8 米视图全部失败时，使用 12 米备用视图重新检索和验证；两者使用相同质量门槛。
2026-10-13 起，Gazebo 算法配置默认用最近 5 秒的同步注册扫描构建匹配窗口，
检索和质量检查使用近期云，接受后仍融合完整历史子图。旧离线输入保持兼容。
原质量门槛保留，另要求立体结构点双向覆盖率均达到 60%，拒绝重复地面/墙角假匹配；
配准通过后用完整子图融合，保留沿途观测。
这些参数见 `start_up/config/stage3_gazebo_algorithm_matching.yaml`；正式 LIO 配置仍由 `config/stage3.yaml` 提供。
[本轮修复、41 组会合回放和新 Gazebo 正负例报告](../../results/stage3_rendezvous/20261011_rendezvous_fix/REPORT.md)
及[沿途历史融合对比](../../results/stage3_rendezvous/20261011_rendezvous_fix/recorded_rendezvous_fusion.png)已保存。
2026-10-12 的 8/12 米视图回退优化已通过当前录制、历史录制、负例与在线匹配检查，
见[优化报告](../../results/stage3_rendezvous/20261012_224403_optimization/REPORT.md)。

## 启动

```bash
cd /home/yang/wzw/multi_robot_ws
source src/start_up/scripts/setup_stage1.bash
catkin_make -j2
source src/start_up/scripts/setup_stage1.bash
roslaunch start_up stage2_sim.launch
```

默认 `robot:=both`，只启动一个 Gazebo。启动前停止上一轮仿真。
`robot:=ugv`、`robot:=uav` 仅作排障；正式链路检查采用 both。
可追加 `gui:=false rviz:=false`，或用 `output_root:=/你的目录` 指定存储位置。
UGV 和 UAV 键盘控制方法沿用 `start_up/README.md`。

Stage 2 入口会在启动 LIO 前关闭其内部回环，避免旧注册点与修正后的位姿混合。
`stage1_sim.launch` 默认仍使用其 YAML 中的原始回环配置。
桥接挂接已运行的 LIO 时必须首先用 `loop_closure:=false` 重新启动它；运行中改 ROS 参数
不会改变 LIO 已缓存的配置。仅挂接桥接节点：

```bash
roslaunch ehpr_bridge stage2_submaps.launch robot:=both
```

## 官方行为与坐标

默认累计 3D 行程 **超过 20 米**生成一次子图，达到 **0.2 米**时锁定本段关键帧。
地图是 1 米体素，每体素最多保留 25 个点，同体素内最小间距 0.2 米。
生成后清除当前位置 80 米以外的体素，近处历史点继续保留，所以相邻子图自然重叠。
这不是仅包含最近 20 米路段的窗口。行程计算沿用官方逐帧位移累计，没有额外运动门槛。
官方体素索引和边界行为也保持原样；未替换成单点体素质心滤波。

| 机器人 | 同步的位姿 / 点云 | 参考系 |
|---|---|---|
| UGV | `/lio_sam/mapping/odometry` / `/lio_sam/mapping/cloud_registered_raw` | `odom` |
| UAV | `/uav/lio_sam/mapping/odometry` / `/uav/lio_sam/mapping/cloud_registered_raw` | `uav_lio_odom` |

使用 PointXYZ 和相同扫描时间戳，队列长度 100，未配对数据超过 1 秒 ROS 时间即过期。
注册云已经在参考系里，**不会再乘当前位姿**。导出局部云执行
`P_key = inverse(T_reference_key) * P_registered`。
保存的 PCD 是未经重力对齐的关键帧局部云；`T_gravity_key` 使用官方 Z-Y-X Euler
分解和 pitch*roll 组合，从 LIO 雷达姿态计算。后续 BEV 输入为
`P_gravity = T_gravity_key * P_key`。该方法要求 LIO 参考系的 Z 轴已经与重力一致。

`RobotSubmap.header` 表示参考系和关键帧时间，`cloud_local.header` 表示独立的关键帧局部系。
局部系名称用于数据约定，不另外发布大量子图 TF。RViz 使用已转换回参考系的
`/stage2/ugv/submap_cloud`、`/stage2/uav/submap_cloud`，设置 Decay Time=0 只显示最新子图。
显示云时间戳设为 0，按 ROS 约定选择最新 TF，支持 RViz 晚于子图导出启动。
它只用于显示固定在参考系中的几何；精确的关键帧时间和本段首末帧时间仍保存在子图消息与 YAML 中。
`viz_world` 只用于共同显示，不能据此判断两端已配准。

## 手动导出与文件

无需等待行驶 20 米，可以导出当前尾段：

```bash
rosservice call /stage2/ugv/flush "{}"
rosservice call /stage2/uav/flush "{}"
```

未达到关键帧门槛时采用该段首帧位姿。手动和退出导出的尾段标记 `partial: true`，
不能作为已完成 20 米行程的证据。没有新同步帧时不重复导出。

`results/stage2/<上海时间_微秒_进程号>/` 保存双方文件：

- `ugv_submap_000.pcd`、`uav_submap_000.pcd` 等二进制 XYZ 点云；
- 同名 YAML：robot ID、epoch、关键帧时间、参考系、4×4 变换、重力变换、输入话题、点数和导出原因；
- `config.yaml`、`parameters.yaml`：冻结的桥接配置及两端启动时的 LIO 参数；
- `summary.yaml`：运行状态、同步/丢弃计数、重置及已完成导出。

元数据的 segment 时间与行程对应本段；地图还包含保留的早期历史点。
PCD 和 YAML 落盘后才发布成功消息。文件写入失败会停止桥接节点并保存失败状态，LIO 仿真入口继续运行以便排查。
ROS 时间倒退、同步输入间断 >1 秒、相邻位姿跳变 >1 米或 >45°时清空当前地图、递增 epoch；
子图编号在本次运行中持续递增，既有文件不覆盖。重复/迟到消息丢弃，不触发新 epoch。

## 检查

```bash
catkin_make run_tests_ehpr_bridge -j2
catkin_test_results build/test_results
rosrun ehpr_bridge stage2_verify_exports.py /完整路径/本轮目录
```

可重复的自动检查入口会为每次测试创建独立 ROS/Gazebo master，保存日志并结束自己启动的进程：

```bash
source src/start_up/scripts/setup_stage1.bash
python3 src/ehpr_bridge/tests/run_ros_checks.py --mode synthetic
python3 src/ehpr_bridge/tests/run_ros_checks.py --mode io_failure
python3 src/ehpr_bridge/tests/run_ros_checks.py --mode loop_guard
python3 src/ehpr_bridge/tests/run_ros_checks.py --mode shutdown
python3 src/ehpr_bridge/tests/check_launch.py
python3 src/ehpr_bridge/tests/run_ros_checks.py --mode live
# 有桌面显示时，同时保存本次 RViz 窗口截图：
python3 src/ehpr_bridge/tests/run_ros_checks.py --mode live --rviz-snapshot
```

实际 Gazebo 检查应在停止上一轮 PX4 仿真后运行，避免多个 SITL 争用 MAVLink 端口。
合成检查覆盖双机自动导出、异常输入和 epoch 重置；`io_failure` / `loop_guard` 是预期失败测试，
通过表示节点按要求拒绝运行/停止，相关运行摘要仍明确标记 `failed`。
`shutdown` 检查 Ctrl+C 正常退出时，两端尾段落盘后仍能向订阅者发送完成消息。
每次正常检查结束后再次验证 PCD，包含退出时新增的尾段。

验证工具检查二进制 PCD、非空/有限点、元数据、刚体变换、双方隔离及官方版本，
生成 `verification.json` 和 `verification.md`。图形外观检查和正式行驶 20 米的检查分别记录；
数值文件检查不会自动宣称通过人工地图检查。

`stage2_ros_test.py --mode synthetic` 是合成输入检查程序，需要测试配置中的
两个 LIO 参数根及 `/use_sim_time=true`，由上述隔离测试入口准备；`--mode live` 可在已启动的 Stage 2 双机仿真中运行，
仅订阅数据并调用两端 flush，不解锁或控制机器人。它比较局部云重建与 RViz 点云，
并检查当前真实注册云是否被子图覆盖。正式验收主入口始终为 both。

## 本次交付记录

2026-10-08：构建、8 项子图 C++ 测试、14 项 Stage 1 回归测试及双机 ROS 检查通过。
实际 Gazebo both 已导出可读 PCD，RViz 截图已检查；正常退出尾段、写入失败停止和回环保护也已验证。
汇总见 [results/stage2/REPORT.md](../../results/stage2/REPORT.md)，完整测试路径见
[implementation_checks.json](../../results/stage2/implementation_checks.json)。

实际场景本次只做静止双机接口检查，导出为 partial 尾段；超过 20 米的自动触发已用合成轨迹验证。
未新增 bag、未执行实际运动精度验收；Stage 1 未量化项目及实机验收继续保留。

## Stage 3–4：双机子图检索与精配准

Stage 2 已根据自动检查及用户实测反馈记录为 `accepted_for_stage3`，见
[development_release.json](../../results/stage2/development_release.json)。这不会补写尚未执行的 Stage 1 运动精度验收。

正式入口使用 both，UGV 持续建库，UAV 查询；匹配通过后融合本次 UGV/UAV 子图，发布 `/fused_submap`；完整历史地图累计融合仍未实现。
原 Stage 2 的单机排障入口仍可用，但匹配节点要求两端 LIO 均存在且内部回环已关闭。
每个 accepted match 都先保存再发布。partial 尾段参与同样的门槛，并保留 partial 标志。

```bash
cd /home/yang/wzw/multi_robot_ws
source src/start_up/scripts/setup_stage1.bash
catkin_make -j2
source src/start_up/scripts/setup_stage1.bash
roslaunch start_up stage3_sim.launch
```

启动时 RViz 显示 **UGV observed cloud**（绿色）和 **UAV observed cloud**（紫色），
分别订阅双方 `cloud_registered_raw`。**Fused matched submaps** 已启用，但 `/fused_submap`
在第一次成功匹配前不发布任何消息，显示项尚无数据是正常状态。
在线显示的 Fixed Frame 为 `viz_world`；两端原始观察云的显示锚点只方便观察，不能作为算法已匹配的证据。

在另一个已 source 的交互终端手动控制 UAV：

```bash
rosrun start_up stage1_uav_teleop.py
# t 起飞；i/k 为本地 ENU X 正/负，j/l 为 Y 正/负，w/s 为 Z 正/负。
# 空格保持当前位置；g 降落；a/d 调整偏航。
```

飞到 UGV 已观察的区域后，具备足够公共墙角、建筑边缘且通过 EHPR/GICP 门槛时，
自动拼接双方子图并发布 `/fused_submap`，无需调用 flush。
要单独查看融合结果，可关闭两项 observed cloud。

在线入口使用 `config/stage3_live_submaps.yaml`：以同步扫描的仿真时间预热 15 秒，
每端至少接收 20 帧；UGV 每 10 秒发布一次 partial 预览，即使静止也能建库；
UAV 完成相同预热和帧数检查后，每 3 秒发布 partial 预览进行查询；不要求先移动。
预览采用最新同步位姿作为局部原点，不消耗累计里程，也不重置官方超过 20 米导出和 80 米清理逻辑。
正常距离导出、手动 flush、退出尾段导出继续保留；独立 Stage 2 默认不启用定时预览。
预览间隔不是成功融合的保证，特征不足或质量不通过时不会发布融合结果，原因见 `/stage3/status`。
这些参数可通过 `submap_config:=<yaml>` 替换，使用仿真时间而非墙上时间。

初次验证也可使用已通过的公共区域出生位置，只改变仿真出生位置，不把它送入检索或 GICP：

```bash
roslaunch start_up stage3_sim.launch ugv_x:=-3 ugv_y:=-3 uav_x:=-2.2 uav_y:=-3
# 在另一个已 source 的交互终端：
rosrun start_up stage1_uav_teleop.py
# 查询诊断时，在其他终端运行：
rostopic echo /stage3/status
rostopic echo /stage3/match
```

UGV 入库前收到 UAV 子图会保留最新有效查询；入库后重试。未通过时保留最新查询等待新 UGV 子图。
已成功查询不会重复发布。重复按 robot/session/epoch/submap 的完整身份识别。
默认保留最近 200 张 UGV 子图，超限重建 HBST 并重新分配内部连续 ID，外部身份不变。
UGV session/epoch 改变清空数据库及待查询；UAV 改变清除待查询；双方重置都会清空旧显示云。
回调只放入队列，单工作线程操作数据库和 GICP。队列上限 64，超限丢弃最旧消息并计数。

只挂接匹配节点：

```bash
# 两端 LIO 必须从启动时就关闭内部回环，并已运行 Stage 2：
roslaunch ehpr_bridge stage3_matching.launch
```

### 坐标与话题

输入保持 `/stage2/ugv/submap`、`/stage2/uav/submap` 的 RobotSubmap 接口。
先把 cloud_local 乘保存的重力旋转 G，再运行官方 BEV/ORB/高度编码。
官方粗变换恢复为 `G_UAV⁻¹ · T_gravity · G_UGV`；GICP source=UGV、target=UAV。
结果 **T_UAVkey_UGVkey** 把 UGV 关键帧局部点变换到 UAV 关键帧局部系。
它描述两个被匹配关键帧，并非两个机器人当前时刻的位姿。

| 话题 | 类型 / 内容 |
|---|---|
| `/stage3/match` | `ehpr_bridge/InterRobotMatch`，双方身份和时间、粗/精变换、质量指标及结果文件 |
| `/stage3/match_cloud_ugv` | `PointCloud2`，`T_odom_UGVkey · P_UGVkey` |
| `/stage3/match_cloud_uav` | `PointCloud2`，`T_odom_UGVkey · T_UAVkey_UGVkey⁻¹ · P_UAVkey` |
| `/fused_submap` | `PointCloud2` XYZRGB，本次匹配双方子图在 UGV `odom` 中的体素融合结果，首次成功前不发布 |
| `/stage3/fused_submap` | 同一融合结果的兼容话题 |
| `/stage3/status` | `std_msgs/String`，YAML 状态、数据库大小、拒绝原因、队列及查询计数 |

两份来源显示云均为 UGV 的 `odom`，绿色 UGV、紫色 UAV，RViz Fixed Frame=`odom`。
默认启用 **Fused matched submaps**，另两项默认关闭供对比。融合云使用 RGB8：
绿色体素仅收到 UGV 点、紫色体素仅收到 UAV 点、黄色体素同时收到双方的点。
这是体素内点来源标记，不表示空间区域是否仅由一方观测，也不是配准置信度。
默认 `fusion_voxel_size: 0.15` 米，每个体素输出双方点的联合质心，保留两端各自补充的部分。
显示云 latched，时间戳 0 供晚启动的 RViz 使用；match 保留准确关键帧时间且不 latched。
本阶段不新增两端 LIO 参考系之间的 TF，`viz_world` 不参与算法。

### 参数与文件

默认配置见 [config/stage3.yaml](config/stage3.yaml)。BEV 为 0.5 米、210×210，高度分辨率 1 米，
密度上限 135，局部邻域 3/3；HBST 16 桶、汉明距离 60、分布参数 115/20，跨机器人 `skip_num=-1`。
GICP 0.8 米降采样、最多 10 次迭代、对应距离 5 米，降采样后每云至少 20 点。
要求 RANSAC ≥7 内点、有效高度解、GICP 收敛且为有效刚体变换，有限的 `0≤fitness<2.5`。
在降采样云上按 0.5 米近邻距离计算双向覆盖率，点数较小一侧覆盖率 ≥30%，对应残差 RMSE ≤0.30 米。
这些是初始项目门槛，不是通用精度保证；fitness 是 PCL `getFitnessScore(20)` 的均方距离。
GICP 变换方向、输出矩阵和质量指标均做检查，点数、高度或描述子不足时记录拒绝原因。

`results/stage3/<上海时间_微秒_进程号>/` 包含：

- `parameters.yaml`、`compiled_source.json`：实际匹配配置、固定版本及补丁校验。
- `bev_*.png`：重力对齐后的官方密度 BEV。
- `match_000/{reference.pcd,query.pcd,reference_in_query.pcd,fused_submap.pcd,match.yaml}`：双方原始局部云、精变换后的 UGV 云、位姿与指标。
- `summary.yaml`、`report.md`：查询、拒绝、接受及终止状态；质量未通过的候选也保存已计算的指标。

前三个 PCD 为二进制 XYZ，reference 在 UGVkey，query 和 reference_in_query 在 UAVkey。
`fused_submap.pcd` 为二进制 XYZRGB，位于 UGV `odom`，配套 YAML 的 fusion 字段记录坐标系、
体素大小、点数和各来源体素数量。融合显示随 epoch 重置一起清空，晚启动订阅者可读取 latched 最新结果。
匹配目录整体落盘及摘要更新成功后才发布完成消息。失败保留临时文件便于排查，停止匹配节点，输出 `failed`。
启动/运行失败时查看节点日志；LIO 仿真不会因匹配节点失败而退出。

### 离线回放与检查

要先判断 EHPR/GICP 和拼接实现是否正确，可绕过 LIO 和 Gazebo，使用已知位姿的合成点云独立检查：

```bash
cd /home/yang/wzw/multi_robot_ws
source src/start_up/scripts/setup_stage1.bash
catkin_make run_tests_ehpr_bridge -j2
catkin_test_results build/test_results/ehpr_bridge
python3 src/ehpr_bridge/tests/run_algorithm_validation.py
```

最后一条命令创建独立的 `results/stage3/<上海时间>_algorithm_validation/`，
保存各场景输入 PCD/YAML、事后评估真值、配准结果、融合 PCD、`checks.json`、`REPORT.md` 和覆盖补全图。
覆盖同位姿、六自由度、双方关键帧均倾斜、部分重叠、8 mm 噪声、互不重叠及无高度范围输入。
正例要求相对位置误差 ≤0.20 m、姿态误差 ≤3°；负例不得产生成功匹配。
融合文件还由独立 NumPy 实现核对坐标变换、双方联合体素质心和来源颜色。
质量门槛沿用 `stage3.yaml`；已知位姿仅用于生成输入和事后评估，不用作检索提示或 GICP 初值。

这项检查验证测试范围内的匹配与最新子图对拼接，不能替代 LIO 飞行稳定性、真实场景匹配或历史地图优化验收。
漂移既可能是整个子图的坐标偏移，也可能造成子图内部重影、弯曲；后者通常无法靠一次刚体配准修复。

2026-10-11 复查：正式 5 m 对应距离配置为 6/7；1 m 候选为 7/7，拼接的独立文件与 ROS/RViz 检查通过。
新增带噪声部分重叠正例暴露了配准参数敏感性，候选尚未替换正式配置。
结果、失败证据及候选配置的复现命令见 [算法独立验收报告](../../results/stage3/20261011_algorithm_audit/REPORT.md)。

离线入口直接读取一轮 Stage 2 的 PCD/YAML。只选双方各自最高 epoch，按 UGV 入库、UAV 查询顺序回放；
这是独立的可重复重放，不重现原在线消息到达顺序。不要把多轮 session 的文件混放在同一输入目录。

```bash
roslaunch ehpr_bridge stage3_replay.launch input_directory:=/完整路径/Stage2运行目录
# 无 ROS master 的命令行计算：返回 0=至少一组通过，2=无通过匹配，1=运行或保存失败。
rosrun ehpr_bridge stage3_match_cli /完整路径/Stage2运行目录 \
  src/ehpr_bridge/config/stage3.yaml results/stage3
rosrun ehpr_bridge stage3_verify_results.py /完整路径/Stage3运行目录
catkin_make run_tests_ehpr_bridge -j2
catkin_test_results build/test_results/ehpr_bridge
```

自动检查均只结束自己启动的进程；SITL 检查前先关闭上一轮仿真，避免 PX4 UDP 端口冲突。

```bash
# 已知非对称场景，生成可复用的正例输入：
STAGE3_FIXTURE_DIR=$PWD/results/stage3/synthetic_known_pose_inputs \
  devel/lib/ehpr_bridge/test_matcher --gtest_filter=Matching.KnownPoseGravityAndDisplayDirection
python3 src/ehpr_bridge/tests/run_stage3_ros.py
# 实际双机静止传感器检查；不解锁、不驾驶、不起飞：
python3 src/ehpr_bridge/tests/run_stage3_live.py \
  --ugv-x=-3 --ugv-y=-3 --uav-x=-2.2 --uav-y=-3 --rviz-snapshot
```

在线检查独立记录 `/stage1/{ugv,uav}/ground_truth/lidar_odom`，在双方关键帧时刻插值后比较
`T_world_UAVlidar⁻¹ · T_world_UGVlidar`；真值不会传给匹配节点。
初始通过标准为相对位置误差 ≤0.50 米、姿态误差 ≤5°。
结果、原始雷达真值、ROS 日志和 RViz 截图保存于 `*_both_live_checks`。
本次交付汇总及未通过尝试见 [Stage 3–4 验收报告](../../results/stage3/REPORT.md)。

## 查看融合后的图像

重新编译并重启 Stage 3 匹配节点后，RViz 中启用 **Fused matched submaps**，
Topic=`/fused_submap`，Color Transformer=`RGB8`；在线 Fixed Frame=`viz_world`，离线回放为 `odom`。
临时关闭两项 observed cloud、UGV matched submap 与 UAV aligned submap，可单独查看融合结果。
两端覆盖相同区域时，融合轮廓会与单方接近；要观察地图补全，需要同时具有公共结构和各自独有区域的子图。
融合保留实际输入点，无法生成两端都没有扫描到的结构。这里仅融合最近一次成功匹配的子图对。
后续预览匹配成功会替换融合显示；没有新匹配时保留上次结果，epoch 重置后清空旧结果。
只启动桥接节点时仍需已有双方 LIO，且应在子图启动入口指定上述 `stage3_live_submaps.yaml` 才能使用定时预览。

自动链路和启动显示可分别复查（前者为合成扫描，后者启动隔离的 Gazebo，均不驾驶或起飞）：

```bash
python3 src/ehpr_bridge/tests/run_stage3_auto_preview.py
python3 src/ehpr_bridge/tests/run_stage3_startup.py
```

无需重新启动仿真，可直接回放本次通过的实际双机数据：

```bash
source src/start_up/scripts/setup_stage1.bash
roslaunch ehpr_bridge stage3_replay.launch \
  input_directory:=/home/yang/wzw/multi_robot_ws/results/stage3/20261008_135921_both_live_checks/submaps/20261008_135923_708152_3872400
```

单独生成双方具有不同覆盖范围的**合成演示**，其结果不作为实际飞行验收：

```bash
catkin_make test_matcher -j2
STAGE3_COMPLEMENTARY_DIR=$PWD/results/stage3/synthetic_complementary_inputs \
  devel/lib/ehpr_bridge/test_matcher --gtest_filter=Matching.ComplementaryCoverageCanMatchAndFuse
roslaunch ehpr_bridge stage3_replay.launch \
  input_directory:=$PWD/results/stage3/synthetic_complementary_inputs
# 该合成场景较大，可将 RViz Views / Distance 调为 80。
```

[真实数据融合显示](../../results/stage3/20261008_154322_fusion_replay_checks/rviz_fused.png)
及合成覆盖补全对比：[仅 UGV](../../results/stage3/20261008_154413_fusion_replay_checks/rviz_ugv_only.png)、
[匹配后融合](../../results/stage3/20261008_154413_fusion_replay_checks/rviz_fused.png)。
合成例中，UGV 覆盖 x 上限约 14.25 米，融合后约 22.05 米，新增部分来自 UAV。
自动复现融合显示与文件检查：

```bash
python3 src/ehpr_bridge/tests/run_stage3_fusion_replay.py \
  results/stage3/synthetic_complementary_inputs --snapshot --distance 80
```

## 稳定性修复后的在线入口

`roslaunch start_up stage3_sim.launch` 默认 stable 仿真及自动分包录制；操作及阶段标记见 ../start_up/README.md 的“双机稳定性修复”部分。
UAV 不再需要先移动 0.5 m，预热后可靠静止匹配同样允许融合。质量门槛保持不变。
每个事件的 diagnostics 及 `/stage3/status` 的 last_diagnostics 记录有效高度特征、RANSAC 高度对应和空高度数量。
`stage3_height_radius1.yaml` 是独立对照配置，默认仍为邻域 3。
旧失败记录未被重新标为通过；悬停/运动修复结论以新的完整手动 bag 为准。

### 输入质量联锁与未通过项

正式 stage3_sim 及 stage3_matching.launch 默认要求健康 LIO 诊断。
当前/关键帧未优化、退化或诊断过期时拒绝匹配；暖机后连续异常达到 2 秒，清空匹配状态并锁定本轮输入，需重启整条 LIO 链路。
现有消息类型和 `/fused_submap` 保持不变；首次正确通过前不发布占位云，已有融合被锁定时发布空云清除旧显示。
本轮实际 UAV 稳定性及正确双机融合尚未通过；详见 results/stability/REPORT.md。
GICP/重叠分数不是绝对精度证明，仍必须独立检查真值和公共结构。

### 2026-10-12 闪退后复查

源码和 devel 中的匹配节点、共享库已统一重编译，不再混用独立验证目录的库。
会合入口增加 `matching_profile:=legacy|enhanced`，当前默认 legacy。
增强配置没有通过 103549、110529 两轮困难录制，不能将此轮标为“完全修复”。
复查结果：[REPORT.md](../../results/stage3_repair/20261012_152051_recheck/REPORT.md)。

```bash
cd /home/yang/wzw/multi_robot_ws
source src/start_up/scripts/setup_stage1.bash
# 当前默认对照链路，UGV 仍由键盘手动控制；UAV 停在地面。
roslaunch start_up stage3_gazebo_rendezvous.launch matching_profile:=legacy
# 停止上一轮后，比较增强配置；此配置目前仍是实验配置。
roslaunch start_up stage3_gazebo_rendezvous.launch matching_profile:=enhanced
```

两种模式均只启动一个 Gazebo，算法入口使用带源时间戳的雷达真值，
不运行 LIO；真值不传入匹配器作为检索或 GICP 初值。这些结果不能用于 LIO 验收。
首次可靠匹配前 `/fused_submap` 无数据，普通拒绝保留上一结果，epoch 改变清除旧显示。

独立长测（先停止现有 Gazebo；脚本只生成静止测试，不驾驶车辆）：

```bash
python3 src/start_up/tests/run_gazebo_algorithm.py \
  --duration 300 --timeout 2400 --continuity --matching-profile legacy \
  --world src/start_up/worlds/stage3_rendezvous.world --max-range 12
```

长测核对实际采集时长、epoch、输出间隔、过期/队列/插值计数、P95 延迟及内存样本。
启动时尚未获得完整真值插值区间的计数单独保留；正式测量从双方首次有效输出后开始。
启动参数、录制数据和最终报告均保存到新的运行目录，不覆盖旧结果。

### 可选高空轮廓与会话地图

新增 `partial_contour` 配置：保留 `partial`，在历史地面可信时增加高空二维轮廓优化；双方持久化历史索引支持反向查询和后台补查；配准后的 `/stage3/fused_map` 持续融合双方完整会话地图。新消息 `MapAlignment`、服务 `/stage3/retry_history` 和 `warm_start_inputs` 不改变现有子图对接口。缓存恢复会重新验证关系，epoch/时间回退会清除旧地图。

rendezvous 默认仍为 `partial`，因为冻结录制尚未满足十个不同高空查询的发布门槛。参数、接口、恢复和测试方法见 [高空轮廓与历史地图说明](../start_up/docs/STAGE3_CONTOUR_HISTORY.md)。
