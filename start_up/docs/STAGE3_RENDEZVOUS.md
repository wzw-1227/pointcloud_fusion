# UAV 地面保持、UGV 手动接近的融合实验

按用户选择让 UAV 停在地面，先排除飞行控制和 LIO 漂移。
使用真实 Gazebo ray 扫描和理想位姿前端，EHPR/GICP 自主检索配准，不读取真值初值。
该入口不验证真实 LIO 或飞行稳定性。

## 预热与初始分离

从每个机器人第一对有效同步扫描/位姿开始，至少 **15 秒仿真时间、20 帧**才允许自动 partial 预览。
UGV 每 10 秒、UAV 每 3 秒更新，运动不是必要条件。
预热用于避免观测量太少，不能证明估计稳定或保证匹配成功。
仿真慢时实际等待超过 15 秒；手动 flush 独立于预览门槛，本实验初始阶段不需要调用。

看不到对方机体不代表没有观测共同环境，匹配依据是共同墙面、地面等结构。
本入口把有效输入量程设为 **12 米**。以下历史路线的初始雷达原点约相距 **24.82 米**，
扫描域之间至少约 0.82 米距离，不会在 0.5 米对应半径内重叠。
原始雷达话题和正式 30 米前端默认值保留。
实际出生位置以 `stage3_gazebo_rendezvous.launch` 中的参数为准；改过出生位置后，
不能沿用本页历史路线的“初始扫描不重叠”结论。
独立世界复用 stable 场景，在左侧增加起点环境并留 2 米宽门口；原世界不修改。

## 手动复现

关闭旧仿真/回放。终端一：

```bash
cd /home/yang/wzw/multi_robot_ws
source src/start_up/scripts/setup_stage1.bash
roslaunch start_up stage3_gazebo_rendezvous.launch
```

UGV 机体出生 `(-26.6,-8.5)`，UAV `(-2.2,-3)`，落稳后不控制 UAV。
先停住 UGV 等预热，确认 `/stage3/status` 有检索活动、`/fused_submap` 还没有消息。
终端二（每个新终端都先 source）：

```bash
cd /home/yang/wzw/multi_robot_ws
source src/start_up/scripts/setup_stage1.bash
rosrun start_up stage3_phase.py separated
rosrun start_up stage3_phase.py approach
rosrun teleop_twist_keyboard teleop_twist_keyboard.py \
  cmd_vel:=/stage1/ugv/cmd_vel_raw _speed:=0.3 _turn:=0.3
```

`i` 直行、`j/l` 原地左/右转、`k` 停止；接近障碍用 `z` 降低线速度。
以下是 **Gazebo 机体 world 坐标**，不是 LIO 参考系坐标：

| 顺序 | 位置 | 操作 |
|---|---|---|
| 1 | `(-26.6,-8.5)` | 初始朝 +X |
| 2 | `(-5.5,-8.5)` | 向 +X，穿过 `(-10,-8.5)` 的门口；左转朝 +Y |
| 3 | `(-5.5,-2.55)` | 向 +Y；右转朝 +X |
| 4 | `(-3,-2.55)` | 向 +X 绕过内墙北端；右转朝 -Y |
| 5 | `(-3,-3)` | 低速接近后停住，与 UAV 约相距 0.8 米 |

路线约 30.0 米，限速 0.3 m/s，仿真时间至少约 100 秒，加上转向。
[路线图](../../../results/stage3_rendezvous/20261011_rendezvous_fix/geometry/route.png)
和 [静态碰撞几何检查](../../../results/stage3_rendezvous/20261011_rendezvous_fix/geometry/checks.json)
已保存。这是手动路线参考，不是自动驾驶或动态避障保证。
另开终端可查坐标和匹配原因：

```bash
rosservice call /gazebo/get_model_state "{model_name: xiaoqiang, relative_entity_name: world}"
rostopic echo /stage3/status
```

不一定到终点才观测到共同结构，也不一定出现重叠就马上匹配。
通过质量门槛且落盘成功后才发布融合，被拒绝时查看 `last_reason`。
抵达后停住、观察几轮预览，并标记：

```bash
rosrun start_up stage3_phase.py rendezvous
rostopic echo -n 1 /stage3/match
```

## 是否包含沿途环境

匹配节点默认为每台机器人缓存最近 5 秒仿真时间的同步注册扫描和里程计，
先以 0.2 米体素降采样，至少有 20 帧才允许使用。近期窗口单独用于描述子、
粗配准、GICP 和质量检查；原建图节点继续保留完整历史，接受后融合完整子图。
因此 UGV 沿途观察过、UAV 没看到的墙面不会持续进入近期匹配云的覆盖率分母。
窗口按最新雷达位置裁剪，仍优先尝试 8 米、失败后尝试 12 米，原质量门槛保留。

参数为 matching YAML 中的 `use_recent_matching_cloud`、`recent_window_seconds`、
`recent_voxel_size`、`recent_min_frames` 和 `recent_max_frames`。在线窗口尚未就绪或
输入超过 1 秒仿真时间未更新时，状态为 `waiting_recent_cloud`；不会用历史云冒充近期云。
老录制未提供近期云时，离线 CLI 保持原历史匹配方式用于兼容对照，结果中的
`matching_scope.reference_cloud_source/query_cloud_source` 会明确标为 `history`。

每轮 matcher 的 `recent_inputs/` 保存实际使用的完整云、近期云及窗口时间、帧数和局部中心，
可以直接用 CLI 重放。已有 bag 可用 `ehpr_bridge/scripts/prepare_recent_replay.py` 抽取
注册扫描和里程计重建窗口；该脚本不使用两端真值相对位姿作为检索或配准提示。

扫描从开始就累计，预热不是延迟开始建图。
同一 epoch、输入连续时，超过 20 米导出子图后 **不清空体素地图**，近处历史点继续保留。
20 米是出图间隔，不是只保留最近 20 米的窗口；时间或位姿异常导致 epoch 重置时会清空累计。
正常导出/flush 后清除当前位置 80 米之外的体素；预览本身不清理历史。
这条 30.0 米路线的正常观测都在保留范围内，因此后续 UGV 子图会继续包含沿途点，
但经过官方体素密度过滤，不保留每个原始点。

融合是 **本次匹配选中的完整 UGV 子图 + 对齐后的 UAV 子图**，不会只裁出公共区域。
0.15 米融合体素以质心代表重复观测。选中包含起点和途中观测的 UGV 子图时，融合也会包含它们。
`/fused_submap` 每次替换为最新接受的子图对，**不是整个历史数据库的累计融合**。
若检索选中较早的 UGV 子图，融合就不一定包含后续全部路段。

检索和配准优先使用关键帧附近 8 米水平半径的 UGV 局部云，UAV 使用完整查询云；
8 米视图未获得可靠匹配时，再检索/验证 12 米视图（`matching_radius_ugv_fallback: 12.0`）。
12 米视图补回可能被 8 米裁剪去掉的公共墙面；保留 8 米首选可避免沿途历史影响原先的成功场景。
两个视图分别建立描述子数据库，使用相同质量门槛；首选通过时不运行备用检索。
这只是匹配输入范围，`/stage2/ugv/submap` 和融合仍保留选中子图的完整历史点。
GICP 降采样为 0.5 米，残差门槛仍是 0.30 米；每个视图最多验证 10 个 legacy EHPR 候选，
前一个高度或几何失败后继续验证后续候选。
`/stage3/status` 和结果 `summary.yaml` 的 `candidate_attempts` 记录每个候选的身份和拒绝原因。
所有候选失败时继续等待新子图，不发布融合。
事件日志的每个候选增加 `matching_radius_ugv_m`；成功结果的 `matching_scope`
记录实际使用的半径，状态计数 `radius_fallback_matches` 记录备用视图接受次数。
设置 `matching_radius_ugv_fallback: 0.0` 可恢复仅用首选视图的行为。
为拒绝地面和重复墙角造成的假匹配，还要求两端立体结构点各自至少 20 个、
双向结构覆盖率均达到 60%。结构点为重力对齐后高于该云 Z 第 10 百分位 0.4 米的点；
这是点云自身的高度统计，没有使用 Gazebo 真值或机器人实际高度。
普通覆盖率 30%、0.30 米残差等原门槛继续执行。
若共同结构过少，仍会拒绝融合；`candidate_attempts` 可区分高度、整体几何和结构覆盖失败。

在 RViz 依次比较：

- `UGV Gazebo scan (ideal pose)`：当前单帧，随车移动。
- `UGV accumulated submap`：`/stage2/ugv/submap_cloud`，最新导出的累计历史。
- `UGV matched submap`：`/stage3/match_cloud_ugv`，本次真正选中的 UGV 子图。
- `Fused matched submaps (algorithm test)`：`/fused_submap`，本次两张子图融合，RGB8。

看融合时关掉其他点云显示；然后单独看 matched UGV，比较沿途墙面是否保留。
`reference_submap_id` 对应 `submaps/<session>/ugv_submap_NNN.yaml`，
`result_path` 对应实际选中双方 PCD 和融合 PCD。
关键帧时刻只是局部坐标原点时刻，不是所有点的采集时刻。
结果和包含阶段标记的 bag 保存到 `results/stage3_rendezvous/` 的 frontend、submaps、matches 时间目录。

## 验证边界

新起点扫描域分离、近处静止匹配均已在新的 Gazebo 运行验证。
你的已有实际手动会合记录按导出时间顺序回放，得到 41 组正确匹配，
独立位姿和融合文件检查全部通过；错误的重复结构对子图被拒绝。
新起点的完整连续手动驾驶尚未重新运行，仍按上述路线手动测试。

最终结果见 [修复与验证报告](../../../results/stage3_rendezvous/20261011_rendezvous_fix/REPORT.md)，
[已有手动会合扫描的融合对比](../../../results/stage3_rendezvous/20261011_rendezvous_fix/recorded_rendezvous_fusion.png)
显示完整 UGV 沿途环境保留在融合中。
这些是理想位姿前端下的算法检查，不能替代 LIO 漂移或飞行稳定性验收。
