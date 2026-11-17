# 在实际 Gazebo 场景中单独验证点云拼接

这个入口使用当前世界和真实仿真雷达返回点，保留量程、视场和遮挡。
它用 Gazebo 雷达原点位姿代替 LIO 位姿，生成稳定子图，再交给现有官方
EHPR/HBST/RANSAC、PCL GICP 和融合代码。检索和配准不读取真值初始相对位姿。

**这是理想位姿前端的算法实验，不能证明 LIO 漂移已解决，也不是实机定位验收。**
原 `stage3_sim.launch` 的 LIO 和质量联锁保留；本入口不启动 LIO。
它不是把两份地图按已知真值直接拼起来：融合只在算法自己找到并接受匹配后发布。

## 运行和观察

先停止之前的 Stage 3 / Gazebo，避免重复传感器、控制器和同名话题。
每个终端都执行前两条环境命令。

```bash
cd /home/yang/wzw/multi_robot_ws
source src/start_up/scripts/setup_stage1.bash
roslaunch start_up stage3_gazebo_algorithm.launch
```

默认双方在公共墙角附近：UGV `(-3,-3)`、UAV `(-2.2,-3)`，仅用于产生公共观测。
不要求先移动；双方累计 20 帧、预热 15 秒仿真时间后自动生成 partial 子图。
UGV 预览间隔 10 秒、UAV 3 秒，原超过 20 米的正常分段流程保留。
默认不自动驾驶、起飞。可通过出生位置参数改为更远的场景，手动接近公共区域。

RViz 的 `UGV/UAV Gazebo scan (ideal pose)` 显示各自当前扫描；
`Fused matched submaps (algorithm test)` 固定订阅 `/fused_submap`，使用 RGB8。
绿色表示 UGV 独有体素、紫色 UAV 独有体素、黄色表示双方贡献的体素。
只看融合时，取消两项观察云。两项 matched submap 默认关闭，可单独打开检查配准。
显示公共坐标用的真值 TF 只是可视化参考，匹配节点不查询该 TF。

```bash
rostopic echo /stage3/status
rostopic echo -n 1 /stage3/match
rostopic hz /fused_submap
rosparam get /algorithm_test/run_directory
```

启动时 `/fused_submap` 有发布者但没有消息；匹配不通过时也不会拼接。
查看 `/stage3/status` 的 `last_reason`，不能靠观察云看起来重叠来认定配准通过。
默认保留原内点数、fitness、整体覆盖率和残差门槛；GICP 对应距离为 1 米、
降采样为 0.5 米。UGV 关键帧附近 8 米局部云用于检索/配准，最多检查 10 个候选。
首选视图失败时再使用 12 米备用视图，补充被裁掉的公共结构；原有质量门槛同时适用于两种视图。
额外要求立体结构双向覆盖率达到 60%，防止地面和重复墙角假匹配；融合使用完整选中子图。
生产 LIO 配置仍见 `ehpr_bridge/config/stage3.yaml`。
对应配置见 `config/stage3_gazebo_algorithm_matching.yaml`。

已保存的实际场景检查见
[运行报告](../../../results/stage3_gazebo_algorithm/REPORT.md)。
这轮是双方静止、UAV 在地面的算法正例；不是飞行/运动验收。

需要初始扫描分离、保持 UAV 地面不动、UGV 手动接近时，使用
[会合测试入口与路线](STAGE3_RENDEZVOUS.md)：`roslaunch start_up stage3_gazebo_rendezvous.launch`。

## 直接重放本轮真实子图

不需要 Gazebo 或 LIO，重新执行算法并查看融合结果：

```bash
roslaunch start_up stage3_gazebo_algorithm_replay.launch \
  input_directory:=$PWD/results/stage3_gazebo_algorithm/20261011_195123_025630_checks/replay_inputs/match009 \
  config:=$PWD/results/stage3_gazebo_algorithm/20261011_195123_025630_checks/matching_config.yaml
```

该目录中的 PCD 来自这轮真实扫描，没有生成新的场景表面。
回放 RViz 默认只显示融合云；勾选 `UGV matched submap` 并关闭融合项，可比较 UGV 自己的子图。
如果需要同时启动新的实时仿真，先退出回放，避免两个匹配节点发布同名输出。

## 手动控制

UGV 限速控制沿用原入口：

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py \
  cmd_vel:=/stage1/ugv/cmd_vel_raw _speed:=0.3 _turn:=0.3
```

UAV 使用相同的仿真键盘控制，等待新的诊断位姿输入：

```bash
rosrun start_up stage1_uav_teleop.py _require_lio_ready:=false
```

`t` 起飞、`g` 降落、空格保持；`i/k` 为 ENU X、`j/l` 为 Y、`w/s` 为 Z、`a/d` 转向。
跳过 LIO 就绪检查只允许在本诊断入口使用，其他入口默认仍等 LIO 里程计。
位姿采用第一次有效扫描时固定的重力对齐参考系；移动和起降不会重新对齐。

## 自动复现检查

```bash
python3 src/start_up/tests/test_ideal_frontend.py
python3 src/start_up/tests/run_gazebo_algorithm.py --snapshot
```

自动检查启动独立 ROS master 和单个 Gazebo，被动采集双方雷达，**不驾驶和起飞**。
它保存实际参数、原始/门控扫描、真值、诊断前端、子图、匹配结果、融合云、TF、时钟 bag，
以及 PCD/YAML、真值相对变换误差、文件检查和 RViz 截图。
每轮创建 `results/stage3_gazebo_algorithm/<上海时间>_checks/`，不覆盖历史记录。
`checks.json` 的 `status=passed` 只代表本轮算法检查通过；`lio_acceptance=false` 始终保留。
必要时固定对照 `--correspondence-distance 5` 或 `--sensor wide`，拒绝结果同样保留。

手动运行的结果分别保存至上述根目录的 `frontend/`、`submaps/`、`matches/` 时间目录。
停止 launch 后 rosbag 自动收尾。`frontend.json` 明确记录真值来源、实际初始参考变换和丢帧计数。
Gazebo `/model_states` 没有时间戳，因此真值使用 `/clock` 接收时刻；
扫描只接受被真值时间包围、间隔不超过 0.05 秒的插值，不做外推。
本测试只支持逐点 `time=0` 的瞬时扫描，不验证旋转雷达去畸变。

通过该实验可检查物理场景中的检索、变换方向、配准门槛和拼接显示。
之后再切回 `stage3_sim.launch` 验证真实 LIO 前端；两项结果必须分别记录。
