# UAV 漂移处理计划与指令（2026-10-09）

最新 [扫描约束与 32 线对照](../../../results/stability/20261009_150010_mapping_replay/REPORT.md) 已完成。
32 线实际悬停 RMSE 0.14146 m，但连续退化 4.114 秒及真实机体运动使悬停整项失败；融合仍被质量联锁拒绝。
实验平面检查、固定预测 RPY=0.1 和 32 线地面稳定后闭环初始化均未提供通过的修正。
今后自动悬停对照统一在起飞稳定后预热 15 秒（`--hover-warmup 15`），不重置 LIO 或阶段对齐。
诊断工具可用 `--vertical-lines 32` 生成运行专用模型/参数/契约；不是生产 profile，不修改默认入口。
下一次完整 32 线长测宜先预留至少 11 GiB 可用空间；本轮完整记录和旧 bag 均保留。

当前定位与融合仍未验收通过。本计划先筛选输入改动，再定位首次失稳，最后完成双机融合验收。
已准备可选的 `wide` 雷达模型。2026-10-09 已执行两组地面记录及 stable/wide 各一次自动起飞、61 秒悬停、降落对照，
两轮均未通过 UAV 定位验收。实验报告见 [实际运行记录](../../../results/stability/20261009_112158_fov_trials/REPORT.md)。
wide 悬停期间无扫描匹配退化，但整次运行起始对齐后的误差仍超限，默认配置仍保留 stable。

后续 [起飞阶段诊断](../../../results/stability/20261009_141204_takeoff_diagnosis/REPORT.md) 已完成：
取消启动落下过程仍未通过；wide 在空中首次初始化的单次 hover 数值检查通过（RMSE 0.17775 m、退化 0%）。
这不包含起飞段定位，运动/高度/融合尚未通过，正式默认配置不变。

## 1. 对照规则

| 项目 | A：stable 基线 | B：wide 候选 |
|---|---|---|
| 物理步长 / 更新频率 | 0.001 s / 1000 Hz | 相同 |
| UAV 雷达 | 16×1800、10 Hz | 相同 |
| 垂直视场 | -15°～+15° | -45°～+15° |
| 安装位置、外参、噪声、LIO 参数 | 当前 stable 配置 | 相同 |
| 姿态权重 | 0.01 | 相同 |
| 配准门槛及输入质量联锁 | 当前门槛，启用联锁 | 相同 |

本轮只改变向下视场，验证是否能增加有效地面约束。保持线数后各线俯仰角间隔会改变，因此也要检查墙面特征和配准质量。
`wide` 是仿真候选，不代表实际 Velodyne 规格；实机参数不变，默认 profile 仍为 stable。
视场扩大能否解决漂移尚未知；IMU/激光首次失稳的因果仍需完整数据。
不同时调整高度邻域、姿态权重或近距离过滤。每次正常关闭上一轮、重新初始化 LIO，使用相同出生位置及相近手动路线。

## 2. 终端准备

关闭旧 launch 和旧控制终端。以下准备在每个新终端执行一次：

```bash
cd /home/yang/wzw/multi_robot_ws
source src/start_up/scripts/setup_stage1.bash
```

模型和 launch 已准备，不需要再次编译 LIO。当前入口仍是 both，一个 Gazebo。
剩余磁盘检查：

```bash
df -h /home/yang/wzw/multi_robot_ws
```

录制器保留少于 5 GiB 时停止的限制，不删除历史结果。

### 独立仿真自动悬停对照

以下工具复用 MAVROS 位置控制，自动起飞、保持目标 61 秒仿真时间，再降落并等待解锁关闭；不驾驶 UGV。
它创建独立 ROS/Gazebo master，检测到已有 Gazebo 时拒绝启动。真值只参与评估，不进入控制器或 LIO。
依次执行，必须等上一条正常结束再执行下一条：

```bash
python3 src/start_up/tests/run_hover_probe.py --sensor stable --duration 61
python3 src/start_up/tests/run_hover_probe.py --sensor wide --duration 61
```

仅复现后续初始化诊断，可单独执行下面的一条；会实际在独立仿真中起飞和降落：

```bash
python3 src/start_up/tests/run_hover_probe.py --sensor wide --duration 61 --init-in-air
```

此模式在起飞稳定后首次给 UAV LIO 输入扫描，预热 15 秒后标记 hover；不在运行中重启 LIO，也不逐阶段对齐。
不包含起飞段 UAV LIO 轨迹，仅为诊断入口；原手动入口和正式默认配置保持。

结果保存至 `results/stability/<时间>_<profile>_hover/`。`progress.json` 显示当前阶段；
`probe.json` 的 `hover_complete` 只表示控制流程完成，不表示精度通过。
使用其中的 `recording_directory` 执行 `stage3_evaluate.py`，查看 hover 阶段报告。
飞控本地位置保持不代表真实机体完全静止；评估仍检查真值运动，不能重设对齐来消除误差。

## 3. A 轮：基线悬停记录

终端 1 启动，保留默认出生位置，避免在已知隔墙负例或地面即退化的位置开始正式悬停对照：

```bash
roslaunch start_up stage3_sim.launch sim_profile:=stable uav_sensor_profile:=stable record:=true
```

终端 2 检查初始化和录制。`health=ready` 只说明节点/流正常，不证明 LIO 无退化：

```bash
rostopic echo -n 1 /stage1/health
rostopic echo -n 1 /stage3/recording
rostopic echo -n 1 /stage3/status
```

终端 3 启动 UAV 手动控制：

```bash
rosrun start_up stage1_uav_teleop.py
```

按 `t` 进入 OFFBOARD、解锁并起飞；稳定后按空格保持当前位置。`t` 的目标为初始位置上方 1.5 m。
UGV 本阶段不动。终端 2 标记悬停：

```bash
rosrun start_up stage3_phase.py hover
```

保持至少 **61 秒仿真时间**，不是墙钟 61 秒。查看 `/clock` 或 RViz 时间。
可在另一终端查看瞬时诊断；它们只供观察，完整判断使用 bag：

```bash
rostopic echo -n 1 /uav/lio_sam/debug/scan2map
rostopic echo -n 1 /stage3/status
# 惯性预测的世界系速度；停止观察用 Ctrl-C
rostopic echo /uav/odometry/imu_incremental/twist/twist/linear
```

关注 `optimization_performed`、`is_degenerate`、`weak_direction_count` 和有限特征值。
若预热后出现连续超过 2 秒的退化、明显漂移或 quality quarantine，记为失败，结束该轮并保留完整录制。
联锁锁定后，偶尔恢复一帧不会重新开放融合，需要整个链路重新初始化。

## 4. 保存路径、正常收尾及悬停筛选

在录制还运行时，终端 2 保存实际目录到本终端变量，避免猜测最新目录：

```bash
UAV_DRIFT_RUN_DIR=$(python3 - <<'PY'
import json, rospy
from std_msgs.msg import String
rospy.init_node('read_drift_recording_directory', anonymous=True)
message=rospy.wait_for_message('/stage3/recording', String, timeout=30)
print(json.loads(message.data)['directory'])
PY
)
printf '%s\n' "$UAV_DRIFT_RUN_DIR"
```

仅做悬停筛选时标记结束：

```bash
rosrun start_up stage3_phase.py end
```

在 UAV 控制终端按 `g` 请求降落，确认落地后正常 Ctrl-C 关闭控制及 launch，让 bag 收尾。
回到保存了变量的终端 2：

```bash
rosrun start_up stage3_evaluate.py "$UAV_DRIFT_RUN_DIR"
cat "$UAV_DRIFT_RUN_DIR/phases/00_hover_uav/report.md"
cat "$UAV_DRIFT_RUN_DIR/phases/00_hover_ugv/report.md"
```

仅悬停筛选轮的总报告会因缺少运动/高度阶段或匹配而返回 failed，这是预期；应看对应 hover 阶段的指标和失败原因。
UAV 悬停位置 RMSE ≤0.20 m、姿态 RMSE ≤5°；UGV 静止误差 ≤0.10 m；输出无超过 1 秒中断，无节点崩溃/反复预积分重置。
连续退化超过 2 秒仍失败。UAV 未实际起飞的记录不能算悬停通过。

## 5. B 轮：扩大向下视场，重复同一悬停流程

结束 A 轮后，在终端 1 使用下面这条启动 B 轮，其他步骤与 A 相同：

```bash
roslaunch start_up stage3_sim.launch sim_profile:=stable uav_sensor_profile:=wide record:=true
```

确认实际 profile，避免误用旧进程：

```bash
rosparam get /stage1/uav_sensor_profile
rosparam get /uav/lio_sam/N_SCAN
rosparam get /uav/lio_sam/Horizon_SCAN
```

应分别为 `wide`、`16`、`1800`。每轮保存自己的目录，不能拿 A 的变量评估 B。
录制目录的 session.json、parameters_start.yaml、runtime_sources.json 会保存实际 profile、世界/模型 hash 与算法源/二进制版本。

判定：若 B 有改善但仍超门槛，不采用为正式修复；若 B 悬停通过，进入完整运动验证；若 A/B 均通过且 B 没有明确收益，优先保留 A。
若 A/B 均失败，停止完整融合验收，进入首次失稳分析。

## 6. 若仍失败：分析首次偏离，而非继续放宽门槛

由保存的 A/B 完整 bag 对齐检查：原始 IMU 与时钟间隔 → 惯性预测速度/位姿 → scan2map 退化/增量 → 地图畸变。
已有 bag 包含原始输入、预积分里程计和扫描诊断；IMU 估计偏置没有专用话题，若时序不能定位原因，再增加偏置诊断。
`health=ready` 或某一帧不退化不能替代这一步。

列出分包和录制摘要：

```bash
ls -lh "$UAV_DRIFT_RUN_DIR"/data*.bag
cat "$UAV_DRIFT_RUN_DIR/session.json"
cat "$UAV_DRIFT_RUN_DIR/report.md"
```

如需快速同房间地面负例对照，先结束手动运行，再分别执行，工具不驾驶/起飞，报告不能算悬停验收：

```bash
python3 src/start_up/tests/run_stability_probe.py --physics stable --sensor stable --near-rich --duration 45
python3 src/start_up/tests/run_stability_probe.py --physics stable --sensor wide --near-rich --duration 45
```

源码修复按首次异常证据决定：惯性预测先异常则修时间/坐标/预积分；激光校正先异常则查特征对应和退化方向处理。
当前不预先指定某个源码改动为根因。修复后需要重新构建、重复 A/B 及实际节点回归；不能仅关闭联锁证明修复。

## 7. 候选通过悬停后：正式双机运动及融合验收

重新初始化选定的配置。本轮依次采集所有阶段，不在 hover 后提前 end。
UAV 仍用手动控制，UGV 在独立交互终端运行：

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py \
  cmd_vel:=/stage1/ugv/cmd_vel_raw _speed:=0.3 _turn:=0.3 \
  _repeat_rate:=10.0 _key_timeout:=0.3
```

按阶段开始时，在监测终端依次标记（命令不会控制机器人）：

```bash
rosrun start_up stage3_phase.py hover
# 两端稳定，静止/悬停至少 61 秒仿真时间
rosrun start_up stage3_phase.py ugv_motion
# UAV 保持；UGV 在无遮挡公共结构附近累计至少 20 m，完成四次约 90°转向
rosrun start_up stage3_phase.py uav_motion
# UGV 保持；UAV 累计至少 20 m，完成四次约 90°转向
rosrun start_up stage3_phase.py height
# UAV 用 w/s 上下变化至少 0.5 m，恢复原高度
rosrun start_up stage3_phase.py end
```

移动键：UAV `i/k` 为 ENU X±，`j/l` 为 Y±，`w/s` 为 Z±，`a/d` 为偏航±；空格保持当前位置。
保持 0.3 m/s、0.3 rad/s 上限；两轮使用相近路线，避免碰撞。靠近但隔墙不作为正例，需双方观察到相同公共结构。
预热后即允许静止 partial，无需人为移动触发子图。

查看可靠融合：

```bash
rostopic echo -n 1 /stage3/status
rostopic echo -n 1 /stage3/match
rostopic info /fused_submap
```

无接受匹配时 `/stage3/match` 的 echo 会等待，可 Ctrl-C 结束。`rostopic info` 有发布者不等于已有融合消息。
RViz 使用独立观察云和 RGB8 的 `/fused_submap`，检查共同墙面/地面重合，保存真实截图。
正常收尾后先运行数值评估；仅在实际人工检查通过、截图存在时再填写人工结论：

```bash
rosrun start_up stage3_evaluate.py "$UAV_DRIFT_RUN_DIR"
rosrun start_up stage3_evaluate.py "$UAV_DRIFT_RUN_DIR" \
  --map-review pass --screenshot /绝对路径/本次RViz截图.png
```

完成门槛：悬停/静止通过；运动位置 RMSE UGV≤0.20 m、UAV≤0.30 m、姿态≤5°；高度变化通过；无超过 1 秒中断；特征丰富路段无持续超过 2 秒退化。
至少一组实际匹配关键帧独立真值误差≤0.50 m/5°、非空可读融合 PCD、公共结构重合截图。
各阶段共用整次运行一次起始刚体对齐，真值不进入算法。只有这些全部通过才标记 UAV 漂移已修复。
