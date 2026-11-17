# 高空轮廓、双端历史检索和完整会话地图

`partial_contour` 是可选配置。rendezvous 默认仍为 `partial`：冻结录制的高空数量验收尚未通过，不能把实现完成当成发布验收完成。其他入口默认不变。

```bash
source src/start_up/scripts/setup_stage1.bash
roslaunch start_up stage3_gazebo_rendezvous.launch matching_profile:=partial_contour
```

此配置保留低空 `partial` 分支。低空失败后，UAV 观测窗口离地至少 2.5 m，并且双方各自历史地面可信，才启用轮廓分支。窗口保存当时的关键帧位姿、实际重力旋转、最后一帧原始三维扫描和质量结论；历史查询不会借用当前飞行高度。单端 session/epoch 更新只清理该端缓存，保留另一端仍有效的完整会话历史；全局时间回退会清理双方，并拒绝旧 epoch 输入。

地面来自各自完整输入云，采用确定性 RANSAC 和局部水平法向见证。支持至少 100 个独立 0.25 m 体素、3 m 跨度、1 m 第二方向宽度、0.03 m RMSE，法向离竖直不超过 5°，高度在自身初始地图原点下方 0.1～1.5 m。多个地面高度或与缓存相差超过 0.05 m 时等待。这里假定双方处于同一平坦地面，不能用于楼层高度未知的情况。

高空 BEV 使用 0.5/0.25 m 二值轮廓，不要求高度编码有效。EHPR/HBST/ORB 检索历史参考，再用确定性线模型补充粗位姿；每参考最多四个不同刚体假设、每查询最多四十个。二维 PCA 法向和一对一对应参与 Huber IRLS，求解 x/y/yaw，z 由历史地面和双方关键帧变换确定。

最终仍需 0.30 m 范围内的 70%/25% 双向覆盖、60 个独立对应、0.20 m 二维欧氏 RMSE、2 m 跨度、两个相差至少 30°且各有 20 个对应的方向，以及至少 10⁻⁴ 的归一化信息特征值比。源、目标法向都参与方向确认。地图变换按 0.25 m/3°聚类，不同簇评分差不足 0.05 时等待。若镜像模型通过相同几何检验且比刚体模型高出 0.05，则拒绝并记录 `reflection explains contour better than rigid yaw alignment`；镜像模型不会成为可发布位姿。

三维自由空间检查只比较真实单帧扫描：独立射线方向容差 0.5°、深度容差 0.75 m、至少 50 条可比较射线，冲突超过 5% 拒绝。累计窗口和二维投影不被解释为自由空间证据。已验证地图关系可以提供下一次配准的初值及一致性约束；是否使用此依据、修订号和矩阵均写入结果文件。首次融合不读取旧跨机器人变换或 Gazebo 真值。

## 历史和恢复

任一方的新输入都能触发查询；后台在输入队列空闲时每两秒墙上时间补查一个历史窗口。位置变化 1 m、朝向变化 10°或新增结构单元达到 10% 的窗口加入持久化索引。静止端最新窗口同时参加检索，不要求进入长期索引。双方最多各驻留 200 个几何条目；这个预算包含低空快速缓存、历史 LRU 和最新窗口。描述子索引和磁盘历史不随几何淘汰而删除。

```bash
rosservice call /stage3/retry_history '{}'
```

服务立即排队返回；新输入仍优先处理。相同历史查询在索引和地面依据未变时不重复计算。

热重启 matcher 时，可以在现有前端和 Stage 2 会话继续运行的情况下恢复自身输入：

```bash
roslaunch ehpr_bridge stage3_matching.launch \
  mode:=ideal_frontend require_lio_quality:=false \
  config:=$(rospack find start_up)/config/stage3_gazebo_algorithm_matching_partial_contour.yaml \
  warm_start_inputs:=/absolute/path/to/previous_matcher_run/recent_inputs
```

先结束原 matcher，避免两个同名发布者。恢复只读取双方自身云、位姿和观测质量，重新建立描述子、地面和累计地图，并等待双方新 Stage 2 预览确认当前 epoch，再重新验证历史配准；不会直接恢复旧有效地图关系。确认完成之前，状态为 `waiting_current_epochs`，手动补查也不会绕过这一等待。完整重启前端或 Stage 2 产生新 session/epoch 时，会清除旧关系和地图，不能把旧原点的缓存混入新会话。此恢复入口目前针对理想前端；正式 LIO 质量隔离不能通过旧缓存绕过。

## 持续地图接口

- `/stage3/match`、`/fused_submap`：原来的配准成功事件与该子图对的完整云融合。
- `/stage3/map_alignment`：latched `ehpr_bridge/MapAlignment`，包含有效标志、修订号、两端帧/session/epoch、`T_UGVmap_UAVmap` 和来源子图对。
- `/stage3/fused_map`：latched `PointCloud2`，位于 UGV 地图系，最多每秒墙上时间更新一次。关系失效立即发布空云。

Stage 3 按 0.15 m 体素独立累计每个有效完整输入。代表点是离体素中心最近的真实点，平局使用坐标排序。重复输入不改变地图，Stage 2 后续清理远处点也不会清理已累计区域。绿色表示 UGV、紫色表示 UAV、黄色表示双方贡献。

地图关系为：

`T_UGVmap_UAVmap = T_UGVmap_UGVkey × inverse(T_UAVkey_UGVkey) × inverse(T_UAVmap_UAVkey)`

首次关系须经两组独立观测确认；新的小幅更新相对固定确认锚点限制为 0.25 m/3°。可靠单帧冲突触发清空融合并重新定位，三组独立观测可确认大幅修正。每次更新都重新投影 UAV 原始地图缓存，不变换已经融合的点。持续地图更新不增加配准成功计数，也不发布跨地图 TF。

运行目录中 `recent_inputs/` 保存可恢复的有效完整输入；`history/` 保存索引、窗口、质量、描述子和地面依据；`map_*/` 保存双方原始地图、融合地图和关系检查点。事件和状态包括历史条目、磁盘加载/淘汰、地面质量、对应数、覆盖、方向、退化、镜像解释、候选评分、关系和累计体素数。RViz 增加持续地图图层。

## 验证

`stage3_verify_results.py` 分别重算子图对变换和颜色、累计地图重投影和颜色，并按实际注册分支校验规则。`verify_map(run, inputs)` 还从全部原始输入独立重算真实代表点缓存，检查历史是否丢失。`run_partial_regression.py` 支持高空不同查询数量、首次高空结果、延迟历史查询和独立真值评分；`make_history_fixture.py` 覆盖四房间、交换顺序、超过 200 条历史和直过道退化；`make_contour_fixtures.py` 提供高空首次融合及几何负例；`run_contour_ros.py` 验证缓存恢复、服务、后续输入、重复、重置和 latched 清除。

冻结真实录制、最终评分和发布决定见本轮 `results/stage3_rendezvous/20261014_115337_contour_history_implementation/REPORT.md`。不能把隔离 ROS 测试当成 Gazebo 高空飞行验收。

`partial` 与 `partial_contour` 共享的双向射线、确认/撤销状态机及更新后的接受规则见 [配准安全说明](STAGE3_ALIGNMENT_SAFETY.md)。
