# 部分重叠空地点云配准

`partial` 保留 EHPR/HBST、BEV/ORB、高度编码和确定性 RANSAC，用重力约束的鲁棒精配准与独立几何检查支持不同的观测范围。原 `legacy` 和 `enhanced` 配置保持可用。该配置首先用于 Gazebo 理想前端实验。
`stage3_gazebo_rendezvous.launch` 默认使用 `partial`；通用算法入口和正式 LIO 入口继续保留原默认配置。

```bash
source src/start_up/scripts/setup_stage1.bash
roslaunch start_up stage3_gazebo_rendezvous.launch matching_profile:=partial
```

出生位置、雷达扫描参数和 12 m 前端量程沿用 launch；几何通过后先等待独立观测确认；首次需要两组、恢复需要三组，再发布 `/stage3/match` 和 `/fused_submap`。两端不必同时移动，静止端重复扫描不加票。融合使用选中双方的完整子图，匹配阶段的局部裁剪不会删除沿途历史。

## 候选与精配准

最近 5 秒的同步注册扫描提供匹配输入；EHPR 在 0.5/0.25 m BEV、不同高度邻域和八个朝向中检索。至少五个独立 RANSAC 内点只允许启动精配准，不直接证明成功。新配置的平面内点距离为 1.5 m，旧 enhanced 默认值仍为 0.75 m。

匹配层使用各自局部姿态的 `Rz(-yaw) * R` 规范重力旋转，避免 Euler 分支造成的额外 180° 水平翻转。原 `T_gravity_key` 保留；有效旋转写入 `T_matching_gravity_key`。两端的全局相对位姿没有作为算法初值。

优化分辨率为 0.5/0.25 m，对应距离 1.5/0.75 m，Huber 尺度 0.3/0.15 m，每级最多 20 次迭代。PCA 法向邻域为粗层 8 点、细层 12 点；粗层允许较大法向差以恢复偏航，最终验证要求法向差不超过 30°。对应采用一对一目标分配。
迭代增量低于 0.02 m / 0.2° 时停止；独立几何门槛和事后真值评分继续判断结果是否可靠。

高度约束来自各端传感器原点下方至少 20 点支持的水平面，分组容差 0.10 m；多个平面产生多个高度假设并检查歧义。水平面支持不足时拒绝接受，避免依赖立面法向的小噪声错误地判断高度可观测。

## 接受检查

- 全部立面结构云作为覆盖分母，较高一侧覆盖率至少 70%，另一侧至少 25%。
- 至少 80 个独立立面对应、0.20 m 以内欧氏对应 RMSE、2 m 以上水平跨度。
- 至少两个相差 30° 的立面方向，每个方向至少 20 个对应；信息矩阵最小/最大特征值比至少 10⁻⁴。
- 对实际单帧扫描执行双向自由空间检查，使用各自真实原点；角容差 0.5°、深度容差 0.75 m，每方向至少 50 个独立方向及 20 个非地面方向，总/非地面冲突率均不超过 5%。累计窗口不充当射线。
- 对缺少近期扫描原点的旧累计云，附加双向立面覆盖至少 60% 的兼容检查，不把累计地图当作单帧射线数据；缺少单帧附件时仅保留几何候选，不确认新关系。
- 将候选换算成同一地图变换，以 0.25 m / 3° 聚类。评分为 `较高覆盖率 × sqrt(较低覆盖率) × exp(-(RMSE/0.20)^2)`；不同位姿簇的分数差小于 0.05 时拒绝歧义结果。

`partial` 对两种 UGV 半径均比较候选，总数限制为 30。旧配置继续使用原来的首选/备用半径流程。

## 状态与验证

`/stage3/status` 给出 profile、优化方法和最近一次拒绝原因；事件与 match YAML 的 `partial_validation` 记录有效对应数、水平面支持、结构方向、自由空间冲突、退化指标和分数。`matching_processing_wall_s` 是单次 matcher 运算时间，另以端到端话题观察检查发布延迟和队列。

离线回放入口：

```bash
python3 src/ehpr_bridge/tests/run_partial_regression.py \
  --inputs <冻结子图目录> --frontend <独立评分用的frontend.json> \
  --config src/start_up/config/stage3_gazebo_algorithm_matching_partial.yaml \
  --output <新的结果目录> --min-queries 3 --required-query 37
```

CLI 只收到点云目录与配置，不读取评分真值。回放检查每个接受结果的 0.20 m / 3° 精度、融合方向、完整点云保留、体素贡献颜色与配置门槛。真实 LIO、RTK 和实机雷达接入需要单独验证。

完整状态机、独立性条件、自动撤销恢复和参数见 [配准安全说明](STAGE3_ALIGNMENT_SAFETY.md)。
