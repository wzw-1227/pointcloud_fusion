# EHPR 源码版本与桥接变更

- 仓库：<https://github.com/SYSU-RoboticsLab/EHPR>
- 固定提交：`b4b7137f7e675974e025fc3c8e0ab891d353739a`
- 下载归档：GitHub API 该提交的 tarball；校验记录见 `third_party/EHPR_SOURCE.json`。
- 子图流程：`third_party/EHPR/test/multisession_video.cpp:369-413`。
- 行程初始位置：同文件第 486 行，使用本段数据的首个里程计位置。
- 默认参数：`third_party/EHPR/config/multisession_video.yaml` 的 `VoxelMapBConfig`。

`third_party/EHPR` 保留上游原文件，未应用源码补丁。Stage 2 编译桥接及其直接包含的
`VoxelMap.hpp`；Stage 3–4 另编译描述子、数据库和 RANSAC 三个核心 cpp。
不编译完整离线演示或 GTSAM 后端。

桥接层的新增职责：

1. 把官方全局变量组织为两个独立 `SubmapSession`，保留距离触发、早期关键帧、体素点距/容量、
   逆变换和导出后清理逻辑；新增尾段导出、异常输入检查、epoch 和 ROS 消息。
2. 直接输入 LIO 注册云；原离线程序从机体系点云变换得到注册云的步骤无需重复。
3. 官方 `generateSubmap()` 接受单独的旋转输入，离线调用可来自 IMU；本桥接采用
   同步 LIO 雷达位姿的旋转，以相同 Euler/pitch*roll 运算保存重力对齐旋转，假设参考系 Z 已与重力一致。
4. 官方先存关键帧局部云，再对后续描述子输入做重力旋转；本阶段分别保存局部云与重力旋转，
   不提前执行描述子阶段。PCD 改为二进制，另存坐标与身份 YAML。
5. 上游头文件含非 inline 的全局函数；生产代码只在一个编译单元包含它。
   对比测试独立包含同一头文件，仅在测试编译单元用宏重命名未使用的自由函数，避免链接重复定义。
6. 使用官方 `Clear()` 释放体素节点；关闭 LIO 内部回环，保证累计注册点和后续位姿处于同一参考系。
7. `/submap_cloud` 仅用于显示，时间戳 0 选择最新 TF，允许 RViz 在导出后才启动。
   `RobotSubmap` 和 YAML 保留准确关键帧时间及扫描区间。

行为对比测试使用原始 `VoxelMap` 和独立重现的 `generateSubmap` 控制流程，比较触发时刻、
关键帧、排序后的每个局部点和重力旋转。测试位于 `tests/test_submap_session.cpp`。

## Stage 3–4 编译副本与补丁

六个描述子/数据库/RANSAC 头文件和实现文件已逐字节对照固定提交的官方原始文件，
校验见 `third_party/EHPR_STAGE3_SOURCE.json`。CMake 配置时验证原件哈希，
复制到 `build/ehpr_bridge/ehpr_stage3` 后应用 `patches/ehpr_stage3.patch`，仅从副本编译。
每轮结果的 `compiled_source.json` 记录提交、补丁以及原件/编译副本哈希。
不编译离线演示、GTSAM；桥接依赖官方 HBST、OpenCV 4、TBB、glog 和 PCL GICP。

补丁只修改 `desc_db.h`、`desc_db.cpp`、`AlignRansac2D.cpp`：

1. 候选遍历使用匹配表的实际编号，避免稀疏候选编号被误当作 0..N-1。
2. RANSAC 无输入或最终内点不足时返回单位变换和空内点，避免除以零。
3. 粗高度解显式返回有效标志；没有高度解不得把默认 z=0 当成有效解。
4. 查询临时 Matchable 在查询结束和异常析构时释放；入库 Matchable 的所有权交给 HBST。
   UAV 查询描述子不入库；重建数据库时重新生成保留 UGV 描述子与连续内部编号。

BEV/ORB/高度编码的原始 `desc_mng.h/.cpp` 未修改，官方默认参数保持。
桥接保留 ORB 数据，不在查询/入库前调用官方 `clearImage()`（该函数同时释放 ORB）。
OpenCV 内部线程设为 1，避免初始化和线程竞争，未改变描述子算法；数据库仍采用上游 TBB 流程。
`tests/test_matcher.cpp` 对固定输入比较 BEV/ORB，并验证变换方向、重力补偿、
不连续候选、首个候选、无高度解、无特征、GICP 无对应、低重叠拒绝、容量重建和重置。

## 匹配对子图融合显示

桥接新增 `src/submap_fusion.cpp`，未修改官方匹配或子图算法。
匹配通过后，将双方局部点分别变换到 UGV odom，再以 0.15 米体素计算联合质心。
保留每个体素的 UGV/UAV 点来源位，用 RGB 表达，并同时保存 fused_submap.pcd 与元数据。
新话题只融合本次匹配对，不跨 epoch 累积完整历史地图，不改变 EHPR/GICP 的接受门槛。

## 在线 partial 预览（2026-10-08）

桥接新增只读 `SubmapSession::preview()`：读取当前保留体素，使用最新同步位姿的逆变换导出局部云及重力旋转。
它不更改官方分段里程、早期关键帧、体素内容或清理触发；原超过 20 米的完整导出流程继续执行。
仅 Stage 3 在线配置启用定时 partial 预览，使静止 UGV 可建库、移动后的 UAV 可持续查询。
ROS 层增加预热、最少帧数、初始移动门槛，保存 reason=online_preview；这些属于接入策略，非上游默认算法行为。
启动前不向 `/fused_submap` 发布占位消息；融合落盘成功后发布，旧 `/stage3/fused_submap` 保留兼容。
单元测试验证预览不消耗 20 米累计段，传感器到融合的 ROS 测试不调用 flush，不提供真值提示。

## 会合匹配修复（2026-10-11）

`desc_db` 编译副本新增 `queryFeatureCandidates()`，保留官方 HBST 投票、
每候选 RANSAC 和高度编码运算，按内点数排序（同分按内部编号）返回有界候选列表。
每个候选单独携带粗位姿、内点数、高度有效标志和空高度数量；旧单候选 API 保留。
桥接按顺序对候选执行原质量检查，失败后继续；记录 `candidate_attempts`。
补丁仍只应用于构建副本，原始描述子文件不变，源码哈希随结果落盘。

桥接增加可选 `matching_radius_ugv/uav`（0 表示不裁剪）和 `max_candidates`。
半径按重力对齐后的 XY 到关键帧原点计算，局部点保持原关键帧坐标。
局部云只用于描述子和 GICP，`MatchResult`、显示和融合保留完整输入云。
会合实验配置 UGV 半径 8 米、UAV 不裁剪、GICP 0.5 米和最多 10 个候选。
现有正式 LIO YAML 保留原降采样配置，未将本次诊断实验当作 LIO 验收。
CLI 的 `--chronological` 按 `segment_end_timestamp` 回放导出顺序，避免提前建好未来数据库。

会合配置增加立体结构验证：在各云重力对齐 Z 的第 10 百分位以上 0.4 米选择结构点，
两端至少 20 点且 0.5 米内的双向覆盖率均达到 60%。这是一项桥接质量检查，
基于扫描数据统计，未调用世界位姿、真值或控制高度；不替代环境语义分割。
该检查由 `min_structural_overlap` 开启（0 关闭），默认正式配置保持关闭。
其结果记录于 `match.yaml/structural_check` 和每候选诊断，融合仍使用完整点云。

## 输入与候选修复复查（2026-10-12）

固定提交的原始文件仍通过 `EHPR_STAGE3_SOURCE.json` 的 SHA256 核对。
`patches/ehpr_stage3.patch` 现在还修改构建副本的 `AlignRansac2D.h`、
`desc_mng.h`，新增内容如下；上文“仅修改三个文件”描述的是早期补丁。

- 二维 SVD 使用 `V * diag(1, sign(det(V*U.transpose()))) * U.transpose()`，
  用校正后的旋转重新计算平移，避免将镜像当成刚体旋转。
- `legacy` 使用固定 RANSAC 随机种子；`enhanced` 使用确定性 1000 次采样、
  0.75 m 内点距离、1 m 最小采样基线、一对一内点和每参考最多三个假设。
- 增强对应保留 HBST 的全部等距引用，去重并限制为 4096 对。
  空高度编码的对应可参与二维求解，在高度阶段单独计数、跳过；
  不提前丢弃这些二维对应。高度无效、GICP 或结构质量失败仍拒绝发布。
- 描述子生成后只释放 BEV、完整高度栅格和构建缓存，保留 ORB、关键点、
  逐特征高度编码。没有调用同时清空 ORB 的上游 `clearImage()`。
- 增强桥接比较 0.5/0.25 m、邻域 3/1 四套配置和 UAV 八个重力轴视角；
  以保存的辅助旋转恢复局部变换方向，最多 10 个参考、30 个 GICP 假设。

增强配置仍为实验对照，尚未通过两轮困难录制的验收，算法入口默认 `legacy`。
复查证据见 `results/stage3_repair/20261012_152051_recheck/REPORT.md`。
原始上游哈希、构建补丁哈希和实际共享库哈希均独立保留。
