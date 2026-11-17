# pointcloud_fusion

多机器人点云融合（multi-robot pointcloud fusion）相关 ROS 包集合。

本仓库包含两个独立 ROS 功能包（各自保留独立提交历史）：

## 目录结构

| 目录 | 说明 |
|---|---|
| [`ehpr_bridge/`](ehpr_bridge/) | EHPR 部分/轮廓配准桥接包（ROS noetic，C++）：跨机器人点云部分匹配、轮廓配准、亚图融合、对齐跟踪。上游 EHPR 源码见 `ehpr_bridge/UPSTREAM.md` 与 `third_party/EHPR_SOURCE.json` |
| [`start_up/`](start_up/) | 多机器人 stage1-3 仿真启动栈（ROS noetic，PX4 + Gazebo + LIO-SAM）：UAV/UGV 稳定与宽视场模型、stage1-3 launch、rviz 配置、仿真世界 |

## 内容说明

上传内容仅包含**代码、配置、启动文件、模型与文档**；测试数据（点云样本、submap）、溯源记录（`*source_record.json`）等数据/结果文件未纳入版本库（见各包 `.gitignore`）。
