# WHILL 轮椅导航仓库

私有研究仓库，仅包含 WHILL 轮椅导航相关内容。

- [WHILL 原生 Nav2 导航](experiments/whill_nav2/README.md)：2026-11-03 Jetson 源码快照、配置、标定清单及覆盖层补丁。

## 项目概述

在非 EDU WHILL 轮椅上运行原生 ROS 2 Nav2 导航栈，运行环境为 Jetson AGX Orin / Ubuntu 22.04 / ROS 2 Humble / Cyclone DDS。当前保留的导航配置为 **turn_memory：Smac Lattice + RotationShim + MPPI + ObstacleLayer 记忆**。

导航链路：

```text
双MID360过滤点云 -> scan -> 两个前角遮罩 -> scan_map / memory_obstacles
Odin -> odom/TF                         -> local/global costmap
RViz目标 -> BT / SmacLattice -> RotationShim / MPPI
        -> VelocitySmoother -> CollisionMonitor -> cmd_vel_adapter -> ESP32 / WHILL
```

## 目录结构

- `experiments/whill_nav2/src/wheelchair_navigation/`：唯一导航 profile 包、八顶点车体、前角 mask、BT、运动原义、启动和标定来源文档。
- `experiments/whill_nav2/src/whill_controller/`：实际共用的底盘 cmd_vel 适配、HID 标定、ROS 接口。
- `experiments/whill_nav2/src/nav2_overlays/`：实际环境使用的两层 RotationShim 源码。
- `experiments/whill_nav2/patches/`、`experiments/whill_nav2/upstream/navigation2.repos`：固定 Nav2 上游版本和本地源码补丁。
- `experiments/whill_nav2/docs/`：系统架构、底层控制、汇报框架与验证证据。
- `experiments/whill_nav2/support/site_runtime/`：现机 Nav2 覆盖层构建入口及隔离仿真 DDS 环境。

## 来源与许可

第三方来源与许可说明见 [THIRD_PARTY.md](THIRD_PARTY.md)。Nav2 上游 commit 固定在 `experiments/whill_nav2/upstream/navigation2.repos`。

## 重要边界

- 原生 Nav2 控制链没有接入眼控 RDS 速度求解器；眼控作为下一阶段上层输入保留接口位置。
- 源码归档不代表已重新启动或验证真实轮椅；现机入口命令见 [experiments/whill_nav2/README.md](experiments/whill_nav2/README.md)。
- 环境依赖（ROS/Jetson/CUDA 库、传感器驱动、眼控共享工作区）不打包，全新机器的独立安装与真车复测尚未完成。
