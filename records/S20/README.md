# S20研究记录入口

## 当前在线抓放代码（2026-10-26）

本节是本聊天当前真机主线。下方旧RL、32/41件及其他聊天结果仍独立保留，不是当前执行器的依赖或验收。

在工作区根目录使用：

```bash
./s20 config                 # 查看当前配置，不访问设备
./s20 run --help             # 查看全部启动参数
./s20 run --execute          # 四件现场抓放，保留1/2/3人工确认
./s20 plan --observation /完整路径/已记录的observation目录
```

- 唯一短入口：[s20](../../s20)。旧online_session命令仍兼容；短入口不包含自动使能、松泵或故障重试。
- 当前配置：[current_profile.json](../../experiments/s20_online_mixed8_20261024/deployment/current_profile.json)。相机/模型、空间与路线参数、默认启动选项集中于此。新会话加载，运行中的进程不热改；显式命令行启动选项优先。
- 新运行结果统一放在`experiments/s20_online_mixed8_20261024/runs/`；用户显式--output可覆盖。旧运行目录/证据不搬动。每轮会话result.json记录配置快照、真实CLI选项和阶段耗时。
- 标定与示教不混入普通算法参数：`deployment/scene_tf.yaml`、`site_robot.urdf`、`pick_waypoints.json`保持原路径。

### 模块边界

以下路径均位于`experiments/s20_online_mixed8_20261024/`。

| 责任 | 当前源文件 |
| --- | --- |
| 本机流程、异步衔接、占据提交 | `deployment/online_session.py` |
| Jetson相机生命周期、真实配对帧 | `deployment/camera_session.py`、`camera_receive.py` |
| 观测流程与RGB-D转换 | `deployment/observe_current.py`、`received_to_observation.py` |
| YOLO分割、顶面/尺寸、中心吸取 | `perception/yolo_sim_perception.py`、`envelope_grasp.py`、`grasp_interface.py` |
| 稀疏贴边候选与横放优先 | `deployment/planning_candidates.py`、`straight_place/contact_layout.py` |
| 抓取准备、稀疏CAD采样 | `deployment/prepare_observed_pick.py`、`pick_reference_recorder.py`、`observed_collision.py` |
| 持箱/空载求路、独立规划实例 | `deployment/prepare_observed_place.py`、`straight_place/async_sparse_place.py`、`optimized_cycle_place.py` |
| 事件与参考组合 | `deployment/compose_cycle.py` |
| 官方SDK执行及真空/人工确认 | `deployment/native_cycle_bridge.py`；`s20-device`和`s20-interfaces`为远端入口 |
| 历史ServoJ接口，不是当前默认执行 | `deployment/ros_cycle_bridge.py`、`servoj_reference.py`、`run_servoj_executor.py`、`start_servoj_executor.py` |
| 诊断与离线实验，不是主流程入口 | `compare_saved_detectors.py`、`movepath_single.py`、`native_down_compare.py`、`vacuum_stationary.py`；不因整理而执行 |

当前行为：YOLO11m-seg→上表面/中心吸取→抬升/示教入舱→异步候选与IK/CAD→先伸入、再横移、最后下放→释放确认→竖直上抬20cm→空载返回。水平每侧5cm代理余量，实际底面支撑另算；5cm高度归并尚未启用。当前是贪心/数值规划，不是RL、严格GSIP/KC复现，也不把现用LM路径称cuRobo V2后端。

规划依赖仍借用`experiments/s20_greedy_async_20261023`、`s20_newton_update_20261020/runtime13`及`reproductions/ludan_greedy_20261023`；这些外部依赖尚未抽成独立可安装包，不能单拷deployment就称完整部署。主机规划Python仍为`reproductions/nvidia_curobo_v2_20261023/.venv/bin/python`，其目录名不代表每次都执行cuRobo。

### 证据与历史

- 最新“先伸入再横移”四份实测输入离线准备：[depth_then_shift_20261026](../../experiments/s20_online_mixed8_20261024/depth_then_shift_20261026/)。不是四件真机通过。
- 四周余量/释放后直升：[release_lift_padding_20261026](../../experiments/s20_online_mixed8_20261024/release_lift_padding_20261026/)。保留第一次trace失败及修正。
- 横放排序：[horizontal_order_fix_20261026](../../experiments/s20_online_mixed8_20261024/horizontal_order_fix_20261026/)。
- 检测模型比较与顶面平行假设修正分别在`detector_comparison_20261026/`、`round2_plane_fix_20261026/`。
- 全部历史结论、失败和限制继续写入[RESEARCH_DIARY.md](RESEARCH_DIARY.md)，不另开同类MD；历史报告里的“当前”以对应日期理解，不覆盖本节。

## 其他研究与原始记录

先看[迭代记录和精选可视化](showcase/README.md)。当前推荐结果集中在桌面 `S20研究/01当前推荐`，原生Isaac入口在 `03原生环境与参考`。

- [当前执行状态](active_current.json)
- [完整技术流水](TRACK.txt)
- [论文和源码阅读范围](placement_source_review.json)
- [历史交接](handoff/)

`showcase/` 内的README、媒体和参数摘要可整体用于后续GitHub展示；原始实验、失败和权重仍保留在工作区。
