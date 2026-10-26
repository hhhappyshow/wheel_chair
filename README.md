# CPS Intern · S20 在线装箱与抓放

当前源码快照：2026-10-26，Asia/Shanghai。私有研究仓库，包含仿真、规划及真机适配，不是整个CPS工作区，也不是免配置的独立安装包。

## 当前流程

```text
回观察位 → 当轮真实RGB-D → YOLO11m-seg → 箱顶/长短边/高度 → 中心吸取
                                      ├→ 异步空间候选与持箱/空载求路
                                      └→ 贴合 → 吸附确认 → 抬升 → 示教入舱
入舱交接 → 向深处伸入 → 左右横移 → 下放 → 释放确认
         → 官方MoveL竖直上抬20cm → 独立空载返回 → 提交占据 → 下一件
```

- 只读取当前来箱与已提交占据，不读取下一件，不按箱序号背固定落点。
- 同层、同一有效深度组先横放，从右向左贴紧；保留90°补窄位及向入口扩展备选。
- 当前是贪心候选、数值IK和CAD/FCL规划，不是RL，不是完整GSIP/KCSVSDF复现；存在cuRobo适配代码不代表当前每次求路都使用cuRobo V2。
- 占据采用规划释放位假设，不是重新观测落点。几何支撑不是力反馈或动态落稳。
- 最新四份实测输入已离线生成抓取、放置与空载返回参考；不据此宣称最新参数四件真机已通过或普遍无碰撞。

## 环境与依赖

以下命令在配置好的Ubuntu主机、仓库根目录执行；真实相机和SDK通过Jetson访问。

```bash
./s20 help
./s20 config
./s20 run --help
```

主机历史解释器为`reproductions/nvidia_curobo_v2_20261023/.venv/bin/python`，其目录名不是当前算法的证明。需要Newton/Warp、PyTorch、NumPy/SciPy/OpenCV/PyYAML、python-fcl、Ultralytics和匹配的CAD。Jetson另需现有控制器SDK、ROS Humble与RealSense DDS。环境、SDK二进制与权重不上传。

同学源码和模型保持`reproductions/ludan_greedy_20261023`相对路径；Newton运行时在`experiments/s20_newton_update_20261020/runtime13`。外部commit见[source_manifest.json](source_manifest.json)。这些依赖未打包，全新机器独立安装尚未验收，不能只复制deployment就称可运行。

YOLO11m-seg权重需按上游许可取得并放入`experiments/s20_online_mixed8_20261024/perception/weights/yolo11m-seg.pt`。

## 仿真与离线规划

### A. 当前部署同款规划，无硬件动作

使用现场已保存的观测目录，需包含`observation/observation.json`、`grasp.json`、`controller_snapshot.json`和解析模型；原始观测不随源码上传。

```bash
./s20 plan \
  --observation /absolute/path/to/round_1/observation \
  --output experiments/s20_online_mixed8_20261024/runs/offline_current
```

已有前序时加`--scene-state /absolute/path/to/scene_state.json`；否则按空舱。不要混用旧无膨胀占据与当前膨胀代理。

可加`--preview-rounds 4`重复当前观测并更新占据，但它是重复同一箱尺寸的规划，不是四次新感知、物理仿真或真机执行。输出包括`result.json`、`session.log`、每轮pick/place、候选、参考与组合recipe；不发送运动或泵IO。

### B. Newton四件合成感知/FK仿真：历史独立入口

```bash
experiments/s20_online_mixed8_20261024/s20-contact-sim \
  --arrivals four20 --size-noise 0 --planning-mode async \
  --weights experiments/s20_online_mixed8_20261024/perception/weights/yolo11m-seg.pt \
  --pose-json /absolute/path/to/saved_pose.json \
  --waypoints experiments/s20_online_mixed8_20261024/deployment/pick_waypoints.json \
  --site-urdf experiments/s20_online_mixed8_20261024/deployment/site_robot.urdf \
  --camera-calibration /absolute/path/to/saved_capture_result.json \
  --record-decisions \
  --output experiments/s20_online_mixed8_20261024/runs/newton_four
```

姿态文件含起始`q_rad`；相机文件含已保存D555 CameraInfo/driver transforms，不访问真实相机。还支持`--arrivals three20-one24`或`four20-two24`及`--size-noise .10`。

该入口为`four20_sparse_cycle.py`：合成场景、理想吸附、FK跟随与冻结释放，不是接触动力学闭环，**不自动继承当前真机profile的全部新参数**。当前5cm膨胀、20cm撤离、伸入后横移的验证以A入口部署规划为准。本次更新未重新执行B入口。

### C. 保存轨迹的原生渲染

仅适用于有`trajectory.npz`、匹配模型和result的Newton运行目录，不适用于普通硬件recipe。

```bash
PYTHONPATH="experiments/s20_newton_update_20261020/runtime13:experiments/s20_greedy_async_20261023" \
reproductions/nvidia_curobo_v2_20261023/.venv/bin/python \
  experiments/s20_online_mixed8_20261024/render_reference.py \
  --run-folder experiments/s20_online_mixed8_20261024/runs/newton_four \
  --site-urdf experiments/s20_online_mixed8_20261024/deployment/site_robot.urdf \
  --suitcase-assets --cuboids --camera-panel \
  --output experiments/s20_online_mixed8_20261024/runs/newton_four_media
```

需要可用OpenGL/EGL和匹配资产；读取保存状态，不重新规划、不证明物理成功。

## 四件真机：人工与DI0系统监督

空舱新会话前确认机械臂空载、现场可控、已使能，相机和外部气源正常；现场保留急停监督。脚本不自动清故障、使能或恢复悬挂箱体。

### 人工监督版

```bash
./s20 run --rounds 4 --vacuum-feedback operator --execute
```

- `1`确认空载分离后运动；`2`确认实际吸稳后抬升，入舱交接再确认；`3`确认松开且有支撑后撤离。
- 不以DI0作为吸稳门槛，不提供自动失吸监测；DO=10只是输出状态，不是压力测量。

### DI0系统监督版

```bash
./s20 run --rounds 4 --vacuum-feedback digital --execute
```

- 当前约定DI0高有效：1吸稳、0释放。恢复上线由现场报告，仍需确认随真实吸附/释放变化，不是仅常亮；禁止伪造DI0。
- 空载重启DI0=1时拒绝观察位返回。
- 吸附后等DO0=1/DO1=0且DI0=1稳定1秒，最多30秒；未确认不抬升。
- 运动反馈轮询中DI0变0时停止、保留吸附，不自动松泵/重试。
- 释放后等DI0=0才撤离；无1/2/3输入，但仍须现场监督。
- DI0不是落稳、底面支撑、力矩检测或独立机器人安全系统。

### 共用代码和IO时序

两版共用`online_session.py`和`native_cycle_bridge.py`，只切换`--vacuum-feedback operator|digital`，感知/规划/示教/路径/速度相同，等待时长随人工操作或DI反馈变化。

DO统一：吸附先DO1=0再DO0=1（10）；释放先DO0=0再DO1=1（01），0.2秒后DO1=0（00），随后分别等人工或DI确认。人工版原有时序不变；数字版统一后的新时序尚未实机验证。

## 当前参数

配置集中在[deployment/current_profile.json](experiments/s20_online_mixed8_20261024/deployment/current_profile.json)，新会话加载，不热改活动进程。result记录profile和实际CLI选项，CLI启动选项优先。

| 项目 | 当前值/语义 |
| --- | --- |
| 检测 | YOLO11m-seg，conf0.15，any不限类别；仍需检测框，取物区仅布置明确单目标 |
| 采集 | 目标6组帧，截止35秒；不是固定等待35秒 |
| 建模/吸取 | 当轮RGB-D拟合长短边、高度及顶面，最大面中心吸取，固定件优先world +X |
| 贴合偏置 | world-Z -0.02m；位置控制，不是力控 |
| 规划包络 | XY每侧+5cm，长宽各+10cm；高度、原测尺寸和吸取中心不变 |
| 支撑 | 去掉XY余量后实际底面至少80%覆盖且中心支撑；不把虚拟边缘算材料 |
| 高度归并 | 5cm同层容差尚未启用，不把较低箱顶当实际接触 |
| 候选 | 最多10个；同层→有效深度组→横放→右往左→贴面/紧凑 |
| 范围 | 1.7m顶部/TCP可达球粗筛，仍须IK；箱顶相对舱底最高1m |
| 规划 | endpoint-first批量IK、约8cm节点、插值与CAD检查；预览不等于整臂通过 |
| 放置 | 示教入口→深度伸入→横移→下放，三段官方MoveL |
| 撤离 | 释放确认后world-Z +20cm，官方MoveL，再独立空载返回 |
| 执行 | 官方MoveL/MovePathJ，不是默认ServoJ逐点发送 |
| 速度 | native-speed-scale2；MoveL60mm/s、100mm/s²；MovePathJ ratio约0.1333，不是实测关节峰值或全局80% |
| 场景更新 | 执行完成提交规划释放位，不重新观测落点 |

各箱每侧5cm意味着相邻真实表面可能约10cm再加原候选间隙；容量会下降、可能更早叠放，不能误解为两件总共只留5cm。

## 代码归属与复用

“本项目代码”指项目集成/适配和Codex辅助开发，不等于每行均由单一作者原创。继承段落尚未逐行核清，不作全原创声明。

| 分类 | 文件或组件 | 归属处理 |
| --- | --- | --- |
| 本项目流程适配 | deployment中的online_session、观测转换、参考准备、native桥、profile、根目录s20 | 感知→异步规划→SDK事件的集成；不将底层SDK计为原创 |
| 本项目规划适配 | compact_prior、straight_place、顶面/吸盘包络 | 贴边候选、排序、余量、IK/CAD和异步衔接；依赖上游几何与模型 |
| 同学 | ludan0414/RoboticArm | hull/候选/route函数、模型CAD、轨迹helper被引用；未整体上传，commit见manifest |
| 旧项目继承 | tools/three_box_original | 旧三箱SDK/执行接口，逐段作者未完全核定，不改称新原创 |
| NVIDIA/第三方 | cuRobo、Newton/Warp、Ultralytics、FCL及数值库 | 外部依赖/比较后端，各自许可证有效，YOLO不是本项目训练原创 |
| 设备组件 | 官方控制器SDK、RealSense DDS、ROS | 外部安装依赖，二进制不上传 |
| 现场资产 | scene_tf.yaml、site_robot.urdf、pick_waypoints.json | 特定安装/示教，不是通用出厂配置，资产权利需独立核对 |

[THIRD_PARTY.md](THIRD_PARTY.md)与[source_manifest.json](source_manifest.json)记录来源和commit；不对继承代码统一重新授权。

## 文件与证据入口

- `deployment/`：主机/Jetson、相机、配置、抓放准备、SDK执行；诊断脚本不是主启动入口。
- `perception/`：YOLO、深度顶面与工具包络。
- `straight_place/`：候选、独立worker、持箱/空载规划与可视化。
- `experiments/s20_greedy_async_20261023/`：继承的项目几何/仿真依赖。
- [records/S20/README.md](records/S20/README.md)：详细模块导航。
- [研究日记](records/S20/RESEARCH_DIARY.md)：成功、失败、改动、证据范围与下一决定。

部分证据链接仅在原现场工作区有效。源码库不含全部图像/视频/原始日志、密码密钥、环境、第三方整树、权重、SDK二进制、其他机器人和旧32/41箱项目。
