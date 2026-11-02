# WHILL Nav2 导航源码交接

2026-11-03，Jetson AGX Orin / Ubuntu 22.04 / ROS 2 Humble / Cyclone DDS。当前保留的导航配置为 **turn_memory：Smac Lattice + RotationShim + MPPI + ObstacleLayer记忆**。

此目录是现机源码、参数和标定清单快照；原生Nav2控制链没有接入新眼控RDS速度求解器。具体边界与共享层变更见 [审核记录](docs/NAV2_ISOLATION_AUDIT_20261103.md)。源码归档不代表本次已重新启动或验证真实轮椅。

## 项目说明入口

按以下顺序阅读，可以从底盘接入一直看到上层导航：

1. [系统架构与工程路线](docs/SYSTEM_ARCHITECTURE.md)：非EDU轮椅的接口条件、完整硬件/软件链、感知与反馈，以及各模块职责。
2. [单片机、蓝牙与速度执行](docs/LOW_LEVEL_CONTROL.md)：固件与烧录、离散方向指令、BLE HID、虚拟摇杆、精细协议、连续速度映射、标定与停止逻辑。
3. [汇报与配图框架](docs/PRESENTATION_FRAMEWORK.md)：可直接扩展为Presentation的叙述顺序、图位、图注与证据要求。

眼控作为下一阶段上层输入保留接口位置；本轮只完善文本，不上传新眼控/RDS实现。后续提供现场图片、截图与录屏后，再按配图清单补入。

## 内容

- `src/wheelchair_navigation/`：唯一导航profile、八顶点车体、前角mask、BT、运动原语、启动和标定来源文档。
- `src/whill_controller/`：实际共用的底盘cmd_vel适配、HID标定、ROS接口；保留原包作者与许可证声明。
- `src/nav2_overlays/`：实际环境使用的两层RotationShim源码；COLCON_IGNORE避免全工作区扫描同名包。
- `upstream/navigation2.repos`、`patches/navigation2_humble.patch`：固定Nav2上游版本和全部本地tracked/untracked源码补丁。
- `docs/jetson_source_inventory.json`、`baseline_comparison.json`：现场文件SHA256和与2026-11-01导航源码快照的比较。
- `support/site_runtime/`：现机Nav2覆盖层构建入口及隔离仿真DDS环境原件，保留现机绝对路径语义。

新眼控包、RDS源码、学习策略、构建产物、原始bag/视频/完整运行日志没有纳入这次导航上传。原包的历史诊断脚本与文档作为源码保留，只有`profiles/turn_memory/navigation.yaml`与其启动链代表当前导航配置。

## 导航链路与参数

```text
双MID360过滤点云 -> scan -> 两个前角遮罩 -> scan_map / memory_obstacles
Odin -> odom/TF                         -> local/global costmap
RViz目标 -> BT / SmacLattice -> RotationShim / MPPI
        -> VelocitySmoother -> CollisionMonitor -> cmd_vel_adapter -> ESP32 / WHILL
```

local/global为10×10m、5cm分辨率，膨胀0.5m、系数2。车身主体x[-0.18,0.62]、y±0.28，窄踏板x[0.62,0.77]、y±0.13；左右前角各15×15cm回波遮罩，区域内真实外物也会被遮掉。其余车体与FootprintApproach扫掠仍检查碰撞，不把遮罩描述成障碍分类器。

前进上限0.536622219601921m/s、倒退下限-0.40367094263942477m/s、角速度±0.8108854254075847rad/s；线加减速度±0.33m/s²、角加减速度±0.8rad/s²。控制/速度平滑20Hz，地图更新10Hz，重规划请求10Hz；请求频率不等于每次求路保证100ms完成。终点位置容差0.30m、朝向20°，StoppedGoalChecker。完整参数以YAML为准。

当前navigation.yaml SHA256：
`89f607f43e419bde17a0aaeca00465befaa6c425bbdd510aca97857510ce4cc8`。

## 取回标准Nav2源码

在本目录执行以下命令，只下载并恢复源码，不编译、不启服务、不驱动车辆：

```bash
bash tools/restore_navigation2.sh
```

脚本固定上游commit、应用本地补丁，并逐文件核对现场SHA256。目标目录已存在时退出，不覆盖本地修改。上游Nav2没有复制整树进本私有仓库，来源和许可证保持可追溯。

## 现有Jetson入口

现机标准工作区为 `/home/whill/Shaki_ws/src/WheelChair_ws`。在已安装依赖的原机器上：

```bash
WS=/home/whill/Shaki_ws/src/WheelChair_ws
bash "$WS/scripts/sensors.sh" start
# 切回导航前需释放眼控对底盘的占用
bash "$WS/scripts/rds.sh" stop
bash "$WS/scripts/nav2.sh" turn_memory start
```

底盘由现场操作员使能，通过RViz 2D Goal Pose导航。上述命令是现机入口，不是仓库克隆后的通用安装器；归档过程未执行它们。

## 构建与外部依赖

导航配置包在已有ROS环境下可单独构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src/wheelchair_navigation --packages-select wheelchair_navigation
```

Nav2与RotationShim覆盖层构建参考`support/site_runtime/build_update.sh`、原`profiles/turn_memory/env.sh`及原工作区文档。脚本仍引用现场runtime/install，不能直接在新机器照搬。该仓库不打包已编译ROS/Jetson/CUDA库。

现机还依赖`/home/whill/eye_ctrl_wheelchair/wheelchair_shared_ws`内的`wheelchair_hardware_bridge`、`wheelchair_nav2_pipeline`、`wheelchair_control`、`wheelchair_interfaces`，原传感器驱动工作区，以及`/home/whill/Shaki_ws/esp32_velocity_interface`的run/set_output_mode脚本和运行状态。它们是独立共享依赖，不是本目录内的新RDS。全新机器的独立安装与真车复测尚未完成。

现场标定原件与只读快照的关系见`src/wheelchair_navigation/docs/CALIBRATION_INVENTORY.md`；不要把归档快照自动当在线加载源。历史诊断目录的五个绝对symlink未上传，避免指向不随源码交付的原始观测；完整原始校准证据树也未重复打包。

## 验证范围

此次核验源码/配置边界、来源补丁及文件一致性。2026-11-02隔离Nav2几何/碰撞监控验证原件见`docs/geometry_validation_20261102.json`，它不是2026-11-03真车重新验收。当前速度映射、标定及底盘真实跟踪能力仍以现场数据为准。
