# WHILL 导航演示与开发交接

2026-11-02更新：仅保留turn_memory导航入口。车身改为窄踏板八顶点轮廓；两个前角15×15cm自回波在地图和碰撞监控前统一遮罩。眼控/RDS结合开发交接见工作区 `docs/NAV2_RDS_HANDOFF.md`。下文历史设备记录不表示当前服务已启动。

对应Jetson：`whill@192.168.8.115`，ROS 2 Humble。交接日期：2026-11-01。

标准工作区：`/home/whill/Shaki_ws/src/WheelChair_ws`。导航版本为`turn_memory`：Smac Lattice + RotationShim + MPPI + ObstacleLayer记忆。

## 演示启动

在Jetson桌面终端运行：

```bash
bash /home/whill/Shaki_ws/src/WheelChair_ws/scripts/sensors.sh
bash /home/whill/Shaki_ws/src/WheelChair_ws/scripts/nav2.sh
```

第一条只启动传感器并检查两个MID360和Odin新鲜数据，不打开RViz；第二条启动默认第三版、RViz和底盘接口。点ARM后，通过RViz的2D Goal Pose发送目标。服务状态不能代替数据新鲜度检查。

重新加载导航配置（先停止占用底盘的眼控服务）：

```bash
bash /home/whill/Shaki_ws/src/WheelChair_ws/scripts/rds.sh stop
bash /home/whill/Shaki_ws/src/WheelChair_ws/scripts/nav2.sh turn_memory restart
```

检查/停止：

```bash
bash /home/whill/Shaki_ws/src/WheelChair_ws/scripts/nav2.sh turn_memory status
bash /home/whill/Shaki_ws/src/WheelChair_ws/scripts/nav2.sh turn_memory stop
bash /home/whill/Shaki_ws/src/WheelChair_ws/scripts/sensors.sh status
```

当前设备和验证结论见`evidence/FINAL_STATUS.json`；物理供电丢失不能通过重复启动软件恢复。新服务统一名称`shaki-nav2-demo.service`，旧路径、旧启动入口保留兼容。

## 交接内容

- `docs/WORKSPACE.md`：目录规范、源码/编译产物关系、回退与开发方式。
- `docs/CONFIGURATION.md`：当前参数、数据流与模块职责。
- `docs/CALIBRATION_INVENTORY.md`：实际加载的标定、坐标系、路径和哈希。
- `docs/DEMO_AND_PRESENTATION.md`：演示顺序、汇报素材与证据边界。
- `visuals/navigation_pipeline.svg`：可用于PPT的矢量结构图；PNG提供预览。
- `visuals/real_rotation_tracking.png`：前一轮真实旋转的命令/里程计曲线。
- `source/`：本轮源码、三套配置及接口快照；独立Git跟踪。
- `evidence/`：迁移映射、配置哈希、仿真结果、版本补丁及历史实车记录。

本机文件是版本快照，Jetson上的标准工作区是下一步开发入口。录屏由用户自行保存；每次录制建议同时记录profile名称和源码快照commit，避免画面与配置混淆。
