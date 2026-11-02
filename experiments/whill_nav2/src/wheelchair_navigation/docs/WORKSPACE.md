# 工作区目录与开发规则

Nav2与RDS共享控制后续开发先读同目录 `NAV2_RDS_HANDOFF.md`；当前眼控独立于Nav2导航运行时，具体操作与历史证据见 `DEMO_HANDOFF.md`。

```text
Shaki_ws/
└── src/
    └── WheelChair_ws/
        ├── src/
        │   ├── wheelchair_navigation/        导航配置与交付ROS包
        │   │   ├── profiles/
        │   │   │   └── turn_memory/
        │   │   ├── launch/                  通用Nav2启动
        │   │   ├── behavior_trees/          重规划/恢复BT
        │   │   ├── motion_primitives/       Lattice运动原语
        │   │   ├── calibration/             标定快照与来源清单
        │   │   ├── scripts/                 操作者入口
        │   │   ├── support/first_run/       原启动与诊断工具
        │   │   ├── docs/
        │   │   └── visuals/
        │   ├── whill_controller/            ESP32速度映射/ARM接口
        │   ├── wheelchair_shared_control/   连续横线意图/RDS适配/网页/二维仿真
        │   ├── rds_vendor/src/              rds_core与rvo2_lib
        │   ├── eye_control/                 原眼动标定/相机/阻尼映射
        │   ├── navigation2/                 原基础Nav2源码，保留回退
        │   └── nav2_overlays/               已验证独立覆盖层源码
        │       ├── COLCON_IGNORE
        │       ├── navigation2_humble/      当前更新Nav2源码
        │       ├── rotation_shim_path_orientation/
        │       ├── rotation_shim_deceleration/
        │       ├── mppi_acceleration_candidate/
        │       └── smac_fullfootprint_candidate/
        ├── build/                           原基础工作区编译产物
        ├── install/
        ├── log/
        ├── runtime/rds/{build,install,log}/  眼控独立构建
        ├── runtime/nav2_humble_update_20261031/
        │                                   更新覆盖层已验证build/install/log及历史材料
        ├── records/                         验证结果、bag、历史诊断
        ├── docs/
        └── scripts/                         指向ROS包内的短入口
```

`nav2_overlays/COLCON_IGNORE`防止多个同名Nav2包被一次colcon扫描到。新代码放`src`，原始观测与运行证据放`records`，图和说明放对应文档目录。新增功能应作为ROS包或明确的覆盖层，不继续在Shaki顶层堆日期目录。

当前旧更新目录`/home/whill/Shaki_ws/nav2_humble_update_20261031`是指向`WheelChair_ws/runtime/...`的兼容链接；旧`first_run`、接口源码路径、部分配置/运动原语也指向标准位置。旧路径仍被既有二进制、安装脚本和YAML引用，不能直接删这些链接。迁移清单逐项记录旧路径、新路径、文件SHA；参数未因整理而调优或覆盖。

更新源码与原基础源码是不同版本：基础commit `a097086719c88f781aa59788eca29ac6ca5e56db`，当前更新commit `3c3db59d6969d8ecee8e68468693d006397f4a0c`，均存在已记录的本地修改。源码快照包含当前修改，不把commit单独当完整版本。

## 构建

配置包可独立构建，不重建整套导航：

```bash
source /opt/ros/humble/setup.bash
cd /home/whill/Shaki_ws/src/WheelChair_ws
colcon build --base-paths src/wheelchair_navigation \
  --packages-select wheelchair_navigation --executor sequential
```

修改当前Nav2覆盖层时使用其原构建入口`runtime/nav2_humble_update_20261031/build_update.sh`；该入口通过兼容链接找到位于`src/nav2_overlays/navigation2_humble`的真实源码，保持原build/install路径。RotationShim减速覆盖层与MPPI/Smac候选不能混为同一层：候选源存在不代表当前加载。默认第三版加载路径以`profiles/turn_memory/env.sh`为准。**不要为了目录整洁直接搬移CMake缓存、复制install到新机后假定可用。**换机器需重新准备依赖并构建。

ESP32控制器是Python包，原symlink-install仍指向标准源码位置；编辑后需要重启接口进程才生效。ARM保持补丁默认关闭，由turn_memory的`latched_arm:=true`启用。

标定快照保留来源及SHA，当前传感器服务继续读取原共享传感器工作区的有效配置。复制快照不等于已切换输入；不要同时修改两份后不更新清单。师兄的源工作区未迁移、未改动。

## 回退与记录

2026-11-02用户授权后只保留turn_memory，旧lattice/hybrid025 profile及安装副本、相应已迁移旧启动链接已删除；用`nav2.sh turn_memory restart`重载。保持式ARM补丁原件在`runtime/nav2_humble_update_20261031/arm_latch_20261101/backup_before_install/`。当前导航仍依赖共享runtime、Lattice原语、BT和编译覆盖层，均保留。旧迁移别名表是历史记录，不保证被用户要求删除的入口仍可用。

本机`source/`为独立Git仓库，记录源码快照；`evidence/source_revisions.json`及`.patch`保存上游commit与本地改动。每次后续改配置，应记录版本、原因、验证场景，不用“最新版”替代可复查的文件SHA。

配置包维护人email使用`whill@example.invalid`占位，不代表已确定项目公开维护者；对外发布前由项目负责人填写。

2026-11-01 16:19：传感恢复后已重启到统一demo服务，保持式ARM参数读回true，当前进程已加载。

2026-11-02：眼动源码迁至`src/eye_control`，旧`Shaki_ws/src/eye_calibration_ws/src/eye_control`保留兼容链接。新增`scripts/rds.sh`短入口，实际脚本在`src/wheelchair_shared_control/scripts/rds.sh`。原Nav2构建/覆盖层不移动，当前操作交接以`docs/DEMO_HANDOFF.md`为准。旧记录中的运行状态不能代替当前服务实查。
