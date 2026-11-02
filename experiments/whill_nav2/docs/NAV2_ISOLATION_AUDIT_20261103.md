# 原生Nav2与眼控改动边界核验

日期：2026-11-03。范围：只读Jetson源文件、启动参数、Git工作树及导出文件；没有启停生产服务，没有运动测试，也没有回滚远端代码。

## 核验结论

当前原生Nav2仍为turn_memory的SmacLattice + RotationShim + MPPI，控制/规划/collision配置未被新眼控shared_solve、TurnRecovery或AssistedTeleop替换。

1. 主配置SHA256为`89f607f43e419bde17a0aaeca00465befaa6c425bbdd510aca97857510ce4cc8`，与2026-11-02用户授权的窄踏板、前角遮罩版本一致。
2. 当前官方Nav2源文件与2026-11-01导航源码Git索引快照比较，1392个文件内容相同；两层RotationShim各8个文件相同。编辑器tasks配置不属于源码比较/恢复范围。结果见`baseline_comparison.json`。
3. active Nav2的上游HEAD为`3c3db59d6969d8ecee8e68468693d006397f4a0c`。相对上游的本地变更仍为VelocityPolygon回移植和SmacLattice地图范围检查；补丁已包含新增的velocity_polygon.hpp/.cpp，反向apply --check通过，未漏掉untracked源码。
4. production导航YAML、launch和nav2.sh没有引用`wheelchair_shared_control`、`shared_solve`或`/shaki/rds`。眼控独立Costmap2DROS及自定义速度搜索不在这条原生导航链上。

## 已授权的导航变化

与2026-11-01快照相比，导航包的代码/配置差异主要是：local/global八顶点车体、三个StopZone分区同步切角、官方LaserScanBoxFilter两个15×15cm遮罩、map/CollisionMonitor统一消费masked scan、只保留turn_memory入口及相应说明。原速度、加速度、膨胀、goal容差和重规划配置保持。所有差异文件列于baseline_comparison.json。

底盘cmd_vel_adapter.py与原快照一致，速度映射/HID标定原件也未见此次变化。共享controller_node.py和velocity_interface.launch.py增加了可选unified_ui支持：默认false；Nav2现有start_navigation.sh未传true，新订阅/网页ARM分支由该开关启用。不是“所有共享文件完全未动”，也不等于原生导航已接入RDS。

共享传感启动start_sensors.sh增加OpenCV库路径和OPENBLAS/OMP单线程调度；DDS配置内容仍与导航快照相同。它们影响共同运行环境，不能笼统说眼控与导航完全没有共享依赖；本次没有把这些已存在的改变擅自回滚。

## 验证边界和交付

归档提供当前导航包、共享底盘包、实际RotationShim覆盖层、固定上游commit、完整本地补丁、标定清单与SHA清单。共享传感器/ROS依赖仍按README中的现机路径安装，未把其他同学工作区或新RDS工程整体复制进来。

没有把源文件比对当作当前真车验收。此轮仅做静态导出和来源核查；原机新运行是否正常需要传感器、TF、加载参数和现场测试共同确认。
