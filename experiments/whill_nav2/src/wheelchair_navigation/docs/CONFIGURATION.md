# 版本、参数与数据流

2026-11-02：当前只保留turn_memory。旧lattice/hybrid025两个profile及安装副本已删除，导航所需Smac Lattice算法、运动原语、BT和共享覆盖层保留。

|项目|Turn-memory|
|---|---|
|Planner|Smac Lattice，464条原语/32方向|
|原地转向执行|RotationShim接管起步/终点方向|
|Controller|RotationShim包装MPPI DiffDrive|
|路径平滑|Smac内置平滑；SmootherServer启动但当前BT未调用|
|速度平滑|20Hz OPEN_LOOP|
|线加减速度|±0.33m/s²|
|角加减速度|±0.8rad/s²|
|终点位置/角度|0.30m/20°|
|StoppedGoalChecker|线速≤0.05、角速≤0.1|
|地图记忆|官方ObstacleLayer运行时记忆|
|allow_unknown|false|
|保持式ARM|启动参数latched_arm=true|

速度限值：前进0.536622219601921m/s、倒退0.40367094263942477m/s、角速度0.8108854254075847rad/s。倒退上限写成负的`vx_min`。原语文件为`motion_primitives/diff_025_32_candidate.json`。

默认第三版RotationShim：起步偏差45°接管，10°以内交回MPPI，前采样0.3m，终点朝向接管开启；`closed_loop=false`表示角加速斜坡基于前一命令，并不是不使用Odin位姿反馈。加载上游路径姿态和余角减速修补。速度限幅不等于真实轮椅已经无误差跟踪，当前底层仍有方向差异和启动延迟；没有jerk/S曲线保证。

## 规划与地图

BT重规划请求10Hz，控制和速度平滑20Hz；10Hz是请求配置，不是对所有地图保证每次规划100ms内完成。原始实测路径更新常约7–10Hz；重规划、路径平滑、速度平滑是不同环节。

global/local地图10×10m、分辨率0.05m，两图inflation_radius=0.5m、cost_scaling_factor=2.0。当前footprint为八顶点窄踏板：主体x[-0.18,0.62]、y±0.28，前踏板x[0.62,0.77]、y±0.13，padding=0。global track_unknown=true，local=false；planner仍拒绝未知目的格。

地图source链：两台MID360点云→滤波/标定变换→`/wheelchair/scan`→官方LaserScanBoxFilter遮罩→`/wheelchair/nav2/scan_masked`→scan_map观测占据图；同一遮罩输出再经过原记忆自回波过滤成为`/wheelchair/memory_scan`供ObstacleLayer使用。Odin用于odom/TF，不作为额外点云障碍源。记忆保留窗口内已观察状态，`observation_persistence=0`并不清除地图历史；移出窗口、清图、重启会丢失运行时记忆。

`profiles/turn_memory/self_mask.yaml`仅新增两个base_link矩形：x[0.62,0.77]、y[0.13,0.28]及y[-0.28,-0.13]。原始点云/scan不被修改，只有Nav2消费的扫描被遮罩，命中点为NaN，不当远处自由射线。区域内真实外物也不可由被遮掉的点检测；这不是“只消除自回波而自动识别真实障碍”。区域外点、剩余车身及转动后的完整轮廓仍参与检测。原scan_map与memory_scan_filter已有自过滤边界保持，本次不扩大。

地图颜色：紫红=内部254障碍，青色实心带=253内接半径膨胀，外围蓝红渐变=1–252普通代价，透明=0自由，灰绿=255未知。激光点也画成青色，不能按颜色把点和实心带混为一个来源。外层梯度不是实体墙，但costmap本身参与规划/控制碰撞检查。

## 运动输出链

```text
NavigateToPose / RViz goal
  -> BT planner (Smac internal smoothing)
  -> RotationShim / MPPI
  -> /wheelchair/nav2/cmd_vel_raw
  -> VelocitySmoother
  -> /wheelchair/nav2/cmd_vel_smoothed
  -> CollisionMonitor
  -> /wheelchair/nav2/cmd_vel
  -> cmd_vel_adapter -> /whill/controller/navigation
  -> calibrated HID mapping -> ESP32 -> Bluetooth/iPad -> WHILL
```

CollisionMonitor读取`/wheelchair/nav2/scan_masked`。StopZone三个速度分区均改为同样切角轮廓，后界仍保留原来的x=-0.15，其他边界同车身，max_points=5不变。FootprintApproach读取local_costmap八顶点车体框，预测2秒、步长0.1秒、max_points=2。真实障碍仍可触发停车，但停车是输出速度为零，不是deactivate整套Nav2；输入恢复后继续按当前新鲜数据计算。它可以覆盖平滑指令立即降速/停车；不把速度平滑后的每一步都说成最终输出必连续。

ARM保持是授权开关与输出暂停分离：新第三版超时/断连等只暂停运动并取消当前目标，保持绿色，状态中提供暂停原因；恢复后新目标或新鲜零速解除输出等待。手动再次ARM才关闭。进程重启仍默认关闭。旧接口进程未重载时不具备这个行为。

## 可修改项

- `profiles/*/*.yaml`：Planner、MPPI critics、goal checker、速度平滑、costmap、碰撞层。
- `behavior_trees/navigate_lattice_10hz.xml`：重规划频率、恢复流程（当前BT不含SmoothPath）。
- `motion_primitives/*.json`：Lattice原语；修改后必须检查分辨率/运动模型和碰撞采样。
- `profiles/turn_memory/self_mask.yaml`：地图/碰撞监控共同前角遮罩；`memory_scan_filter.yaml`保留原记忆输入过滤。不能当作全局关闭碰撞检测。
- `whill_controller`：接口/ARM/HID映射；底层映射改动须同时核真实Odin速度。
- `calibration`：只读交接快照，实际加载源见独立清单；不凭文件名推断在线使用。
