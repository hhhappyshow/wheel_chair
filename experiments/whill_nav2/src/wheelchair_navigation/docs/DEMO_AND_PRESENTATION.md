# 演示与汇报材料

建议展示顺序：硬件与后轴坐标系→双雷达/Odin数据流→turn_memory导航与窄踏板轮廓→原地旋转与前进倒退→真实速度曲线→眼控共享控制与地图长期维护。

1. 运行`sensors.sh`，看到两台MID360和Odin新鲜数据检查通过。
2. 运行`nav2.sh`，确认默认turn_memory及所有Nav2 lifecycle active。
3. 在底盘窗口ARM，查看串口/BLE、是否输出暂停及原因；在RViz给目标。
4. 录制目标、路径、车体框和实际轮椅运动，若需要速度说明可展示此前实测曲线。
5. 记录本次profile、配置SHA、视频文件名与时间。录制视频本身由用户保存。

## 可复用材料

- `visuals/navigation_pipeline.svg/png`：数据流结构。
- `visuals/real_rotation_tracking.png`：真实Odin角速度反馈，含平滑命令与最终碰撞监控输出。
- `docs/CONFIGURATION.md`：当前单套导航参数与遮罩数据流。
- `docs/CALIBRATION_INVENTORY.md`：传感器与标定来源。
- `evidence/validation_summary.json`：目录整理后逐版本仿真结果。
- `evidence/source_revisions.json`、`migration.json`：实现与文件管理可追溯证据。

## 表述范围

已完成的真实测试包括官方Spin左右转、普通NavigateToPose同位置旋转。±60°目标普通导航实测+61.38°/−70.27°，符合当前±20°容差；临时测试角速度限0.3rad/s，结束恢复原上限。此前存在一次无响应试次、一次短暂碰撞预测介入，日志保留。不能称所有窄通道、最高速、所有转向都已验证。

本轮整理后仿真测试验证启动、前进倒退和第三版原地旋转，不能代替整理后真实硬件复测。传感器缺电时应明确显示blocked，不以服务active冒充可录制演示。

当前Nav2启动的是`eye_control_enabled=false`的原生导航接口。眼控采用连续速度意图加RDS，后续复用Nav2地图能力；不把眼位转为一串NavigateToPose。眼控端到端状态以自身入口和实时数据为准，本交接不把RViz目标测试写成眼控闭环验证。切换模式须释放底盘控制权，开发交接见工作区`docs/NAV2_RDS_HANDOFF.md`。

下一步地图：在稳定坐标下维护长期概率占据图，结合短期动态障碍层。当前只部署滚动窗口记忆；OctoMap/STVL/ROG-Map的研究不能写成已部署长期地图。跨重启复用需明确定位坐标一致性。

## 录屏索引模板

|录制时间|视频文件|profile/源码commit|目标动作|结果与备注|
|---|---|---|---|---|
|待用户录制|待填写|turn_memory / 待填本机snapshot commit|原地转向、前进/倒退|填写实际表现|
