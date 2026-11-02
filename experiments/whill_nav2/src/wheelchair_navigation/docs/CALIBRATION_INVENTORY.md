# Jetson 三套 Nav2：传感器、标定与上游眼控交接清单

只读盘点：2026-11-01 Asia/Shanghai。ROS 运行快照 2026-11-01T04:11:06.994405+08:00 至 2026-11-01T04:11:13.108262+08:00；标准目录解析 2026-11-01T04:12:25.500172+08:00。文件逐项时间、完整 SHA-256、旧 alias、canonical sourcepath 与结构化参数见同目录 `inventory.json`。远程只读取文件、进程、systemd 状态、DDS 参数/订阅；没有改文件、控制服务、ARM、发目标或速度。本机保存原始证据在 `calibration_evidence/`。

## 当前状态与交接边界

- 运行服务：`shaki-nav2-sensors-20261101`（PID 4007）、`shaki-nav2-turn-memory-20261101`（PID 18592）；两者为 transient user unit。Lattice / Hybrid025 的文件已核，未切换运行。
- 03:59起日志显示Odin USB断开、双雷达数据停止；实查 `eno1 DOWN`。4 秒订阅只收到 9 条静态 TF 和 4 条控制器状态，双原始/过滤点云、Odin 里程计、轮椅里程计与 scan 均未收到。服务 active 不等于传感健康。
- 状态消息当时为 `armed=true, direction=idle, serial_connected=true, bluetooth_connected=true, firmware_test_mode=false`；只是读取原状态，本轮没有操作 ARM。不能由 serial/BLE 状态推出 Odin/MID360 在线。
- 实际眼控未接入当前导航：live `eye_control_enabled=false`，无眼控节点/话题；`/goal_pose` 唯一发布者为 RViz，`/whill/controller/navigation` 唯一发布者为 `whill_cmd_vel_adapter`。
- 主线程正在整理和独立仿真复测；本文只记录本轮证据，不宣称 demo、迁移后构建、仿真或实车验证通过。

## 标准源码目录与三版本

工作区 `/home/whill/Shaki_ws/src/WheelChair_ws`；标准源码根 `/home/whill/Shaki_ws/src/WheelChair_ws/src`。已按用户最新整理重新实查 `resolve()`：

| 内容 | canonical 位置 |
|---|---|
| 导航 package | `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation` |
| 底盘接口 | `/home/whill/Shaki_ws/src/WheelChair_ws/src/whill_controller` |
| 新版 Nav2 | `/home/whill/Shaki_ws/src/WheelChair_ws/src/nav2_overlays/navigation2_humble` |
| 路径朝向 shim | `/home/whill/Shaki_ws/src/WheelChair_ws/src/nav2_overlays/rotation_shim_path_orientation` |
| 减速 shim | `/home/whill/Shaki_ws/src/WheelChair_ws/src/nav2_overlays/rotation_shim_deceleration` |
| 运行/构建产物 | `/home/whill/Shaki_ws/src/WheelChair_ws/runtime/nav2_humble_update_20261031` |
| 传感/DDS 公共入口 | `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/support/first_run` |

`nav2_overlays/COLCON_IGNORE` 实际存在，父层用于避免同名包重复。新版 Nav2 HEAD `3c3db59d6969d8ecee8e68468693d006397f4a0c`，有本地改动；原 `src/navigation2` HEAD `a097086719c88f781aa59788eca29ac6ca5e56db` 仍保留，也有本地差异。不能只靠 HEAD 或 1.1.20 版本号认定完整源码一致。

| 版本 | canonical 入口 | canonical 配置 | 当前运行 |
|---|---|---|---|
| lattice | `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/profiles/lattice/start_navigation.sh` | `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/profiles/lattice/hardware_lattice_10hz.yaml` | 否，仅文件核对 |
| hybrid025 | `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/profiles/hybrid025/start_navigation_hybrid025_10hz.sh` | `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/profiles/hybrid025/hardware_hybrid025_10hz_analytic125.yaml` | 否，仅文件核对 |
| turn_memory | `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/profiles/turn_memory/start_navigation.sh` | `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/profiles/turn_memory/navigation.yaml` | 是，既有进程 |

旧 `Shaki_ws/nav2_humble_update_20261031` → `WheelChair_ws/runtime/nav2_humble_update_20261031`；旧 `WheelChair_ws/first_run` → package 的 `support/first_run`；旧 ESP32 工作区的 `src/whill_controller` → 标准 `src/whill_controller`。各 profile 旧路径继续解析到上述 canonical 文件。服务 cmdline 仍可显示旧路径，不能把 alias 当第二份源码。未改师兄/shared sensor 目录。

三版共用：双 MID360 + Odin、sensor_visualization 静态 TF/车体过滤、wheelchair_hardware_bridge、ESP32 velocity interface、domain 0 Cyclone DDS。第三版另用官方 laser_filters 的盒过滤和减速 shim overlay；第三版 memory_scan 不改变碰撞监控对原 `/wheelchair/scan` 的订阅。

## Sensor 服务入口和实际加载路径

`/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/support/first_run/start_sensors.sh` → `/home/whill/eye_ctrl_wheelchair/sensor_visualization/start_sensors.sh` → `all_sensors.launch.py mid360:=both` → 安装索引中的 dual_mid360 launch、Odin host_sdk_sample、`sensors_tf.launch.py`。sensor 启动加载 `/opt/ros/humble`、`/home/whill/livox_ws/install`、`/home/whill/odin_ws/install` 和 shared workspace/install；这几处是标准源码包外的实际依赖，不能只交付 Nav2 package 就认为传感器可复现。

- MID360 live `user_config_path=/home/whill/livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/dual_MID360.json`，canonical 为 `livox_ws/src/livox_ros_driver2/config/dual_MID360.json`。右 `.201`，左 `.202`；主机数据 IP `.15`，均属 `192.168.86.0/24`。配置 UDP 雷达端口 56100/56200/56300/56400/56500，对应 host 56101/56201/56301/56401/56501。当前网口无地址且 DOWN，不把配置地址当当前已绑定地址。
- driver：`xfer_format=0`（PointCloud2）、`multi_topic=1`、`publish_freq=10.0`（配置值）、`data_src=0`。JSON 两套 extrinsic 全零；旋转单位 degree、平移 mm，由 pub_handler.cpp 的角度换算和 `/1000.0` 可核；点云输出坐标为 m。倒装和微调只由下表 TF 应用，不能重复翻转。
- Odin：安装路径 `odin_ros_driver/share/odin_ros_driver/config/control_command.yaml` resolve 到 `odin_ws/src/odin_ros_driver/config/control_command.yaml`。源码从 package share 选取该文件；launch 的 `config_file` 覆盖未出现在 live 已声明参数中，不能说是参数服务读回了该文件内容。
- 配置包含 `use_host_ros_time=1`、`send_odom_baselink_tf=0`、`tf_extra_publish_rate=0`、`sendodom=1`、`sendimu=1`、`custom_map_mode=1`、`sendcloudrender=1`、`sendrgbundistort=0`；DTOF/RGB 的 `100` 单位为 Hz×10，不能写成 100 Hz。

## 坐标、静态 TF 和标定

`base_link` 按源码注释定义为后轴中心，+X 前、+Y 左、+Z 上；这是本次核实的配置约定，未重新机械测量。下表是 `T_parent_child`，即 `p_parent=R*p_child+t`；xyz 单位 m，RPY 为 fixed-axis rad，`R=Rz(yaw)Ry(pitch)Rx(roll)`。数值均由源码、运行命令和实际收到的 `/tf_static` 三方一致核验，四元数在 JSON 中保留。

| parent → child | xyz [m] | roll,pitch,yaw [rad] |
|---|---|---|
| `base_link` → `lidar` | `[0.45, 0.25, 0.65]` | `[0, 0, 0]` |
| `base_link` → `imu` | `[0.45, 0.25, 0.65]` | `[0, 0, 0]` |
| `base_link` → `camera_0` | `[0.457415, 0.279663, 0.63946]` | `[-1.564317, 0.012941, -1.577013]` |
| `base_link` → `seat_link` | `[0.27, 0, 0.42]` | `[0, 0, 0]` |
| `base_link` → `base_footprint` | `[0, 0, -0.036]` | `[0, 0, 0]` |
| `base_link` → `mid360_201_local` | `[0.55, -0.21, 0.32]` | `[3.141592653589793, -0.017453292519943295, 0]` |
| `base_link` → `mid360_202_local` | `[0.55, 0.21, 0.32]` | `[3.141592653589793, 0, -0.017453292519943295]` |
| `mid360_201_local` → `mid360_192_168_86_201` | `[0, 0, 0]` | `[0, 0, 0]` |
| `mid360_202_local` → `mid360_192_168_86_202` | `[0, 0, 0]` | `[0, 0, 0]` |

MID360 右侧 .201：roll=180°、pitch=-1°、yaw=0°；左侧 .202：roll=180°、pitch=0°、yaw=-1°。Odin lidar/imu 的 x=.45、y=.25、z=.65 对应前、左、上，旋转为 identity；不能据此声称真实 IMU 与 lidar 原点完全重合已测量。`base_footprint.z=-.036` 是现用地面参考，`seat_link.z=.42` 是另一参考；不要把 base_link 误写成地面/座椅原点。

Odin 运行标定路径由 `odin_calib_path.h` 的 `ODIN_CALIB_DIR → ROS_HOME/odin_ros_driver → HOME/.ros/odin_ros_driver` 决定。进程未设置前两者，HOME 为 `/home/whill`，因此主文件为 `/home/whill/.ros/odin_ros_driver/calib.yaml`。运行文件、源码文件、install alias 当前 SHA 均为 `8d61df9cb1a194266bc185739fcaa2aaeb537912c58966e039f59559fa678883`，首行设备号 `O1-P080200001`。源码先从设备取标定，主文件供 renderer/camera 参数加载，再镜像到 config。没有从 journal 获取本进程 CALIB 读回行，也未读出内存矩阵，故证据级别为源码+进程环境+当前文件一致。

`calib.yaml` 的 `Tcl_0` 是 camera←lidar（行主序 4×4；平移按 m 使用），完整矩阵：

```text
-0.00621 -0.99990 -0.01294  0.02957
 0.00656  0.01290 -0.99990 -0.01097
 0.99996 -0.00630  0.00648 -0.00716
 0        0        0        1
```

相机 `FishPoly`，1600×1296，A11=731.6419279041597、A12=0.20538432870345655、A22=731.43447332719575、u0=790.3923599698114、v0=645.3867472402917（图像坐标/像素）；完整畸变系数保留在 JSON，不替换为 pinhole 模型。相机 optical 约定 +Z 前/+X 右/+Y 下。launch 注释使用 `T_base_camera=T_base_lidar*inverse(Tcl_0)`，但实际发布的是六位小数硬编码外参，修改 calib 不会自动同步 TF。

动态 TF 由 `odom_relay` 唯一设计拥有 `odom→base_link`：输入 `/odin1/odometry_highfreq`，输出 `/wheelchair/odom`；live `source_linear_in_world=true`，先把源线速度从 world 转回源体坐标，再用 TF 杠杆臂转换 pose/twist/covariance 到后轴。时间戳保留，接收年龄门 [-.05,.3] s。`/wheelchair/imu` 标记 orientation_covariance[0]=-1，不能把 IMU identity quaternion 当姿态估计。`wheelchair_control_center_estimated` 当前偏移为零，只是同位辅助输出。当前无新 odom，未验证动态 TF 新鲜度。

## 点云、地图与时间依赖

链路：双 `/mid360_20x/points` → body_cloud_filter → `/mid360_20x/points_filtered` → cloud_to_scan → `/wheelchair/scan` → scan_map → `/wheelchair/planning_map`（odom）。Odin 点云没有接入该 scan 输入。

- 车体过滤在 rear-axle base_link：x[-.15,.80]、y[-.275,.275] m；由 body_filter.yaml 和 hardware_bridge.yaml 两处提供，live 一致。
- 地面参考 z=-.036 m、容差 ±.15 m、obstacle_height=.90 m；保留低于地面的非地面回波。源码旧 docstring 的 3 cm 与当前 live 15 cm 不同，交接以运行参数为准。legacy height slice [.217,.817] m 仅地面过滤关闭时使用。
- 单传感器单帧聚类：以 seat 对应 x=.27 为中心，半径1 m、3D连通距离 .05 m、少于5点抑制（1.05 m 支持圈）。cloud_to_scan 启动日志残留 min 4 字样，实际 `sizes < 5`；与 collision monitor 的 max_points 无关。
- scan angular_resolution 配置单位 degree/bin（.5°）；消息 angle/range 单位 rad/m。cloud age .3 s，合成时间戳取所用输入最早值。`scan_time=.1` 是写定元数据，`range_max=1e6` 是逻辑哨兵上限，不是雷达量程，当前无流不能报告实测10 Hz。
- scan_map：.05 m、10×10 m rolling map、history .4 s、max_age .3 s；SELF_ECHO x[-.38,.82]/y[-.35,.35]。侧后 blind_band .10 m 是人为 free 策略，不是已观测空闲。
- turn_memory 额外 `/wheelchair/scan→/wheelchair/memory_scan`：LaserScanBoxFilter 盒 x[-.38,.82]/y[-.35,.35]/z[-1,1]、invert=false。此近车过滤只用于记忆输入；collision monitor 仍用原 scan。合成 scan 共用 base 原点，不能等价成两个真实雷达射线原点。

## 速度映射和实际加载标定

三版正式链：Twist raw → velocity_smoother → collision_monitor → `/wheelchair/nav2/cmd_vel` → cmd_vel_adapter → `/whill/controller/navigation`（TwistStamped）。adapter 为未 stamped 输入加当前 ROS 时间和 `base_link`；stamped 输入保留原时间/frame。源码 `/home/whill/Shaki_ws/src/WheelChair_ws/src/whill_controller/whill_controller/velocity_mapping.py`。

| 项 | 值/单位 |
|---|---|
| 输入半径 | 8 HID count |
| 前进增益 | 0.06707777745024013 m/s/count |
| 后退增益（绝对值） | 0.050458867829928096 m/s/count |
| 转向增益 | 0.10136067817594809 rad/s/count |
| 正/负线速度极值 | +0.536622219601921 / -0.40367094263942477 m/s |
| 角速度极值 | ±0.8108854254075847 rad/s |
| 线加/减速度 | ±0.33 m/s² |
| Lattice / Hybrid025 角加/减速度 | 配置 ±0.6 rad/s² |
| turn_memory 角加/减速度 | 配置和 live ±0.8 rad/s² |

逆映射 `x=-w/(8*TURN_GAIN)`、`y=v/(8*相应正/反增益)`，再按 `max(1,hypot(x,y))` 径向归一化，组合速度不是独立矩形饱和。x 是向右的归一化 HID 输入、y 是向前；HID 屏幕点 `center+8*[x,-y]`。细分支持 1 或 25，当前状态 fine_hid_enabled/supported 均 true；没有测实际 HID 或车体精度。

实际控制器 `calibration_file=/home/whill/Shaki_ws/esp32_velocity_interface/ros2_ws/install/whill_controller/share/whill_controller/config/ipad_calibration_20261009.json`，resolve 到 `/home/whill/Shaki_ws/src/WheelChair_ws/src/whill_controller/config/ipad_calibration_20261009.json`；live status 校验值 `d515e4c0c0aacaab961e4d5af1f4e94e9345acc84a232ccb18c5548bb2dd64dd` 与读取文件一致。此 iPad 标定基于屏幕右下边界的 HID 计数，不是像素或米；方向盘中心 [-16,-19]，记录的 forward [-16,-27]。状态中的 app_target_offsets 可以是运行时重映射值，不用它覆盖标定文件。旧 controller.yaml 内 .3717/.3642 m/s、.3888/.4033 rad/s 是旧运动显示参数，正式 velocity_interface.launch.py 不加载该 YAML；不能当当前连续映射增益。

当前 velocity_smoother 是 OPEN_LOOP、20 Hz、timeout .2 s；映射无速度反馈闭环证明。串口 `/dev/ble2wchair→ttyACM0`。磁盘第三版入口已带 latched_arm:=true，但 PID18880 的 live 参数没有 `navigation_latched_arm`，与用户先前“旧进程未重载”相符；不能宣称新保持式 ARM 已在旧进程生效。

## DDS 与上游眼控契约

公共 `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/support/first_run/dds_profile.env` 配置 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、`ROS_DOMAIN_ID=0`、`ROS_LOCALHOST_ONLY=0`。进程环境一致，未设置 CYCLONEDDS_URI。实际 maps 加载 `/home/whill/Shaki_ws/src/WheelChair_ws/runtime` 外的 `/home/whill/Shaki_ws/dds_eval_20261030/stage/opt/ros/humble/lib/librmw_cyclonedds_cpp.so` 和 `lib/aarch64-linux-gnu/libddsc.so.0.10.5`。本轮仅 source /opt/ros 的首次只读探测因缺 Cyclone 库失败；加载既有 dds_profile.env 后成功，未安装软件或改配置。不要默认依赖系统 /opt/ros 中的 Cyclone。

眼控已有上游入口是共享仓库 `scripts/run_hardware_eye.sh`，启动 EYETLE `start_fusion.sh --publish-ros` 与另一套 `wheelchair_hardware_bridge hardware.launch.py input_source:=eye eye_sample_mode:=fusion interaction_mode:=directional_intent`，还依赖 `projects/Ble2WChair` overlay。它不会自动接入这里三套原生 Nav2。三套入口均未启动 eye_intent、direction_manager 或 gaze→Nav2 目标转换器。

| 接口 | 类型/约定 |
|---|---|
| `/eye_tracking/fusion_samples` | std_msgs/String，JSON schema=1；主推荐上游契约 |
| `/eye_tracking/signals` | Float32MultiArray [horizontal, closed, valid]，旧接口 |
| `/eye_tracking/samples` | wheelchair_interfaces/EyeSample，带 session/sequence/header |
| `/wheelchair/eye/direction_event` | DirectionEvent：1继续/启动、2偏左、3偏右、4取消偏好、5停止、6掉头；ttl_ms |
| `/wheelchair/eye/health` | InputHealth；健康和时效，不是速度 |
| `/wheelchair/intent_ack` | IntentAck；必须保留事件确认语义 |
| `/wheelchair/navigation_status` | NavigationIntentStatus；交互上下文 |
| `/eye_control/feedback` | std_msgs/String |
| `/eye_control/start` | std_msgs/String JSON {session,stamp_ns}，原眼控授权接口 |
| 当前 Nav2 `/goal_pose` | PoseStamped，odom 坐标；当前发布者 RViz |
| 原生目标契约 | nav2_msgs/action/NavigateToPose；本轮未发送目标 |

fusion JSON 必须含 schema/session/sequence/stamp_ns/source_t/horizontal、fresh/calibrated/open/motion_valid/gesture_continuity/long_close、blink_event；扩展包含 eyes/input_grace/left_blink_event/interaction。horizontal 为无量纲 [-1,1]，负左正右；源代码把右偏转成负相对 yaw。stamp_ns 是 epoch ns，source_t 是 monotonic s，不可混用；接收端约 .25 s 源时效、session/sequence 去重和 ACK/上下文门均需保留。完整字段与 parser SHA 在 JSON。

EYETLE 独立 eye_ros_env 默认 domain=58、localhost=1；旧 hardware wrapper 才把 EYE_ROS_DOMAIN_ID 设为硬件域默认0。当前 Nav2 为 domain0/localhost0/Cyclone，不能仅 topic 同名就声称互通。默认 start_fusion 每次启动做中心标定，--three-point 才选 saved profile；本轮未启动眼控、未核用户视线标定或 eye→Odin 外参，不能猜眼控角度/像素映射。camera_paths.py 按 USB hub 4.4 或4.1、left端口1/right端口2选定，未打开相机验证。

## 关键 sourcepath 与 SHA-256

下表为本次读取文件内容的 SHA；不是对正在运行 Python 内存字节的证明。全部 inspected/reference 文件（含不在运行链的旧接口源码）及其 alias 在 inventory.json.files；文件加载地位以上文和 runtime 参数为准。

| canonical sourcepath | SHA-256 |
|---|---|
| `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/profiles/lattice/start_navigation.sh` | `2c09d0c2492e3a9a34b23a729b2931956086b1b9faf485a68e3369d97da65c30` |
| `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/profiles/lattice/hardware_lattice_10hz.yaml` | `320ec899c88b998c5dbfbbe342cabf85b32d3c8bac384e0bf00426dfd9c26a72` |
| `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/profiles/hybrid025/start_navigation_hybrid025_10hz.sh` | `056ecf9bed5b952c3204e8b00ce64b618a912925cc25681fff24774128a4a584` |
| `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/profiles/hybrid025/hardware_hybrid025_10hz_analytic125.yaml` | `31454ccc7570f0264ee96fb70b4e0eb15fab87730c26ec66c104652c29518dbb` |
| `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/profiles/turn_memory/start_navigation.sh` | `a9f453c4a86a93e4d761efd9fd9f09ceefef816a6609f00373d8bc044be75cc2` |
| `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/profiles/turn_memory/navigation.yaml` | `7d2a2c0d48da9ff52cd1d9af755654d5e08d3af326080ec1529e46633b717404` |
| `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/support/first_run/start_sensors.sh` | `2835322978ce98c778969b8ac2aec8956acb2344458ebf32b69719059db73685` |
| `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/support/first_run/dds_profile.env` | `c237c0d0a15ad6689b2c20e817eea85153ff553aae694a11117f31bcd10421ce` |
| `/home/whill/eye_ctrl_wheelchair/sensor_visualization/all_sensors.launch.py` | `f30e6dba8d1b8086af6f04522f4969f3bf478eda0b9de5125273f393bd496862` |
| `/home/whill/eye_ctrl_wheelchair/sensor_visualization/sensors_tf.launch.py` | `375708a0bc976e2b07bcf11006d54fe72cf73e4637ac338479acadded42d1596` |
| `/home/whill/eye_ctrl_wheelchair/sensor_visualization/body_filter.yaml` | `8a06a39d34632c7f0855237e3f474ab1030588d9a3c30c0e42b75f72201c9920` |
| `/home/whill/eye_ctrl_wheelchair/sensor_visualization/body_cloud_filter.py` | `faf761f69d53e3e74da29be90a1a58a24713bf2074d03d62e21b89848446df81` |
| `/home/whill/livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/dual_mid360_launch.py` | `40b8ab1bd22b498fc9d9ca5a03124d51ddd828a6362bfd26a5c8d43d478f1820` |
| `/home/whill/livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/dual_MID360.json` | `89dda12fc6740dc79432b05c377bbc248e6e852966213784c871c4ea6379eef2` |
| `/home/whill/odin_ws/src/odin_ros_driver/config/control_command.yaml` | `03ccce26f212643040dda8406ae577ac6d9d316d4900b0644e97ad592a98f1d9` |
| `/home/whill/.ros/odin_ros_driver/calib.yaml` | `8d61df9cb1a194266bc185739fcaa2aaeb537912c58966e039f59559fa678883` |
| `/home/whill/eye_ctrl_wheelchair/wheelchair_shared_ws/src/wheelchair_hardware_bridge/config/hardware_bridge.yaml` | `986e0b39ab48f026b278a1300bed03af3b53cb375806275d756fc02227fdabb8` |
| `/home/whill/eye_ctrl_wheelchair/wheelchair_shared_ws/src/wheelchair_hardware_bridge/wheelchair_hardware_bridge/odom_relay.py` | `27a7908d058e712a31fb4250a84d2f604b5430624ea9260c1b0ed70f19e818c2` |
| `/home/whill/eye_ctrl_wheelchair/wheelchair_shared_ws/src/wheelchair_hardware_bridge/wheelchair_hardware_bridge/cloud_to_scan.py` | `79a124ca30e5cc3c0e8ef94d6dda7e1b9b5aefd91af7b13dcce23fcf02a1724d` |
| `/home/whill/eye_ctrl_wheelchair/wheelchair_shared_ws/src/wheelchair_hardware_bridge/wheelchair_hardware_bridge/scan_map.py` | `f9ed81816345e739e75678dc563d35a9418a65f84e177abc0a6d7a4e0bd882c3` |
| `/home/whill/Shaki_ws/src/WheelChair_ws/src/wheelchair_navigation/profiles/turn_memory/memory_scan_filter.yaml` | `ca79cb5cf6ff674dc0807468868b7bea442ab71de59a05b07ce6ec01cfc9feab` |
| `/home/whill/Shaki_ws/src/WheelChair_ws/src/whill_controller/whill_controller/velocity_mapping.py` | `b395e876184f2c7f0583b5e73bd45c16bd4c1184b72d120a69e7d8a5880974c7` |
| `/home/whill/Shaki_ws/src/WheelChair_ws/src/whill_controller/whill_controller/cmd_vel_adapter.py` | `37d4726c728fa6ca8f2734ec4cc2e32d2b0332c992d64d9c84ce2aa4e5ee5915` |
| `/home/whill/Shaki_ws/src/WheelChair_ws/src/whill_controller/launch/velocity_interface.launch.py` | `6991e4ff71a49f7ff822e514281972db4948075f7a52c50bfd7cdc7d9947ff8e` |
| `/home/whill/Shaki_ws/src/WheelChair_ws/src/whill_controller/config/ipad_calibration_20261009.json` | `d515e4c0c0aacaab961e4d5af1f4e94e9345acc84a232ccb18c5548bb2dd64dd` |
| `/home/whill/eye_ctrl_wheelchair/projects/EYETLE/start_fusion.sh` | `b58d83864556104a4bea800b4b9d36d3f9e69ea5d437e76a1cc873dbda684850` |
| `/home/whill/eye_ctrl_wheelchair/projects/EYETLE/tools/fusion_tracker.py` | `505e7d21fd6565b0108d79ed48ed972e09f5cd9453a343bcf745f2b7d1ece580` |
| `/home/whill/eye_ctrl_wheelchair/wheelchair_shared_ws/src/eye_shared_control/eye_shared_control/confirm_node.py` | `1d8cc6e2340586ae7b77b7b11392528899220e1009231e94d27530b642b30d73` |
| `/home/whill/eye_ctrl_wheelchair/wheelchair_shared_ws/src/eye_shared_control/eye_shared_control/fusion.py` | `9ef8f3a2b3fe928b3572ed43eb9d89205f7bd83b7c72228d538ce66d05e1560b` |

## 尚未证实的事项

硬件断电下未复测外参精度、频率/时钟同步、动态 TF、眼摄像头/用户标定、车体速度响应。Odin runtime calib 内存加载和静态 camera TF 的原始标定过程没有重新验收。运行进程早于源码搬迁/ARM补丁，磁盘路径和 live 配置分别保留。主线程正在进行的整理、仿真不计入本次验证。Git dirty 文件列表已保存，本文未重新研究 planner，也未修改任何远程或师兄文件。

最终内容复核：2026-11-01 04:16:31 +08:00 核对 144 个 manifest 文件；并行整理仅 package.xml 有变化，已重新读取并更新记录。其余传感、TF、三版入口/配置、映射与眼控接口文件 SHA 均与本清单一致。本机 JSON、证据文件及静态 TF 数值一致性检查通过。
