# S20研究总track：成功、失败与路线修正

更新日期：2026-10-25（北京时间）。持续维护正本：本文件。用途：组会、阶段汇报与后续接续，不以最新成功覆盖历史失败。

本地可视入口：[研究日记、视频与贪心对照](research_site/index.html)。10月23日至25日新增内容在下方独立记账；旧32／41件、旧三箱、旧训练与作者源文件不覆盖。10月23日曾只读核对Jetson；25日完成隔离接口部署与后续离线衔接，没有发送运动、IO、使能或控制柜配置写入。25日本轮最新SSH及SDK回读成功，FSM33/StandBy；此前连接失败保留为历史，不代表当前离线。

## 1. 现在到底做到了哪里

### 2026-10-26 感知等待优化：会话复用与媒体异步回传

- 用户指定舱内持箱顺序改为先伸入→再左右水平移位→最后竖直下放。新增仅部署启用的insert_then_shift：入舱横向通道由示教entry TCP及当前吸附变换推算箱体位置，并夹至门框可用开口；按既有路线高度候选先到目标深度、再移至目标Y、最后下放，保留持箱整臂IK/CAD和箱体通道检查。三段独立标记INSERT_DEPTH/SHIFT_LATERAL/LOWER_FINAL并由native官方MoveL执行，不合成MovePathJ斜向插入。保留XY每侧5cm代理余量、真实支撑判定、释放后worldZ+20cm直升和原空载返回；速度参数与真空IO/确认逻辑未改。
- 052527四份实测输入从空舱顺次更新新布局，pick/place/空载返回准备4/4通过，place计算4.140/2.437/2.218/2.291秒（初始化另计）。证据depth_then_shift_20261026/{before/,verification.json,replanned_1/至replanned_4/}；三文件同步Jetson，无运动/IO下发。此处是离线参考与SDK分段接入，非新版四件真机验收；5cm高度同层归并尚未启用。
- 用户要求释放后先竖直抬升20cm以免拖箱、检测箱体水平四周各膨胀5cm。当前部署place使用测得长宽各+10cm、高度不变的碰撞/布局代理；原尺寸、吸取中心、接触下压偏置及半高吸附变换不变，提交占据/catalog明确为带余量代理。支撑判定将XY代理每侧缩回5cm后用实际底面计算，不能把虚拟余量当材料支撑。此配置不适合不注明来源地继承旧无膨胀占据，重跑需空舱新会话。释放后只保留world-Z+20cm分离首段，无斜向后退替代；native桥将_EMPTY_SEPARATE交官方MoveL，再接原独立空载返回；速度数值与阀门/人工确认时机不变。
- 第一次四件离线到第3件时，底层无有效候选、trace记录拒绝候选触发上一轮新增深度锚映射KeyError0.0；失败目录保留。修复仅拒绝候选缺层锚时的记录排名fallback后attempt2用052527四份实测输入从空舱顺次规划4/4成功，place计算2.946/1.874/1.813/1.721秒，第三件转上层；原速物理/真机未执行。FK核对四件释放→首段终点均约worldZ+0.200m，XY终点变化小于1微米；非实机跟踪测量，native实际MoveL尚待现场验证。证据release_lift_padding_20261026/{before/,replanned_3/原失败,attempt2/verification.json,release_geometry.json}，五文件同步Jetson；未发运动/IO、未改速度/泵，不宣称四件真机通过。
- 横放排序修正：052527实际候选记录显示前三件均长边沿ULD深度X，首选直接成功；ContactLayoutPrior原_rank把姿态偏好放在深度/左右/接触分数之后，不是横放求路失败或YOLO导致。新增隔离启用项horizontal_row_first，仅当前deployment预览/完整place启用，旧调用默认不变；按层→相对本层有效最深候选的既有10cm深度档→横放（长边沿ULD Y）→右往左→贴面/紧凑排序，保留90度及其他深度备选。深度档锚定候选前沿，避免任意坐标网格边界压倒朝向偏好；不改变可达球、间隙、支撑、IK、速度或泵。
- 用052527四份实际逐件观测/吸附变换、从空舱顺次重规划并提交新规划占据，四件pick/place/独立空载返回准备均通过：前两件横放相邻，第三件90度补剩余窄带，第四件在底层下一深度排横放；不是固定槽位回放、不是新真机验收。place计算1.721/1.774/2.014/2.935秒（初始化另计），原失败不覆盖。证据horizontal_order_fix_20261026/{before/,verification.json,replanned_1/至replanned_4/}；四文件同步Jetson，native执行器/速度/真空代码未改，无运动/IO下发。
- 051353新m-seg真机第1件完成观察→贴合→人工确认吸附→抬升入舱→放置释放→空载返回，第2件RGB/depth/info各6帧仍在转换退出，最终1/4。重放第2件真实输入定位为fit_box的top/floor平行硬门槛dot>=0.999（约2.5626°），实际拟合夹角2.56498°，不是相机断流或YOLO漏检；m-seg识别suitcase置信度0.94758，平面RMS3.163mm。原异常被主会话泛化为capture/conversion failed，已改为保留实际异常。fit_box新增require_parallel默认True保留原独立实验语义，真实观测适配关闭此平行假设，保留测得法向、尺寸及倾角，不伪造水平面，不放松碰撞或IK；这仍是箱体代理，不证明底面真实形状。用实际第1件place_execution/scene_state.json和第2件重新拟合输入、同-2cm偏置完整准备pick/place/独立空载返回已成功；没有发真机运动/IO，尚未宣称4件通过。备份和原失败保留，证据round2_plane_fix_20261026/{before/,diagnose_conversion_fixed/,offline_input/,offline_actual_prefix/}。已同步观测侧三个文件到Jetson，不改速度、泵、标定和观察位。
- m-seg接入后追加离线完整接口验证：使用045104实测RGB-D重新拟合的几何、原关节快照、中心吸取与-2cm接触偏置，调用原online_session的--observation/--preview-rounds 4，四次抓取、放置及独立空载返回参考均生成成功，逐次累积规划释放占据。整体25.820秒，各次准备窗口6.932/6.193/6.277/6.417秒，place内部计算1.388/1.368/1.429/1.645秒，初始化另计约3.58–4.28秒。输入是重复同一实测箱体，不是四次新感知、动态执行或四件真机通过；hardware cycles=0，actual_async_overlap_measured=false。证据detector_comparison_20261026/{offline_input/,offline_four/result.json,offline_four/round_*/}。当前先保留已通过此接口验证的m-seg，不因模型名字更新盲换YOLO26/YOLOE。
- 045104轮次真实RGB-D已恢复，但YOLO11s-seg在不限类别/conf0.15下无框，0/4停止。原图为完整居中橙色箱体，不据此移动观察点。对同一原图离线CPU比较官方YOLO11分割权重：s在640/960均无框；m在640/960为suitcase置信度0.8764/0.9227、首次调用墙钟3.875/5.279秒；x为0.6317/0.2528、7.600/12.985秒（初始化及并发负载影响耗时，不作为稳态延迟）。m640经原RGB-D几何链得到48.33×34.72×24.67cm，status仍为SURFACE_FITTED_COMPLETENESS_UNCONFIRMED，不称完整顶面或四件真机验收。切换observe_current的实际与预热权重s→m，原s与before源码保留，x保留离线对照；conf、观察位、运动速度、泵、接触偏置均未改，不发运动/IO。Jetson时钟异常导致官方HTTPS下载证书失败，改在本机正常HTTPS下载后SSH复制，未关闭TLS验证或改系统时间。原始对比及深度拟合留在detector_comparison_20261026；仅模块级检测升级，未做现场闭环成功声明。
- 043957采集失败为RGB/depth/info均0、paired=null，官方驱动与枚举均报告No device；不是any类别模式或接触偏置引发的识别拒绝。网络只读核对eno1 UP/MTU9000、路由正确，camera邻居由FAILED探测后转REACHABLE但ICMP不响应，不能仅凭ping判断电，也不能将旧内核链路记录归为刚刚事件。用户确认通电并重插相机后，旧驱动仍0帧/失效流句柄；没有改IP、路由、MTU、速度、泵或机械臂状态。
- 经driver.json PID/start_ticks/进程组/命令/本任务日志归属核实，仅清理本任务14组已退出轮次残留相机驱动（SIGINT，2组随后SIGTERM），未停止其他任务。单套新驱动初期仍未发现设备，随后重新枚举并实际出帧；恢复同时涉及用户重插、旧驱动清理和重新发现等待，不宣称已证明唯一根因。第一次旧节点采集0帧的失败保留。
- 新驱动两次真实采集：12RGB/13depth/13info与8RGB/9depth/8info，匹配RGB-D时间戳推进23.8秒，深度非零比例94.756%/94.829%，640×360；本机查看当前灰色箱子图像。首次重连等待17.446秒、实际收图0.733秒，不能冒称整个重连仅0.7秒。证据 `artifacts/s20_camera_replug_20261026/{capture_2/,capture_3/,summary.json}`，远端清理记录 `runs/camera_replug_check_20261026/owned_camera_cleanup.json`。验证后仅关闭本次临时相机驱动，避免四件重启重复占用；没有发送机械臂或泵指令，尚未重新验收四件。
- 用户要求识别上表面之后最终贴合再下走2cm。增加显式 `--contact-offset-z-m -0.02`（默认0保留），只偏置选定接触TCP的world-Z；原上表面/尺寸/AABB、接近点、抬升绝对终点、示教点、速度与泵不变。按新接触位重新计算T_tcp_box_estimated，pick/place消费同一估计，不把箱体整体向下平移。几何目标与偏置目标均记录。
- 同041736第三件any模式原数据加2cm偏置，完整pick准备3.810秒、place求解1.926秒成功、recipe组合成功；离线FK最终contact位相对表面Z=-0.0199991m。证据 `contact_down20mm_20261026/{planning_result.json,contact_offset_verification.json,recipe/}`。仅位置偏置，不是力控或真实吸盘压缩量/接触力验证；代理箱不形变，原吸盘/源物体接触规则沿用，不能由离线通过宣称真实下压安全。本机/Jetson三文件均备份和同步，本次没有发送运动或泵指令。
- 后续041736实际2/4完成，第三件尚未抓取便因NO_YOLO_LUGGAGE_DETECTION退出；图像/深度均已取得、单物体完整可见，YOLO预测cell phone/0.2877。旋转90/180/270度离线识别分别无检测/toaster/banana，不能声称旋转已修复，也不将地面颜色相似断言为唯一成因。用户明确授权单物体地面工作区“不限类别，有框即用深度建模”的原型模式，覆盖最初的纯几何分割讨论。
- 新增显式 `--perception-target any`，默认luggage保留：同一YOLO检测框/掩码、置信度0.15、深度RANSAC和上表面拟合，不以类别拒收；检测类名和实际luggage数量仍如实记录，不把phone改标suitcase。完全无框、无有效深度/上平面或抓放规划失败仍会报错，不宣称任意形状都可吸附；仅适用于工作区内单个指定物体。速度、泵、点位、SDK运动和空间放置算法不变。四文件本机/Jetson备份并同步，没有发送运动/IO。
- 用041736原失败第三件新模式离线建模成功，尺寸[0.458800,0.335374,0.243019]m，中心world=[0.033038,1.609441,0.221031]m；保留标签cell phone。随后与前两件已提交占据一起验证，完整pick准备4.077秒、place求解1.444秒（初始化4.142秒）均成功，recipe组合成功。记录在 `any_target_20261026/{recorded_round3/,recorded_planning_result.json,third_recipe/}`，仅原数据复算，不是第三件真机已完成；新模式需要用户命令显式加 `--perception-target any`，从空舱重启须清空已有两件。
- 040933第三件零位移前导段修复，用户明确授权且速度/泵不变：仅部署native桥增加连续完全相同关节点去重；实际起点0.05°核对仍先执行，整段完全相同则记录skip、不上传MovePath，也不补成5个重复点；真实非零路径保留原每5点采样和少点插值，非相邻重复点/闭环绕行不被折叠。MoveL参数、MovePath比例、阀逻辑和确认均未改。
- 用第三件原pick_recipe和SDK替身完整离线回放，注入已被原起点条件接受的0.015°回读偏差：原版复现20031，修复版跳过重复CONTACT/ATTACHMENT前导段，保留贴合及垂直20cm抬升两次MoveL和后续TAUGHT_LIFT/ENTRY，两次MoveL仍60mm/s、100mm/s²，开阀调用相同。该回放是假的SDK响应，不冒称实际控制柜轨迹预计算或真机运行通过；文件夹明确标记offline_emulated。
- 另通过真实官方SDK只调用HRIF_GetInverseKin核对第三件贴合与20cm抬升端点，两者return_code均0、约2.03/1.81ms，未发送MoveL/MovePathJ、未写IO/使能/复位。不能据端点逆解声称整条实际插补已验收。证据 `native_duplicate_fix_20261026/{verification.json,lift_ik_result.json,verify_recorded_third.py}`；本机/Jetson旧桥脚本备份到该实验before目录，新桥已同步，未自动启动四件；重跑须先支撑并释放当前箱、空载、清空已放前缀或显式携带scene_state。
- 用户重跑040933，要求只监听、不改其他代码：前两件完整真机抓取→放置→释放确认→空载返回成功（周期113.884/92.954秒，含人工确认）；第三件贴合、开阀10、人工确认成功，但在lift_and_entry开始0.176秒后官方MovePath预计算报 `['5','20031','4','0']`，尚无第三件抬升MoveL下发记录，第四件未开始。会话240.343秒、2/4完成；泵退出前最后记录10，程序没有自动释放，不伪称当前现场IO已持续验证。
- 040933前三件真实抓取准备内部3.736/3.921/3.742秒；按本机pick.log创建到末次写入估算含Python启动4.982/5.255/4.990秒（文件时间估算，非新增monotonic计时）。设备感知3.391/2.884/3.927秒，完整主机感知7.169/3.815/4.892秒，首次包含worker未被完全覆盖的启动；放置求解1.436/1.335/1.414秒，前两件到入舱后的异步等待仅0.000032/0.000023秒。第三件失败不是抓取准备又变慢，也不是DI拦截。
- 只读定位第三件失败段：lift_and_entry开头依次CONTACT1点、ATTACHMENT标记1点、VERTICAL_CLEARANCE；前两点关节完全重复。native分段逻辑在垂直抬升前先尝试这两点MovePathJ，非跳过时再补至5个相同点；本次尚未记录MoveL即在路径计算阶段失败，与零长度前导段处理不稳一致。厂商20031不是物理碰撞证明。只保存原始失败/计时摘要 `native_four_operator_20261026_040933/listening_timing_summary.json`；本次没有改执行器/泵/速度、没有补发抬升、没有复位或自动重跑。
- 040025抓取准备修复（本聊天负责）：保留原粗粒度Cartesian IK、样条、retime及密集SDK参考；新增部署专用 `pick_reference_recorder.py`，同一site URDF批量CPU FK记录参考，按3cm几何行程采样并保留样条knots/各关节反向点做原FCL CAD检查，不按20ms时钟逐帧重复GPU FK/回读；去掉示教段重复两遍CAD。`observed_collision.validate`新增可选CPU poses输入，其他调用默认路径不变。相邻CAD采样的所有连杆/持箱表面行程用碰撞几何外接半径作采样尺度，本数据最大2.9996cm；这是离散检查，仍不冒称SDK原生插补的连续扫掠证明。没有修改native执行器、速度profile、泵IO/确认、观察/lift/entry点或放置搜索。
- 复用040025真实RGB-D/抓点/关节，两次完整离线pick准备均成功：含Python进程启动5.351/5.257秒，内部3.325/3.273秒，其中初始化2.630/2.566秒、CAD0.166/0.170秒；2927个运动参考点只验证266个几何点。对照原040025是124.502秒时被中断、仅1987帧，不将其说成旧完整耗时或固定倍速收益。第一版仅改CPU密集CAD也成功（内部11.552秒），用户强调稀疏规划后再完成上述采样分离。
- 验证范围：原040025已有1987帧与新参考的q逐点完全相同，CPU记录的连杆位置最大差1.02微米；CPU密集版与稀疏版完整2929帧（含初始化/吸附标记）的q、时间、阶段完全一致。与040025原已成功place结果组合成功，四个SDK运动参考JSON及全部动作/吸附操作列表在密集/稀疏版间完全相同。旧完整版不存在，因此不把该对照冒称旧完整真机周期已验收。证据 `pick_prepare_fast_20261026/{benchmark.json,reference_comparison.json,sdk_payload_comparison.json,040025_recipe/}`。
- 主机准备器更新、Jetson三文件副本同步，旧准备器及碰撞文件在本机和Jetson `pick_prepare_fast_20261026/before/`保留；本次未发运动/IO、未自动开始四件。四件现场结果仍待用户重新运行原native/operator命令验收，不能据此宣布四件真机成功。
- 按用户“监听”只读跟进040025，无代码/运动/IO插入。新观察位真实返回成功，原生MovePathJ运动1.740秒；设备感知3.469秒。放置后台成功（compute35.987秒、含初始化68.658秒），但抓取准备尚未完成便以KeyboardInterrupt中断，pick准备已124.502秒。已完成的离线阶段包括PRECONTACT、CONTACT与VERTICAL_CLEARANCE，说明本件不再是前述CONTACT IK失败；它们只是规划参考阶段，不是真实贴合/吸附/提升。终止堆栈在observed_cycle.Runtime.frame→body_q.numpy→wp_memcpy_d2h，暴露逐执行点仿真/FK与GPU回读的开销；采样时GPU显示180MHz/14.06W/100%，仅为现场快照，不直接断定因果。未下发pick、开泵或place；会话无最终result.json，不能将pick的success=false/error缺失解释为完整硬件失败原因。原始pick.log/reference/结果保留，未自动重跑。
- 后续035351实际感知含传输降到7.067秒，仍有返回准备/运动22.136秒、pick准备9.040秒；此次CONTACT IK位置差12.361mm而终止，未发送抓取。追加8轮35次Newton迭代、两种完全容纳夹具的朝向和多种种子未消除误差；独立CPU有界最小二乘同样收敛到关节2下界，官方SDK `HRIF_GetInverseKin` 返回20031/Point calculation failed（查询5.82ms，无运动）。这些结果不能称全局不可达证明，但不能用扩大误差阈值冒充接触解；日志的collision_free=True在本IK调用中是关闭碰撞查询时的占位，不是通过碰撞证明。
- 035626又在YOLO处退出，原相机图像中的箱子被作者YOLO误识别成tv/0.24；这是独立感知负例，未将tv自动当行李继续抓取。失败原件不覆盖。
- 第二轮减耗：相机采集直接复用感知进程的ROS/Python导入；在同一SDK连接内采集控制柜快照和逐帧关节，去掉额外snapshot子进程/断连等待；首轮空图仅预热推理器，不产生目标或占据。复用现存本会话所属相机驱动，未重启驱动、运动或写IO，实际新鲜两次RGB-D→建模→抓点→候选分别3.628/3.002秒；热态采集+快照1.219秒、转换1.716秒。首请求含worker加载/预热9.479秒，不能省略冷启动；正常入口将其与回观察位重叠。证据在 `perception_reuse_20261026/live_inprocess/`。
- 用户随后要求观察位沿机械臂base正X加10cm。仅部署 `pick_waypoints.json` 的observation更新，保持原高度/姿态、lift/entry不变；官方SDK逆解成功1.70ms，新q_deg=[189.057,-125.801,-104.642,-36.932,91.890,13.259]。独立FK的TCP位移base=[0.099989,0.000034,0.000277]m，当前TF下对应world-Y约减10cm。原点本机/Jetson均备份，已同步但没有实际移动；这不增加实体机械臂可达范围，不冒称之前远端抓点已解决。证据在 `observation_base_x10_20261026/`。
- 已结束的 `native_four_operator_20261026_033955` 实际完成3/4真机周期，第四件报 `Controller stopped: 21`；不能称四件通过。本次不重试运动、不复位、不操作泵。
- 原四件感知含传输约12.68–13.02秒；设备内部9.40–10.33秒，实际配对图像采集窗口约0.20–0.34秒。瓶颈不在强制收图等待，不能把减少帧数称为主要优化。
- 仅修改部署的 `online_session.py`、`observe_current.py`、`received_to_observation.py`：每会话保留一个SSH感知进程，YOLO模型与Python导入复用；各件仍实时重新采集RGB-D和关节，不复用旧目标。首轮模型加载与返回观察位重叠；抓取/候选算法、0.15识别阈值、运动与IO逻辑不变。设备增加snapshot/capture/conversion/grasp/candidates分段计时。
- 规划先回传必要JSON/URDF，完整原图、深度和点云后台SCP归档，会话结束等候归档并记录返回码；归档失败保留远端原件并提示，不静默删除。同份实际第四件记录的传输对照：整套6,833,598字节耗时1.032秒，规划tar184,320字节耗时0.299秒。
- Jetson记录回放实测：相同RGB-D的首次识别建模4.907秒，缓存模型后1.675秒；两次拟合尺寸/姿态一致。完整感知入口在复制SDK快照/相机文件、没有调用硬件的两次请求中均成功，热态转换1.683秒、抓点0.049秒、候选0.017秒。该验证不是新鲜相机采集，不把1.75秒回放处理当完整现场感知时间。
- 必要文件包已独立接通离线pick/place准备与recipe生成成功，没有发送运动或IO；新版本已同步Jetson，旧文件备份在远端 `runs/perception_reuse_20261026/before/`。新鲜采集及下轮全周期耗时待实际运行确认，不宣称四箱已重新验收。
- 本机证据：`experiments/s20_online_mixed8_20261024/perception_reuse_20261026/`，含 `transfer_benchmark.json`、`pipeline_68571316967097/benchmark.json`、`offline_preparation/`；原真实失败与媒体不覆盖。

### 2026-10-26 原生MoveL/MovePathJ接入并完成单件全流程

- 人工模式033731的真实失败：输入1后接近/贴合均成功，DO0写入成功后约4.1ms立即单次回读，operator分支错误地直接以旧DO值终止，未显示输入2；这是新适配缺陷，不是DI0拦截。随后独立SDK只读IO为DI0=0/DO0=1/DO1=0、StandBy33。仅修复operator吸附输出确认：每50ms轮询DO0=1/DO1=0，最多2秒、到位即返回，而非强制等2秒，之后才询问2；DI不参与。语法核对并同步，未切阀/移动/自动重试；尚未在下一真实周期验收，当前吸盘仍开启，不能直接确认空载重启。
- 用户明确说明自制泵未做真空反馈硬件，任务配置因此改为可显式选择 `--vacuum-feedback operator`（仅native）：不等待或使用DI0作吸稳/掉压/空载判断，也不伪造DI值；以现场EMPTY、HOLD、RELEASED输入确认替代，明确没有自动吸附丢失监测。原digital默认保留，不静默切换已有命令。HOLD在吸附开启后及入舱交接处要求现场确认，RELEASED在0.2秒释放脉冲结束后确认才撤离；拒绝/EOF中止且不自动释放。主机将该模式远端提示直接显示并继承stdin，不再将确认提示藏在session.log。三文件语法及CLI帮助核对完成并同步Jetson，未启动新的运动/IO/四件，不宣称无反馈无人干预四件已通过。
- 按用户要求进一步分离默认/运行/接线证据：本次实际ROS GetParameters回报box/DI0，但当前节点带preserve_input_or_slow_v2，属于本适配执行器，不能冒充师兄现场原运行。ROS图未见vacuum_controller，Jetson/本机目标进程扫描未发现师兄hardware_pick运行；原hardware_pick.sh→hardware_pick.launch.py配置backend=hardware，vacuum_controller默认订阅/vacuum/di0，上游vacuum_io默认box/0。该链证明原启动默认，不证明师兄某次实际启动的覆盖参数。我们native分支SDKInterfaces.read_vacuum直接硬编码ReadBoxDI(0)，独立于ROS映射参数。物理反馈线是否接DI0仍未确认，已请求厂商IO接线图或设备型号/端子照片；未切换输出、移动、改映射或强制DI。
- 用户质疑DI0是否额外强加：只读核对Jetson师兄 `RoboticArm-master/src/luggage_planning/luggage_planning/vacuum_backend.py` 99–116行，HardwareVacuumBackend确实开DO1=0/DO0=1后轮询DI0=1，默认8秒超时；`suction_pick_session.py`371–378行也检查搬运前DI丢失。因此“师兄完全不处理DI0”与当前存储版本不符，但不能据此证明现场当时启动了这一后端。厂商CPS SDK只提供读DI/写DO，没有规定DI0天然就是压力；硬件映射属于项目接线约定。当前适配新增的连续稳定1秒及30秒超时由本聊天负责，不冒称厂商规定；此次未发运动/IO或改代码。
- 030328再现同类未lift：贴合MoveL约2.320秒成功，发DO0=1/DO1=0后30秒DI0始终0，native桥接已退出而非仍等待。独立SDK新鲜回读StandBy33、当前仍在贴合位、DI0=0/DO0=1/DO1=0。未改代码/阈值、未补发lift或切阀；现场称吸紧与DI不一致，需压力开关OUT指示和控制柜DI0对照，不能用输出10替代输入确认。
- 用户确认箱子已落地支撑并授权仅原地释放：`supported_release_20261026_attempt1`执行DO0关闭→DO1释放2秒→DO1关闭，无机械臂运动/使能/复位。执行开始IO实际已为0/0/0（相对此前DO0=1是现场状态变化，不推断原因），结束独立SDK回读DI0/DO0/DO1均0、StandBy33，六关节三位小数前后差均0。原始结果回传本机；未自动开始四件，外部泵电源保持开启由用户现场确认，空舱重启仍需清空已放箱子。
- 025843首件已贴合并写DO0=1/DO1=0，但DI0全程未高，30秒确认超时，未发lift。用户现场称已吸紧且有时直接关闭泵电源；本聊天随后独立SDK只读回读仍StandBy33、DO0=1/DO1=0/DI0=0。当前代码只控制阀输出，无独立泵电源/真空传感器使能或电源回读，不能将DO0=1说成泵与压力开关已通电；用户触感与数字确认目前不一致。未改DI极性/通道，未开关阀/断电/移动；需现场压力开关数值/OUT灯及设备供电证据定位，不能仅由关闭电源经历断定原因。
- 用户随后允许5cm贴边间隙。仅部署候选入口与放置worker选项改为 `neighbor_gap_m=.05`；是水平相邻候选留间距，不是允许5cm穿透，也没有修改碰撞判定/运动速度/吸附逻辑或旧算法默认。基于024608第二件原真实观测及第一件已提交占据，独立 `b2_gap5cm_20261026_attempt1`重新规划成功：求解1.323秒、含初始化4.883秒；没有发送任何运动/释放，不冒充第二件已经放入。两文件已同步Jetson。用户计划从头再跑四件，必须先可靠支撑并释放当前持箱、清空此前已放B1；不能在仍持箱时空载重启。
- `native_four_2x_20261026_024608`第一件2倍完成，第二件YOLO已通过且真实抓取/抬升/入舱成功，但放置后台失败，未发第二件place。23条保留失败为原路径CAD拒绝10、终点CAD拒绝5、终点IK未解4、途中IK未解4；前10条路径接触均B1/B2，首次相交深度0.00046–0.039mm，说明紧凑零间隙候选与曲线/数值误差的接触判定不一致，不能称所有候选都物理碰撞或空间无解。另5条终点CAD日志未给深度，8条IK仍须独立区分。只读新鲜回读当前入舱关节、StandBy33、DI0=1/DO0=1/DO1=0；第二件仍持箱，无自动移动/释放/重跑。遵守用户只改YOLO0.15、不改其他代码，本轮仅查证并记录；下一候选间隙/接触一致性修复未实施，不能说四件已完成。
- 单件成功后用户要求速度加倍并自行启动四件：显式 `--execution native --native-speed-scale 2`传到初始回观察位与后续所有执行调用。MoveL为60mm/s、100mm/s²（速度与加速度同比，使起步斜坡时长不缩短），MovePath比例约0.13333，原1倍默认不变。吸稳1秒、IO流程、规划预算不缩短。仅代码/参数核对及同步Jetson，本次不自动启动运动；2倍四件尚未实测，不能承诺总时间减半。空舱重新开始前须取出上一轮B1，否则应传递已有scene_state。
- 用户现场确认20cm MoveL“非常丝滑”，明确授权移植并实际执行全流程。隔离新增 `native_cycle_bridge.py`，`online_session.py --execution native`显式选择；ServoJ默认与原服务/同学源码不改。接触和垂直抬升用MoveL（30mm/s、50mm/s²），其余路径由当前规划关节参考每5点保留一个并含终点，上传MovePathJ（ratio约0.06667、半径1mm），保持源路段顺序；合并抬升参考位/入舱位为一段原生路径，不再由主机20ms逐点推送。原生插补和源曲线不同，已有源曲线几何检查不能冒称原生连续扫掠证明；比例也不称实测12°/s或10°/s²。
- `native_full_20261026_attempt1`同一轮真实新鲜感知、并行抓取/放置规划、回观察位→贴合→真实开阀→吸稳→提升→入舱→在线放置→真实释放→空载返回，1/1完成，无人工续跑，无吸附丢失/控制柜错误。全轮175.107秒，含相机启动/清理会话177.698秒。返回准备和执行16.441秒；感知含传输12.293秒（设备内部9.707秒）；抓取准备6.806秒；放置初始化3.528秒、求解1.413秒，入舱等待约0秒，异步确实覆盖了本轮放置计算。
- 原生SDK动作回执耗时：初始返回5.805秒；接近10.102秒；贴合MoveL3.991秒；真空DI变高后连续稳定1.018秒才提升；直线提升7.304秒；后续搬运25.750秒；放置27.658秒；空载撤回37.855秒。释放DO0=0/DO1=1后约0.16秒DI0=0，随后DO1=0。最终独立SDK回读StandBy33、q回观察位、DI0/DO0/DO1均0；本轮任务已结束，无自动第二件。主要剩余耗时是搬运与空载返回，并非放置求解。
- 本地完整结果和事件在 `experiments/s20_online_mixed8_20261024/native_full_20261026_attempt1/`；实际执行report.backend为native_MoveL_MovePathJ，首轮session旧scope还写ServoJ是过期描述，后续已改通用motion dispatch并新增execution_backend字段，不改首轮原件。放置后占据按用户明确要求使用规划位姿，不重观测、不宣称实测落稳/高成功率。当前舱内已计入B1；继续试验必须带此scene_state，不可将有箱场景误当空舱重新跑。

### 2026-10-26 单段MovePathJ实机对照

- 用户随后明确授权空载当前位置向下20cm并声明现场安全。新增隔离 `native_down_compare.py`：官方MoveL单次指令、基座Z减200mm、朝向不变；不是MovePathJ，也不是ServoJ。prepare1因当前固件不支持ReadActCoord返回20005，未运动；改用已验证ReadActPos且核对命名TCP/Base与当前TCP/UCS均为零后，prepare2官方IK每20mm连续求解11点成功，无翻腕分支。用户确认空载延续本次对照，未改变泵/使能/override/原服务。
- `native_down20_20261026_execute1`真实执行完成：30mm/s、50mm/s²、末端圆角0；约7.337秒后StandBy33，无错误，Z由−4.693到−204.642mm（下降199.949mm），X/Y基本不变、姿态保持。未自动上升/返回，DO0仍1、DO1=0、DI0=0。原始准备失败与执行反馈回传本机；现场抖动改善尚待用户观察，不把一次空载直线成功外推为持箱/舱内全流程完成。
- 用户确认故障已清除、重新使能、结束平板点动，并另行确认吸盘与箱子分离且空载。只读回报StandBy33，DI0=0、DO0=1、DO1=0。本聊天新增隔离 `deployment/movepath_single.py`，仅本次获准空载J1增加1°，不自动返回、不启动四箱、不切换ServoJ服务、不写泵IO或使能/reset。
- 初始离线CAD命令缺deployment导入路径，修正后当前位置到J1+1°的21个离散状态通过；这不是全程连续扫掠证明。SDK使用StartPushMovePathJ→5个共线关节点→EndPush→ReadPathState计算完成→MovePathJ，计算比例约0.06667、半径1mm。比例不是已测12°/s或10°/s²保证，控制柜插值不保留原MINCO时间。
- attempt1上传/计算成功，但日志函数event(name, name=...)参数重名，发生于MovePathJ调用前，异常处理GrpStop返回0；原result的motion_sent已提前标true，不能据此误称执行过。实际q回读未变。修正日志参数后attempt2真正完成：J1由188.413°到189.411°，其他关节未见三位小数变化，调用后至目标StandBy约0.809秒，FSM33、无错误、IO原样。两个原始目录均回传本地，失败不覆盖；短段无报错不等于长搬运或持箱平滑已验收，等待用户现场感受。

### 2026-10-26 ServoJ节拍只读核对

- 用户授权按同学方式修改后，已在隔离适配层贯通抓取／回观察位原曲线解析速度，compose保存速度、桥接填入FJT velocities，若参考整段减速则速度同比缩放，供原执行器Hermite插值使用。首轮离线命令缺Newton导入路径失败，按online_session实际PYTHONPATH重跑 `velocity_handoff_20261026_attempt2`规划成功；首次compose因累计浮点时间造成尾部多一个采样点失败，改为将速度同步采到实际输出网格后 `recipe2`成功，原失败保留。代码已同步Jetson，不重启服务、不发运动；50Hz、20%/12°/s、参考10°/s²、吸稳1秒保持。此改动只补齐速度传输，不将离线成功说成抖动已消除：放置worker尚无解析速度输出，中间参考位仍停稳，同步SDK反馈读取与实际发送抖动尚未修改／验收。
- 随后按用户要求对照同学本地 `ludan_greedy_20261023` commit `1f0600802ed6c4b7beb4b4187aba4c2f299449b6` 与Jetson师兄原目录 `RoboticArm-master` commit `12a7998dd6126ce1f53c5ab91a2afe3a634557d0`（origin adamliaoyifan）。同学实际文档明确也是20ms ServoJ，并记录过60°/s²加减速触发CollisionStop，文档并非所有参数已复测通过；不能将其当无条件丝滑的硬件证明。旧8143e88快照只有WayPoint混合执行，不代表师兄当前没有ServoJ，已从Jetson原目录补读当前ServoJ源码。
- 确认适配差异：同学 `trajectory_timing.py` 生成带速度/加速度的整段梯形时间律，执行器收到速度时用Hermite插值；我们桥接仅填positions/time，故进入线性插值分支。我们拿取链把VERTICAL_CLEARANCE、TAUGHT_LIFT、TAUGHT_ENTRY分别做成起止静止参考，会增加中途停顿。上述仅能解释潜在拐点/停顿，不单凭代码宣称剧烈抖动或20018唯一根因。
- 同学和师兄当前ServoJ发送循环均同步调用完整 `_refresh_positions()`；ROS发布关节状态的定时器也会调用refresh。因此同步读阻塞不是我们独有差异，但可能因机器/通信负载而显著放大，仍需实际发送间隔验证。厂商C++样例使用秒级time()和sleep(小数)，不直接复制其定时实现。此次未改同学/师兄源码、未切换服务或发送运动；不把100Hz离线加密称问题已解决。
- 用户要求插值提高频率：在隔离启动器增加可选 `--servo-period-s .01`，复用作者FJT插值到100Hz，并将前瞻显式从配置传入；默认仍20ms，不自动重启或切换。对021012首件抬升真实计划离线检查1496→2991点，首尾一致、总时长29.90秒不变。仅完成插值及入口准备，不代表100Hz实机吞吐／平滑性通过；分段线性插值不消除原曲线拐点，同步读取阻塞仍未处理。启动器已同步Jetson，当前内存执行器仍50Hz／0.1秒前瞻。
- 用户使用平板点动更平滑，要求核查频率。本聊天只读ROS参数：当前运行执行器周期0.02秒（50Hz）、前瞻0.1秒；应用配置写0.2秒但启动入口未显式传入，实际使用作者默认0.1秒，存在配置未贯通，不冒称配置值已生效。
- 已读本地厂商Python SDK V1.1.1.1手册3.12.1–3.12.2：官方更新周期示例0.02秒、前瞻0.2秒；HR6.5.19以前参数只表示周期，不能将新版刚度8000直接套用当前记录HR6.5.13。手册未据此给出当前机型可任意提升到250/500Hz的保证。
- 当前发送循环每次PushServoJ后同步执行完整位置/速度/电流/指令位置及三路IO读取，再推进固定deadline。若读取超期则没有sleep、可能追赶发送；代码路径存在节拍抖动风险，但旧日志无逐次发送时间，尚未实测归因。未在用户平板操作期间发运动、IO、重启或修改运行参数。后续先记录实际发送间隔／SDK耗时，保留吸附监测，避免未经测量直接提高频率。

### 2026-10-26 吸附等待与加速度减半

- 用户手动启动 `live_four_20261026_021012`，本聊天仅只读监听。首件贴合成功；DO0开启后约0.972秒DI0首次变高，约28毫秒后回落，稳定计时正确清零；再次变高并连续1.011秒后才发抬升，开阀至确认共2.094秒。抬升过程没有记录吸附丢失，但在第1094个PushServoJ处控制柜进入RobotCollisionStop，SDK返回20018，首件未到入舱／放置、累计完成0/4。保留原失败，不自动复位、重试或释放。参考10°/s²设置生效且执行器点数未再扩张（1345→1345、1496→1496）；这些只能证实参考与发送行为，不能证明实际加减速已平滑或排除实际碰撞／负载模型等控制柜触发原因。
- 只读核对旧 `tools/three_box_original/manual_hardware.py`：开DO0后每50ms读取反馈，连续匹配1秒才确认吸附；12秒为超时而非固定等待。当前按用户要求将桥接默认连续稳定确认从0.3秒恢复为1秒。此前原地吸附/释放实测可用不等于移动持箱已通过。
- 用户要求加速度减半：参考上限20→10°/s²，全部阶段速度仍为20%（共同60°/s基准下最高12°/s）。桥接对包括放置、撤回在内的每段发送参考按同一上限整段延时，保留路径与事件；不改变控制柜100°/s²参数，不重启现有执行服务，不下发运动或泵IO。新增发送点数和实际参考时长日志；尚未通过下一次真机周期验收。

### 2026-10-26 真机执行服务与四件累计规划衔接

- 现场SSH、控制柜新鲜回读StandBy33，启动隔离r2中同学原版`trajectory_executor`，PID15420。实际参数回读real/servo_j；动作服务、DO0/DO1服务、关节与真空IO订阅均取得真实反馈。原连接逻辑尝试Connect2Box/Electrify/Connect2Controller被控制柜以当前StandBy状态拒绝并按原版处理，未执行reset/enable、轨迹或泵IO；不能把启动服务说成物理周期完成。服务器保留空闲等待，不重复启动。
- 新鲜D555采集30 RGB/30深度，接收时刻配对SDK读数差约3.37ms，仅软同步、不称标定采集同步；当前拟合约61.85×42.26×25.43cm。`fresh_online_prepare_20261026_attempt1`的拿取参考成功，放置worker因旧uniform20初始化限制失败，原日志保留。仅为当前观测与已提交前缀增加显式catalog模式，旧uniform20默认不变，修复后`online_prepare_catalogfix_20261026_attempt2`通过。
- 首件加后续`planned_prefix_four_20261026_attempt1`的三件，共完成同一记录箱型重复输入的4件累计占据离线规划，逐件后台计算6.123/9.478/8.026/6.260秒，初始化另列；不冒充四个新感知箱、物理仿真自然落稳或真机完成。各后继场景累计B1至B4，前三件平铺贴邻，第四件叠放。用户指定放置位置按规划位姿估计，不要求释放后重观测。
- `online_session.py`衔接每轮真实observe、主机并发place/pick准备、Jetson执行拿取、入舱等待后台结果、放置与空载返回、计划占据写回下一轮。默认不执行；显式执行不能复用旧观测。`ros_cycle_bridge.py`复用原ServoJ动作和真空服务，接起点/事件与取消反馈，不改原控制器。Jetson此版缺少`rclpy.parameter_client`导致首次接口联调导入失败，已改ROS原生GetParameters服务，第二次只读接口联调实际成功。尚未发送动作，吸附DI由0到1、实际释放与四轮时长仍待现场执行；不把接口就绪称已完成硬件验收。

### 2026-10-26 放置占据采用规划位姿假设

用户明确要求当前不做释放后位姿重观测，按规划落点更新下一件的占据。已将新循环清单改为`commit_planned_occupancy`：真实轨迹与释放IO序列成功返回后写入下一轮scene_state，包含累计catalog、occupied及placed_world_poses；记录`occupancy_source=planned_release_pose_assumption`，不称实测落稳。`cycle_planned_occupancy_attempt2`已生成首件的后继场景预览；预览不等于硬件已经提交。新增`ros_cycle_bridge.py`对接同学现有ROS real/servo_j动作与DO服务，不自动启动/使能/reset，不用仿真吸附回执替代DI反馈；本轮只生成配方并同步Jetson隔离r2，未执行动作、未开泵、四轮真机尚未完成。旧要求实际占据观测的记录保留为历史，不再作为当前流程的新增前置条件。

10月25日本轮真机衔接：已生成同一真实观测的四段ServoJ参考及显式开泵/释放事件清单，入口六关节衔接差为0。`deployment_ready_20261025_r2/cycle_moved_attempt1/cycle.json`记录参考运动总时长31.88秒，不含真实IO确认和等待；仅为待执行计划。`prepare_observed_place.py --scene-state`新增实际已提交前缀输入，不再只能空舱规划；新增多件输入尚未实际运行验收。四个修改文件已同步Jetson隔离r2，原交付未动。完整硬件事件执行、释放后实测占据提交、四轮闭环仍未完成，没有启动运动，不以仿真冻结释放替代实机完成。

当前方向（10月25日最新）：保留原三箱cost与在线占据，布局脚本的前沿贪心默认不覆盖；用户随后明确选中Contact V2，现通过显式选项完成统一20寸四箱及3×20＋1×24随机顺序的完整理想运动学异步流程，并保存同轮候选/观测/机械臂联合录像。混流使用当前观测匹配20/24名义尺寸，不是任意尺寸纯观测闭环。**旧前沿基线的“靠右早于深排”缺陷仍保留记录；本轮不是恢复旧PPO，也不是已完成真机装载。** 用户已确认硬件未通电，当前只在本机仿真；定时发送不恢复，旧训练/真机不重启。

最新只读连接修正：用户随后确认Jetson已开机，本机经局域网192.168.3.12成功SSH认证到`hkureconova`，不能继续称Jetson离线。Tailscale两端均异常：本机控制面注册超时且无peer列表；Jetson为NoState，系统/RTC停在1970-01-01、NTP active但未同步，TLS明确因时间早于证书有效期拒绝控制面连接。本轮只读诊断，没有修改时间/代理/路由/配置或连接机械臂SDK；该事实不等于相机、控制柜或吸附已就绪。更早断线与关机记录保留为当时状态。

- 传统三箱：保存了完整仿真及历史真机执行证据，但真机有续跑、落点未复测等条件，不能称高成功率自主系统。
- 固定32／41件：有完整理想仿真及视频，特别是41件采用理想导向／释放固定支撑，不能当自然堆叠、在线泛化或真机容量。
- 几何学习：先后做过自写候选PPO、复用PCT编码器、作者GOPT主干训练。部分容量提高，但未证明所需顺序、紧凑和机器人可执行性共同达标。
- 机械臂残差PPO：三组长预算最终明显劣于零残差参考；属于必须报告的负结果。
- 随机尺寸教师：全20控制明显退化，后确认奖励依赖未观测的历史状态；保存暂停。修正仅CPU验证，未训练成新策略。
- 同学规则旧复现（10月19日）：全20寸30来件几何接受30件；混尺寸26/30且搬移5次。旧首箱IK到目标上方30mm，不是30次机器人闭环。
- 新现场隔离对照（10月23日）：三叠与2+1完成独立入口放置/撤回运动学诊断；平铺偏好第三件回退2+1。有限domain优先减少重复不可达查询；实际6秒Pick运动学回放＋新单箱求路，串行12.34秒、异步6.52秒。不是完整地面Pick／自然释放／真实起点连接或真机验收。
- 标定更新（10月23日只读核对）：Jetson感知启动参数所指向的安装版配置，与同学`scene_tf.loading_site.yaml`一致；相对旧粗配置Y−11cm、Z+10cm，完整RPY未变。相对旧用户−6cm偏移版则Z+16cm。旧文件未覆盖；配置一致不等于本轮独立测量完成。

状态来源优先级：实际运行原件／checkpoint元数据 > 实验summary与protocol > 本总track > 旧聊天结论。`active_current.json`里同时保存多代记录，有些旧字段仍写ACTIVE，不代表当前任务应恢复。

## 2. 阅读与整理范围

本次通过聊天读取接口回看可访问的S20相关聊天，并与本机原始结果、已有交接和开发track交叉核对；不是仅复述本聊天。对话包含大量重复心跳，本文件合并重复状态，保留关键变化、负结果和纠错。

| 聊天 | 对总track的主要贡献 |
| --- | --- |
| S20 (RL) | 早期几何训练与完整物理后端脱节；0完整周期、力矩／跟踪与任务接口问题 |
| S20 (真机运动) | MoveJ／ServoJ／MovePathJ、吸附与撤回、软件降速、感知障碍归属错误 |
| S20 (找种子） | 历史轨迹复用、候选调度、6–7秒命中条件与随机场景变慢 |
| S20 (Issac 传统三箱) | V1–V4、三种布局、第四箱、5/6及7/8外推失败、随机地面三箱 |
| S20 (Issac 32箱) | 固定32／41、几何37件溯源、P／U耦合、第二件碰撞与返回失败 |
| S20 RL | 作者工程、全20／混流、官方规划与残差PPO、教师奖励诊断、Newton复现与标定 |
| S20 RL (2) | 观测训练推进偏差、资源／NAS记录；不把清理工作写成算法收益 |
| 评估两项强化学习装箱任务 | 模块边界、完整周期与局部成绩、同流评估与泛化定义 |
| 讲解 RoboPacker 论文（两条同名聊天） | 正文缺失、作者代码输出被覆盖、不能将公开代码等同论文完整复现 |

聊天分页读取覆盖单独记录在[来源与媒体清单](research_media/source_coverage.json)。如果接口不能返回某段，明确保留缺口；不会声称读过不可访问或从未保存的每条输出。原三箱和固定Demo已有详细历史，本文件引用它们而不复制所有版本日志。

已有持续记录确实存在，但分散，且总入口没有及时对齐：

- [传统三箱旧总结](/home/ubuntu/Documents/ChatGPT/CPS%20Intern/S20_LEGACY_SUMMARY.md)
- [固定32–41件Demo](/home/ubuntu/Documents/ChatGPT/CPS%20Intern/S20_DEMO.md)
- [早期学习路线／实际训练审计](/home/ubuntu/Documents/ChatGPT/CPS%20Intern/S20_RESEARCH.md)
- [10月12日起传统开发track](/home/ubuntu/Documents/ChatGPT/CPS%20Intern/artifacts/s20_development_track_20261012/DEVELOPMENT_TRACK.md)
- [全量历史track](/home/ubuntu/Documents/ChatGPT/CPS%20Intern/artifacts/s20_full_development_track_20261012/README.md)
- [近期逐项轨迹](TRACK.txt)、[交接](HANDOFF.md)、[当前实验索引](active_current.json)

## 3. 研究日记：按问题、尝试、证据与修正记账

### 早期至10月1日：过早把几何放下当成机器人完成

**问题。** 希望实现持续来箱、布局P与执行U交替、实际落点更新，而不是固定三箱或槽位播放。

**做过。** 搭候选环境、mask、PCT输入、MoveIt／implicit-SVSDF／KC接口、Isaac跟踪与分阶段状态机。

**没做对。** 部分`env.step()`只做几何边界／支撑后直接提交位置，完整运动后端虽然存在，却没有进入rollout。代码检查通过不证明机器人闭环；当时完整物理周期为0，相关四组几何在线训练随后停止。

**进一步失败。** 5kg负载诊断出现模型200Nm上限与姿态约237.5Nm静重力估计冲突，Isaac跟踪尚未通过。这是模型／控制／姿态问题，不证明真实S20不能承载，也不能靠改大参数宣称硬件验证。

**得到的认识。** 布局可行、端点IK、全程运动、自然释放、撤回与占据提交是不同接口。若上层只按几何训练，应诚实称几何教师，不包装成完整robot-aware RL。

### 10月6–8日：种子确实能提速，但固定命中不能代表泛化

**做过。** 用师兄2.5D支撑候选与Atlas作先验，历史完整阶段曲线热启动，再对当前起点、吸附关系与场景修正；不是盲播放旧轨迹。

**结果。** 固定工位经验命中的主规划约5.97／5.68／7.05秒；加入前瞻约8–9秒。另一随机三箱试验只有1/3整轮、7/9箱登记，一次成功的三轮搜索合计163.83秒。

**原因线索。** 过早给很多候选逐一求完整轨迹；浅处候选优先占掉搜索预算；入口代理曾用X=0而非真实入口X=-0.745；首次成功后比较窗口太短。增加种子数量不能修复候选调度。

**真机成果与失败。** 10月8日旧MoveJ带箱三箱最终返回HOME，但含修正／续跑与SSH恢复，落点未复测。随后ServoJ关闭泵演练7/9，不能算九次真实带箱。一次抓取抬升被目标箱轮子附近的`rgbd_component_11`拦住，疑似目标附属物重复作为障碍，尚不能仅凭投影确认归属。

**控制提速教训。** 跟踪误差包络把约0.2秒时延按各关节独立最坏组合，导致11.32秒接近／贴合参考被拉到26.18秒，实测26.55秒。MovePathJ还曾被未运动腕关节的速度比例拖慢。单轴10°往返修正后5.81秒，不等于正式多轴带箱已提速。

### 10月9–16日：传统cost迭代，少件成功与数量外推失败并存

| 阶段 | 主要改变 | 实际成绩／问题 |
| --- | --- | --- |
| V1 | 师兄候选、平放／侧放、MoveIt＋MINCO | 有三箱链，腕部大绕转 |
| V2 | 最大面吸取、工具朝下、yaw0/90、绕转代价 | 姿态更符合任务，仍有等待和返回缺口 |
| V3 | 布局cost、支撑、增量占据、反馈和返回 | 固定三箱完整；早期悬挑版本撤回 |
| V4 | 同时改变支撑、预算和权重 | 因难以判断收益、按用户要求弃用 |
| v31→v36 | 修正候选内归一化、热启动与重复分支 | 三种布局完整；规划合计较v31约降19%–33%，非严格单变量消融 |
| 第四箱v33 | 使用前三箱实际落点继续规划 | 4/4完整周期 |
| 5／6件 | 数量外推、候选去重 | 仅4/5、4/6；均第5轮入舱受阻 |
| 7／8件 | 更长前缀与路径修复 | 聊天记录为5/7、5/8，未完成整批 |
| 随机20–24寸地面三箱 | 同工位、位置／yaw／代理尺寸变化 | 三组各3/3；共9个理想吸附仿真周期，不是统计泛化结论 |

v36平铺／两叠一铺／全叠规划合计134.74／119.33／138.73秒，整轮墙钟313.30／282.63／315.39秒；第四箱262.73秒规划、466.79秒墙钟。随机三组规划170.05／144.33／152.33秒、墙钟318.40／294.32／420.86秒。不能用删除规划等待后的录像长度证明系统周期很快。

**认识。** 规则法已能产生有价值的小规模闭环，不必因没有RL就贬低它；反过来，3/3也不能直接外推32件。终点、箱体进入通道、转腕时机和空载返回共同决定是否可执行。

### 10月9–14日：固定32到41件，工程Demo成功但假设很强

- 水平32件：三次独立空舱整轮；每轮130阶段、65092条实测记录，最终32件同时稳定。
- 侧放／上层：大量条件后缀试验，不能把静态前序与新增周期拼成新的空舱容量。真实失败包括邻箱封住转姿通路、自然释放后支撑不足、返回失败，以及部署接口遗漏。
- 最终v1312：同一世界空舱连续41件，207阶段、144876条记录；仿真时间2896.364秒、进程墙钟9247.486秒。
- **关键条件：32水平＋4侧面＋5上层，理想吸附、理想导向及释放时在实际pose固定支撑。** 未把箱子搬回名义槽位，但仍不是自然释放稳定性的证明；侧放也不满足后来“全部最大面水平”的任务限制。
- 使用固定目标／成功曲线和顺序，没有PPO在此整轮在线选位。公开归档中的数值平滑、PPO辅助对照不能追认原41件是学习成果。

**认识。** 这条线证明了复杂场景的路径复用、执行和接口整合能力；没有证明任意工位、随机来箱的学习策略。汇报可保留，但必须单列理想化条件。

### 10月中旬：自写／PCT几何PPO并未稳定胜过规则

**做过。** 候选MDP、动作mask、PCT图编码器、BC→MaskablePPO；三种子各200704几何决策，保留权重和全部失败评价。三种子训练＋评价墙钟约2.09／2.11／2.12小时，不能相加当单次串行耗时。

**同流比较。** 32来箱流平均接受：旧规则23、新规则25、三种子学习策略22／25／28。全部32箱流完整完成数均为0；不能只挑28的种子宣称普遍优于规则。总602112条训练决策的回报审计不等于物理周期成功数。

**缺口。** 几何奖励没有实测时间／effort，配置写有系数也不等于训练使用了它们。M／I／joint接口和计划预算不是已经完成的物理学习。

**认识。** 学习有局部收益，但不稳定；弱规则／强规则、多个训练seed、相同箱流和相同约束都要保留。不能把“用了神经网络”作为选它的理由。

### 10月15–17日：37件溯源与P／U耦合问题

**37件真实来源。** `uld_online_rl_20261015`自写几何候选MLP＋SB3-Contrib MaskablePPO，seed1516、64env、262144决策、rollout64、batch512、epochs4；不是作者完整GOPT／STEP。记录`_n_updates=256`是优化epoch累计口径。

**尺寸／容积不同。** 55×40×25cm、yaw0/90；五层8＋8＋8＋7＋6，装载高1.25m，只余顶部23cm。不能与后来1.00m装载高下35件直接比。该视频没有机械臂运动。

**较难物理任务。** 旧局部放置254/256与入舱251/512不是同难度。第二件失败诊断中，403次失败有303次箱撞箱，碰撞时距目标中位数仍约54cm；说明下层过早下降，不能只归因落点相交。另一批335次第二件失败中276发生持物、59发生释放后，固定撤回也会自碰撞。

**尝试与边界。** 2cm目标间隙、抬升／返回修正、BC／PPO等隔离对照，不能把放稳后返回失败记成完整成功。最终原计划记录用户暂停六训练分支，不恢复旧课程。

**认识。** P生成的目标分布与U的训练起点／障碍分布不匹配时，两个模块单独好看也接不起来；控制／返回脚本可能主导成败，不能全部归给RL。

### 10月17–18日：回到作者GOPT，容量和顺序分开验收

**选择理由。** GOPT用EMS＋Transformer＋MaskedPPO，适合当前件已知、在线选落点；原奖励为成功装入体积比例。STEP的缓冲选件／抓取面并非当前单件已吸稳任务的直接替代。

**全20结果。** 三组各8.192M决策，冻结最终贪心均35件，但晚回低层9／17／20次，同层返深6／8／4次。最低z→后壁priority规则辅助仍35件、顺序计数0；33步是在同一anchor选两个yaw、2步只有一个位姿，因此规整位置主要由规则给出，不能称网络学会了整套顺序。

**名义混流。** seed23、128env、rollout5、epoch1、batch128，8.192M有效决策／12800轮／64000梯度步；训练墙钟58204.644秒，共享CUDA上下文16.203小时，非独占GPU收费。

| 评价条件 | initial | 容量best（6291840步） | final |
| --- | ---: | ---: | ---: |
| 两尺寸体积利用率 | 48.926% | 58.559% | 58.303% |
| 四尺寸体积利用率 | 56.514% | 61.942% | 60.958% |

有容量增益，但顺序和持箱通道未通过。best不能替代final，也不能跨不同高度／箱流直接比较。

### 10月18日：官方规划／控制接入与机械臂RL负结果

**官方执行成果。** 037同一PhysX rollout从空舱连续5件落座、撤回ACK，理想已吸稳5kg初始化；释放误差0.57–6.64mm。只覆盖一个同层深度anchor，不是35件全舱。039从5箱快照独立完成第6件，不能拼成空舱连续6件。040／041专家查询12/18、5/6通过，仅路径候选，未等于BC或动态释放成功。

**运动质量尚差。** 037切换jerk峰约1262.44，039约1264.57；保留原曲线，不能只展示成功视频忽略尖峰。

**三组残差PPO。** 复用官方RSL-RL PPO训练器并适配S20单路径任务；不是官方原任务无修改复现。每组98.304M步，4096env×rollout24×1000轮，8epochs×4minibatches＝32000梯度更新；seed31／47／31（第三组改终止奖励且含迁移）。同192集零残差／initial均192/192，三组final仅0/192、5/192、3/192。

**诊断。** raw Normal std最大约12.82，而执行残差经tanh；原始熵增长不等于有用动作探索。关闭熵的9.928704M／101轮对照仍0/192；再移除正proximity也0/192，且192次timeout。后者末速度下降但误差约41.08mm，不能把“更慢”当成功。

**认识。** 对已有效的参考路径叠残差RL，必须先说清要改善什么。单路径长训练在本适配中没有收益；熵是候选机制，有限消融不支持“只改熵就修好”的结论。累计三组294.912M只代表交互量，不能与几何packing决策数混加成统一有效样本。

### 10月18–19日：观测路线先行后纠正为教师优先

**做过。** 原生Isaac理想深度14帧，深度→点云／高度图／未知→冻结GOPT；部分提议仅IK+self通过，无world证明。先筛可行再截80EMS能修观测候选遗漏，但不是网络学会。

**执行偏差。** 曾一直做冻结前向和专家查询，未及时交付用户要的观测RL；后来实际开训。Actor用简化三视角Warp射线深度，不是把真值高度图加噪冒充相机。

**实际停止状态。** 观测单卡日志181760、保存179200；四卡日志新增8960，未保存尾步单列。用户明确先理想特权教师，故保存暂停，不应称观测策略已经学好或失败收敛。

**性能诊断。** 相同12深度投影CPU平均128.586ms／CUDA4.638ms，局部约27.72倍；这不是完整RL吞吐27.72倍。

### 10月19日：特权教师早期顺序改善，几何统计域也出过错

全20教师从旧all20final骨干开始：163840→327680评价保35件，晚回11→0、返深4→1、提前升层23→0。后查底部void55000cm³中6749.31cm³属于CAD斜壁外，仅修正统计域，真正内部48250.69cm³保留；评分减少不算学习收益。

随后用户授权随机尺寸，旧全20日志858240、保存857600，10小时watcher取消。不要把暂停尾步或计划长预算写成已训练完成。

### 10月19日：随机教师退化，不能再仅回答“继续跑预算”

**任务。** seed59、4rank同一模型，混合20／24／28／32代理每轴真实±10%并1cm取整，容器尺寸／高度条件化；128逐件arrival，不窥未来，不搬旧箱。准确mixed6291840骨干＋新10维context＋freshAdam。计划20.48M并未完成。

**并行。** 正式比较1024／4096／8192env，热态423.29／460.03／502.06有效决策/s，按最快10%内较小方案选4096，32CPUworker、rollout5、epoch1、globalminibatch128；各阶段样本和梯度保留。显存占满并非目标。

**结果轨迹。** 初始mixed G/S利用率52.773/53.899%；5.775M约59.827/60.665%，12.595M为59.001/58.505%，13.639M为57.786/56.428%。G晚回／返深／提前升层最终约2.688/.781/.313，初始12.031/2.5/9.031。顺序代理变好并不等于布局可执行。

**全20控制恶化。** 初始32件，期间长期24件，13.639M仅18件。先前把它解释为分布外波动并持续预算，诊断推进不够及时；即使进程健康、loss正常，也不能证明任务实现正确。

**确认的错误。** 两段不同放置历史有完全相同obs／mask／heightmap，同下一动作奖励却为-0.034749与-0.009369：`last_anchors`隐藏历史影响返深罚分，但没有被obs充分表达。严格说，应让奖励依赖的必要状态在obs中可辨，或移除该历史依赖；未知未来箱子本身不是这里问题的理由。

**冻结因子对照（0新RL）。**

| 权重 | 扩展候选 | 原候选 | 加最低z／后壁priority |
| --- | ---: | ---: | ---: |
| 旧all20final | 35 | 35 | 35且顺序0/0/0 |
| mixed initial | 32 | 34 | 34 |
| 当前13.639M | 18 | 24 | 24 |

旧35逐件在当前CAD完整重放；675个mixed状态无top80候选丢失。故不能说“现在舱体放不下35”或一概归因80截断。候选适配解释了部分18→24，不能解释旧权重35与新权重24；纯all20流不在随机训练分布也是退化可能因素。**这些证据没有量化每个因素的因果占比，也没有证明奖励全局偏爱18件。**

**暂停。** 最后日志14,172,160步；准确保存13,824,000步／680轮／108000同步SGD，未保存尾348160。13.639M是上述诊断评估权重，不是暂停时最后保存计数。原件备份，env／RNG未存，不称完整逐步恢复。

**最小修正方案。** A为作者正体积奖励；B为同体积＋0.05×(Phi后−Phi前)，Phi基于TAP C/P与明确S20适配的rearfill。gamma1下累计shaping差值有界，不让逐步历史罚分吞掉容量。旧all20骨干、零context、freshAdam；同seed61、512env、8worker、4rank、rollout5、batch128、epoch1，各1.024M／400轮／8000SGD，25%exactall20＋25%nominalmixed＋50%随机mixed。**仅protocol／CPU验证完成、0新训练，后来用户转论文阅读，没有运行成效。**

### 10月19日：tight packing文献没有自动解决容量

TAP的C/P参照同流CPU33episode／675决策，利用率52.635%，同流PPO59.001%；底部void更少但容量更低。不能因为指标名字叫compactness就立即叠入奖励。

Tetris空洞、井与轮廓、TAP紧凑／填充、PPN的SDF评分各有启发，但原任务允许的重排、开口、物体形态与S20不同；读过、离线算过不等于已经进入训练并验证收益。此前没形成系统的“作者基线→同任务对照→单项适配→实际训练”是实质缺口。

### 10月19–20日：同学贪心＋swap复现，先把强规则基线跑通

上游`ludan0414/newton_box`，commit `3ade6e48b09b9fc3f91f94aef1e5d1c210f1b802`。原源码保留，当前隔离运行Newton1.7.0／Warp1.18.0／Python3.11.16；README写Newton1.3，不能宣称完全同版本复现。未发现顶层LICENSE，不据此自动公开再分发。

**逻辑。** 墙／旧箱边界生成候选，过滤舱体、重叠、全底支撑和箱体进门路径；真实舱体版优先深处，再低处、再侧边及接触分数。直接放不下才抽走一个顶层可取旧箱，放当前件，再放回旧箱。需要外部暂存；这是规则／搜索策略，不是RL，也没有看未来来件。

**原样CPU对照。** seeds1/8/13，原六朝向（允许侧立）：矩形greedy 9/8/7，＋swap 9/10/9；舱体greedy15/16/12，＋swap15/17/12。swap在个别流有收益，不是每流都改善。README历史数字与本commit实测不同，以日志为准。

**适配当前全RPY、平放两yaw后：**

| 场景 | 结果 | 规划耗时 | 边界 |
| --- | --- | --- | --- |
| 全20寸30来件 | 30接收，0拒收，0swap | 中位5.58ms，最慢11.85ms，总0.223s | 未测装满，机器人静态；深优先会先堆高后补前面低处 |
| 20–32寸30来件seed8 | 26接收，4拒收，5swap | 中位0.356s，最慢61.77s，总146.08s | 允许搬移，与原不可重排约束不一致；历史场景尚有取箱台 |
| 原IK路径组30来件 | 0接收／30拒收 | 243.01s，8204目标求解尝试 | 不能当硬件不可达证明，接触判据需诊断 |
| 修正碰撞核对后单20首箱 | IK候选9.86s＋运动生成4.96s＋550点复核0.47s | 合计约15.29s；名义轨迹9.17s | 理想吸附，停目标上方30mm，无释放／退回／动力学 |

**IK问题。** Newton原接触判据存在正间隙也拒绝；同一被拒姿态按相同CAD的FCL复核，link5到ULD约251.43mm。不能推断所有旧接触都是误报。低入口0.4m确实使持箱撞台座；改0.5m入口，目标／TF未改。

**单箱运动边界。** FK与原URDF差约2.22e-7m／9.32e-6度；550个60Hz离散状态通过原CAD检查，非连续扫掠保证。工具相对ULD最大倾角2.80°，关节参考速度1.484rad/s；分段插值加速度145.90rad/s²、jerk9102.58rad/s³，尚不平顺，不应直接部署真机。

**10月20日场景。** 导出师兄schema的`scene_tf.yaml`，移除取箱台／黄色标记，保留机器人台座、原工具与ULD，原生Newton加WORLD／S20／ULD的XYZ轴。静态场景加载成功不是标定精度验证。用户提出约下降6cm，待核对是世界竖直方向、基座方向还是ULD局部轴，先保留原标定与明确偏移分支。

## 4. 论文：读过、跑过、适配过分别是什么

| 方法 | 实际材料／复现层级 | 项目中实际作用与缺口 |
| --- | --- | --- |
| GOPT | 原TeX、作者训练／EMS／网络源码，commit a2e42de1c0ab62c5e0a356e363349c32beb9e05b | 近期packing PPO主干；CAD、候选、context、奖励为适配，不把适配错误归原论文 |
| STEP | 原TeX和两阶段作者源码，commit 6f362a4e11877555ec923151638aec60eb091ac1 | 选件／抓取面及空间—时间偏好；不是下层关节控制；并非GOPT同一作者团队 |
| Online 3D Bin Packing with Constrained DRL | 原TeX／作者源码与发布权重核查 | 可行mask和在线决策参考，未完成S20整套适配训练 |
| Learning Practically Feasible Policies… | 原TeX相关方法／奖励／机器人章节 | 从远到近与稳定性参考，不能写成已完整接入 |
| PPN-Pack | 原PDF（未取得TeX）、源码；checkpoint strict load＋CPU前向 | 监督候选排序＋几何SDF，不是PPO；未S20闭环 |
| Tight Robot Packing | 原TeX＋作者旧Blender模拟器 | 给定目标布局的接触／感知纠偏，不负责在线混尺寸选布局；不能照搬可变形纸箱壁原语到刚性ULD |
| TAP-Net | 原TeX、C/P、EMS／MACS代码与CPU对照 | 规划／评分参考；global重排能力不符合当前不可重排，不等于迁入PPO成功 |
| Tetris相关两篇 | 原TeX目标章节、离线指标 | 0新RL，不能把游戏策略当装箱复现 |
| RoboPacker | 作者摘要／仓库／网络核查，准确正文／TeX未取得 | 公开快照学习输出被手工分数覆盖、packing权重缺失；不是HRL部署复现 |
| PCT | 历史图编码器有实际训练使用；其他版本仅资料核查 | “复用编码器”不同于原论文完整复现 |
| implicit-SVSDF／KC／GSIP、EAIK等 | 局部原型与保存路径对照 | 部分几何／IK加速，不继承整条新路径的连续证明或完整论文成绩 |

准确作者、commit、许可证、材料缺项与阅读范围以[独立审查目录](/home/ubuntu/Documents/ChatGPT/CPS%20Intern/reviews/2026-10-17_s20_independent/)及各`source_review.json`为准。不要把列表长度当成适配完成率。

## 5. 可以汇报的insight，以及不能说的话

1. **结构化规则是应认真建立的强基线。** 本项目中最低层／后壁规则能让旧GOPT35件规整，简单greedy也能毫秒级接收30件。同一方法不需要通过训练才能叫policy：policy是状态到动作的决策规则，learning-based描述它如何得到。
2. **尚不能下结论“规则全面打败RL”。** 已有对比的高度、来箱、支撑、是否允许swap、朝向和机器人验证层级不完全一致。可信结论是：当前若干RL适配没有证明比可解释规则更值得；单路径残差PPO明确输给零残差基线。
3. **几何紧凑、最终容量、装载顺序和机械臂可执行性是四个指标。** 少孔洞不一定容量高；终态无重叠不等于持箱能穿门；深优先可能先堆后排，不等于整层先铺满。
4. **奖励必须与可辨状态及任务目标一致。** 隐藏历史依赖已被反例确认；重复空洞／高度罚分可能掩盖体积目标。修正需要固定分布／预算对照，不能继续跑旧预算当诊断。
5. **接口与候选是方法的一部分。** 同权重18→24、旧权重35、零残差192/192→训练后接近0，是可以定位问题的证据。loss正常、权重确实变化、GPU很忙都不是交付指标。
6. **路径复用有工程价值，泛化仍需测量。** 固定工位几秒命中与随机场景几分钟不是矛盾；应报告首选／回退、查询预算和真实墙钟。
7. **恢复前序不能拼接完整成果。** 039第6ACK、条件侧放、上层预检查均有价值，必须标明起点，不能拼成新空舱整轮。
8. **接触／标定误差是tight placement瓶颈。** 应先核对TF与实际工具，再接可验证规划，最后考虑放后纠偏；粗标定偏差不能靠网络训练或挪场景掩盖。

汇报建议主线：需求与任务拆分 → 传统少件闭环 → 固定32/41的工程能力及理想假设 → 几何学习与规则的同流正负结果 → 接口／奖励诊断 → 回到强greedy基线 → 下一项可测实验。失败是产生这些认识的证据，不是需要删掉的部分。

## 6. 可视化与统计账本

精选素材已归到[分阶段媒体入口](research_media/)，使用指向原件的链接，避免复制大视频或制造多个版本。全部已发现本机媒体路径、大小与阶段见[media_inventory.tsv](research_media/media_inventory.tsv)；连续帧PNG保留在原位置，清单数量不是独立实验数量。

**素材仍须按证据层级使用：**几何布局／动作回放／离散运动学／动态仿真／真实执行分开；加速片、剪掉规划等待的片子注明；旧图标题或旧聊天“当前”以源日期解释。清单只统计本机可读取文件，不声称远端唯一原件已全部回传，本次未走VPN拉取大媒体。

| 汇报素材 | 主要用途 | 必须附的限制 |
| --- | --- | --- |
| 传统三箱、第四箱及随机三箱 | 完整少件流程、前缀更新 | 已知pose／理想吸附，小样本 |
| 固定32和41件三视图 | 路径复用、工程整合 | 固定顺序；41理想导向／固定支撑 |
| 37件几何布局 | 几何学习早期结果 | 五层1.25m、自写PPO、无机械臂 |
| 全20规则辅助35与mixed曲线 | NN／规则贡献、容量和顺序分离 | 不同条件不可混分母 |
| 第二件失败视频、机械臂PPO诊断 | 中段碰撞、固定返回、学习退化 | 保存失败，不改标成功 |
| 随机教师曲线与all20退化 | 非单调学习、评估回归、诊断 | 旧奖励有确认缺陷，未交付教师 |
| Newton30件与首箱IK/FK | 强规则基线、几何和执行分层 | 30件不是装满；单箱只hover无释放 |
| TF场景与XYZ轴 | 核对相对位姿 | 粗标定，尚未下降6cm重验 |

训练预算不要粗加：几何决策、物理控制步、失败前截断步和评估回合的语义不同。上文列出关键seed／env／rollout／epochs／minibatch／transitions与已证实墙钟；原件没有的GPU成本明确缺失，不按瞬时占用推造GPU小时。

## 7. 接下来怎样维护

每次有实质进展，更新本文件对应日期及本节当前状态：**问题→准确基线→改动→固定输入／预算→实际结果和全部分母→成功／失败原因的证据强度→下一决定**。补上原JSON／protocol／checkpoint／视频链接；训练计划与真实launch分开。更正旧结论时保留原结论及为何更正，不抹去失败。

不因为维护track恢复定时发送、训练或真实机械臂操作。用户约下降6cm和Jetson连通性是待办；先明确偏移方向、保留原TF，再做独立场景对照。下一真正的算法验收应在同场景／同箱流上，先对无swap规则、greedy＋swap（允许重排的参照）和冻结GOPT分别记账，再接连续实际机械臂周期。不能为复现同学代码而悄悄改掉原不可重排约束。

## 8. 精选媒体逐项入口（实际核对文件存在）

| 阶段 | 媒体 | 原件 |
| --- | --- | --- |
| 01传统三箱 | [three-cases-comparison.mp4](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/01传统三箱/three-cases-comparison.mp4>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/artifacts/three_box_spatial_20261011/summary_20261016/three-cases-comparison.mp4>) |
| 01传统三箱 | [layouts-all.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/01传统三箱/layouts-all.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/artifacts/three_box_spatial_20261011/summary_20261016/layouts-all.png>) |
| 01传统三箱 | [timing.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/01传统三箱/timing.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/artifacts/three_box_spatial_20261011/summary_20261016/timing.png>) |
| 02固定32与41 | [demo32_routes_20261009__five_minutes.mp4](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/02固定32与41/demo32_routes_20261009__five_minutes.mp4>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/artifacts/demo32_routes_20261009/clean_smooth_multiview_v312/five_minutes.mp4>) |
| 02固定32与41 | [ideal_empty41_upper_dispatch_fixed_v1312_three_views__five_minutes.mp4](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/02固定32与41/ideal_empty41_upper_dispatch_fixed_v1312_three_views__five_minutes.mp4>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/artifacts/demo32_routes_20261009/ideal_empty41_upper_dispatch_fixed_v1312_three_views/combined/five_minutes.mp4>) |
| 02固定32与41 | [final_three_views.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/02固定32与41/final_three_views.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/artifacts/demo32_routes_20261009/ideal_empty41_upper_dispatch_fixed_v1312_three_views/final_three_views.png>) |
| 03早期学习与失败 | [packing_final.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/03早期学习与失败/packing_final.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/uld_online_rl_20261015/results/media_geometry_layout_v01/packing_final.png>) |
| 03早期学习与失败 | [packing.mp4](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/03早期学习与失败/packing.mp4>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/uld_online_rl_20261015/results/media_geometry_layout_v01/packing.mp4>) |
| 03早期学习与失败 | [recorded_failure_isaac.mp4](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/03早期学习与失败/recorded_failure_isaac.mp4>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/uld_sensor_planning_rl_20261016/results/diagnosis_second_20261017_v01/original_isaac_replay/recorded_failure_isaac.mp4>) |
| 03早期学习与失败 | [failure_path.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/03早期学习与失败/failure_path.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/uld_sensor_planning_rl_20261016/results/diagnosis_second_20261017_v01/failure_path.png>) |
| 04作者GOPT与规则 | [all20_completed_final_numbered.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04作者GOPT与规则/all20_completed_final_numbered.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_gopt_multisize_20261018/height_order_20261018/all20_completed_final_numbered.png>) |
| 04作者GOPT与规则 | [initial_best_final_rule.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04作者GOPT与规则/initial_best_final_rule.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_gopt_multisize_20261018/training/author_mixed128_s23_20261018/evidence_008192000/initial_best_final_rule.png>) |
| 05官方运动与PPO失败 | [diagnosis.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/diagnosis.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/ppo_diagnostics_20261018/diagnosis.png>) |
| 06Newton规则复现 | [Newton_全20寸_无取件台.mp4](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/06Newton规则复现/Newton_全20寸_无取件台.mp4>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_newton_single20_20261019/Newton_全20寸_无取件台.mp4>) |
| 06Newton规则复现 | [Newton_最新标定30件测试.mp4](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/06Newton规则复现/Newton_最新标定30件测试.mp4>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_newton_calibrated30_20261019/Newton_最新标定30件测试.mp4>) |
| 06Newton规则复现 | [Newton_IK与FK_持箱进入.mp4](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/06Newton规则复现/Newton_IK与FK_持箱进入.mp4>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_newton_single20_ik_20261019/Newton_IK与FK_持箱进入.mp4>) |
| 06Newton规则复现 | [Newton_IKFK_门口.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/06Newton规则复现/Newton_IKFK_门口.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_newton_single20_ik_20261019/Newton_IKFK_门口.png>) |
| 07标定场景 | [scene_overview.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/07标定场景/scene_overview.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/exports/scene_overview.png>) |
| 04随机教师与退化 | [随机尺寸教师_学习进展.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/随机尺寸教师_学习进展.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_学习进展.png>) |
| 05官方运动与PPO失败 | [S20_cuRobo_首条持箱接近_原生Isaac.mp4](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/S20_cuRobo_首条持箱接近_原生Isaac.mp4>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/trials/003_neighbor_joint1_branch/results/S20_cuRobo_首条持箱接近_原生Isaac.mp4>) |
| 05官方运动与PPO失败 | [010_gravity_bias_implicit_pd_diagnostic__S20_空载关节控制跟踪_原生Isaac.mp4](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/010_gravity_bias_implicit_pd_diagnostic__S20_空载关节控制跟踪_原生Isaac.mp4>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/trials/010_gravity_bias_implicit_pd_diagnostic/results/S20_空载关节控制跟踪_原生Isaac.mp4>) |
| 05官方运动与PPO失败 | [012_author_retime_quarter_speed__S20_空载关节控制跟踪_原生Isaac.mp4](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/012_author_retime_quarter_speed__S20_空载关节控制跟踪_原生Isaac.mp4>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/trials/012_author_retime_quarter_speed/results/S20_空载关节控制跟踪_原生Isaac.mp4>) |
| 05官方运动与PPO失败 | [016_consistent_quintic_reference__S20_空载关节控制跟踪_原生Isaac.mp4](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/016_consistent_quintic_reference__S20_空载关节控制跟踪_原生Isaac.mp4>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/trials/016_consistent_quintic_reference/results/S20_空载关节控制跟踪_原生Isaac.mp4>) |
| 05官方运动与PPO失败 | [018_lula_scaled_native_tracking__S20_空载关节控制跟踪_原生Isaac.mp4](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/018_lula_scaled_native_tracking__S20_空载关节控制跟踪_原生Isaac.mp4>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/trials/018_lula_scaled_native_tracking/results/S20_空载关节控制跟踪_原生Isaac.mp4>) |
| 05官方运动与PPO失败 | [handoff_numbered.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/handoff_numbered.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/trials/019_packing_handoff_sensors/results/handoff_numbered.png>) |
| 05官方运动与PPO失败 | [S20_全20规划衔接_独立可达目标与传感器.mp4](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/S20_全20规划衔接_独立可达目标与传感器.mp4>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/trials/021_native_sensor_attributes/results/S20_全20规划衔接_独立可达目标与传感器.mp4>) |
| 05官方运动与PPO失败 | [sensor_observations.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/sensor_observations.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/trials/021_native_sensor_attributes/results/sensor_observations.png>) |
| 05官方运动与PPO失败 | [reach_spheres_audit.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/reach_spheres_audit.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/diagnostics/reach_visual_audit/reach_spheres_audit.png>) |
| 05官方运动与PPO失败 | [clear_pose0_view0.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/clear_pose0_view0.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/diagnostics/reach_visual_audit/clear_pose0_view0.png>) |
| 05官方运动与PPO失败 | [live_pose_check.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/live_pose_check.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/diagnostics/reach_visual_audit/live_pose_check.png>) |
| 05官方运动与PPO失败 | [live_view0.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/live_view0.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/diagnostics/reach_visual_audit/live_view0.png>) |
| 05官方运动与PPO失败 | [measured_workspace_comparison.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/measured_workspace_comparison.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/trials/024_measured_workspace_endpoints/results/measured_workspace_comparison.png>) |
| 05官方运动与PPO失败 | [S20_连续5件完整放置_原生.mp4](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/S20_连续5件完整放置_原生.mp4>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/trials/037_continuous_five_validated_expert_library/results/S20_连续5件完整放置_原生.mp4>) |
| 05官方运动与PPO失败 | [ack_numbered.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/ack_numbered.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/trials/037_continuous_five_validated_expert_library/results/ack_numbered.png>) |
| 05官方运动与PPO失败 | [actual_motion_metrics.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/actual_motion_metrics.png>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/trials/037_continuous_five_validated_expert_library/results/actual_motion_metrics.png>) |
| 05官方运动与PPO失败 | [独立第6件_释放撤回.mp4](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/05官方运动与PPO失败/独立第6件_释放撤回.mp4>) | [原文件](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_curobo_placement_20261018/trials/039_sixth_from_five_snapshot/results/独立第6件_释放撤回.mp4>) |
| 04随机教师与退化 | [mixed_nominal_h148_greedy_fixed_seed_prefixes.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/mixed_nominal_h148_greedy_fixed_seed_prefixes.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/mixed_nominal_h148_greedy_fixed_seed_prefixes.png>) |
| 04随机教师与退化 | [mixed_jitter_h100_greedy_fixed_seed_prefixes.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/mixed_jitter_h100_greedy_fixed_seed_prefixes.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/mixed_jitter_h100_greedy_fixed_seed_prefixes.png>) |
| 04随机教师与退化 | [mixed_nominal_h148_worst_stochastic_three_views.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/mixed_nominal_h148_worst_stochastic_three_views.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/mixed_nominal_h148_worst_stochastic_three_views.png>) |
| 04随机教师与退化 | [mixed_nominal_h148_worst_stochastic_prefixes.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/mixed_nominal_h148_worst_stochastic_prefixes.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/mixed_nominal_h148_worst_stochastic_prefixes.png>) |
| 04随机教师与退化 | [mixed_nominal_h148_greedy_fixed_seed_three_views.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/mixed_nominal_h148_greedy_fixed_seed_three_views.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/mixed_nominal_h148_greedy_fixed_seed_three_views.png>) |
| 04随机教师与退化 | [mixed_jitter_h148_worst_stochastic_prefixes.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/mixed_jitter_h148_worst_stochastic_prefixes.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/mixed_jitter_h148_worst_stochastic_prefixes.png>) |
| 04随机教师与退化 | [mixed_jitter_h148_worst_stochastic_three_views.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/mixed_jitter_h148_worst_stochastic_three_views.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/mixed_jitter_h148_worst_stochastic_three_views.png>) |
| 04随机教师与退化 | [mixed_jitter_h100_greedy_fixed_seed_three_views.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/mixed_jitter_h100_greedy_fixed_seed_three_views.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/mixed_jitter_h100_greedy_fixed_seed_three_views.png>) |
| 04随机教师与退化 | [mixed_jitter_variable_container_greedy_fixed_seed_prefixes.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/mixed_jitter_variable_container_greedy_fixed_seed_prefixes.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/mixed_jitter_variable_container_greedy_fixed_seed_prefixes.png>) |
| 04随机教师与退化 | [mixed_jitter_variable_container_worst_stochastic_three_views.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/mixed_jitter_variable_container_worst_stochastic_three_views.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/mixed_jitter_variable_container_worst_stochastic_three_views.png>) |
| 04随机教师与退化 | [mixed_jitter_h148_greedy_fixed_seed_prefixes.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/mixed_jitter_h148_greedy_fixed_seed_prefixes.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/mixed_jitter_h148_greedy_fixed_seed_prefixes.png>) |
| 04随机教师与退化 | [all20_h100_worst_stochastic_three_views.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/all20_h100_worst_stochastic_three_views.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/all20_h100_worst_stochastic_three_views.png>) |
| 04随机教师与退化 | [mixed_jitter_h100_worst_stochastic_three_views.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/mixed_jitter_h100_worst_stochastic_three_views.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/mixed_jitter_h100_worst_stochastic_three_views.png>) |
| 04随机教师与退化 | [all20_h100_greedy_fixed_seed_prefixes.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/all20_h100_greedy_fixed_seed_prefixes.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/all20_h100_greedy_fixed_seed_prefixes.png>) |
| 04随机教师与退化 | [all20_h100_greedy_fixed_seed_three_views.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/all20_h100_greedy_fixed_seed_three_views.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/all20_h100_greedy_fixed_seed_three_views.png>) |
| 04随机教师与退化 | [all20_h100_worst_stochastic_prefixes.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/all20_h100_worst_stochastic_prefixes.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/all20_h100_worst_stochastic_prefixes.png>) |
| 04随机教师与退化 | [mixed_jitter_variable_container_worst_stochastic_prefixes.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/mixed_jitter_variable_container_worst_stochastic_prefixes.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/mixed_jitter_variable_container_worst_stochastic_prefixes.png>) |
| 04随机教师与退化 | [mixed_jitter_variable_container_greedy_fixed_seed_three_views.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/mixed_jitter_variable_container_greedy_fixed_seed_three_views.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/mixed_jitter_variable_container_greedy_fixed_seed_three_views.png>) |
| 04随机教师与退化 | [mixed_jitter_h148_greedy_fixed_seed_three_views.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/mixed_jitter_h148_greedy_fixed_seed_three_views.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/mixed_jitter_h148_greedy_fixed_seed_three_views.png>) |
| 04随机教师与退化 | [mixed_jitter_h100_worst_stochastic_prefixes.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/mixed_jitter_h100_worst_stochastic_prefixes.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/随机尺寸教师_最新编号三视图/mixed_jitter_h100_worst_stochastic_prefixes.png>) |
| 04随机教师与退化 | [greedy_layers_order.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/greedy_layers_order.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/教师_编号三视图/greedy_layers_order.png>) |
| 04随机教师与退化 | [worst_stochastic_three_views.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/worst_stochastic_three_views.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/教师_编号三视图/worst_stochastic_three_views.png>) |
| 04随机教师与退化 | [worst_stochastic_layers_order.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/worst_stochastic_layers_order.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/教师_编号三视图/worst_stochastic_layers_order.png>) |
| 04随机教师与退化 | [worst_stochastic_prefixes.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/worst_stochastic_prefixes.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/教师_编号三视图/worst_stochastic_prefixes.png>) |
| 04随机教师与退化 | [greedy_prefixes.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/greedy_prefixes.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/教师_编号三视图/greedy_prefixes.png>) |
| 04随机教师与退化 | [greedy_three_views.png](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/research_media/04随机教师与退化/greedy_three_views.png>) | [原文件](</home/ubuntu/Desktop/S20研究/01当前推荐/教师_编号三视图/greedy_three_views.png>) |

本次本机媒体清单共28345个文件（包含连续帧和历史版本，不是实验数量），精选62个独立链接；字节数未去重，不作释放空间估计。原始视频、曲线与失败均未移动或删除。

## 9. 关键数值原件

- [奖励退化反例与因子对照](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/all20_regression_diagnosis_20261019.json>)
- [奖励修正协议（尚未训练）](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_teacher_reward_repair_20261019/protocol.json>)
- [机械臂PPO三组及诊断](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/ppo_diagnostics_20261018/diagnosis.json>)
- [Newton全20几何实测](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_newton_single20_20261019/summary.json>)
- [Newton混尺寸及原IK失败](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_newton_calibrated30_20261019/summary.json>)
- [Newton首箱IKFK](</home/ubuntu/Documents/ChatGPT/CPS Intern/records/S20/newton_single20_ik_fk_20261019.json>)
- [TAP原文与源码范围](</home/ubuntu/Documents/ChatGPT/CPS Intern/reproductions/tap_net_compactness_20261019/source_review.json>)
- [Tetris阅读范围](</home/ubuntu/Documents/ChatGPT/CPS Intern/reproductions/tetris_packing_features_20261019/source_review.json>)
- [论文模块审查](</home/ubuntu/Documents/ChatGPT/CPS Intern/reviews/2026-10-17_s20_independent/step_gopt_3dbp.md>)
- [RoboPacker与PPN审查](</home/ubuntu/Documents/ChatGPT/CPS Intern/reviews/2026-10-17_s20_independent/robot_packer_ppn.md>)
- [Tight Packing审查](</home/ubuntu/Documents/ChatGPT/CPS Intern/reviews/2026-10-17_s20_independent/tight_packing.md>)

## 10. 2026-10-20续：更新同学仓库、6cm偏移与通信核查

用户要求回主线复现同学更新，并查Jetson、软同步、ZMQ和SQLite。本轮实际拉取同学新版48f6edd17e3ac5d0c5e211c1a53aaf29a148f2d0，独立worktree，旧3ade6e4不动。新增baseline使用深度体积目标、最多两次搬移／4m预算和IK可达球初筛；它仍是规则／搜索，球内不是完整可达证明。

**关键纠错：Newton版本不能忽略。** 相同新版原场景baseline同尺寸10件，在Newton1.7首箱阻塞；隔离装作者声明1.3.0后，10/10规划通过，原作者关节跟随回放1063帧通过（17.72秒名义时间、位姿更新均值.320ms／p95 .456ms）。规划累计56.817秒、中位.747秒，首件27.587秒含首次编译影响。旧1.7的0/30不能用来判机械臂物理不可达。当前匹配的是Newton版本，Python3.11/Warp1.18仍是本机依赖，并非作者全锁文件复现。

**当前工位独立偏移。** 以导出scene_tf.yaml为源，仅world→ULD的Z减.06m，旋转／机器人不变、无取箱台。这是用户指定校正偏移，非新测量标定。1.3下全20 baseline接受6件、第7件阻塞（目标30，38.867秒），仅离散IK／作者arm-vs-ULD判据，不是6次实际释放ACK。两几何策略各接收30件且无搬移，约.080／.119秒；mixed同seed8平放下baseline14件、heuristic19件，均遇第一阻塞停止，不能与此前拒收继续的26/30直接比较。

**原场景对照不可忽略朝向条件。** default随机流三seed：baseline7/10/8，heuristic15/17/12；前者强制最大面水平，后者默认六排列，故这些数不能当纯策略优劣消融。当前适配的mixed两组才都限制两平放yaw。

**Isaac已运行。** 本机原生Isaac加载S20 USD、ULD／台座、完整TF和下降6cm，逐件回放新版baseline30箱路线并保存USD／截图。机械臂为实测q静态，箱体为规定变换路径；没有接触求解／自然释放／机器人整批闭环，不能称全量物理仿真成功。首轮静态q未刷新到USD，修正为reset后step再pause，重跑截图已更新；不保留错误姿态为推荐。

**Jetson本轮状态。** 100.97.142.96的SSH服务可达，但hku_reconova身份认证被拒绝，未进入系统，未发SDK／运动／IO。正在等待用户提供已配置SSH别名或密钥信息，不能声称当前传感器状态已读取。

**师兄仓库核查。** adamliaoyifan/RoboticArm的ros2_humble，当前8143e88c093b9ef7c332eb1d1e7dc67d539cc6a7，fetch无新变动；核对部署d555_host_stamp_node.py和motion_compensation.md。部署版将已配对RGB／depth及CameraInfo用一组host ROS时间重打stamp，Livox／CPS已有host时间不重写，用于避免D555硬件时钟回退打乱关联。此前本机另一个DeviceClockMapper是保留设备间隔、估计偏移的适配实现，二者不能混称同一部署算法。统一接收stamp不等于同时曝光，也不会消除传输时延；不要求必须师兄电脑当主时钟，Jetson也可做主机。

ZMQ是消息通信库，常用TCP／IPC，非HTTPS、非运动规划器；Python可给机器人侧进程发送目标，后者仍由驱动／控制器执行。Franka常见Python高层＋C++/libfranka低层实时环，不据用户口述断言具体架构。ROS2、MoveIt和ZMQ职责不同，换传输不会自动解决IK／碰撞／实时性。SQLite为嵌入式单文件数据库，项目sqlite3用途包含ROS bag的topics/messages查询，不是控制协议。

原件：[本轮summary](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_newton_update_20261020/summary.json>)、[下降6cm场景](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_newton_update_20261020/scene_tf_lower6cm.yaml>)、[Isaac画面](</home/ubuntu/Documents/ChatGPT/CPS Intern/experiments/s20_newton_update_20261020/Isaac_新版baseline_30件_几何回放.png>)。0训练，旧预算／automation／原标定不恢复或覆盖。

本轮补测完成：作者1.3原场景20–32寸随机流seed1，目标10件，第1件通过、第2件阻塞（与作者README口径一致）；不是混尺寸10件成功。Isaac交互窗口2312522显示当前下降6cm的30件几何结果，非实时传感器／机器人控制。

Jetson连接补记（2026-10-20）：经用户授权，用其提供的密码对hku_reconova@100.97.142.96交互认证一次，服务器拒绝；已取消连接，无远端命令执行，未持久化凭据。待核对SSH用户名／认证方式。

## 11. 2026-10-20：用户要求舱内点云平面精调

新LAN192.168.3.12可ping且SSH服务有响应，本机192.168.3.15走WiFi。SSH指纹与100.97.142.96相同，确认同台；交互密码认证一次被拒绝，取消连接，无远端命令／运动／IO。不是网络完全不通，尚待用户确认当前登录身份。

找回10月17日来源归档中的原标定程序：Jetson /home/hku_reconova/lianaiwei/src/uld_observation_ws/recalibrate_current_uld.py，调用capture_current_pose_scene.py、audit_current_pose.py和Tag双棱拟合；本机只恢复5个原程序至experiments/s20_uld_plane_refinement_20261020/original，不提取或保存凭据。旧aperture配置角点到真实STL约68mm，不能继续错配门框边。

新目标是从原粗TF＋同时间当前实测q/相机TF出发，用实际可见CAD内表面与点云配准，比较角度／面距残差。单平面不能约束完整6DoF；不把6cm先验当测量结果，不靠改相机／工具TF凑贴合。当前未获得新点云、未生成精标定。状态experiments/s20_uld_plane_refinement_20261020/status.json。

认证复核：用户再次确认hku_reconova。新LAN SSH握手、已知主机指纹均通过；服务器明确提供publickey/password，单次密码认证仍拒绝。客户端证据不足以区分密码不匹配、PAM或账户限制；需Jetson本地SSH日志，未修改认证／网络／机器人配置。

## 11. 2026-10-21：拼接同学policy与师兄PICK，目标20件完整Newton周期

用户睡前授权自主推进及sub-agents，目标固定为20只55×40×25cm／5kg，从地面PICK到持箱、落位、释放、撤回、返回并实际占据ACK，同一world不重置。原训练／自动发送不恢复，无真机操作。JetsonSSH本轮可达但密码仍被拒绝，未登录。

复用Newton1.3作者48f6edd及师兄waypoint_generator。查到最新生产orchestrator将VacuumCommand放在完整pick／place结束后，而阶段表已含retreat，不能直接照搬；当前仿真采用attach处开理想吸附、descend后解除再retreat的明确事件时序。作者policy原样还缺空载回pickup、完整撤回和对已放箱的机械臂碰撞，必须补接口后才能验收全链。

后端实际验证：5kg箱由z=.5释放，2秒后中心z=.124991879；idealattach随q6变化误差4.2e-8m，解除首帧自然下降1.533mm。机械臂仍为规定关节轨迹／FK，不是电机动力学。attempt001空载取箱路径未通过、0ACK；发现直接横移穿台座、直接关节连线撞ULD，一条RRT工具倾角10.0901°也按10°拒绝。沿舱深撤出再绕台座的379态通过同CADFCL且最大倾角1.5075°，attempt002正实际执行该首周期。来源与状态见该case protocol/status，不能把这些单元通过称全流程成功。

首轮实际attempt002失败：源箱从首帧开始穿地自由落体，13.18s后Z=-852.90m，非真正爆飞；未吸附、0ACK，实际trajectory完整保存。根因已定位为XPBD缓存初始KINEMATIC物体有效逆质量，运行时改flags却未调用官方BODY_PROPERTIES刷新；后端修正与同路径复核正在进行，未用锁箱／重置箱位置修饰失败。独立首箱loaded/return几何路径已通过，但释放穿ULD地板也属于此物理后端问题，不能记作已成功PICK&PLACE。

attempt003在修正后已实际完成PICK（真实接触gap9.66e-7m、理想吸附、抬升），但持箱经过台座角落前执行检查停止，0ACK；实际原始轨迹和失败录像保留。发现规划用.025rad检查、执行又插入更密采样会暴露中间碰撞，不能把端点通过当整段。后续将规划关节采样收紧至.01rad（直接覆盖每个执行态），并把载物入口外XY/yaw对齐与下降分开，避免斜线掠过台座角；attempt004重新从地面完整验证，不拼接attempt003成功PICK。

用户最新强调整个运动复用师兄连续ServoJ，只替换同学packing，不能点控MoveJ。attempt004从地面完整完成1周期，56.20模拟秒／171.32墙钟秒，释放后最终中心误差10.76mm；这是FK机械臂＋理想吸附＋XPBD释放的诊断，不能记为ServoJ或精准20件成功。已转向师兄原生MoveIt→ServoJ和Newton关节驱动对接；既有S20 MINCO不是师兄原创，来源分别记录。

连续运动审核：旧attempt004参考最大速度0.600rad/s，但段间加速度70.999rad/s²、jerk4259.91rad/s³，且持箱q6累计6.667rad而端点最短差仅−.478rad，不能称平滑。已改走师兄FJT→ServoJ重采样/速度与刹车限制。真实驱动首轮暴露q1实测等价角超URDF窗、XPBD驱动不稳定，失败保留；MuJoCo CPU外部contacts接口不支持真实CAD的路径也不能用于结论。正使用官方GPU接触接口及等价合法关节角修正。

师兄完整MoveIt已在ROS_DOMAIN73/localhost隔离启动，加载当前机器人全工具URDF、完整TF与原ULD网格，不接硬件、不碰domain0的PiPER。地面pregrasp与Cartesian下降实测通过。首次完整整合001因空段tilt11.13°拒绝；002启用工具约束后MoveIt报−2而非成功（TOTG路径平滑后约束超限，原始轨迹保留）；003用同任务明确走廊目标、师兄Cartesian规划及原FJT→20ms ServoJ/刹车限制开始20箱尝试。当前执行器仍理想跟随，不冒充动态ServoJ。官方MuJoCoGPU实际驱动已独立跑通连续50Hz与落箱，肩关节重力静差尚不满足精准PICK。

驱动诊断纠正：前段“q1超URDF窗”判断错误，原S20 q1为±6.28rad，3.773是一次失败的停止位置，不是限位；canonical仅等价角表示。真实驱动高增益输出的actuator_force也属夹限前请求，不能直接当实际关节力矩，须看qfrc_actuator。上述早期结论保留并在此撤回，不据此改硬件参数。

师兄连续接口005从空场实际运行83.04模拟秒/241.76墙钟秒：首箱完整ACK，最终误差11.76mm；第二箱完成取件/释放/返回，但首箱累计位移27.39mm，按20mm原阈值停止，只有1ACK，不算2或20。保存轨迹分析两箱全程最小表面距离45.8cm，排除第二箱直接碰移；单独保持原ACK状态/速度、无机器人运动与新来箱的20秒XPBD同条件复核仍漂移21.26mm。正在比较2/8子步，不冻结已放箱、不放宽阈值、不改TF凑成功。另实际连续参考仍有6.67rad冗余转腕和角点加速度尖峰；ServoJ接口本身不保证平滑，正在核师兄Cartesian输出等价角及原刹车采样限制。

004正式整轮前固定修正：原003第二箱单帧qvel角速.184rad/s导致拒绝，但保存实际1秒位姿仅.0857mm/.0138°；第一箱该窗口却真移动5.765mm/1.631°，不能简单忽略所有速度。新采用所有已释放箱1秒实位姿窗口（位移跨度≤2mm/转角≤.5°），保留累计旧箱20mm上限，单帧qvel照实保存。release缓存条件配对0.1秒：无事件.0436mm、原notify1.5908mm、清qacc_warmstart .1091mm；质量/逆质量未变，故在source/attach/release状态切换后清求解缓存，不改箱pose/速度、TF/μ。004从空场20件正式重跑，源码与协议保存在该attempt。该条件配对非原求解器隐藏状态恢复，长期修复仍待实际整轮。

证据范围补充：较早Newton整合尝试002/003的原始result/trajectory/accepted plans完整保留，但未在启动时独立冻结全部适配器源码，不能声称字节级复现。004已保存实际运行源码与协议快照。视频按仿真时钟回放，规划等待期间仿真暂停，绝不据此称真实硬件50Hz实时闭环或无停顿在线系统。

004连续2ACK后在第3件已吸附/待抬升时停止；原始source静置实际穿入软地面1.315mm，绝对0.5mm几何检查拒绝起始支撑接触。该停止非手臂路径碰撞或第三件不可达。005从空场开始，支持接触只按实际起始深度（≤5mm）允许数值0.1mm余量，规划不得继续加深；搬运/其他障碍不放宽，源码与协议自动归档。前2件1.884/14.346mm误差如实保留，后者不能称毫米级精准。


2026-10-21 续：mujoco_servoj_attempt004最终2ACK，005最终1ACK/B2下降与B1碰撞，均已停止保存，不能沿用旧running状态。真实joint-drive cycle001在2×10ms初始保持发散；cycle002使用官方sample10×2ms，同gain/geometry/effort执行首个地面完整周期，尚未完成。准备隔离5mm XY候选间隙，物理支撑使用实际箱，不扩大支撑面；不移动旧箱、不放宽碰撞。原MoveIt C2+师兄20ms FJT/SDK形状仿真接口复用，无MoveJ、真机、旧RL或定时恢复。

2026-10-21 实际驱动002结果：0ACK，26.62simsec/435.16wallsec，载物搬运中TCP倾斜11.9906deg停止，目标最大逐帧增量0.0201rad；保存实际body/joint/target/torque。不能把旧FK2ACK当实际ServoJ成功。正在查物理接触约束或积分机制，无放宽10deg/碰撞或参数扫描。5mmXY间隙及实际top精度CPU适配已准备，仍待真实完整周期验收。

2026-10-21 true_drive_full20_attempt001已真实启动/session34514：GraphDriveWorld10×2ms实际jointdrive，师兄SRDF/fixedrigid52pair物理过滤，FCL独立不放宽。旧接触force配对active21→0；held-tool候选全inactive，已纠正初始归因；savedstate重新积分未复现，不当成功。参考clearance_reference20_attempt001最终1ACK/B2稳定窗口失败，完整保留。新actual20显式5mmXY及精确XY/top（原支撑判定不改）、原10terminalhold+S20adapted3read、最多5s额外等相同稳定窗口，非重置旧箱。目标20，尚无新ACK，不称修复完成。

2026-10-21 actual001终点门槛负结果：0ACK/14.92simsec/41.66wallsec，实际q误差<=.000123rad、1s span<=1.03e-6rad，但额外S20瞬时qd<=.05deg/s不满足。核对证明nativeMuJoCo与eval_ik速度一致，非映射bug：2ms子步250Hz微振荡，峰峰.001694deg、峰速.8415deg/s、最大均速.00134deg/s。原失败保留。actual002恢复师兄10terminalhold，仿真完成以实际q误差/0.2s位姿窗口<=.05deg连续3读，不宣称电机零速或真机已验；rawqd/torque保留。已启动20同world实际驱动，尚未ACK。

2026-10-21 actual002最终0ACK/15.94simsec，已地面接近/吸附/抬升但候选预筛0。保存态无积分配对确认：actual gripZ=.127067782m（理想.125）使旧名义抓点映射把箱底压低2.07mm；actual+nominal0候选、nominal+nominal21、actual+实测标准刚体映射21，非packing策略无位置。新增measured_grasp_pose.py，预筛/规划/执行统一T_box*inv(T_grip)及一次3mm releasegap。actual003同world20已启动/session74131，尚无新ACK；原全部失败保留，不称已完成。

2026-10-21 actual0030ACK/27.58simsec/144.83wallsec：实际grip映射已消除候选拒绝，但Newton fullULD↔link4错误接触爆发。保存态official force27.56s Newton距离−.393m/69.2kN，而同CAD同pose FCL无碰撞、距离+.241944m；27.58s仍FCL+.109788m。不是单纯无接触高gain发散。原资产/TF/失败保留；不关舱壁，不盲重启。查官方mesh-mesh parity/SDF及每robotlink保守convex-vs-fullULD triangle路径，先核对空隙和真实碰壁静态证据。

2026-10-21 当前真实结果：004为保守凸碰撞但support扫描太慢，0ACK主动停；005因root结果字段名接错在0ACK阶段停更正，源码/intentional_stop保留。006实际驱动1ACK（18.469mm），B2全部抓放/返回动作已执行但旧B1累计20.4486mm移位，未ACK2。独立20s full8k/作者physics2k漂13.602/13.547mm，证实creep，未换mesh重跑。原生Newton实际状态录像已看地面抓取与门口最终图。额外控制对照指出20ms ZOH大脉冲；原FJT2ms子步同参考将4s峰速9.17→1.07rad/s、torque19.9k→725Nm，仍非平顺/硬件通过。

现成Isaac5.1无安装，fullCAD同TF/Z-6、同B1pose/velocity、μ.7/2ms静置20s：初落座2.25mm，1→20s仅.000134mm额外漂；只是PhysX对照，不能代称Newton20箱。根据用户早前允许全Isaac，开始隔离当前URDF真实articulation适配，复用原同学policy/师兄MoveIt/FJT、037成熟gravityFF，不用旧037虚拟墙/OBB资产、不恢复训练。完整20仍未交付。

2026-10-21 PhysX完整20正式启动：physx_full20_attempt001/session80945，实际B1地面接近/吸附/抬升已执行，尚无ACK。复用师兄MoveIt C2+原FJT20ms/2ms同参考连续采样，同学packing，原TF不改/隔离Z−6cm；理想吸附无载荷反作用、不是真机ServoJ认证。Newton006失败原件保留，不能拼接称20成功。

2026-10-21 PhysX001真实0ACK/31.18simsec：已完成地面取件，下降时实际箱底−2.228mm，指令箱底+10.902mm；FCL接触normalZ≈1，是底板不是侧墙。实际相对指令向下偏13.13mm，末端下行约.153m/s。diagnose_physx001.json与实际录像保留；正查gravity/control并将运输与最后低速接近分开，不能放宽碰撞或增加跌落高度冒充精准。

2026-10-21 PhysX002已启动session53687：同师兄ServoJ/nativeMoveIt曲线，运输到8cm preplace，最后独立Cartesian≤.02m/s/.05rad/s/.1rad/s²，原3mmreleasegap。gravity正确；native质量矩阵预测减速误差q2−.006225/q3−.002130rad，实际−.005686/−.002041吻合。未加自写逆动力学或扫gain。释放接口有首2ms后写初始twist近似，低速停稳释放；不能称载荷/真机验证。

2026-10-21 PhysX002最终同world5ACK：落位误差1.00–1.50mm，最终旧箱相对ACK最大位移.0108mm；5cycle共206.40simsec，总211.08simsec/377.37wallsec。actual20ms峰速1.005/acc1.166/jerk63.37，非2ms上界；倾斜2.164deg，末接近.01984m/s。第6已取起但author预筛0candidate，geometry105floor/0stack，正在核对exacttop-route舍入及author低位进舱与senior高位路径重复约束。保存原实际录像，非20成功。

2026-10-21 PhysX003 fresh空场景20启动/session20945。B6实际态配对确认旧0yield→exactroute/defer重复IK185candidate/25strictstack，保留几何/排序/reach筛选与senior全FCL；去24cap不改支撑EPS/TF/碰撞。不是恢复0025箱或拼接；新实际结果待验。

2026-10-21 PhysX003最终5ACK，B6已释放但15cm撤回在elbow近伸直时fraction.625失败。相同真实释放态8cm+整6段返回seniorCartesian/fullCAD FCL全部通过，仅规划非ACK。PhysX004从空场景重新20/session89133，首撤回改为已通过的8cm preplace高度，未拼接旧5箱；其他条件不变。

2026-10-21 PhysX004最终10ACK：完整cycle383.66simsec，总402.20simsec/730.78wallsec；误差.201–1.488mm，旧箱max漂.160mm。10prefix20ms采样v1.007/acc1.166/jerk58.76；包含B11失败则acc4.970/jerk255.88，均非2ms上界，负结果保留。B11preplace3秒后q2−.004016/q3−.001886rad未停稳，未释放；FCL无hits，正在核对原生接触，不放宽门槛。实际录像与final图已查看并链接桌面PhysX完整抓放；仍非20交付。

2026-10-21 B11原因纠正与证据：native direct读contact抓到link3↔ULD向下11.849N；held↔robot0，原CAD gap4.687mm但完整凸包相交，规划精确CAD与实际PhysX凸包语义不一致。先前zero matrix有读取时序问题，不能排除contact；decomp只小幅改善未解决、gravityeffort未解决，不作默认修正。选择保留物理凸包/舱壁，规划增加同原vertex凸包检查，提前避免天花板卡住；不改gain/TF/停稳阈值。

2026-10-21 PhysX005从空场景完整20启动/session92809。新增原CAD+原robotvertex物理凸包双检查，初始/74旧actualphaseendpoint全通过，B11actual原CAD通过而凸包明确拒绝，匹配native约12N顶壁接触。物理/控制/TF不改，未改decomp/friction/gain。保留00410ACK，不拼接。

2026-10-21 PhysX005完整20尝试最终10ACK/760.80wallsec/388.34simsec。B11全部464author候选枚举完，无完整路径，429native规划拒绝/35凸包顶壁；物理阶段未再撞顶停稳。仍不是20通过。当前仅保存态有限核对师兄已有OMPL fallback（main此前强制Cartesian）；不盲重跑或降低碰撞标准，不称当前policy全局无解。

2026-10-21 本轮最终交付口径：PhysX005实际10/20完整地面取件→搬运→低速下放→释放→撤回→返回；10cycle383.66simsec，总388.34simsec/760.72wallsec，误差.201–1.488mm，旧箱最大漂移.160mm。连续原FJT/MoveIt接口、无MoveJ，但非vendorServoJ硬件。Newton显示实际PhysX状态；理想吸附无载荷反作用。第11已取起但不入舱；464候选耗尽，9代表Cartesian/OMPL+3默认IK及1次30.078秒RRTConnect仍无完整连接路径（存在一个静态安全IKgoal，不能称全局不可达）。不盲重跑或重复上述诊断；新批次前须在真实prefix上有完整连接路径证据。最新完整录像/第十箱下放/最终图已实看与桌面固定入口更新。Jetson仍认证拒绝，无真机命令、无新RL或定时恢复。20箱成功尚未达成。

2026-10-21 继续目标：author自带rankdepthTrue vs原False的端点准入配对20，CPU21.70s，8 vs10；检查preplace8cm/final3mm/empty8cm，0物理/0ROS/0训练，非path/ACK。排序改变不够支持重跑20。现成Newton1.7源码未找到对应creep默认修复，不安装/不盲版本试验。下一有限检查用户明确允许的传送带逐件到站（stationtop.75/TCP1.0）是否能使用已有正肘IK分支；地面10ACK不覆盖、不冒称20。

2026-10-21 新进展（0新完整物理轮）：远端HEAD确认仍48f6edd。单一输送面.75m正肘source及senior取件approach/contact/retreat可规划且原CAD+物理凸包通过，平台未建；连接高目标仍未完成。作者SETTLE_CLEARANCE_M=.02对照自加.08：ground True8→9/False10→12，conveyor正肘True9→12/False9→11，均仅端点非path/ACK；8cm适配确有额外限制，仍无20证据，不盲重跑。隔离原生FixedJoint吸附组件正在修正scaledCube localanchor单位，尚未通过；原10ACK依然理想吸附无反作用基线。

2026-10-21 原生动态抓持组件进展：外部FixedJoint接link6+完整实测TCP变换，scaledCube localanchor需÷scale，修正前attempt001/002失败保留。真实地面contactgap.231mm→抬升→1s保持→仅disablejoint释放，无pose/twist重置。复用037原epsilon1e−5 COM Jacobian已知5kg重力补偿，同固定goal baseline误差.60818deg→FF .009114deg（<原.05）；grip最大1.606μm/.000379deg，释放jump0；J2负载反力差42.978Nm与补偿42.980一致。19.7simsec/12.07wallsec、1env/0训练，实际状态NPZ及19.7s原生回放已保存并实看抬升/落地。仍非carry/place/20ACK，不替换005理想吸附10箱。两输送合法分支+作者2cm端点20配对True12/False9，无20前缀；不得把这些贪心计数称全局容量。下一须先找到20位置布局与完整连接路径，避免重复上述诊断或盲重跑。

2026-10-21 本轮布局搜索进展：明确标S20离线lookahead而非作者greedy/RL，beam宽4/每node4不同footprint、120s/2000nodes上限；实际48node/5090candidate/10412IK，最深13endpoint前缀（前5铺底层）后时间预算停止，非穷尽/非容量上限/非13ACK。best13完整目标/branch/qs在beam_author_endpoint20.json，无新物理。独立run_native_load_fullcycle.py+protocol_native_load20.json已准备：原师兄FJT/MoveIt+原生5kg约束/037FF/作者2cm，尚未launch；原00510箱不改。新20整轮前仍须20布局与完整连通路径证据。

2026-10-21 link3碰撞表示隔离实证：同savedB11原CAD gap4.687mm，hull最大q误差.23073deg/顶壁peak87.22N；CPU原CAD SDF256/remeshfalse误差.00464deg/hold顶壁0N。两者真穿透样本均有碰撞；SDF强制穿透产生大量material-face-index warnings，保留而不称鲁棒接触/全路径通过。可选physx_sdf_payload.SDFPayloadPhysXWorld组合原生5kg抓持+037FF实际19.7simsec/12.38wallsec组件通过，未carry/place。新Collision allowlist仅link3省额外hull，全部原CAD/其他hull不变，真穿透仍拒。新SDF映射beam单次120s到13endpoint/47node/5047candidate/10324IK，未20/未穷尽/非ACK；独立文件不覆盖旧13。run_native_load_fullcycle.py支持明确S20_LINK3_SDF=1，仍prepared_not_launched。下一应从已有前缀验证完整连通/来箱source和分支切换，平台未建，不重复从空搜索或盲重跑20；20目标仍未完成，原10实际闭环与所有失败独立保留。

2026-10-21 继续完整20件任务：固定五塔四层有限pattern191footprint/1228IK/13.51s，45个独立四层端点可行，但有限池互不重叠最多4塔（DFS与MILP一致）；没有20目标，不称全局容量上限。saved13完整路径从实测q到conveyor负肘取件/抬升/高位入口均连通，首低entry失败已直接定位：author entry z=.5的实际持箱与原pedestal重叠32.2576mm，ULD无碰撞。作者几何route未查当前底座，不能将其归IK故障或放宽碰撞；正检查作者已有更高入口。link2同191pool的38shape9拒绝中28原CAD真碰、10凸包额外碰，但10例仍被其他真实/保留碰撞拒绝，未发现独立恢复可行目标，不扩SDF2。无新物理ACK，原10完整周期仍为理想吸附PhysX基线，native5kg+FF仅组件验证。

2026-10-21 新原生负载完整周期：在同一PhysX世界重放冻结005第一箱原师兄C2连续参考（原8cm接近保持），原生5kg吸附+037FF实际完成地面pick→carry→place→仅disablejoint释放→withdraw→source return，53.12sim/40.358wallsec，完整原CAD/其他hull/link3SDF映射实际状态逐采样碰撞及10°检查，目标中心误差1.110mm，最后1s箱位跨度.0000723mm，grip最大2.258μm，releasejump0。不是新鲜规划、不是Newton物理、不是20/真机，也不能与旧10理想吸附拼成11。视频已实际看取件/下放/返回3帧（广角舱壁遮挡需门口视角补看）。conveyor简化75cm薄台native5kg取件/抬升/保持/释放回台单独17.98sim/12.60wallsec通过，尚非ULD交付。作者.6入口已让B1完整规划通过；audit自身returnquat与真实joint限位wrap错误已纠正并保留原失败，不归作者/物理。B2真实失败是取箱负→正肘腕分支过渡违反tool约束，不是30s搜索耗尽。q3=0有界12点6.53s证实两侧仍不同腕分支，未连通。18参考局部74新点/898IK/12.34s仍未到20；只有限池组合、非全局上限。当前正进行1件conveyor完整原生周期，不盲开20。

2026-10-21 逐阶段口径核对：nativeground第一版固定10hold虽完整运动/最终落位1.110mm，carry末.071deg>原.05deg，降为fullsequence诊断非strictACK，源/结果/录像保留。最小修正复用原main位置误差及.2s位置span各<=.05deg、3freshreads门槛（非qd门槛），新独立native_load_gated_B1实际54.30sim/41.589wallsec全7motionstage通过，中心误差1.218mm、grip最大2.411μm、释放无跳变，完整地面pick/place/return1次。门口原生回放已实看箱体释放及退爪。conveyor第一版同样完整sequence但preplace末.05668deg>门槛，原summary降级strictACKfalse；仅一次同门槛修正验证进行中。腕奇异q5=0的12候选均工具约89.64deg不合10deg，不能跨腕branch。新物理有效自由度是顶面中心toolyaw180（箱不转/不翻、mount与TF不变），8次源IK找到同腕branch，原生initial→hover→contact/loadedretreat均通过；实际contactFK重新测Ttcpbox修复理想map的.227μm支撑反例，未放宽EPS。正用此新grasp评估既有pool，不混入旧反腕branch，仍0新增20箱。

2026-10-21 严格来箱完整B1通过：native5kg简化75cm入站→ULD→释放→撤回返回，48.12sim/53.529wallsec，2396实际状态，18/18motionstage原位置/窗口<=.05deg且3freshreads通过，箱误差1.212mm、旋转.01745deg/静止、grip最大2.318μm、tool倾角2.299deg、releasejump0；final参考峰.019775m/s，实际20ms峰.020591m/s，保留超参考瞬态，未称硬速度全过。当前18stage逐停是诊断，正合并原seniorC2/FJT为少量连续逻辑流，不能把连续PushServoJ等同已经取消中间停顿。四cardinal顶面抓姿0/90/180/270源链都同腕branch规划通过，箱yaw仍0/90、原mountTF不变；90/270各265pool/3744新IK/42.314s，合并既有0/180cache，69正heightfootprint有限MILP最优13，非全局上限/非联合13或20ACK。停止新角度/候选池扫，仅检最优13真实prefix。录制时本机根盘满，已只清旧退出PID S20NewtonNVRTCpch及S20下载wheel缓存共401628916bytes，安装环境和所有原始轨迹/失败/源码/TF/权重不动；render临时PCH改/dev/shm专用目录。20件目标仍未完成，不给虚假完工。

2026-10-21 连续四流实际单件结果：已把18waypoint参考合为empty-contact、loaded-preplace、slow-final、empty-return四条原生seniorISP/FJT流，19865个2ms参考FCL/deck/10deg通过。隔离conveyor_fused_B1真实native5kg+037FF同一世界完整1cycle，42.72sim/44.82wallsec，2126状态；4/4原logicalgate通过（preplace额外.62s收敛，中间waypoint不停车），目标误差1.329mm/旋转.02313deg/静止，sourcegap.09586mm、releasejump0，grip最大2.275μm、tool倾角2.295deg；实际全程TCP峰.75498m/s、joint峰1.00358rad/s，final参考<=.02m/s、实测20ms峰.020573m/s瞬态保留。原速42.55s双视角2.58MB已实际看取件/ULD放箱退爪/返回，桌面固定推荐更新为此视频，旧18stage诊断保留不拼接。最新native1与旧idealgrasp10分开，20仍未达。best13联合prefix39IK/.599s端点全部通过，但第一箱原.6/.55高处深入IK失败，直斜降亦断开；只最后检一个几何推导的先水平越过pedestal再下降corner，不任意高度/角度扫。13为S20离线pattern非同学greedy已完成，更非13ACK。

2026-10-21 本轮真实停止点：固定13路径的几何pedestal-aware corner先水平.158364m使持箱与原底座分离5mm，再下降，接通B1完整planningcycle。B2同腕抓姿切换/取件/入口/corner也过，但corner→预放点Cartesian仅.06818，按有限授权停止；best13_connected_paths.json保留全部源/目标/实测抓姿映射/失败。该离线13pattern没有实际13ACK，不作为同学policy成功。当前实物理推荐是连续四流native5kg完整单件1ACK，旧10件仍理想吸附历史基线。20件尚未完成；本轮未恢复训练/定时或下发真机命令，没有活动物理/渲染任务。下一必须从具体失败状态解决完整路径或独立核标定，不能无界扫角/加高/挪TF/重跑20掩盖问题。

2026-10-21 继续20目标的新物理依据（非中心角度/候选池扫）：只读原suction CAD全部5部件相对原TCP投影均在18x18cm内。55x40顶面boxlocal X−15cm及对称+15cm仍整面保留约35mm边缘，满足既定20mm；Y−10cm仅10mm，明确拒绝。固定中心抓点是当前reach限制的一部分，允许合法顶面偏置抓点的S20 PICK适配不改mount/TF/箱平放姿态。偏心5kg增加7.3575Nm重力矩，nativeweld/037实际COM FF可模拟但不证明真空抗剥离；原CADcap距名义TCP约3mm也不证明seal。只检±15cm两源链（每方向≤8IK，原来箱center不动），actualcontactFK新graspT后才对既有265pool新增条件，绝不重跑中心池或加新角度。protocol_offset_pick20.json固定该边界，0新训练/0新物理，截至记录仍20未达。

2026-10-21 合法偏置抓点结果：X±15cm实际contactFK两源链（每方向8IK/yaw0）均原生FCL/deck/10deg同腕通过，箱仍原站位。既有265pool只新增两offset条件正式3488IK/39.59s，原4centercache不重测，有限83正heightfootprint最优组合13→17，独立几何收益存在但仍非20/联合17/path/ACK/真空承载评级。attempt001在首10proposal首次MILP遇空pairwise矩阵shape接口错误退出，源/log保留；原进程未存点cache，实际失败IK数unknown。只修空矩阵并小JSONcheckpoint后按root授权唯一恢复首10并继续全265，失败次数不混正式有效IK。基于新offset可达场确有4件组合改善，仅授权围绕新17最优footprints做一次≤125新proposal/90s的±2/4cm局部相位，原265全部cache，6既有sourcecontexts/所有几何约束不变；不到20不继续任意域扫。仍0偏置物理和0新训练。

2026-10-21 两缺层与层高/IK界限复盘：Y±8cm顶面边距30mm合法（5kg矩3.924Nm），两源各8IK完整planning通过；仅对固定178两缺层4目标检查仍失败。额外12近邻初值/14IK/.31s并未补齐：第3层Y−8cm三个合法同腕/无碰seed均约34.35mm残差，第4层约99mm或以上；这是有限35iter IK诊断非永久不可达。固定5footprints×8抓点第五层40IK/.61s无可新增端点，至少14个pose已收敛但真实CAD腕/工具碰顶。只读roof审计纠正Hull2.01并非实际roof（CAD顶约2.112–2.118/边梁2.077），吸盘p01可clear11mm而link6/adapter碰顶，不能用p01长度做全球五层不可行证明。固定tower/MILP路线停。主线回用户指定authorcurrent-arrivalgreedy：8合法抓点接原候选/排序，rankdepthTrue/False各20件端点前向、总120s，不用中心TCP解析球提前误删offset候选、不抽旧箱/不窥未来、不新角/offset/容器/训练。有限18组合参考不是作者policy容量极限或实际ACK。

2026-10-21 作者在线两流与Cartesian接口修正：同学原排序rankdepthTrue/False各20来箱，8合法顶面抓姿、完整CAD/deck/关节窗/同腕端点筛选实际15/16，5931IK/67.692s，仅端点非完整路径或ACK。False16首入口诊断发现senior实验bridge的Cartesian分支提前返回，未传向下约束；服务fraction1的原始knots最大倾角124.784度，执行前已拒绝，非采样遗漏。隔离adapter补官方OrientationConstraint及absolute revolute jump检查，师兄原源码未动；同入口fraction0、4同腕种子无合法入口，不追加错误腕OMPL预算。当前native5kg推荐仍连续四流完整1周期，历史无负载反作用10周期分开；20未完成。下一从已清底座起点到已知同腕preplace解决合理连接路线，不将任意入口失败当目标全局不可达。

2026-10-21 新作者首箱原生验证：绕过非必要entry，采用实体底座推导corner(-.491693,.983,.55)；两段官方路径独立通过后精确复用起终q，避免随机重规划换支。缓存scene同步漏项造成remove不存在已定位修正，非接触碰撞。author_online16_B1_batch_preflight四流20402个2ms参考全FCL/10度通过；run_native_conveyor_batch仅增加显式countN(default20)前缀实验接口，不改物理控制/gates。实际count1同世界native5kg完成pick→place→release→withdraw→return，43.26sim/43.12553wallsec，2163状态，1ACK，位置1.127mm/旋转.01953deg/静止。此为新authorpolicy目标，仍单件非20。后续planning已连接3cycle；B2某段5sOMPL失败但原同腕jointedge328样本全碰撞/真实倾角通过，授权交原ISP/FJT重定时后再次全程检验，明确S20连接适配非作者OMPL成功，不调用MoveJ。

2026-10-21 适配器负结果补记：新Cartesian约束adapter设置x=1时漏清消息Quaternion默认w=1，导致请求[1,0,0,1]非单位；此为本次适配实现错误，不归作者。正在改为明确x1/w0，旧fraction0诊断不能当正确约束下不可达证据。已接受路径均另经真实FK10度与全碰撞检查，原生B1物理亦逐状态验证，不据服务fraction作成功。B13另发现attach setup使用1秒q→q OMPL结果错误判ApplyScene，正拆为直接ApplyScene成功回执；不把setup超时归抓取失败。

2026-10-21 完整路径前缀16/16接通（作者20来箱中接受16、拒绝4，不是20容量/ACK）：新单位四元数与ApplyScene-only已生效。B4关节插值侧扫碰门，现成作者Newton analyticJacobian IKcontinuation252节点保持末端直线通过；重复preplace改实际FK同pose保持q，不切支。所有路径精确q/contactT保存。最终四流融合前5通过，B6loaded起始deck检测1.542e−7m接触重叠，正在定位接触映射/样条微下探；不盲增EPS，也不因微米数值差无限阻塞，当前source支撑与搬运碰撞分开。下一一次原生连续16前缀，尚未启动。

2026-10-21 原生16一次实际结果与具体闭环缺口：64连续流328313个2ms参考全部通过后真实同世界run停2ACK/B3slowfinal，100.94sim/109.0395wall，前2位置1.127/4.303mm，非16/20。B2静置相对nominal倾.5963度，top最高角+2.311mm，ACK后仅31.6μm/.014度漂移；B3 nominal3.042mm gap在actualsupport下只剩.588mm，真实TCP跟踪误差.6466mm，最终触顶角。actualvsplannedgrasp仅.0819mm/FK1.19μm，不归IK不达或乱排。证据native16_B3_support_diagnosis.json；失败tick100.94未存（step后valid先失败），仅NPZ100.92，后续runner需最小记录修正。新双视角已实看并保存桌面，标2ACK/16planned/20goal未达。下一最小实际支撑反馈：每ACK实际旧箱pose/currentgrasp重算slowfinal保持原3mmgap，连续接withdraw；原目标/修正目标分记，原失败保留，不加穿透阈值/不重排/不恢复RL。新run尚未启动。

2026-10-21 实际支撑反馈新一轮真实完成16ACK：独立physx_author_online16_support_attempt002从空场景/原起点连续16次sourcepick→carry→place→release→withdraw→return，真实旧箱全程保留，failureNone；696.10sim/1067.2396wallsec/34805实际状态，1env/0新RL。作者20逐件来箱仅16通过当前greedy+机器人端点准入、其余4拒绝，故20目标仍未完成，不称16全局容量。原2ACK失败不抹去。11次actualsupport局部反馈墙钟133.480s，目标高度修正−.993..+2.390mm，保持3mmreleasegap；作者IK+师兄ISP/FJT四流连续，无MoveJ；物理在局部规划时暂停，非硬件实时承诺。最大原/修正目标误差4.303mm、oldbox累计位置漂移.563mm、工具tilt3.867deg、grip4.192μm，实际20msTCP峰.7554m/s/joint1.00355rad/s。suction仍理想nativeweld无seal/leak，PhysX物理/Newton实际状态显示，非真机ServoJ/机器人硬件成功。metrics_summary.json及原逐件ACK/trajectory/source全部保存；双视角媒体生成中，后续实看才标review。

2026-10-21 用户明确停止后续执行/渲染，要求只看规划层：质疑同学原Newton直线路线被我加入外部高点/底座corner/MoveIt等复杂化。停止新实验与媒体处理，回原commit源码逐项区分作者route_to的水平进舱/可选横移/垂直下放和S20额外transport。16ACK仅独立适配结果，不作作者原样复现成功；不再继续规划或真机。

2026-10-21 用户明确严格按同学方案。已核远端HEAD与clean本机均48f6edd；使用原online_packing_real.py --ik --count20 --seeds1，原defaultheuristic/原vendoredTF/原relocation，固定20寸默认55×40×25cm，无额外MoveIt/绕行/多抓点/支撑反馈。首次因新Warp编译cache写满根盘失败，原件保留，仅将可再生1.18cache移入/dev/shm并通过原支持WARP_CACHE_PATH环境参数恢复。attempt002放入11、relocation0，第12阻塞，输出author_original_ik20_attempt002.json；用原online_packing_newton.py显示该新结果，属于作者Newton几何/IK运动学回放，非PhysX/真机。原TF下降6cm本轮未使用，先原配置原样复现，不把旧16ACK适配结果混入。

### 2026-10-22 原版 Newton 重新演示
- 按用户要求重开未修改同学原版 `online_packing_newton.py`，回放原版 IK seed1 / 20输入样本，原TF；日志 `experiments/s20_author_strict_20261021/author_original_ik20_viewer_reopen.log`。已确认播放至11件、第12件blocked后循环。运动学展示，无师兄Pick/PhysX融合或新增绕行。

### 2026-10-22 原版为何未贴后壁核对
- 本地作者commit48f6edd：heuristic insertion_priority先按箱远端x排序，确实优先深处；开启--ik后先用reachable_x_interval球裁剪，再逐段IK/碰撞，首个通过即返回。B1 x=.45/长度.40，远端.85，容器深1.49，后方余.64m。不能据此宣称实体机械臂不可达，具体拒绝环节尚无诊断。
- 本轮GitHub git及API均TLS失败，未确认远端是否有更新；未修改作者源码/TF。当前回放是原版--ik样本，不等同于旧不带IK几何演示。

### 2026-10-22 取消球裁剪首箱实际对照
- 用户要求不要主动增加限制。隔离wrapper关闭作者2.65m reach sphere与x区间裁剪，作者源码/TF/直线进舱/实际IK及碰撞不改；只运行1箱，不启动训练。
- 实际首箱结果：[{"kind": "place", "box": {"id": "B1", "dims": [0.4, 0.55, 0.25], "pos": [0.45, 0.057, 0.0]}, "route": [[-0.5, 0.057, 0.4], [0.45, 0.057, 0.4], [0.45, 0.057, 0.0]]}]；规划秒12.659369。不能以凸舱体保证机械臂IK有解，也不能用单求解器失败证明无解。

### 2026-10-22 重新联网核对同学仓库
- git ls-remote/fetch main成功，远端HEAD与本地均48f6edd17e3ac5d0c5e211c1a53aaf29a148f2d0；README/规划/IK源码无差异。
- 作者自带online_packing_ik_10_samples首箱x=.50、dx=.40，距后壁.59m；自带baseline IK样本首箱x=.45、dx=.40，距后壁.64m。纯几何real样本首箱x1.09+dx.40=1.49贴后壁，但尺寸.4/.5/.6且允许立放，不能冒称同一全20平放条件。
- 独立关闭球裁剪首箱仍x=.45，证明本首箱退化不能仅归球筛；实际IK/模型/碰撞需进一步定位。README基线与默认heuristic不同，后续演示需清楚标识。没有修改作者源码或恢复旧训练。

### 2026-10-22 全20寸无IK原版演示
- 统一55×40×25cm，使用作者原生baseline（自带最大面平放约束）及原TF，30输入全部放入；通过作者validate_sample。仅输入wrapper，未改作者源码，无IK、无机械臂跟随、无动力学。原生Newton循环回放日志：experiments/s20_author_strict_20261021/author_all20_noik_viewer.log。

### 2026-10-22 全20无IK侧边空隙诊断
- 作者候选确实包含(.4,.55,.25)及(.55,.4,.25)两平放朝向。空舱两种各6条合法路径；第5箱后侧边旋转候选(.94,1.135,0)仍合法。
- baseline按中心深度×体积优先、同深度更高层优先，先堆高近入口投影列，侧带入口横移通道逐渐被堵。30箱末probe(.94,1.135,0),dims(.55,.4,.25)满足舱内/支撑/无重叠，但feasible_route=None。第10/20箱后仍有更靠门侧边旋转候选。说明既有排序忽略通道保留，不是没采yaw或空间放不下。未改作者策略。

### 2026-10-22 用户要求同深度先填宽再升层
- 隔离修改作者baseline的直接候选排序为后端深度优先→低z→横向位置；复用作者原生insertion_priority，保留两种平放朝向、原TF、几何/支撑/扫掠路径及fallback；无IK。原版源码不改。
- 30/30、0搬移，通过作者validate_sample。B1/B2填后壁底层两列，B3旋转成55×40cm footprint放(.94,1.1,0)，再B4升第二层，B6/B9重复填侧带。与原版相同30输入，证明侧带已用，不称容量上限或机械臂执行通过。Newton已切换该适配结果。

### 2026-10-22 接入机械臂、最多四层
- 用户要求吸盘贴箱并向前下放、最多4层、先不考虑机械臂碰撞。30箱布局top≤1m，箱体原扫掠/支撑检查通过。连杆碰撞检查关闭，真实IK位置/姿态/关节窗保留，原TF不变。
- 首箱水平进舱终点TCP世界(-2.045,.653,1.18)m，原默认初值失败；25种初值各235迭代均未通过，最佳位置误差.380862m/姿态.313244rad。不归因碰撞、不称无解证明、不改TF冒充接通。
- 当前Newton诊断窗口显示该目标箱与最佳实际FK机械臂姿态，明确无吸附/无完整周期；实际吸盘贴合搬运0件。原几何布局和失败日志均保留。

### 2026-10-22 底座沿X移动与固定点位IK对照
- 单轴首箱扫描三场景：作者原TF、我们粗TF完整RPY、粗TF独立下降6cm场景。底座自身X在作者场景≈world+Y，在我们的粗TF≈world−Y；±1m/25cm采样未找到首箱完整路径解。朝舱内world−X移动.75/1m，三场景首箱均通过。注意轴不能混用。
- 同30箱四层布局、5cm离散路径，6种固定底座/TF对照（碰撞关闭；每失败点6初值、195迭代，前态也作种子）。结果：[{"scene": "rough", "base_delta_world_m": [0.0, 0, 0], "route_samples_pass": 6, "endpoint_pass": 9, "ordered_prefix_pass": 0}, {"scene": "rough", "base_delta_world_m": [-0.75, 0, 0], "route_samples_pass": 20, "endpoint_pass": 29, "ordered_prefix_pass": 2}, {"scene": "rough", "base_delta_world_m": [-1.0, 0, 0], "route_samples_pass": 22, "endpoint_pass": 30, "ordered_prefix_pass": 2}, {"scene": "lower6cm_scenario", "base_delta_world_m": [0.0, 0, 0], "route_samples_pass": 6, "endpoint_pass": 9, "ordered_prefix_pass": 0}, {"scene": "lower6cm_scenario", "base_delta_world_m": [-0.75, 0, 0], "route_samples_pass": 21, "endpoint_pass": 29, "ordered_prefix_pass": 2}, {"scene": "lower6cm_scenario", "base_delta_world_m": [-1.0, 0, 0], "route_samples_pass": 22, "endpoint_pass": 30, "ordered_prefix_pass": 2}]。总墙钟285.19s，本机单GPU运动学，非RL。
- 固定粗TF通过路径仅B25..B30，非连续前缀/容量6结论。靠近1m终点30通过但路径22，原序前缀2，B3在旋转箱入口水平段失败；不是整条30件闭环成功。下移6cm没有解决主问题。不把有限IK失败称数学无解；无连杆碰撞/吸盘物理/真机验证。证据base_translation_results.json与base_fourlayer_routes.json均原件保存。

### 2026-10-22 用户决定固定底座，独立IK复核
- 停止以移动底座为下一执行方案，保持我们的粗TF完整RPY。用作者原始URDF独立SciPy有界IK复核，不只依赖Newton数值求解器；同q两FK位置差3.29e-7m/姿态2.15e-7rad。
- B1/B2/B3入口有解，深处水平/最终目标48初值均未通过（不称无解证明）。独立近门B25目标的向前→横移5.7cm→下放路线2cm采样66点全部通过，最大相邻关节变化.058855rad；单独目标非已完成25件prefix，armcollision关闭，无物理/真机动作。
- 明确结论：固定点位可以有直线IK；不应说所有直线无解。现有最深布局与工具姿态存在求解失败，要按真实固定点位适配目标，不能以移动底座或篡改TF冒充解决。证据fixed_independent_ik.json/fixed_straight_path_verified.json。

### 2026-10-22 用户要求球限制→同学规划→IK
- 固定我们的粗TF完整RPY，肩部世界(0,0,1.031)m；24组带向下工具姿态约束的臂展优化得到最大已找到半径1.853557m，作为初筛外球，不称球内必达或严格最大。保留作者几何/支撑/路径、rearface→lowz→widthfill顺序及yaw0/90，箱顶≤1m。因无可验证搬移轨迹，隔离配置关闭搬移恢复并明确标注，非默认原版。
- 30来箱上限，第11阻塞、接受10件；球拒绝176/IK拒绝47候选，正式规划5.41s。作者源码未改，连杆碰撞按用户要求关闭。
- 全10件持箱进舱路径再1cm采样独立IK逐点细化，1600回放帧，最大TCP误差8.49e-9m；Newton实跑已到B5，吸盘对齐箱上表面。全RPY渲染适配避免作者yaw-only丢roll/pitch。运动学规定箱pose，无吸盘物理/动态释放/取件/撤回，箱间q重置，非完整周期成功，不称容量上限10。

### 2026-10-22 纠正错误球心与半径解释
- 用户明确要求base_link球心；此前用肩部(0,0,1.031)和局部姿态优化1.853557m半径不符合要求，结果撤下，Newtonviewer停止。局部最大已找到不构成外包上界；不能称可信可达边界。
- 重新以base_link世界(0,0,.86)m为心，完整URDF至吸盘contact的origin长度总和2.671450m作宽松外包，不当厂家额定臂展/球内保证。原TF不改、机械臂碰撞关闭。同候选实际球拒绝0、IK拒绝223、10路径接受，14.36s。因此不能仅靠换球心宣称已解决深处IK，也不能把10件解释实际容量；下一需模型/TCP/求解分支核对。

### 2026-10-22 base_link球重新原生可视化
- Newton静态诊断窗口已打开并实际看图：青色线框以base_link世界(0,0,.86)m为心，URDF链长度总和2.671450m仅作宽松外包。完整原粗TF/RPY、固定底座不改。黄色为目标方向，红色为数值解TCP至目标的残差。
- B1深处水平目标距base_link为2.298175m，位于球内；保存48初值IK的最佳近似位置误差.526630m、姿态.214713rad。这不是球拒绝或碰撞拒绝，也不是数学不可达证明。未执行物理/真机。证据experiments/s20_author_strict_20261021/base_link_sphere_view.json与同名png，脚本show_base_link_sphere.py。

### 2026-10-22 按用户要求：厂家臂展95%球
- 纠正上一轮半径定义错误：用户要标称臂展95%，不是URDF链长度和。已读取厂家S系列官方规格表，S20 Reach=1700mm（S30才1800mm）。来源https://www.huayan-robotics.net/s-heavy-payload-collaborative-robot。
- Newton已改为base_link世界(0,0,.86)m球心，R=1.700×.95=1.615m，不加吸盘长度、固定原TF；已实际查看原生渲染截图。旧2.67145m展示撤下，证据base_link_sphere_95pct.json/png，未重新规划或操作真机。

### 2026-10-22 95%臂展domain接同学贪心+yaw采样
- 使用用户指定base_link世界(0,0,.86)m/R1.615m，原粗TF完整RPY；球约束施加于吸盘上表面中心抓点和全部直线路径端点（球凸，因此各段也在球内）。箱体仍使用原舱壁/支撑/扫掠几何，不要求整个箱体在球内。
- 原作者candidate_positions/route/validate_sample复用，平放yaw0/90；按后表面深度→低z→宽度顺序。球面与各y/z接触点直线的解析交点补充候选，向球内取整到mm，不另设虚拟平墙。最多四层，禁止搬移恢复；无IK/连杆碰撞/动力学/真机。
- 30输入上限，11接受（5个yaw90），第12贪心阻塞；max抓点半径1.614803m，0搬移，作者验证通过。CPU逐件规划总.040486s，逐件.00048–.00694s，非含渲染/IK。不是全局容量上限。新源码run_sphere95_layout.py、结果sphere95_yaw_layout.json/metrics.json位于experiments/s20_author_strict_20261021，Newton隔离回放show_base_link_sphere.py --packing。

### 2026-10-22 用户确认同排右→左填完再升层
- 定位原11件版的球面逐列最大深度相差约2cm，深度优先导致先叠高。用户确认先对齐同排，再从入口视角右→左填完同层后升层。
- 隔离--rows版在固定全20已知规格下，提交第一件前比较同后表面x的整排候选，优先容纳件数、再选更深排；逐件提交不移动已放箱。原作者候选/路径/支撑验证保留；局部y从小到大对应当前入口视角右→左，yaw0/90不变。此为新增行对齐规则，非作者原版或学习收益，不宣称未知尺寸在线泛化。
- 后表面对齐x=.772m；每层3/3/2/2共10，4个转90，无搬移。最大路径抓点半径1.614191m，规划总.052582s，作者validate_sample通过。旧11件版保留不覆盖；10不是最优容量。Newton --packing --rows回放，机器人静止，无IK/动力学/真机。

### 2026-10-22 标称臂展100%与90度优先
- 用户明确半径拉满，改base_link R1.700m（厂家标称，不额外加工具）；此前R1.615m原件保留。平放90度footprint为进深40×横宽55cm，先采90，再用0度55×40cm补空。排内各朝向右→左，90优先高于0补空的位置；排对齐、最多四层、原TF完整RPY与不可搬移保持。
- 固定全20行模板比较先90件数再总件数再深度，沿用作者几何/路径验证。13件成功、10横放3个0度；第14贪心阻塞。各高度4/4/3/2件；同层前后排分别后表面x=.806/.406m。最大抓点路径半径1.699836m，0搬移，作者validate_sample通过，总规划.134572s（含所有失败行候选），并非含IK/渲染。
- 旧10件与11件结果保留，不将多3件单独归因半径：本轮同时改了yaw优先。Newton原生回放sphere100_yaw90，机器人静止、无IK/动力学/真机，不宣称13为可执行容量或全局最优。

### 2026-10-22 仅最终箱顶中心筛球，修正外排侧墙间隙
- 用户明确中心指最终箱顶吸盘抓点。球R1.700m只筛最终抓点，进舱路径/箱体角点不裁球，保持原TF、90优先与行对齐。单独放松球约束对照仍13件，未虚报容量提升；箱体几何中心另作诊断不用于推荐。
- 侧墙5.7cm间隙来自门洞y0=.057和作者route_to在x<.04时直接拒绝侧移。原B4的x=.006/y0贴墙提案原hull/支撑/无重叠均通过却被硬门槛拒绝。原完整8000三角形container_collision.stl配FCL扫掠核对：进门、横移、下放均无穿透；也保存原/.006/.04三路径对照endpoint_sidewall_cad_audit.json。
- 隔离--wall-contact仅在原4cm门槛拒绝时追加侧移路径，用原path_clear及完整CAD扫掠筛选；不改作者仓库、实体门框/粗TF。10um内缩仅允许名义表面接触。最终17件、15横放2个0度，四层5/5/4/3，前排每层最右箱贴y=0；所有最终路径CAD扫掠通过，0搬移、作者validate_sample通过，规划.212667s。终点抓点max1.699836m，箱角可在球外。第18贪心阻塞，不称最大容量。
- 保留深排统一后表面x=.806m：右侧抓点已接近球面，中间列若单独深入会打破用户要求同排对齐，未擅自改成错列。当前无IK/连杆碰撞/动力学或真机，Newton只作几何回放。

### 2026-10-22 同规则半径对照与固定入舱位/ServoJ
- 同一最终箱顶中心球筛选、90优先、同排行对齐、最多四层协议：R1.300/1.500/1.615/1.700m分别7/9/9/17件。作者几何及完整CAD箱体扫掠通过；不是全局容量。Newton四幅同视角截图已实看，桌面可达球半径对比.png。
- 复用师兄保存的sourcecontact→lift→standardtransfer0/1/2，固定入舱位世界(-.493965,-.126395,1.400)m；实际URDF独立FK、原完整TF，吸盘刚性抓取变换保持。17目标首轮终点16过/路径13过。定位作者min_entry_z=.4让底层抬高后拉满臂展；只改实际净空高度、原落点不变，原prefix/舱门/完整CAD扫掠通过后路径15过。前10连续目标都可解，B11终点残差约23.8mm；B17终点可解但高位直进残差4.15mm。反向180度对称吸附姿态补查未解这两项，不称数学不可达。
- 用户再次强调ServoJ，不要点控。复用留存师兄servo_j.py的fjt_to_servo_j_deg+limit_servo_braking，20ms绝对关节流，持物从抬升到固定入舱位/对列/直进/下放是一流，无内部stop命令；吸附/释放/返回为逻辑边界。Newton本地SDK形态指令sink+FK回放，非真机ServoJ或动力学。首件pick流2.44s/loaded19.96s/return21.26s是参考模拟时长，不是实测硬件节拍。
- 几何17件≠15条独立路径≠10件原顺序可解前缀≠完整物理ACK。当前回放在B11前停，不跳过失败补件；无新RL/硬件/自动化恢复，源码与三类失败原件保留。证据experiments/s20_author_strict_20261021/fixed_entry*.json。

2026-10-22 用户反馈卡顿：实际中断栈定位Pyglet image._convert的RGBA→RGB正则逐像素转换占满主线程，不是IK在线慢；去掉录屏后约290FPS，亦暴露作者viewer按帧推进而未按墙钟节拍。已原生RGBA直送ffmpeg并60Hz墙钟呈现，原ServoJ20ms流不改；带录制实测59.89/59.94FPS。当前运动学预览，非动态控制验证。日志fixed_entry_servoj_realtime.log，原错误日志保留。

2026-10-22 用户质疑方向，暂停运动改静态Newton三坐标系正轴展示。红+X/绿+Y/蓝+Z，W世界、B实际base_link、U门口舱底居中平移显示的ULD平行轴（非container_link原点）。原TF不改：baseyaw−90度，ULDyaw约−179.425度；向舱里=ULD+X≈世界−X≈base−Y，不是base+X。原生/俯视侧视图已实看，scene_xyz_audit.json保留矩阵；未据图擅改TF。

2026-10-22 用户确认机械臂base−X约2m左右地面上料。独立对照baseY±.5m，平放20寸地面z0、原TF：−X2m双侧抓取残差约.346m不通过，1.7m约.095m不通过；1.5/1.3/1.1m双侧均抓点与后续连续IK通过。1.5m候选worldxy约(±.5,1.5)，TCP地面抓点z约.25。先竖抬TCP1.2m，再移到门外对列上方，再降到进舱高度，接原B1直进/横移/下放；删旧高位绕行。留存师兄ServoJ20ms采样左右各596/624条，11.92/12.48s参考时长。Newton两个独立starts-at-contact理想吸附方案，非从空手完整pick、释放或物理ACK，未验证连杆/转运碰撞；原舱内B1箱体路径已有CAD检查，不能外推新转运段。原2m失败与全源码保存floor_supply_ik.json；静态俯视对比已实看。

2026-10-22 用户确认开始base−X1.5m地面全周期。prepare_floor_cycle首版直接joint连线两侧扫舱壁失败保留floor_cycle_direct_joint_failure.json；发现实测起点TCP已在舱内，改先世界−X舱内姿态直退到x−.55、舱外升至1.2m，再连续到sourcehover、下降接触；同分支重求loadedIK，双侧完整几何644/738节点通过，原TF明确覆盖旧lower6cm默认，未改原标定。独立floor_cycle_newton_attempt001已启动，Newton原XPBD箱体/规定FK机械臂/理想吸附/20ms师兄采样器；地面到入舱首件进行中，无物理ACK或带载articulation声明。首件后才决定多件。

2026-10-22 地面首件Newton全周期实际结束floor_cycle_newton_attempt001：原TF、base−X1.5/Y−.5地面平放20寸，先实测舱内姿态直退再上料接近/下降/吸附/抬起/门外对列/直进侧移下放/释放/撤回/返回，0碰撞异常，54.78模拟/599.70墙钟秒/2740状态。师兄20ms采样按5逻辑流678/76/596/591/698指令；规定FK机械臂，5kg源/释放NewtonXPBD动态、理想吸附，无真实articulation载荷反作用。接触gap.0843mm，release后移5.930mm，最终nominal误差5.931mm；这是单件规定运动验证，不冒称真机/PhysX带载控制或连续20ACK。原生保存状态54.8s录像与首尾帧已生成/实看，桌面地面上料_完整首件_Newton.mp4。未恢复旧训练/定时。

2026-10-22 用户要求20件：floor20_newton_attempt001首件途中停止，未ACK，源码/status/log保留；发现首件演示回原舱内姿态不宜连续多件，改每件退回sourcehover；上层改用作者fully_supported核对成功prefix，禁止缺箱虚撑。attempt002同一Newton世界20来箱启动PID3594493；原TF/地面−X1.5/Y−.5、20ms师兄采样器/10×2ms XPBD不变，复用官方Warp CUDA graph加速。仍规定FK机械臂/理想吸附/动态释放，非关节驱动力学；17固定几何提案不等于20成功。

2026-10-22 用户明确撤去Pick阻塞，先直接吸盘末端生成已持箱的全20寸±5%各轴随机箱，每件只给当前尺寸重新在线规划和执行。旧floor20 attempt004处理20输入但仅2ACK/4持箱路径拒绝/11缺支撑/3固定模板耗尽，原件保留，不能称20全周期或在线布局成功。新run_attached_online20不沿用17模板，作者candidate_positions/route_to+真实prefix保守AABB，yaw90/0、最终抓点球R1.7/原TF；current-box-only候选、逐个IK/CAD尝试，再20ms师兄ServoJ参考/动态箱体释放/空手返回。机械臂仍规定FK、理想吸附，非Pick或关节驱动力学。attempt001因NumPyfloat64进入Warp模型构造失败，0动作；castPythonfloat修正attempt002启动。

2026-10-22 术语纠正：同学rule-based online greedy policy，在线重规划不是DynamicProgramming或RL。已持箱attempt002前2ACK，第3途中旧箱漂移碰撞；attempt003增加实测3.7mm漂移对应6mm横向间隙，3ACK后17零候选。定位适配错误：作者zs取整毫米但actual AABB top浮点未取整，fully_supported EPS1e-7全部拒绝上层，故3件不是物理满箱。attempt004将真实占据AABB向外取整统一1mm候选/支撑网格，保留失败，继续同seed73/20来箱/每轴±5%/无Pick验证。

2026-10-22 用户再次明确禁止改已调好policy。撤出本轮attached_online20自改候选排序/6mm间隙/1cm支撑规则；attempt005实际5ACK、第8落点偏42.19mm失败，原件保留，不推荐。恢复UNCHANGED run_sphere95_layout.py既有rows/fullreach/yaw90-first/landing-only/wall-contact；20标准来箱的原规则独立回放仍17几何接收/3blocked，全部17box字典逐项与先前用户认可layout相等。新run_attached_approved20只改直接吸盘已持箱初始化，不覆盖几何落点或按执行失败换候选；先标准20寸隔离初始化，±5%误差暂不叠加。原规则运行时支持/排序不做新修改；物理执行失败与policy容量分开。

2026-10-22 原policy已持箱接口attempt001：0运动/0ACK，在首件连续IK的6.191mm位置残差（原3mm阈值）停住，未改落点/未换候选，不声称20执行完成。原几何17件完全一致；现在问题定位执行接口，而非再修改用户policy。

2026-10-22 原policy执行接口修复：误接回旧40cm运输高度导致首件残差，恢复既有fixed_entry_clearance_ik执行轨迹后首件通过；第三件门外下放碰底座，执行层对原目标尝试.35/.45/.55/.65入舱高度后前3动态周期通过，旧箱被刚性吸附箱挤动约2.8cm/旋转导致B4阻塞，未改policy。另隔离attached_approved20_kinematic_attempt007，已持箱初始化、原标准20寸/原落点顺序/yaw、20ms师兄采样，释放位冻结而非XPBD动态。前10连续运动学周期完成，B11 IK残差1.0–1.5cm在运输段停止（历史B11终点23.8mm亦未通过），不是20成功或动力学成功；保持首10前缀不跳件。原失败/源码/状态保留。

2026-10-22 用户要求箱尺寸噪声20件：seed73、每轴真实±5%并1mm取整、已持箱、原policy direct/rows保持不变。attached_approved_noise20_attempt001处理20，1运动学放置/10行缓存尺寸不符拒收/9执行IK或CAD拒收，0新训练/真机；这是冻结释放位的运动学演示，不能称动力学成功。定位run_sphere95_layout.py原先固定全20行规划会缓存首件尺寸，后件变尺寸仍pop旧dims，噪声不能直接套用旧固定行缓存。没有修缓存或改排序/硬编码yaw以冒称policy不变；原失败/完整尺寸/20项结果/源码保留。后续需明确区分保留排序意图与修复固定尺寸适配，不能把1件归因同学作者原仓库容量。

2026-10-22 用户纠正要在线规划，覆盖保留固定行缓存的实现方式。run_live_online_noise20仅以当前尺寸+成功prefix生成作者candidate_positions/route；去掉假设未来同尺寸的整排trial/cache，不读固定17布局/路径。保留底层、当前对齐行、右到左、同位置yaw90/0意图。每次online_rule记录currentdims/committedcount/candidatecount/选位时长/future_arrivals_used=0，NativeViewer在求路和执行期间实时更新；录像若后编码仍标注actualstates replay。seed73、20来箱/±5%，吸盘已持箱、20ms师兄采样，释放冻结仍非动态验证。已真实启动且首1周期通过，不称20完成。

2026-10-22 真逐件在线live_online_noise20_attempt001完成20输入：8运动学周期/10当前几何无候选/2当前所选目标执行失败，无程序中断。每件当前size和成功prefix新调用candidate_positions和route+IK，未读保存布局/路径或整排缓存，实际在线rule耗时中位9.443ms/max26.739ms，8成功件完整规则+IK约1.3–4.1s（以result逐件值为准）；nativeviewer在实际运行更新。±5%各轴seed73，20ms连续参考，冻结释放位：不是Pick/动力学/真机。可视化后编码明确saved-state4x回放；不把录像回放称在线求解本身。

2026-10-22 用户明确允许叠放投影重叠/少量悬边/1cm高度差，不允许箱体相穿。隔离live_online_support20同seed73/20真实±5%来箱，不缓存整排；原author100%exactsupport改为顶面差<=1cm、每侧最多1cm悬边、实际支撑union>=90%且箱底中心有支撑，candidate仍需原collides/pathCAD通过。高度差1cm内作为同层，升层优先复用底层最深首排右侧，右→左填充。此为用户明确授权的规则适配，不再宣称原policy不变或作者原版；单独对比上一8件结果。

2026-10-22 live_online_support20_attempt002完成20真实±5%逐件在线输入：13运动学放置/4无候选/3所选目标IK或CAD未过，334.70模拟秒/140.86墙钟秒。第二层第6输入落点(.390,0,.244)、第7(.416,.532,.244)，从底层最深右侧起叠再向左；支撑高度1cm/单侧悬边1cm/union90%/中心有支撑、原几何无实体穿插。初版attempt001被标准25cm贴地起点污染：更厚箱初始穿地；attempt002改既有sourcehover，未改TF，故8→13同时含支撑与初始化变化不能独归某项。规则选位耗时median31.20ms/max77.90ms，IK单列保留。13是冻结释放位的运动学周期，非完整动力学/Pick/硬件；明确已授权的支撑/顺序规则适配，不称原作者policy不变。


2026-10-22 5cm支撑容差用户要求：分别20理想20寸和20混合20/24来箱，seed73混合各10件打乱，无额外尺寸噪声；代理55×40×25/65×45×30cm。仅当前尺寸+成功prefix在线生成候选；5cm指支撑顶面差与每侧支撑投影扩展，箱底中心仍有支撑，撤掉此前自加90%面积阈值；原nominal实体碰撞/CAD与IK1cm阈值不放宽，继承运动学箱间<=1cm浅接触例外仍记录，不能称严格实际无穿透。首次误将排对齐也扩成5cm：ideal attempt001完整10/20、mixed attempt001人工中断，原source/log/results保留。attempt002恢复原排对齐1cm，支持/同高度带5cm。
实际完整运行：理想20来箱完成10运动学周期、0无候选、10所选目标执行失败；同prefix同尺寸反复选第三层首目标(.399,0,.5)，不是10独立失败，也不能说几何已满。混合20来箱完成6周期（5个20寸+1个24寸）、0无候选、14所选目标执行失败。两组墙钟137.31/117.30秒，规则中位8.818/8.287ms、最大10.159/11.332ms，IK另计。冻结执行接口后的独立纯几何在线对照17/7件，接收prefix不同，不能将差值视独立因果收益；候选耗尽非数学满箱。
20ms师兄ServoJ参考、已持箱理想吸附、规定FK机械臂、释放位冻结，不是地面Pick、关节驱动力学或真机。原完整TF/RPY未改。实际在线运行nativeviewer更新；两份录像为真实保存状态4x回放，非录像阶段在线求路。已看两组native最终图，最终视频已生成；桌面在线规划_5cm支撑_理想20寸/20与24寸混合.mp4/png，原件experiments/s20_author_strict_20261021/live_online_5cm_*_attempt002。下一步保持规则，单独查连续IK/CAD执行接口，不恢复RL或定时。


2026-10-22 5cm支撑容差用户要求：分别20理想20寸和20混合20/24来箱，seed73混合各10件打乱，无额外尺寸噪声；代理55×40×25/65×45×30cm。仅当前尺寸+成功prefix在线生成候选；5cm指支撑顶面差与每侧支撑投影扩展，箱底中心仍有支撑，撤掉此前自加90%面积阈值；原nominal实体碰撞/CAD与IK1cm阈值不放宽，继承运动学箱间<=1cm浅接触例外仍记录，不能称严格实际无穿透。首次误将排对齐也扩成5cm：ideal attempt001完整10/20、mixed attempt001人工中断，原source/log/results保留。attempt002恢复原排对齐1cm，支持/同高度带5cm。
实际完整运行：理想20来箱完成10运动学周期、0无候选、10所选目标执行失败；同prefix同尺寸反复选第三层首目标(.399,0,.5)，不是10独立失败，也不能说几何已满。混合20来箱完成6周期（5个20寸+1个24寸）、0无候选、14所选目标执行失败。两组墙钟137.31/117.30秒，规则中位8.818/8.287ms、最大10.159/11.332ms，IK另计。冻结执行接口后的独立纯几何在线对照17/7件，接收prefix不同，不能将差值视独立因果收益；候选耗尽非数学满箱。
20ms师兄ServoJ参考、已持箱理想吸附、规定FK机械臂、释放位冻结，不是地面Pick、关节驱动力学或真机。原完整TF/RPY未改。实际在线运行nativeviewer更新；两份录像为真实保存状态4x回放，非录像阶段在线求路。已看两组native最终图，最终视频已生成；桌面在线规划_5cm支撑_理想20寸/20与24寸混合.mp4/png，原件experiments/s20_author_strict_20261021/live_online_5cm_*_attempt002。下一步保持规则，单独查连续IK/CAD执行接口，不恢复RL或定时。
### 2026-10-22 PiPER 抓放视觉 RLPD 接口试验复核

- 实际三盒视觉 RLPD 小预算已结束：1024 online decisions、897 gradient updates、19 episodes，成功 0；其中 11 局为纯 actor，成功 0。离线先验实际只有 1 条同接口合格专家、692 transitions，因此该结果不能证明 RLPD 或全部历史专家不可行。
- 失败证据来自真实 100Hz 物理记录：`actual_arm_dq` 10 次、夹爪/容器禁碰 4 次、requested joint limit 3 次、URDF limit 1 次、TCP tracking >5mm 1 次；最大实际臂关节速度 2.5369 rad/s，最大 TCP tracking error 11.722 mm。没有任何真实 pick、support-release 或 withdraw 成功。
- 原速训练回放保留在 `piper_rl/piper_adaptation/pick_place_frontend_20261019/rlpd_preparation_20261022/three_results/video/actual_training_actor_1x.mp4`；它是失败 actor 的实际观测回放，不是专家重演或仿真动画。
- 39 份历史专家已完成非破坏性阶段/接触证据标注，共 48,143 个 20Hz 控制区间、240,715 个 100Hz物理样本、1,235 个规划阶段段落；分为 23 份三方盒立方体变体和 16 份方盒+双圆柱平铺。当前全部 `training_ready=false`：缺统一动作边界/控制语义、显式 command 版本、同一 reward、terminated/truncated/mask 及整局物理资格。不能用阶段名或结束文件替代这些字段。
- 39 份不是 39 个独立条件；同一初始条件的路线变体需按 source family、初始物体/容器 pose、尺寸/yaw、质量/摩擦和任务命令去重，再拆整局 train/dev，避免泄漏。方盒+双圆柱组不能直接混入当前三立方体任务的 RLPD replay。
- NVIDIA Isaac Lab Bin-Mimic 核对结果：官方注册的是 robomimic BC-RNN-GMM 示范学习，stack 配置 `rewards=None`，使用 `grasp_1/stack_1/grasp_2/stack_2` 子任务切分、示范重定向和噪声重放；默认成功阈值约 5 cm，不能替代本项目双 5 mm、真实支撑、松爪和保持门。它提供数据生成/切分结构，不提供可直接用于 PiPER 的抓放权重。
- 当前决定：先统一并重建同任务专家转移，做多专家 BC 的闭环基线，再接 50/50 offline/online RLPD；不因数据“看起来少”直接切换 Diffusion/Flow Matching。生成模型可作为后续多模态动作基线，但不能修复控制越限、碰壁或缺失 reward/done 接口。真实物理成功事件仍由程序化接触/支撑/释放规则定义；阶段标签仅用于采样、课程和覆盖审计，不喂给部署 actor。

### 2026-10-23 长序列抓放重新核对与新数据契约

- 重新读取 Isaac Lab `DataGenInfoPool`、`DatagenInfo`、SkillGen 文档和官方 Stack/SkillGen 配置。官方数据字段是 `obs/datagen_info/eef_pose/object_pose/target_eef_pose/subtask_start_signals/subtask_term_signals` 加 `actions`；SkillGen要求起止信号由示范标注、按0→1边界切段，再用cuRobo连接片段并只保留通过任务终止检查的生成episode。官方Stack配置 `rewards=None`，是Mimic/BC入口，不能称为PiPER的裸RL配方。
- 固定12件作为唯一训练长度会学习到长度、顺序和槽位；只用单件也不会自动获得多件调度。采用外层贪心布局器选择当前物体/槽位，内层可复用单件抓放技能；训练混合1/2/3/6/12件，分开报告单件技能、已占前缀和长时累计误差。
- 新契约写入 `piper_adaptation/pick_place_frontend_20261019/dataset_contract_20261023.json` 与 `DATASET_CONTRACT_20261023.md`。阶段标签只用于审计/SkillGen/DAgger采样，不进入部署actor；四路视觉、本体、上一动作和显式command是actor接口；GT物体pose/接触/stage只保留teacher/physics sidecar。
- 新采集预算先固定为12条单件严格合格来源专家（4个独立source family），再48条单件几何/物性条件，之后24条真实2/3件连续前缀。旧39条和失败RLPD不删除，均标记audit-only，待同一reward/done/mask和控制codec重建后再决定是否入池；本轮未启动新RL。
- 原 `piper-2` 每小时自动检查已查询并删除，返回 `not_found`，因此不再把不存在的定时任务当作运行监督。
### 2026-10-23 PiPER 长序列抓放官方路线复核

- 重新核对 IsaacGymEnvs `FrankaCubeStack` commit `aeed298638a1f7b5421b38f5f3cc2d1079b6d9c3`：官方确有纯 PPO 堆叠，使用 cube 真值和稠密接近/抬升/对齐/松爪奖励；成功条件约 XY 2cm、高度2cm。它证明两物体长动作链可由RL学到，但不等于视觉、多物体、双5mm PiPER。
- 重新核对 Isaac Lab SkillGen 文档与 `isaaclab_mimic`：子任务 start/termination 由人工标注，官方明确不自动标注；stack配置 `rewards=None`，这是示范/BC/技能生成入口。PiPER事件标签必须由100Hz接触、支撑、抬升、开爪、撤回、保持证据重放得到，标签不进入部署actor，也不要求阶段停车。
- 修订抓放契约：目标改为 `destination_pose_xyz_yaw`（堆叠必须有z），选中物体增加视觉grounding引用，容器位姿命令明确来自外部估计/规划而非隐藏GT。固定12件不能作为唯一训练长度；外层贪心调度与内层单件技能分开验证，混合1/2/3/6/12件前缀。
- Shaki1资源核对：L40S无计算进程，/data剩余约116GB。现有单件专家619控制转移、严格物理通过；固定条件基线，不能冒称泛化。历史39条保留审计，因统一动作/命令/reward/done/mask和整局资格缺失暂不入RLPD。
- 决定：先在Shaki1按Stage A生成12条单件立方体/圆柱体、四个独立场景族，逐条物理验收并保留原速四流视频和失败原件；12条通过后再做48条单件条件化，再做2/3件占据前缀。当前不启动PPO/RLPD。

### 2026-10-23 S20：同学新标定、贪心与原三箱隔离对照

**本轮任务与责任。** 本聊天主进程执行；查同学仓库、新现场TF、AABB与异步，保留原三箱和几十箱独立成果，不联系真机运动聊天，不恢复旧训练。新增实验在`experiments/s20_greedy_async_20261023/`；可视入口`research_site/index.html`，桌面`S20研究/01当前推荐/S20研究日记与规划对照.desktop`。原件不挪动、不删除。

**源码阅读。** `ludan0414/RoboticArm`默认分支`greedy`，冻结commit`1f0600802ed6c4b7beb4b4187aba4c2f299449b6`（10月23日17:22）。已读policy README、greedy_sequence、有限工作域/IK、在线候选/支撑/路线/score、MotionCompiler/保存bundle、现场YAML、上表面/支撑估计、矩形细化和ServoJ适配。根目录未找到全仓LICENSE，嵌套组件许可不能自动视为整个policy许可；不把作者源码当自己作品发布。

**关键区别。** 同学默认greedy会看全部剩余箱并重排，四箱保存顺序B2→B4→B1→B3；准备阶段求IK/路线，回放阶段不规划，不是严格未知未来的当前来箱policy。online_packing_demo另一支还允许搬移旧箱，本轮不使用搬移。其Newton回放理想附着、释放冻结，非完整动力学；新仿真复用真实CAD/工具几何，未改作者源文件。

**现场TF核对。** 通过`ssh s20-jetson`既有密钥只读访问；Unix用户`hku_reconova`，hostname为`hkureconova`。实际感知launch加载安装目录scene_tf，world→container_link为`[-1.6587986012786127,-0.04373921696351268,0.00081639199681018]`，RPY`[-0.0045321426851967755,-0.03136878010657762,-3.131557004068548]`，与上游loading_site一致。相对records/S20/exports旧粗配置Y−0.11m/Z+0.10m；相对旧用户−6cm偏移分支Z+0.16m。未覆盖旧TF；配置一致不等于本轮独立复测。

**新鲜相机／关节。** 18:10:23完成3秒既有静止捕获，118次有效关节回读；控制柜RobotDisable，仅监测连接，无使能、运动、IO或参数写入。RGB-D640×360，画面朝地面，无完整行李箱，未检出tag36h11 ID3，因此没有生成新标定或现场尺寸。原标定脚本`/home/hku_reconova/lianaiwei/src/uld_observation_ws/recalibrate_current_uld.py`仍在。eno1 MTU已9000、Tailscale1280均未改，无LiDAR订阅。本轮capture目录保留原数据，无凭据。

**配对几何结果。** 12条相同箱流，每条8件，55×40×25cm每轴±5%；当前来箱、不窥视未来、不可搬移、平放yaw0/90、完整支撑。四方法共384个决策：旧三项布局score适配87/96、深度贪心80/96、贪心top4短路优先76/96、低层贪心80/96；候选/排序/箱体通道p95约9–10ms。旧score只是新候选池评分适配，不假称完整V3后端原样复现。该结果不支持贪心必然更好，前端毫秒级评分不是主要瓶颈。

**机械臂结果与边界。** 新鲜候选＋有限domain关节初值＋NewtonIK＋连续CubicSpline＋20ms FCL整臂/持箱CAD检查，三箱分别入口初始化，前箱作为后续占据。缓存JIT后贪心/10mm逐箱4.27、8.75、8.17秒；贪心/3mm为4.38、8.54、8.04秒；旧score/10mm为5.26、14.26、2.23秒，均三个入口放置/撤回诊断通过。贪心首件更深；不是完整地面Pick、连续三箱起点连接、动态落稳或连续SVSDF证明。对照同机并行、有竞争，不能给严谨单次加速比。首次冷运行包含26.18秒内核编译，不混为每箱在线搜索。servoj_reference.json复用同学20ms角度转换函数，无SDK调用或新增限幅。

**AABB与异步。** 选定目标三维点经RANSAC＋最大连通区域、平面内矩形＋地面高度差，输出OBB/世界AABB/box TF，不统一膨胀。仅平面拟合在合成杂点中错成95.44×63.71cm，补连通后为54.96×39.98×24.995cm；不当现场精度。保留`T_world_tool=T_world_box·inverse(T_tool_box)`。常驻进程异步几何已与前台真实点云建模计算重叠；场景变更重算几何，吸附/起点变更重算运动。尚未测得完整Newton求路被真实Pick隐藏的收益，不冒称完成联动。

**控制／文献决定。** 同学实际有SERVO_MAX_ACCEL_DEG=20和DECEL=20，硬件入口调用densify/braking；不能认为其完全靠SDK且无限速。本轮6秒/段连续样条，不逐点停靠、不新增软件速度gate，不承诺实机速度。检索GOPT/EMS、PPN剪枝、STEP时间偏好、cuRobo并行goalset、OMPL GoalRegion/LazySamples、VAMP、Implicit-SVSDF。建议保留旧cost，用贪心作启发，端点先筛再少量完整路径；合法目标区域容差不等于碰撞穿透。论文级GSIP/KC与完整感知—异步求路—真机链未完成。

**可视记录。** 新Newton三视图、同学四箱保存轨迹重渲染、三类目标布局、TF和AABB图均在media。原生headless FBO抓帧修复初次黑帧；俯/侧视只隐藏视觉遮挡、保留CAD特征边，碰撞模型未改。网站覆盖9月末至今可验证阶段，明确历史分页缺口；本轮刷新六个聊天最近两轮，另读S20 RL最近六轮、标定聊天，不声称补齐所有原始输出。旧训练/源码/权重/媒体保留。

**本轮继续：真正有效的先验排序与异步。** 初始贪心有76次端点拒绝，主要并非求解精度不够。按同学有限domain关节见证优先（其余候选仍保留回退），相同三箱目标的尝试次数从10/35/38降至2/3/4；该轮已测运动后端1.65/0.44/1.51秒，不含候选、初始化与撤回。随后将近邻域匹配向量化，并单列candidate_s与query_total_s，避免把前端开销漏掉。首次实现把种子优先置于高度偏好之上，导致不同profile都回成混合布局；该中间记录保留，已将高度cost优先级恢复到先验之前。

**最终偏好对照（新现场，不替换原三箱）。** 三种偏好各完成三个独立入口运动学周期；三叠输出三叠、两叠输出2+1，但平铺偏好在已放两件的固定前缀下没有找到第三件可执行平铺路线，最后回退2+1，不能称三种目标都实现。最终含候选的入舱规划：三叠2.603/0.271/2.526秒、两叠2.487/0.274/3.834秒、平铺偏好2.097/2.275/7.090秒；并发单次诊断，不是分布成功率。对应newton_profile_{flat,pair,stack}_final/result.json与独立三视图保留。最终释放允许与上游同口径的仅箱体接触≤1mm，非途中碰撞豁免；检查器只返回首个hit的问题已补齐，允许该最终接触对后继续扫描其它对，不能让合法支撑遮住臂/壁冲突。

**完整关节求路异步的新增证据。** `async_motion.py`在同一场景按真实墙钟20ms/帧回放已保存地面Pick前半段，共301帧/6.02秒，同时独立为当前单箱做新Newton关节求路。异步总墙钟6.52秒（入箱位后额外等0.50秒），串行实跑12.34秒；后台本体分别4.36/4.15秒，进程与导入开销没有藏掉。只是一次配对仿真回放，不是真实抓取或实际起点连接验证，返回reference_executable=false。前文仅几何异步的边界仍作为先前阶段记录保留；本新增结果不自动授权真机。

### 2026-10-23 S20继续：直进放下与NVIDIA GPU碰撞实跑

- **任务／边界。** 本聊天主进程负责，沿用上述新现场完整TF、真实S20/工具CAD和原三箱当前来箱搜索，不碰几十箱原件，不连Jetson、不写真机、不恢复训练；本机RTX5070Ti Laptop足够，本轮未占服务器。新增代码仍隔离在`s20_greedy_async_20261023`。这是规则/IK/运动学实验，不是新RL。
- **方法。** 当前尺寸＋已放占据→同一候选池/高度偏好＋有限domain初值→入口先对齐Y/朝向→固定朝向X直进→Z下降→20ms全CAD检查。Cartesian每2cm延续求IK，连续关节样条不逐点停车。凸性的论断只适用于固定朝向箱体经尺寸侵蚀后的自由区域，不能据此跳过整臂、自碰撞、工具与持箱验证。已测实际FK到折线偏离约1.8mm，不为这个数另设终止门。
- **结果。** 三叠和两叠各3个入口放置—撤回诊断通过；平铺偏好完成3件但第三件回退到叠放，仍不是纯平铺成功。`straight_stack_committed_final`最终逐箱含候选的放入规划3.201/2.475/2.298秒，空载新求路及回放准备2.194/2.160/2.202秒；初始化2秒量级另列，整次计算墙钟16.97秒，不等于执行时间。两叠/平铺尝试与GPU诊断并发，保留竞争下原始计时，不据此比较算法加速比。
- **撤回改动。** 空载从实际释放关节状态起，重新解直接返回入口的Cartesian线，不再强制倒放持箱折线。三叠逐件累计关节行程相对倒放减少20.4%/12.6%/5.9%，合计13.2%；这不是能耗实测。按同一Cartesian平均速率分配参考时间，空载4.60/5.26/5.82秒，持箱各6秒，参考运动合计33.68秒；尚不含任意Pick/公共入箱位连接/吸附等待，不能称完整三箱周期33.68秒。另将冻结释放先提交为同一障碍场景，再规划与回放空载路线，去掉固定相同场景下重复的整条空载检查；保留倒放回退路径的实际检查。
- **NVIDIA今年方案。** 核对官方cuRoboV2（arXiv2603.05493），取得完整作者仓库commit`78fd485fa82d9b9a063fb4985e371814587e666a`，Apache-2.0、示例资产许可另列。阅读README/依赖、SceneCollision/CollisionChecker、autograd、Warp离散与扫掠内核及作者碰撞例子，不冒称本轮重新完成整篇论文复现。独立venv为Python3.11/Torch2.8.0+cu128/Warp1.18，未覆盖原Newton环境或源码。
- **实际GPU数据。** 从相同保存FK/CAD导出4个静态mesh＋当轮已放箱；每条301帧×1363个连杆/工具/持箱覆盖球。作者批量世界碰撞中位12.23/12.89/11.27ms；作者swept-sphere为46.41/45.63/40.62ms，含球心梯度的前向/反向约48–51ms。首次JIT/场景构造另计，不将该数字当IK、FK导出或整体规划耗时。不是多环境RL或已接入在线关节修正。
- **GPU表示限制。** 为先接通作者接口，本轮采用各碰撞形状AABB的体积球覆盖，未改任务箱体尺寸/CAD；覆盖会在最后支撑接触前4/5/6帧提前报接触，没有因此推翻原CAD结果或缩小实际箱体。只跑世界碰撞，不含自碰撞；原CAD接受后端保留。作者扫掠代码`SWEEP_STEPS=3`，不是连续GSIP证明，更不是链上/关节动力学可行方向的KCSVSDF；拿到球心梯度不等于已完成关节轨迹修复。后续价值是持久场景＋多候选GPU批量筛查，需先减少覆盖空隙/过估计并明确接触阶段。
- **负结果保留。** Newton原生GPU串行接触接口同244状态中，FCL35个碰撞、GPU214个，179分类不一致，主要为link3/link5；含GPU读回14.04秒、CPU FCL0.488秒（并发诊断，非严格单独bench），没有冒称GPU一定更快或直接替换。该分歧尚未独立完成几何接触根因修复。初次GPU导出缺container字段、NumPy float32序列化均已修；离线Torch缓存不全、NVIDIA轮子CDN握手失败后用PyPI安装成功，未修改DNS/Clash/路由。日志与结果见`gpu_contacts.json`、`gpu_scene.json`、`curobo_v2_collision.json`、`curobo_v2_sweep.json`和protocol。
- **交付／证据。** 三种偏好各有原生三视图MP4；本机日记界面新增直线与GPU表。桌面`S20研究/01当前推荐/S20直进放下仿真.desktop`打开Newton原生已保存FK回放，3秒原生窗口实际运行已通过；窗口标题明确非实时物理。释放冻结、每件持箱入口独立初始化，未验证公共入口连接、地面Pick、自然落稳、连续扫掠证明或新真机速度。原版本/作者源码git工作区保持未修改，本轮不新增MD。

### 2026-10-23 S20继续：观测建模、区域搜索和完整单件异步流程

- **用户目标与执行范围。** 检查贪心选区域后局部搜索的开销，接相机建模后立刻异步规划，并在入仓位消费参考。本聊天主进程完成；仿真本机运行，Jetson只做独立订阅/CPU实测，没有SDK连接、使能、运动或IO，没有改其现有环境、网络、感知源文件和进程。源码/失败/结果仍在`s20_greedy_async_20261023`，不混入几十箱任务，不重启RL。
- **空间方法。** 已实现16cm XY分区→先试top4区域代表→8/4/2cm局部候选；Z只取地板/已有支撑顶，保留yaw0/90。当前尺寸与占据每次重新检查，有限domain只提供种子，其余区域保留回退；不是固定槽位或RL。前端本来已是接触候选，不是裸百万体素，所以不把区域采样夸成前端必然加速：本轮空舱基础47候选，区域/局部总98候选。真正的收益目标是减少昂贵IK/整臂尝试。先细化一个差区域会尝试15次；代表优先后2次、放入求路3.52秒，所选终点也改变，不能视为严格单变量加速。保留旧support谓词，不加新支撑门；单箱地板通过不证明堆叠容差已验证。
- **完整感知链。** 原S20腕部camera_link＋原TF，在Newton原生raytrace生成640×360 RGB-D，未用shape-ID/GT mask或隐藏尺寸喂感知。用已知拾取工作区内的深度连通物体，拟合地面/上平面/平面矩形，得到高度、OBB和AABB。首帧触边使55cm长度被看成36cm、中心偏9cm；实际抬高/平移观察后重拍，恢复54.99×39.98×25cm。没有用已知55cm回填尺寸。图像2像素下采样后约5571点，避免原3万点近邻图的无谓开销；保留两次原图、mask、点云和拟合数据。
- **从Pick到Place。** 完整单件同一世界/同一连续q：观察→调整视角→建模→后台区域/IK/带箱验证；前台接近/贴合→理想刚性吸附→提升并向内收→舱外横向对齐→入仓→按实际达到的分支延续IK→直进下降→冻结释放→空载撤回→返回观察区域。不是用原参考初始化每件入口，未把历史Pick录像当这轮执行。模型仍有已知CAD/标定/拾取工作区、模拟器真实吸附变换与碰撞几何、理想关节跟随/吸附和冻结释放等特权，不是自然落稳、载荷动力学、未知杂物全感知或实机成功。
- **真实失败和修正。** 纯竖直抬到人为高点遇到IK分支困难（1.4–1.8cm残差），改为抬升同时向可达区域内收，不放宽原IK判据。侧边斜着切向入口时持箱碰真实门框，改为舱外rail先横向对齐再接近；没有关闭碰撞。完整规划失败日志和实际失败前缀保留。中间unpaced17.65秒墙钟对应32.74秒参考，不报作实时全周期；其中旧staging_wait字段在等完后计算不正确，已在后续版本明确用wait_at_entrance和future.done记录。
- **串行/异步实跑。** `observed_async_paced_final`、`observed_serial_paced_final`都完成从新RGB-D到单件放置/返回，20ms真实墙钟节拍、同场景同箱。异步39.384秒、串行50.993秒，单次配对节省11.609秒/22.77%，不是分布成功率。初始化2.32秒；视角调整2秒、接近3秒、贴合1秒、提升1.5秒、抬升内收3秒、搬运6秒、入舱放置6秒、空载退出4秒、回观察6秒，加事件采样参考32.74秒。后台10.336秒（包括启动/模型/放入和撤回求路）被前台覆盖，入仓等待约0；实际分支放入续解0.49秒。6秒是运动段时长，没有另加6秒吸附等待。流程墙钟不含最终大状态归档和视频编码。
- **继续去冗余。** 发现后台还预算了前台不用的空载返回，又在工作进程内开一层Python。新增placement-only返回`validated_loaded_reference`/`loaded_references`，不把它计作placed周期；独立Python工作进程直接调用规划，不再套子进程。实际释放后的撤回仍正常求路/检查。优化版unpaced完整流程通过，worker5.59秒；这是不同代码版本的诊断，不能把13.29秒墙钟当作实时运动。CAD模型目前仍按查询初始化，不虚称已完全常驻。后续新试验开始即保存源码/场景快照；更早结果保留日志与状态，但没有事后假造对应源码快照。
- **Jetson前端实际基准。** 真实Orin aarch64/R36.4.7/Torch2.5 NV24.08/CUDA12.6，独立目录`/home/hku_reconova/s20_online_perception_20261023`。同5571点，建模warm中位36.67ms、warm p9539.44ms，首调用含导入421.58ms；0/1/2占据前缀区域搜索中位12.10/20.79/29.25ms。只是12次建模、每前缀20次的小型实测，不含YOLO/传输/IK/执行，不报成Jetson端到端。原系统SciPy1.8/NumPy1.26有兼容范围警告，本轮未改系统包；实际这些调用完成。作者几何两文件和原场景/domain只复制到独立目录，原安装源文件未改。
- **软同步落地与真实阻点。** 复用作者已存在的D555设备时钟→ROS时钟映射，不另造时间基准。`soft_sync_observer.py`将RGB与对齐深度按同header配对，在采集时间对关节历史插值，再查该时刻TF；可选发布独立String元数据topic，不改原topic或SDK。新鲜5秒得到71对RGB-D/相同header，raw图像年龄中位13.47ms、p9538.81ms；但/joint_states发布者0，实际关节配对0次。原检测器TF滞后约57秒并出现511.93ms lookup等待，不能偷偷用latest旧TF掩盖。该状态源没恢复，本轮不能宣称Jetson完整移动相机同步通过；未启动/重启他人硬件进程。旧118次真实静止SDK回读＋旧相机记录可离线插值（40.05ms括区），只是接收时钟/静止回放，不证明动态曝光同步。现有YOLO一次predict44.61ms、segmenter update69.96ms，但该帧0检测，不据此宣称真实箱感知成功。
- **cuRobo可加速的范围。** 对本次829个实际已持箱FK样本×1363覆盖球重新跑作者V2：世界碰撞批量中位28.56ms，3步swept查询107.84ms，13样本近吸附/最终支撑处被球覆盖标记。仍不含自碰撞、FK导出/IK/全规划，也未接替在线CAD接受，不能把这些数当Orin结果或端到端GPU加速比。V2官方当前文档driver>=580.65.06，现场JetPack6环境未改、未部署V2；旧cuRobo官方有Orin/Docker路线，不能拿旧版支持声明代替本V2验证。优先方向是常驻后端＋top-k goalset并行IK/局部验证，GPU世界距离作为候选与修正部件，不为此重写控制SDK。
- **研究依据与展示。** 本轮核对Wang/Hauser Stable Bin Packing（1812.04093）的高度图搜索动机、ROS2 message_filters官方源码和cuRoboV2 goal-set/安装/碰撞接口；仅该阅读范围，不宣称新论文全量复现。完整四画面MP4为实际保存FK加速回放＋两次真实仿真锁定观测，锁定后相机面板保持，不是假称全程腕部录像。桌面`S20研究/01当前推荐/S20相机建模完整抓放.desktop`打开原生保存状态，网站补全分段计时/Jetson/软同步边界，原视频全解码并查看，未新增MD或速度曲线。

### 2026-10-23 S20贪心顺序纠正与内部区域实时搜索

- **问题／决定。** 用户要求先最深右侧、再堆上方或向左展开、最后向入口移动。检查发现旧region key先排地板/历史端点接近度，且仅分组contact/domain点，不能说已均匀探索内部；保留旧结果，修改本聊天隔离实验，不改作者/旧交付/几十箱文件。新key深度优先，侧向按16cm区域右→左；深度排序四舍五入到1cm避免毫米差主导yaw选择，这不是碰撞/IK容差。flat/pair/stack只决定次级展开偏好。
- **实现。** 实际16cm内部XY候选＋边界/支撑面，yaw0/90；Newton官方batch IK每目标两种初始化/35迭代，收敛前4区域再8/4/2cm细化，保留其余回退。最终`--no-endpoint-hints`禁用历史XYZ候选，历史q只作初值；当前占据逐件提交，未知未来不输入。沿用当前源完整支撑谓词，没有新增更严支撑条件。未收敛不标为数学不可达。
- **负结果／修正。** 单纯扩大采样并batch端点仍很慢：`greedy_batch_live3`放入43.17/17.07/41.31秒、总计算111.27秒。先求完整IK再查早段碰撞浪费，且作者pose-only cache不区分分支，静态Newton近似接触与最终FCL又不一致。现在沿当前分支直接续解，每个IK结点尽早查同一CAD，再查20ms完整曲线；去掉重复近似静态接触调用，以完整FCL替代，不是取消碰撞。`greedy_batch_depth3_attempt1`第三件到入口、`greedy_depth_flat3`仍叠放，均不冒称理想布局或纯平铺。
- **真实结果。** `greedy_grid_frontier8`四偏好各8/8几何放置，单件中位4.70–6.50ms/max12.87ms；图只画前三件。`greedy_batch_earlycad3`3/3新鲜入舱—冻结释放—空载撤回，放入10.121/3.513/8.042秒，空载新求路0.856/0.874/0.817秒，总计算26.60秒。首两件位置(.48,.72,0/.25)、第三(.32,.16,0)，实际两叠＋一外侧；深候选先被当前种子/路线拒绝再退浅，不是全舱不可达证明。没有完整地面三箱Pick、自然落稳或连续GSIP声明。相比111.27秒含多项修改且第三目标改变，不作严格单因素加速比。
- **完整感知流程。** `observed_greedy_batch_paced`在原相机/场景中实际从观测/完整视角调整→新尺寸建模→异步区域规划→接近/理想吸附→提升/舱外搬运→放置→冻结释放→撤回/返回完成。实时节拍墙钟39.546秒、后台12.630秒，被15.904秒前台覆盖，入仓等待约0；放入查询10.425秒。148端点×2初值批量0.286秒，三层局部细化合计约0.067秒；后续整臂路径仍主导。部分运行与自有渲染并发，不将差值称受控加速；媒体编码/最终归档不计入流程墙钟。理想关节跟随、吸附、已知CAD/TF、冻结释放边界不变。
- **Jetson／展示。** 仅向独立`/home/hku_reconova/s20_online_perception_20261023/greedy_frontier_20261023`复制本次前端并实测：5571点暖拟合32.94ms/p9534.23ms，首360.56ms；0/1/2前缀区域中位13.46/20.47/31.00ms。12次建模、每前缀20次查询，不含YOLO/IK/传输/运动；没连接SDK、改网络或动硬件。新三件及完整单件MP4/三视图保存，现有本地界面新增贪心对照，原生桌面入口指向新完整单件保存状态；不是录像时在线求路。继续“空间实时贪心＋经验公共通道＋实际分支求路”，没有退成固定放置点回放，也未新开RL。

### 2026-10-24—25 本聊天：四箱观测、示教中转与异步规划

- **任务与来源。** 延续原三箱实时空间搜索，不套用32箱槽位。用户指定观察、抬升、入舱中转，再由当前来箱尺寸和已提交占据在线求放置。`pick_waypoints.json`中的观察点是旧实测姿态加世界Z方向10cm后的仿真IK，不是假称又读取了新姿态；抬升/入舱点有当时SDK只读回读来源。未来箱子不传给规划器。
- **真实完成范围。** `four20_attempt6`四件完成，非实时节拍计算墙钟135.85秒；`four20_async_attempt1`四件完成，实时节拍墙钟248.80秒。后者四次后台规划10.06/10.33/21.53/57.54秒，入舱等待0.23/0.24/1.53/36.04秒。两者节拍模式不同，不能用总时长直接声称异步加速。异步确实从尺寸建模及吸附目标确定后启动，与前台接近、吸附、提升、搬运重叠；到入口用实际状态复核，失败则回退求路，而不是把未验证后台结果直接执行。
- **证据边界。** 包含仿真观测和完整关节参考，但仍有已知统一20寸任务先验、理想关节跟随、刚性吸附、已知CAD碰撞和冻结释放；20ms离散曲线检测不等于独立连续扫掠证书。不能写成真实传感器闭环、自然落稳或硬件四件验收。
- **原件。** [实验目录](../../experiments/s20_online_mixed8_20261024/)；[异步实际节拍录像](../../experiments/s20_online_mixed8_20261024/media/four20_async_attempt1/online_newton_perspective.mp4)。媒体与当前日志索引见[本聊天近期清单](research_media/current_chat_recent_20261025.json)。

### 2026-10-25 稀疏候选、端点预筛与重复计算

- **定位。** 原第四件132候选生成仅52ms，主要时间不是空间采样，而是前四位置反复试两朝向×两高度的16次IK失败，以及重复公共路径。隔离改动保留旧版：贴边/箱顶锚点替代密网格，最多交付10候选；先批量终点IK，公共路径按完整上下文缓存；规划结点约8cm/0.15rad求IK，再用PCHIP生成密集20ms参考并查CAD。执行采样密度与求解密度分离，并未因此获得连续碰撞证明。
- **加速来源。** 官方Newton批量40个pose-seed问题约10–12ms，是端点预筛的小批查询，不是完整轨迹时间，也不是cuRobo V2或RL加速。当前轨迹仍由邻近分支续解、实际场景CAD和事件分段组成。
- **失败不删除。** `four20_sparse_attempt1`第二件因冻结释放后的底部约1cm变化而缺失地板锚点；attempt2是NumPy整数JSON序列化退出，不是IK无解；attempt3虽完成四件，但缓存成功值`(path,event)`被调用层误解为错误。修正缓存包装为`((path,event),None)`后，attempt4重新从空舱完成四件，不拼接旧运行。
- **当前结果。** attempt4墙钟46.49秒，后台1.391/1.859/1.722/1.591秒；入口等待0.007/0.023/0.038/0.056秒。第二件后台结果失败，实际入口重规划与提交3.245秒，其余三件消费后台结果。46.49秒为非实时节拍，不能与248.80秒实时节拍作为受控端到端加速比。候选交付8/7/10/10；固定前缀三顺序对照12/12，只证明这些同输入查询，不是连续12件或通用成功率。
- **媒体口径。** [四件三视图](../../experiments/s20_online_mixed8_20261024/media/four20_sparse_attempt4/online_newton_three_views.mp4)约112.9秒，按保存参考轨迹时间回放，不含规划等待；5637状态、1129渲染帧、stride5、10fps。不能把该片长度当实际计算墙钟。

### 2026-10-25 纯布局：接触排序失败、修正与用户回退

- **设置。** 每次仅给当前来箱，55×40×25cm各轴±8%，8件；最大面水平、yaw0/90、不移动旧箱。此阶段只验证几何排列，尚无整臂IK、吸附或物理执行。V1接触排序seed73第一件深度x约1.103，第二件退到x约0.510且仍靠右，出现提前向外和局部高塔，用户不接受。
- **修正与负结果。** V2把深度行排在右侧列前，再比较接触；最初脚本import顺序却载入旧目录较松支撑实现，旧结果不能冒充正式验证。修正为本实验目录优先，`layout_preview_verified_attempt1`两方法×三seed73/74/75均8/8，独立几何核对通过。V2单件平均13.4–15.3ms、最大29.1ms；加入实际粗球后的三seed为8/8/7，不掩盖少一件。此处不是机械臂全流程成功率。
- **当前选择。** 用户要求回退，默认恢复`frontier`；contact V1/V2及失败媒体隔离保留。V1源码归档是由V2恢复旧排序的重建件，不能假称当时启动源码快照。整臂`SparseCompactPrior`没有被悄悄切到contact版。
- **已发现的遗留问题。** 回退版也不完全满足“同一深度行先填右到左、再向外”。其排序先高度5cm分组，再右侧2cm分组，最后深度2cm分组，因此有更深但偏左的候选也可能输给靠右但更外的候选。当前可视化如实揭示，不在生成图时偷偷改成理想顺序。

### 2026-10-25 存储、部署与相机核对

- **存储。** 5637帧状态的三次热缓存写入对照：压缩NPZ中位60.38ms/1.85MB，非压缩2.24ms/4.04MB；不含fsync，不能声称磁盘持久化速度。隔离runner已改为实时非压缩、结束归档压缩；该存储改动后未重跑完整四件，不追认其已作用于上述计时。
- **GPU边界。** cuRobo V2已有常驻与CUDA图机制；`enable_graph=False`控制的是PRM图，不是CUDA graph开关。CPU/GPU拷贝和世界模型缓存可能有优化空间，但本次只是源码审查，未做新V2 GPU整臂基准。内接payload覆盖球可能漏边角，不能取代当前CAD验收。
- **部署范围。** 新Jetson包为独立接口部署，不是完整三箱自动真机验收。旧代码已有ServoJ、泵IO和压力逻辑，新wrapper不等价于旧完整工作流；不同版本lookahead100/200ms记录保留。不自动使能、复位、开泵。早期只读SDK可用、后续返回39504/连接不可达，不能把之前通了说成当前仍在线。
- **相机。** 旧启动参数640×360@15可覆盖YAML；raw、YOLO、3D输出频率不同。1280×720@30及896×504@60只是待设备确认的候选档，不是已在D555应用或实测最高能力。本轮没有修改MTU、DNS、路由、IPv6。用户此前确认相机未通电，不能宣称已完成实时感知验收。

### 2026-10-25 候选决策可视化与研究track复核

- **展示什么。** 展示真实候选、过滤结果、排序键、选择与占据更新，不展示或推测个人隐式思考。当前为锚点候选＋字典序评分＋逐件贪心，不是RL，也不是一个统一加权cost。历史三箱加权代价与独立PPO实验要分别介绍，规则收益不能计为网络收益。
- **实现不改选择。** `sparse_prior.py`只加可选的选择完成后`trace_sink`，另由`candidate_trace.py`记录已实际构造的候选；8件选择与回退原件逐一相同。未生成XYZ就被高度/横向区间剪掉的组合不虚构为点。原始锚点数可大于10，最终交给下游的shortlist不超过10。
- **具体反例。** seed73第2件13个已构造候选，10个入短名单、3个支撑拒绝；更深但靠左的候选列第5/6，而靠右较外的列第1。第8件179个已构造点，只提交10个。由此回答“为什么直接往外走”：这首先是排序优先级，不应全归咎于IK、碰撞或采样太少。
- **可查看证据。** [候选原始记录](../../experiments/s20_online_mixed8_20261024/candidate_trace_20261025/trace.json)、[第二件决策图](../../experiments/s20_online_mixed8_20261024/candidate_trace_20261025/overview.png)、[八件过程图](../../experiments/s20_online_mixed8_20261024/candidate_trace_20261025/timeline.png)、[原生透视渲染](../../experiments/s20_online_mixed8_20261024/candidate_trace_20261025/candidate_step2_seed73_view1.png)。绿色线框是备选位置，不是同时放入的物体，也不是机械臂运动录像。
- **维护方式。** 当前聊天主进程负责结论与版本记录，协作代理仅整理历史/媒体/已有界面。原件不移动、不删除，不改几十箱、师兄源码或真机进程。本次进行人工证据复核，不恢复已取消的周期任务。后续每个真实新实验按问题→改动→结果→负结果→证据层级→下一决定追加同一日记。
- **从头追溯的真实覆盖。** [本聊天历史清单](research_media/current_chat_20261025_history.json)按七阶段索引：旧三箱→固定32箱历史引用→回归实时三箱→STEP/PPN调研→规则泛化失败→Newton/贪心八件→当前感知/异步/稀疏候选。13份同聊天本地rollout流式扫描到文件末尾，筛出1116条公开用户/助手文本；仅抽取主题及时间段人工复核，不声称逐条精读全部文本。直接可追溯范围为本地时间10月11–25日，九月及十月初依赖旧记录，父聊天未全量读取。API本轮读取2页且仍有后页；旧清单记录的大回合缺口保留，不宣称所有原件已回传或完整验收。仅保存概述与证据路径，不保存原始凭据或私有推理。
- **本次check。** [人工复核清单](research_media/current_chat_review_20261025.json)记录八次选点与回退原件一致、前缀只含既有占据、候选最多10个、四张诊断图及历史覆盖限制。近期11段视频只做元数据清点，不冒称本次全部解码；旧单片全解码证据另行保留。已有界面由协作代理检查桌面/移动布局及控件，本主进程打开file协议被浏览器策略阻止，未绕过；候选静态图已直接查看。

### 2026-10-25 用户选定Contact V2：20/24寸随机混流、尺寸±10%

- 用户认可`layout_preview_verified_attempt1/contact_seed75_view2.png`的排列，要求混尺寸配置。保持该Contact V2排序、同学精标场景、1m装载高和yaw0/90，不改回退版默认、不修改任何旧图。仅为`layout_preview.py`加可选`--size-mix 20/24`；原全20默认及随机采样不变。
- 每件到达时才独立以等概率抽20/24类别，再对名义长宽高独立均匀±10%；不预告未来、不重排旧箱、不回放轨迹。20寸代理55×40×25cm，24寸代理65×45×28cm，非实测尺寸。噪声在本轮指实际尺寸变化，不是位置误差或感知误差。
- seed73/74/75分别8/8、8/8、8/8几何接受；单件平均14.01/11.49/10.15ms，最大28.07/24.21/18.03ms，不含渲染/IK/运动。类别序列分别24,20,24,20,24,20,24,20；20,24,24,24,20,24,20,20；24,20,24,24,24,24,24,24。最后一组偶然有七件24寸，因此比全20图明显更高，不能人为挑成四大四小冒充随机流。
- 最低同高底面支撑比例分别100%/83.54%/87.33%，原>=80%及中心支撑谓词未放松，未将不同高度顶面合成理想平台。当前为纯几何静态Newton渲染，未启可达球，也没有整臂IK/动态落稳；不声称机械臂可执行或尺寸不确定下的鲁棒感知。已查看seed73、75第二视角原生图。
- [结果与六张双视角图](../../experiments/s20_online_mixed8_20261024/layout_preview_mixed20_24_jitter10_20261025/)、[逐件尺寸/位置/耗时](../../experiments/s20_online_mixed8_20261024/layout_preview_mixed20_24_jitter10_20261025/results.json)。不启动真机、不新增训练。

### 2026-10-25 本机交付候选：Contact V2接全流程、容量与硬件边界

- **最新授权。** 用户先要求部署与试跑，随后明确Jetson、相机、控制柜、真空泵均未通电；本轮改为本机准备，明天再另行部署。停止连接尝试，不使能、不发运动/IO、不修改网络。此前Tailscale S20显示离线，100.97.142.96连接超时，192.168.3.12 banner交换超时；不是再次归因于密码或标定。
- **抽离真实数据接口。** 新`deployment/planning_candidates.py`接收当前观测尺寸、已提交占据、request_id/scene_version，常驻JSONL stdin输出Contact V2候选及舱体/场景坐标目标。无Newton Runtime、SDK、未来箱或自动占据提交；24条已保存混流请求均与原图选点一致，常驻进程24/24返回并回传版本。目标姿态不是吸盘工具姿态，仍需实际吸附变换、IK及持箱/空载路径。最初内存tuple与list直接比较报false，归一化JSON数组后全相同，保留说明，不误称规划差异。
- **真实新失败。** 为四箱runner新增可选`--layout-prior contact`，默认仍frontier。attempt1第一件16次终点IK未收敛：Contact几何类默认关闭粗球，而原机械臂Sparse类默认启用1.7m粗球，替换类时遗漏显式参数。修正机械臂调用显式保留原1.7m球；不是添加新限幅，也未以球筛代替IK/碰撞。失败原件保留。
- **新全程仿真。** [four20_contact_delivery_attempt2](../../experiments/s20_online_mixed8_20261024/four20_contact_delivery_attempt2/result.json)从空占据连续4/4观察/选择/拟合→后台规划→中心贴合→理想吸附→抬升/示教入口→放置→冻结释放→独立空载返回。全部placement标为contact、returned_observation。非实时节拍计算墙钟38.4358秒，分箱9.439/9.804/9.138/9.149秒；不等于真实运动时间。后台1.658/2.148/2.350/1.379秒，入口等待0.0013/0.0035/0.0059/0.0083秒。第二件后台18次失败后actual入口重算并提交2.009秒，未把失败后台当成功；其余三件实际状态离散CAD复核后消费后台结果。
- **没有夸大的部分。** 四箱保持统一20寸任务先验、理想跟随、刚性吸附及冻结释放；不是混尺寸整臂成功，也不是自然动力学、独立连续GSIP或实机验收。没有新增速度限幅或冒烟/回归框架。保存状态三视图回放不含规划等待，视频长度与计算墙钟分开。
- **全20寸容量。** 同一Contact V2、1m高、关闭可达球、零人为间隙：精确55×40×25cm三seed各接受26/64，首次拒绝第27件；尺寸±10%三seed最终14/16/17件，首次拒绝分别第14/15/16件，拒绝后允许下一件较小箱继续，故首次拒绝前连续前缀为13/14/15。所有接受项几何支撑/舱内/不重叠复核通过。它是有限稀疏搜索找到的数量，不是理论最大容量或实机可执行数量，也不混用旧32箱。
- **容量原件。** [精确尺寸汇总](../../experiments/s20_online_mixed8_20261024/capacity20_contact_20261025_exact_attempt1/summary.json)、[尺寸噪声汇总](../../experiments/s20_online_mixed8_20261024/capacity20_contact_20261025_jitter10_attempt1/summary.json)。布局更大规模时包络利用率仅70.6%/35.1–45.2%，说明尚有排列空洞，不宣称贪心最优。
- **明天仍需完成。** 原旧版有ServoJ/IO实现，新版仍缺由新候选到实际SDK全循环的硬件编排；不得给用户一个伪装为已验收的自动带载启动命令。设备通电后先读实际关节/控制状态、相机观测及既有IO语义，再接实际吸附、入口复核、定时ServoJ和释放/撤回回执。当前本机完成不追认为远端部署完成。
- **交付前实际自查。** 首次便携包漏当前compare/compact_prior/support模块，静默导入旧sibling的5cm支撑放宽版本，24条请求仅22条首选相同，确为算法行为差异而非浮点。坏包/失败清单保留，补齐当前依赖并移除旧版本后，干净解包首选及完整有序候选均24/24严格匹配；没有靠加比较容差掩盖。最终[本机交付清单](../../experiments/s20_online_mixed8_20261024/deployment/local_delivery.json)明确区分便携几何服务、原仿真环境和硬件缺口。
- **视频与复跑。** [新四箱三视图MP4](../../experiments/s20_online_mixed8_20261024/media/four20_contact_delivery_attempt2/online_newton_three_views.mp4)5553保存状态、1112渲染帧、111.2秒，1x参考时钟、不含规划等待；ffmpeg全解码无报错，已查看终态三视图。新增`experiments/s20_online_mixed8_20261024/s20-contact-sim --output <新目录> --seed 73`本机快捷命令；它调用原本机环境，不是可在Jetson直接执行的硬件命令。未继续调整用户速度口径。

### 2026-10-25 Contact混流采样与选择过程可视化

- 用户要求可视化采样点，供研究track及汇报。新增Contact V2最终选择之后的可选trace：父类初筛记录延迟至Contact联合顶面锚点补充与最终排序完成后输出，避免把父类中间短名单错当Contact最终决策。包括实际构造后被拒绝的联合顶面候选，不虚构早期区间剪枝的XYZ。未改变候选生成/评分/选择或新增执行门槛。
- 复跑20/24混流seed73，各轴尺寸±10%，8个选点逐一与用户认可的原示例一相同。各件已构造唯一候选数8/21/27/73/86/109/133/159，最终shortlist8/10/9/10/10/10/10/10。点记录包含XYZ、尺寸、yaw对应尺寸交换、排序键、接触量、同高支撑比例、拒绝原因和当时已提交占据；不含未知未来。
- [36秒候选动画](../../experiments/s20_online_mixed8_20261024/contact_mixed_candidate_trace_20261025/candidate_decisions.mp4)逐件以生成→过滤→选中三阶段展示，各阶段人为保持1.5秒方便观看，不是实时推理延迟或机械臂执行。红叉拒绝、灰点其余可行、绿点短名单、蓝星选中；俯视投影可能重合，准确高度查表/JSON。
- [第4件采样排名图](../../experiments/s20_online_mixed8_20261024/contact_mixed_candidate_trace_20261025/overview.png)、[8件过程总图](../../experiments/s20_online_mixed8_20261024/contact_mixed_candidate_trace_20261025/timeline.png)、[原生三维线框候选](../../experiments/s20_online_mixed8_20261024/contact_mixed_candidate_trace_20261025/candidate_step4_seed73_view2.png)、[原始trace](../../experiments/s20_online_mixed8_20261024/contact_mixed_candidate_trace_20261025/trace.json)。绿色线框为替代候选，不是同时放置物体。视频全解码通过，静态排名图/三维图已查看；仅几何诊断，无新IK/动力学/真机验收，不重启训练或定时任务。

### 2026-10-25 同轮机械臂、感知、异步候选与计时联合录像

- **目的。** 用户要求把采样决策与完整pick→place放入同一段录像，逐件展示建模后后台规划、选点、放置及实际耗时。本轮不把混流八件几何图拼到四件机械臂上：使用统一20寸、原示教观察/抬升/入口和Contact V2的新连续四件实验。20/24混流整臂仍未验收。
- **最小记录改动。** 可选`--record-decisions`保存实际持箱规划的候选trace；增加request_id、同机monotonic起点、每次观测完成时刻、候选生成/路径选定时刻。父基线保持原速度/几何/碰撞；新增`render_decision_cycle.py`按真实时间对齐，不改变选点。几何第一名蓝星与通过完整路径的目标橙星分开，不能把几何偏好当IK已通过。
- **保留负结果。** `four20_contact_decision_attempt1`开启原实时camera预览，每次运动publish反复raytrace/保存，144.76秒仍未完成第一周期；人工中止保留原状态/观测/源文件，不归因于IK失败。新增可选`--saved-camera-only`只跳过额外预览渲染，观察时真正的RGB-D、YOLO和建模仍运行；默认旧行为不变，无新增限幅或测试框架。
- **新运行。** `four20_contact_decision_attempt2 --record-decisions --realtime --saved-camera-only`同一新空占据运行连续完成4/4，四次空载撤回观察位。初始化21.51秒单列；节拍墙钟193.72秒，不与旧38.44秒unpaced计算墙钟当同口径速度比较。仍为理想跟随/刚性吸附/冻结释放/离散CAD，不是动力学落稳或真机。

| 当前件 | 空间候选选择 | 后台计算总计 | 舱口等结果 | 后台采用 | 候选后路径搜索 |
| --- | ---: | ---: | ---: | --- | ---: |
| 1 | 5.87ms | 10.321s | 4.31ms | 是 | 7.003s |
| 2 | 8.54ms（实际入口重算） | 3.667s（失败） | 9.91ms | 否 | 2.983s（实际入口） |
| 3 | 16.10ms | 3.640s | 20.21ms | 是 | 3.377s |
| 4 | 26.02ms | 2.269s | 37.60ms | 是 | 2.015s |

- **计时边界。** 后台计算包括当前worker求路/参考构造及其内部理想检查，不等于纯空间采样，也不包含worker初始化/排队。候选后路径搜索从实际候选记录完成到完整路线选定，排除先前入口公共段、几何生成及后续执行。第二件重规划＋节拍执行22.339秒，不能全标为规划时间。当前轮首件10.32秒不能偷换成上一轮1.66秒；记录实际值，不声称已受控解释全部差异。
- **同源复核。** 4个request_id/scene_version与对应候选占据前缀、最终通过路径的候选逐一匹配；5553状态的wall时间单调，候选生成早于路径选定。实际构造点8/14/40/73；本轮包含原1.7m粗球与2cm侧向设置，不是无球混流纯布局的点集。复核文件`decision_provenance_review.json`仅是本次证据审查，不增加执行gate。
- **证据入口。** [逐件计时](../../experiments/s20_online_mixed8_20261024/four20_contact_decision_attempt2/decision_timing_summary.json)、[同源复核](../../experiments/s20_online_mixed8_20261024/four20_contact_decision_attempt2/decision_provenance_review.json)、[原始完整结果](../../experiments/s20_online_mixed8_20261024/four20_contact_decision_attempt2/result.json)。合成面板采用该轮保存状态和观测；相机画面明确为最近保存观测，不冒充全程相机直播，不提前显示尚未生成的候选或尚未完成的耗时。
- **完成媒体检查。** [同轮合成MP4](../../experiments/s20_online_mixed8_20261024/media/four20_contact_decision_attempt2/pick_plan_place_combined.mp4)为1920×1080、10fps、192.5秒，沿wall_times_s包含实际暂停与规划等待；与193.72秒运行墙钟差额为末状态之后保存/收尾，不补造运动。视频全解码通过，已查看中段和末帧，文字未遮挡，第二件重算/第4件返回均明确展示。[媒体时间与来源](../../experiments/s20_online_mixed8_20261024/media/four20_contact_decision_attempt2/pick_plan_place_combined.json)保留同轮路径和计时口径。旧录像与人工终止尝试不删除；仅更新原固定查看入口，未新增网站或训练。

### 2026-10-25 三件20寸＋一件24寸随机顺序整臂全流程

- **任务/输入。** 用户要求3×20寸、1×24寸随机顺序的完整流程。隔离运行`mixed3plus1_contact_attempt1`，seed73随机置换为20→20→24→20；地面朝向继续随机。20寸名义55×40×25cm，24寸65×45×28cm，本轮未另外叠加尺寸±10%噪声。先前混流±10%的八件仍只按原几何证据保留。
- **改动与边界。** `four20_sparse_cycle.py --arrivals three20-one24`为可选模式，旧four20默认不变。先YOLO/深度锁定/上平面拟合，再用当前拟合尺寸在20/24两个名义类别之间按对数尺寸距离取最近项，无新阈值；不是从仿真actor读取尺寸，也不是任意尺寸纯观测估计。将当前名义尺寸接入吸附半高、目标/朝向、持箱代理和释放后占据，去掉这些路径中的固定20寸计算。观测值与所用名义值分别记录、视频分别显示。
- **未来隔离。** 后台初始化只接收四个20寸占位模型，不接真实来箱catalog或顺序；每个请求只有当前拟合/当前匹配尺寸、已提交占据及前序位姿。第三件实际出现并匹配24寸后才重建该slot模型，旧前缀保持、未到达slot仍为隐藏占位模型。不是预先将未来24寸塞进规划器，也不预存整段轨迹。旧默认统一20寸仍沿原名义尺寸运行。
- **实际结果。** 新空占据同轮连续4/4完成观察/建模/中心吸附/抬升/入舱/放置/冻结释放/独立空载返回。节拍墙钟149.600秒，初始化4.845秒另列；不与unpaced38.44秒比较速度。后台分别1.594/2.181/4.406/1.444秒，入口等待3.77/4.89/10.65/23.45毫秒。第2件后台失败后实际入口重算，其候选后路径搜索1.841秒；第3件4.406秒包含当前24寸模型更新。后台实际采用3/4，不掩盖第二件失败。
- **逐件复核。** 当前拟合匹配20/20/24/20，全与生成类别一致；前序占据数、最终接受候选、当前请求尺寸与模型尺寸、四次空载返回逐项核对通过。第3件拟合约63.27×41.86×26.75cm，规划名义65×45×28cm，不能声称无先验的任意尺寸准确建模。完整[计时与来源复核](../../experiments/s20_online_mixed8_20261024/mixed3plus1_contact_attempt1/mixed_timing_and_provenance.json)、[原始结果](../../experiments/s20_online_mixed8_20261024/mixed3plus1_contact_attempt1/result.json)保留。
- **未增加的声明。** 仍是本机图像/深度观测、理想跟随与吸附、冻结释放、离散CAD检查；不是自然落稳、连续扫掠证明或真实硬件。没有网络/IO/运动操作、没有新增速度限幅或训练，也未改旧三箱交付和几十箱任务。
- **联合媒体完成。** [混流完整MP4](../../experiments/s20_online_mixed8_20261024/media/mixed3plus1_contact_attempt1/pick_plan_place_combined.mp4)148.7秒、1920×1080、10fps，沿同轮wall_times_s含等待，不混接旧统一20寸录像；相机是当前件保存观测，尺寸先验明确显示。已全解码通过，并查看101秒的24寸放置帧及最终4件返回帧。运行149.60秒与片长差约0.90秒为末状态之后收尾，不造帧补周期。原生三视图、5553前轮与本轮5543状态分开保存；本轮来源/耗时见同目录合成JSON。复跑命令：`experiments/s20_online_mixed8_20261024/s20-contact-sim --output <新目录> --arrivals three20-one24 --record-decisions --seed 73 --realtime --saved-camera-only`。
- **汇报结构建议。** 一小时可按任务建模8分钟、原三箱与泛化10分钟、感知/TF/异步10分钟、候选排序及失败12分钟、GPU/存储加速8分钟、学习路线及负结果8分钟、边界与协作4分钟组织。可讲清工程insight，但不能把离散碰撞、冻结释放或规则前向包装成完整GSIP/KC复现、动力学证明或RL成功。

### 2026-10-25 Jetson隔离迁移与授权MTU恢复

- 新部署根目录`/home/hku_reconova/s20_contact_mixed_20261025_r1`，旧部署不覆盖。194个源码/必要资产逐文件存在与大小核对，额外加入有界相机收帧脚本；不迁移凭据、缓存或整套历史媒体。ARM实际运行24次候选查询，全部排名、箱坐标及尺寸与本机严格一致；完整字典比较最初失败仅来自变换浮点末位差4.44e-16，原失败保留，未更改算法或容差。Newton/Warp未安装，不称整臂后端已在ARM可执行。
- 用户明确批准短暂down/up `eno1`恢复MTU9000。先备份，在线修改因Device or resource busy失败；批准后down/set/up成功，三个IPv4地址保持，网口稳定后主路由与原始逐项相同。未改持久配置、代理、DNS或IPv6。恢复后SDK真实回读正常，q为187.938/-135.923/-88.869/-43.091/91.055/10.097度；真空DI0/DO0/DO1均0，无运动、泵IO或控制柜配置写入。
- 相机失败分列：默认多流冲突已按原驱动参数明确关闭次要流；MTU1500收帧为0；attempt2虽目录含9000但修改实际失败，不能当9000测试；attempt3在网口恢复期间启动，白名单接口全部过滤，0帧。等待地址/路由稳定再启动attempt4，收到421 RGB、423对齐深度、422 CameraInfo，640×360@15，含发现/reset/收尾共47.278秒，官方驱动正常退出。无需改DDS白名单。当前保存并查看的首张RGB很暗，不能据收帧成功宣称场景曝光/YOLO已可用。
- Jetson时间仍1970，未擅自校时；本轮SSH走192.168.3.12局域网，不把Tailscale故障称SSH故障。深度单位、base-camera标定及真实ServoJ全周期衔接仍待核对，真空泵关闭，本轮未做真机周期。[迁移结果与未完成项](../../experiments/s20_online_mixed8_20261024/jetson_migration_20261025_r1/migration_result.json)、同目录网络前后备份及相机原始日志保留。硬件接口联通、候选服务可运行与完整抓放可交付分开。

### 2026-10-25 实机清晰画面、YOLO与完整链路复核

- 用户质疑相机是否真正可用，重新运行真实D555采集`camera_observation_attempt5_last`。修复采集脚本只保存启动首帧的问题：保留首帧，另外保存最后RGB/深度及曝光/非零深度统计，不用增强或仿真图替代实际图。本轮413 RGB、423深度、422 CameraInfo，含启动及收尾47.272秒；最后RGB均值131.91/255，已查看清晰黑色箱体，蓝箱在画面上沿部分可见。深度非零比例93.835%，这不是度量准确率。
- 使用已迁移`yolo11s-seg.pt`在Jetson CPU执行实际末帧前向，黑箱被识别为suitcase，置信度0.6202，单张推理2.449秒。已回传并查看[真实识别图](../../experiments/s20_online_mixed8_20261024/jetson_migration_20261025_r1/camera_observation_attempt5_last/yolo_last.png)、原RGB/深度/计数/YOLO结果。不是仿真识别，不宣称已得到base坐标抓点。
- 新`s20-live`仍指向旧20261025接口目录，现仅在新实验中改为新隔离根目录，实际snapshot调用成功。厂家CPS源码`dic_FSM`明确定义24=Disable、33=StandBy；新鲜回读FSM24、Enable state=0、错误状态0、DI0/DO0/DO1=0。接口snapshot增加厂家状态名称，不新增运动阈值；已询问现场使能及泵通电状态，未自动使能/reset或模拟吸附。
- **软件责任/未完成：** 新版只有候选服务、SDK方法及离线参考接口，完整实时观测TF→抓点→全路径→定时ServoJ→吸附/释放反馈→占据更新的真机编排尚未接通；本机四件主程序明确是Newton理想吸附仿真，不能改个入口当真机流程。此次没有执行物理全周期，不能拿清晰相机、迁移文件或旧空载回执替代。旧流程、旧TF及原始失败保留，无新增训练、运动或泵IO。

- **现场使能后的新回读与指令变更。** 新鲜SDK回读FSM33/StandBy、Enable=1、无故障，override=1.000；DI0/DO0/DO1均0，不据此推断外部泵电源。用户随后明确为避免噪音先不开泵，真正开始运动前再通知；本聊天未启动运动、未写泵IO、未自动使能。回读在`jetson_migration_20261025_r1/controller_enabled_20261025.json`，Jetson1970时钟问题仍保留标注。
- **复用而非重写核对。** 已读旧`Workflow.pick/lift/place/pump`及early-prefetch、`ServoHardware.execute`和同学`hrif_start_servo/hrif_push_servo_j/vacuum_io`。可复用的是状态机、定时连续ServoJ和事件IO分工；同学源代码的DI0是吸附数字输入，不是力反馈，当前实物接线/有效性尚无吸附验收。远端旧`manual_cell.json`仍是base_T_ULD平移约(0.600,1.500,-0.860)m的旧固定工位，且measured=false；不能盲启旧入口覆盖当前精标工位。新Newton运行器的吸附/释放/状态结构也不能直接冒充旧执行器schema。完整[复用清单与缺口](../../experiments/s20_online_mixed8_20261024/jetson_migration_20261025_r1/existing_workflow_review.json)保留；没有为求启动而改旧源码/TF、跳过碰撞或模拟硬件成功。
- **2026-10-25 新位置实测输入的离线衔接。** 用户将原不可达箱子搬近后，停止优化旧抓点。Jetson r2实际取得30 RGB/30深度及当前关节回读，重新拟合当前箱体；旧目标参考不复用。`deployment_ready_20261025_r2/real_pick_moved_attempt3/result.json`生成观察、接近、贴合、垂直抬升、示教抬升及示教入舱参考成功，总参考时长15.12秒，离线计算墙钟11.345秒；吸附仅为规划假设，没有运动或泵IO。此前attempt2在离地起始处报地面接触；独立碰撞模块修正倾斜观测地面的法向投影，原模块与失败保留，未全局关闭碰撞。
- **本轮放置与异步计时边界。** `deployment_ready_20261025_r2/real_place_moved_attempt1/result.json`基于新箱尺寸、当前精标、空舱占据及示教入舱起点生成放置和空载返回成功；实际规划9.9265秒，含初始化总墙钟17.6995秒。常驻worker计算小于15.12秒拿取参考时长，说明存在异步覆盖机会；这是分段离线测量，不声称本轮已实际并发或已完成真机周期。用户要求把工作重点转到舱内异步放置，不继续纠结旧抓点和拿取地面检查；未关闭真机保护、未执行吸附/释放、未提交真实占据。
### 2026-10-26 Jetson 六件混流仿真：改用普通立方体

- **现场确认原地实吸与自动确认修复。** 用户明确反馈原地测试能实际吸紧并松开，不再将控制链路完全无效作为已证事实。对015423接触成功回读q做当前URDF FK，模型TCP相对目标约0.106mm/0.0051度，距拟合平面约0.553mm；这只说明模型内跟踪一致，TCP原定义仍是近似接触面，不独立证明实物压合，未据此盲加下探量或改TF。自动流程此前DI1仅保持60–84ms便接受且在抬升启动期翻转，是确认过早的具体薄弱点；不能断言已证明唯一根因。
- **短稳定确认而非固定睡眠。** 无活动编排/桥接时，wait_io对吸附与初始持箱交接增加可配置seal-stable-s默认0.3秒，要求新IO消息连续DI0=1/DO0=1/DO1=0；任何不符重置，未收到新消息不靠时间自行通过。释放确认仍保留原处理；持箱中DI失效仍取消，未加丢吸屏蔽期。0.3秒依据已观测60–84ms脉冲做首轮去瞬态设置，非厂商压力标定或真实负载能力证明；取消错误快照竞争已修复。源码已同步，无新运动/IO，12deg/s不变，尚待用户正常流程实机验证。

- **015423丢吸报错快照竞争。** 实际DI0在61592.315685变0，61592.387657恢复1；原代码在等待cancel回执后才读取可变io，导致报错显示DI0=1。现于触发取消前复制并记录lost_io，报错打印触发值，不将其误称无真实0信号。未屏蔽丢吸，也未擅加稳定等待；单次DI=1确认的局限仍保留。
- **用户授权原地开松测试。** 用户明确确认箱子已落地/有支撑，运行vacuum_stationary_20261026_attempt1，仅调用现有ROS阀门服务，无机械臂目标。释放前DI1；DO0=0/DO1=1后约0.161秒DI0=0；再吸DO0=1/DO1=0后约0.714秒DI0=1并保持至3秒采样结束；再次释放约0.140秒DI0=0。结束SDK独立回读DI0/DO0/DO1全0、FSM33、姿态不变，未让破真空输出一直开着。完整日志已本机归档。仅证实电气输出/输入随命令响应，不证明物理提起、密封、承载或DI接线语义已独立标定。
- **采集窗口与减少导入。** camera_receive新增首个同步对之后最多3秒的采集窗，仍允许收足6帧提前结束；冷启动等待单独记录，不能称冷相机整段3秒。received_to_observation.main支持argv参数，observe_current直接调用转换及EnvelopeGraspPlanner，不再各启动独立Python，未改检测/抓点数学。Jetson旧015423录制输入同进程转换+抓点含导入5.579秒、成功选点；未宣称跨轮YOLO常驻已实现。
- **新鲜无运动观测的负结果。** observe_merged3s_20261026_attempt1在当前贴合附近姿态收到6RGB/6深度/5内参，但既有fit_box报parallel top/floor不满足，未生成新抓点或动作；不是假定与观察位输入等价。原记录和异常保留，不通过删除几何条件伪造成功。软件已同步，20%/12deg/s保持，无新机械臂运动。

- **015423感知等待实测。** 用户反馈Acquiring阶段慢；只读当前远端记录，采集接收循环7.02085秒，感知转换脚本4.82717秒，其中geometry_fit_wall_s=0.20992秒；observe_current总墙钟17.41320秒（还不含外部SCP）。不能将差额精确归因为某个未计时模块，包含子程序启动/导入、关节快照、抓点/候选与存盘。首次冷相机仍可能占接收时间；之前1.56秒只为已热驱动接收循环，不能宣称线上整个阶段1.56秒。当前用户任务活动中，未改程序或采集配置、未新增订阅、未发运动/IO；感知模型常驻尚未实现。

- **用户统一20%/12deg/s。** 用户反馈仍快，要求全部20%。发现015223仍在pick/place规划、无远端桥接；向该本机编排PID1007370发SIGINT取消启动，确认相关客户端退出，未让旧40%继续下发抓取。normal/approach/final_place与executor均0.2，统一60deg/s基准即每轴上限12deg/s，place/return时间倍率4.0，既有加减速设置不变。观察位StandBy/DI0=0确认后退出自有PID27103，用户平板重新使能后启动PID30443；真实日志executor_uniform20_v2_20261026_attempt1显示command_fraction=0.200、READY、v2策略。readonly1参数/关节/DI读取通过，未发新轨迹或阀门指令；吸附反馈原因及碰撞停止尚未因减速自动解决，不称已完成四件。

- **同40%仍忽快忽慢的进一步缺陷。** 用户再次反馈偏快。014751参考4.8秒、峰31deg/s，执行端60×0.4=24deg/s上限触发整倍细分，实际参考9.6秒/峰约15.5；014830参考3.64秒、峰23.53，未细分，实际峰反而更高。两轮均由控制柜报20018/RobotCollisionStop，不视为已安全。问题不只是phase百分比：生成端以控制柜各轴80/80/100/180/180/180为基准，执行端却用max_velocity_deg=60，整数细分进一步放大速度档位变化。此前百分比表格不能代表实际执行速度。
- **只读定位与离线修正。** 无活动任务后，将生成端百分比统一到当前执行器应用基准60deg/s，40%对应每轴至多24deg/s，厂家各轴限制独立保留；启动显式设置同一基准，桥接读取并核对真实参数。20deg/s²和40%不变，不再另调速度档位。以两次原起点重新生成且CAD检查通过：6.22秒/3.64秒，再经现有v2完整两层处理仍为6.22秒/3.64秒，312→312/183→183点，无执行端额外整倍拉伸。仅这两条历史输入的离线证据，不是完整抓放或普遍消抖/碰撞证明；未发真机运动/IO、未自动复位。源码与配置同步Jetson，提醒用户先暂停而非继续反复试跑。

- **吸附/释放时序只读排查。** 014338实际回执：接近goal成功后才调用DO1=0、DO0=1；约1.094秒后DI0=1即允许抬升，约60ms后DI0=0并cancel，DO0仍1/DO1仍0，未发现软件中途释放。当前wait_io只需一次符合条件的新消息，没有持续稳定确认；这是可能过早接受短脉冲的具体薄弱点，但不能据此认定物理泄漏或信号错误。上轮异常保留DO0=1，因此新一轮接近前本来就在吸附输出开启状态，不能表述为每轮都首次于贴合后开泵。正常释放代码在place目标成功后执行DO0=0→DO1=1→新DI0=0→DO1=0；本聊天尚未执行到此分支，不称释放时序已实机验证。DI0=0只说明配置输入，不证明落稳；外部气源供电不受这些DO软件状态认证。当前仍有用户活动任务，未热改时序/阈值、未切阀或发运动。

- **014600持箱反馈下重启。** 用户新启动在prepare_observation_return退出，原始回读DI0=1/DO0=1/DO1=0、FSM33，属于空载重启与持箱反馈冲突，不是IK/相机或新运动失败；没有发回观察位运动。保留源反馈处理，不自动关闭吸附。入口在原有同一回读处直接输出具体状态和安全放稳释放后再重启提示，避免被CalledProcessError掩盖；本机/Jetson同步，未增加新阈值或IO操作。

- **用户统一40%请求。** normal/approach/final_place均设0.4，executor_limit_fraction从0.5改0.4，独立放置/撤回倍率改2.0；既有20deg/s²与v2时间策略不变，不据比例相等推断抖动必然消失。无活动编排/桥接、当前观察位StandBy确认后退出自有PID26782，等待用户平板重新使能再启动40%服务；未下发新运动/吸附IO。运行服务实际配置另待回读，不以文件值替代。
- **40%实际服务核对。** 用户确认重新使能后启动PID27103，日志`executor_uniform40_v2_20261026_attempt1`实际command_fraction=0.400、READY、preserve_input_or_slow_v2；同名readonly1真实ROS参数及关节/DI接口读取通过，不发运动或IO。原命令重新生成参考后生效，不改写旧轨迹或恢复中断任务；尚无统一40%真机全周期成功或无抖动结论。

- **“只慢不快v1”严重负结果。** 实机013712回观察位输入240点/4.78秒，作者逐区间ceil限速产生355点并引入速度变化，新增统一加减速拉伸随后误将其扩为8143点/162.84秒。此前仅用抓取参考离线核对，未覆盖两层叠加后的回程，是本聊天适配错误，不是用户要求30%必然如此慢。原始日志保留，不能用修复后的速度抹去负结果。
- **v2与50%请求。** 独立适配同时替换逐段变倍数限速：对整个输入统一细分，消除相邻段细分倍数不一致导致的速度跳变，再做仅减速加减速处理。与013712同一参考离线核对，240→479→479点/9.56秒，原点逐一保留，离散速度15.399deg/s、加速度9.919deg/s²，均符合该次执行端30deg/s与既有20deg/s²口径。整数统一细分有保守取整，不把50%上限叫实际恒定50%运动。
- **请求边界与部署。** 按用户仅调执行器的要求新增独立executor_limit_fraction=0.5；生成端normal0.3、approach/final_place0.15未擅自提高，旧place倍率不改。因此这不是整套动作已改50/25。确认任务结束/StandBy后退出自有PID25779；用户再次在平板使能后启动executor_preserve_timing50_v2_20261026_attempt1。源码已同步，不发运动/IO；后续仍须真实验收。桥接要求v2策略与当前执行端比例匹配，避免旧服务继续执行。

- **执行端速度遗漏与修复。** 用户再次反馈偏快，实际发现原PID15420始终使用controller_limit_fraction=0.8；仅改启动脚本未更新运行服务。作者旧`limit_servo_braking`也曾将480点缩为424点，不严格保留输入时序。新独立`run_servoj_executor.py`复用作者节点/SDK，仅替换进程内重定时函数：对每个原始20ms区间采用同一整数细分倍数，保留全部原节点/停留、无路径跳跃、各区间不缩短；按已有20deg/s²离散起停约束只慢不快。同学源码未修改。
- **生成端与执行端一致。** 参考生成采用既有执行器20deg/s²加减速口径，而非先按100生成再被重改；不是修改控制柜的100deg/s²上限。30/15速度配置保持。旧013050参考直接适配需5倍时长，调整生成后`preserve_timing_20261026_pick1/recipe1`实际观测离线通过：接近19.76秒、抬升入舱14.52秒，适配前后时长相同、全部节点保留。未把此离线检查冒作真机验收。
- **实际替换服务。** 用户确认空载/箱已落地并批准重启后，只向自有旧PID15420发SIGINT，原驱动退出导致FSM24 Disable；用户在平板重新使能后启动新PID25779，输出目录`executor_preserve_timing30_20261026_attempt1`。实际日志command_fraction=0.300、READY及preserve_input_or_slow_v1。`executor_preserve_timing30_20261026_readonly1`真实参数/关节/DI读取成功，未发轨迹或阀门命令。桥接在发目标前核对运行服务速度与时序策略，防止以后只改文件未换服务。尚无新策略真机抓放周期，丢吸原因仍未确定。

- **012540新增日志的兼容性失败。** 新增IO日志回调错误假定接收`msg,info`两参数，Jetson当前Humble只调用`callback(msg)`，在回观察位读取IO时TypeError退出。远端回执motion_requested=false/io_requested=false，未下发回程运动；与吸附丢失、空间规划无关。已改回单参数，保留DI/DO变化及接收单调时刻，不再记录本版本不支持的publisher GID/源时间；此前GID功能不能称已生效。回观察位入口补充回传执行结果并显示具体失败，不再只输出整条SSH异常。
- **修复后真实接口读取。** `io_callback_humble_20261026_readonly1`在同一Humble/真实servo_j服务成功接收关节与真空回读，interfaces_available=true，未发goal或阀门指令。DI0=0/DO0=1/DO1=0，不据此推断物理吸附状态。源码已同步，本次仅证明回调与接口读取恢复，不称真机全周期完成；30/15速度保持。

- **相机复用实际提速。** 四件入口改为任务开始即启动一次本任务官方D555驱动，与回观察位重叠，逐件到位后才创建接收器并采集新同步帧；四件间不停止驱动，结束/异常仅关闭本任务记录PID与启动时刻匹配的驱动进程组，不触碰控制器/泵。既有单次独占采集模式保留。无活动相机/桥接时部署。
- **提速证据与范围。** Jetson `camera_reuse_timing_20261026_attempt1`同一PID23676两次采集：cold8.72497秒、warm1.56475秒；均RGB6/depth7/info6、同步对存在、毫米深度单位、4条静态TF。耗时为接收循环至收尾，不含调用前Python/ROS导入及后续YOLO/建模/传输，不能把旧整段25秒与1.56秒直接比较。已回传完整采集/日志，结束后确认无本次相机节点遗留。未发运动或IO，未宣称整体四件真机验收。观察位等价跳过逻辑继续保留；仍需实际返回时不能承诺省掉路径验证时间。规划器常驻尚未实现，30/15速度不变。

- **再次减速。** 用户要求normal40%降30%、approach/final_place20%降15%。无活动编排/桥接客户端时更新参考配置；独立放置/空载撤回时间倍率从2改为8/3，即相对上一档速度降至75%。新执行器启动上限改0.3，既有服务不重启或热改；新生成参考使用新配置，旧参考不改写。未发运动、未写吸附IO，反馈丢失原因尚未确定。

- **丢吸信号诊断边界。** 011638执行器日志确认抬升目标被桥接主动cancel，不是该段IK/放置求解失败；现有事件只保存确认DI0=1和随后丢失判定，未保存失败帧DO0/DO1或发布者，无法区分物理泄漏、抖动或重复发布。桥接新增仅IO变化留痕、publisher GID/源时间及失败时三路值，终端不刷逐帧数据；未加去抖延迟、未关闭丢吸处理。无活动桥接时同步Jetson，无新运动/IO。后续需现场确认实际吸附状态并依据记录判断，不能凭用户以前认为吸附牢固就抹去此次0信号。

- **011638实测耗时与真实停止。** 回观察位准备16.323秒（参考运动2.04秒），观测/传输25.064秒，pick准备15.427秒，place初始化10.568秒+计算5.111秒与pick并行，不能重复累加。接近实际9.580秒后DO0置1、约1.246秒出现DI0=1；抬升开始约0.199秒后DI0失效，桥接取消轨迹并保留真空输出，未完成放置。不能称完整周期成功，也不为优化等待而屏蔽丢吸反馈。
- **避免观察位重复初始化。** 新鲜关节回读相对既有观察位各轴均在0.05度以内时，视为已到观察位，不导入Newton/FCL、不生成/发送空返回；这是位姿等价判断，不是新增拒绝门限，实际需要返回时仍做原CAD检查。011002历史真实快照误差0.001002度，已离线验证直接生成空操作列表。代码已同步Jetson，未发新运动/IO。相机和抓放规划器仍逐轮初始化，尚未实现常驻复用，不能把本次优化宣称消除全部启动等待。

- **真实失败与日志精简。** 用户`live_four_20261026_010730/011002`均已回观察位并取得当前图像，但抓取参考在VERTICAL_CLEARANCE阶段因`world_ground/B1_collision_box`拒绝，未执行抓取。原“部署完成”不能等同全周期跑通。终端现在只保留阶段、真实完成数和原因；子程序JSON、Warp、警告及SCP详情写`session.log`，原始结果/失败不删除。
- **两处几何模型不一致修正。** 对011002实际观测计算，倾斜上表面OBB对观测地面存在14.653mm初始穿透。仅为该已观测源箱与地面的初始接触使用测得的代理重合深度，离地规划不允许加深；其他碰撞检查不变。离线attempt1抓取参考通过，但放置16次均在箱/舱终点CAD拒绝。进一步发现抓取使用上表面OBB吸附变换，而放置worker的碰撞箱使用另一套地面投影变换；仅真机部署选项`observed_top_obb`统一目标与持箱代理，默认仿真分支不改。
- **离线实际输入复算结果。** `ground_contact_fix_20261026_attempt2`复用011002真实观测，pick与place均成功并完成组合，放置compute4.877秒、含初始化15.283秒；参考接近8.60秒、抬升/入舱8.94秒、放置10.44秒、空载返回17.16秒，理想事件不计为实测吸附等待。当前40/20速度配置保留。所有改动已同步Jetson；未自动下发运动或IO，不把本次离线成功称真机验收。若后台已完成且已知放置失败，在抓取前直接报具体原因，避免明知失败仍拿起。

- **用户要求降低真机速度。** 无活动在线编排/桥接客户端时，将部署参考的normal从0.8降至0.4，approach/final_place从0.4降至0.2。放置worker使用独立时间轴，组合器将放置及空载撤回参考统一延长2倍，避免只改抓取配置而漏掉放置。新执行器启动上限同步0.4；已运行执行器不热改、不重启，仍使用此前上限，但新生成参考已减速。本机与Jetson配置同步，未发运动/IO，未声称减速后真机验证通过；已有磁盘旧参考不自动改写。

- **观察位回程接口修复。** 用户实际运行`live_four_20261026_010455`在桥接读取`period_s`时退出，未发回程轨迹；原因是新生成器直接保存`prepare_reference`原格式(`servo_time_s`)，未转换为执行器格式(`period_s`)。生成器现显式输出与既有抓放参考相同的`period_s/positions_deg`，用该次真实起点重新生成1.3秒参考并核对执行器同一时间索引读取，复制Jetson；保留原失败，不声称修复后已真机通过。

- **修复真机采集顺序。** 原`online_session.py`先在当前姿态采集，再在抓取参考中回观察位，现场中断后视角可能错误。现在每轮执行先新鲜关节回读、生成空载观察位返回参考、通过现有ServoJ执行器等待到位，再采集当前RGB-D。已有舱内占据用于返回路径CAD检查；未观测地面物体不假称已纳入。吸附DI有效时禁止把持箱状态当空载重启。新增单一参考准备脚本`prepare_observation_return.py`，实际SDK动作仍复用原执行器；未修改同学源码。已用此前真实关节快照离线生成0.6秒返回参考，未执行真机返回；桥接改动已复制Jetson。新入口的完整四件真机执行尚待用户启动，不声称已通过。

- **真机首次执行与启动开销。** `live_four_20261026_execute1`第一件接近轨迹成功、实际DO0打开后DI0在约1.12秒建立，抬升/入舱段收到厂家`20018 / RobotCollisionStop`，流程中止，未放置、未提交占据。用户说明现场原因，但具体碰撞/触发机制未独立定位，不归为规划已通过。未自动复位或关闭碰撞保护。
- **启动优化。** `live_four_20261026_prepare1`总准备59.588秒，其中观测39.397秒，抓放准备窗口20.191秒；放置计算5.775秒、初始化12.181秒，后二者包含于准备窗口，不能重复求和。确认无活动采集/执行客户端后，独立部署相机`initial_reset=False`、观测默认30帧改6帧，保留同步RGB-D及CameraInfo。该改动不调整机械臂速度、吸附反馈或控制器故障处理；相机驱动仍逐次启动，规划器尚未常驻复用，不宣称已消除初始化时间。
- **缩短采集的实际结果。** `observe_noreset6_20261026_attempt1`成功取得6RGB/7深度/6内参，采集驱动生命周期13.028秒，随后感知4.499秒；当前姿态已由现场改变，YOLO未检测到箱子，本次未生成路径、未发运动或IO。该次只证明缩短采集可收到同步数据，不是同姿态端到端提速对照或抓放成功。

- 按用户要求停止复古箱外观处理，使用普通彩色长方体，观测与碰撞使用一致的代理尺寸。测试为4件20寸、2件24寸，每轴独立正负10%变化，逐件揭示，不提供未来来箱。
- Jetson隔离Newton环境实际运行：seed73完成4/6，墙钟256.866秒，第5件没有通过当前支撑/占据/可达粗筛的稀疏候选；seed74完成3/6，墙钟201.353秒，第4件同样没有有效候选。未完成六件，不声称硬件可交付。
- 当前为贪心接触候选、GPU Newton IK及采样FCL检查；执行为关节参考FK与理想吸附/冻结释放，不是实际ServoJ跟踪或完整动力学释放验收。未下发真机运动或泵IO。
- seed73原始记录：`experiments/s20_online_mixed8_20261024/jetson_sim6_20261026/sim6_noise10_20261026_attempt3/`。对应本机保存状态三视图渲染：`experiments/s20_online_mixed8_20261024/jetson_sim6_20261026/media/jetson_seed73/online_newton_three_views.mp4`；画面显示四件已放、第五件仍持有，不将其计为第五件完成。

