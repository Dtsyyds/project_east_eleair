# 整车模块接入、验证与调试操作手册

本手册按当前源码实际调用链整理。入口/参数全量机器清单是 manifests/entrypoints.json；topic/message/QoS是 manifests/topic_contract.yaml；逐项历史与本次证据见 reports/refactor_v1/final_report.json.closeout，新D405实验另见 ../../d405_seam_tracking/reports/framework_validation.json。测试组有重叠，不累加成“总通过数”。源码存在、离线运行、设备活性、真实准确性和生产就绪分别判断。

## 1. 工程分工与权威源

|工程/包|职责与接入点|权威配置/安装|
|---|---|---|
|tank_inspection_system|环境、独立构建、原入口包装、离线上位机|scripts/env.sh；原配置路径引用；core_env/install_cpp/install_physic/install_vision/install_wireless_preview|
|tank_vision_inspection/tank_vision|工业图像stable/guided-anchor、异步证据/快照/ROI|原configs，不更改算法参数；已安装tank-vision-inspection wheel|
|tank_vision_ros/interfaces|工业ROS节点与像素几何/状态消息|原launch参数；install_vision|
|inspection_rerun|选中源真实SHA、严格视频时间映射、RRD原子提交|原rerun.yaml；FIELD_RRD_PYTHON独立环境|
|inspection_tools|公共video_index、timeline工具|install_physic；不跨仓库复制|
|inspection_bringup与physic scripts|唯一相机owner、驱动/MCAP/视频、SSD/PID、停止封存、回放/交付|原launch/YAML、run frozen_configs；原underlay和包装脚本|
|eddy_driver/inspection_sync/interfaces|原始20×8mV与均值分开、时间关联FusionIndex|原配置，sync显式启用；install_cpp；空间坐标未验收|
|agv_protocol/agv_bridge|本地/远程WebSocket、被动设备自检、latest JPEG、Odometry、可选控制|唯一 physic_bev/src/system_websocket/agv_bridge/config/tank_inspection.yaml；install_wireless_preview|
|D405实验|Infra1或Color＋Depth、局部罐壁几何、融合/时序、训练/评测、独立ROS|d405_seam_tracking/configs/experimental.yaml；自己的.runtime/.venv/ros2_ws/install|

父目录、总控、physic_bev根不是有HEAD的Git仓库；physic_bev/src为未产生HEAD的仓库；vision有HEAD但含用户既有修改。不得用Git reset/clean/checkout恢复或删除未知工作。未找到agv_bringup，不能以其他机器人代码里的旧名称认为已安装。源码中的ROS1/vikit/研究工具与当前运行包须分开核查；不复制同名包解决依赖。

实际数据流：原collect入口→launch→唯一工业/D405/Livox/Eddy owner→ROS topic→分组MCAP/工业H264录制MKV＋frames.jsonl/frame_map→监视/SSD/PID→原stop→finalize complete/hash/各项验收→多bag/视频时间轴回放→选中源严格RRD→角色handoff。无线桥只订阅已运行topic，不启动设备；新D405实验也只订阅，不接底盘。

## 2. 日常环境、构建与定位实际执行版本

```bash
cd /home/sunrise/east_eleair
S="$PWD/tank_inspection_system"
source "$S/scripts/env.sh"
ros2 pkg prefix agv_bridge
ros2 pkg prefix realsense2_camera
python3 -c 'import tank_vision; print(tank_vision.__file__)'
bash "$S/scripts/preflight.sh"
python3 "$S/scripts/check_installation.py"
```

preflight当前媒体不可见会非零，不能因此伪造SSD目录。source/install SHA检查检查它的实际记录集合，不等于新模块自动有测试。source是当前修改但install仍旧时，先只构建受影响包及依赖闭包；Python同版本wheel要force-reinstall/no-deps/no-index，避免pip保留旧内容。

原总控build.sh只在需要时执行；无线修改用 `bash "$S/scripts/build.sh" --wireless-only`，保留旧install_wireless。新D405独立构建命令见 ../../d405_seam_tracking/README.md，不进入全车默认launch。source colcon脚本需 `set +u; source 绝对install/setup.bash; set -u`，因为总控env启用nounset而生成setup会访问未定义变量。

## 3. 加入一个新模块的固定流程

1. 明确模块是否拥有设备或只订阅；有owner时只由一个launch创建。算法默认不再打开USB/SDK；记录实际Git根/HEAD/dirty和要编辑文件SHA，保留原入口。
2. 确认实际输入topic/type/QoS/header/frame/单位、输出语义、配置权威文件与消费者。均值不可伪装波形、像素不可伪装米、外参未知不可填单位变换。参数缺失应拒绝或标unavailable。
3. 优先在现有合适包增加小模块；独立实验用新命名空间和独立安装。跨仓库公共模块必须显式安装依赖；不要依赖临时PYTHONPATH、十几个空包或源码副本。
4. 先写具有明确真值的小fixture和异常输入。原CLI/help/launch参数/import/topic/格式保留；异常行为改变登记。普通路径不增加全图复制、逐帧SHA或第二tracker。
5. 有界输入/输出/writer，明确online latest与offline完整策略，输入/处理/保存/drop守恒；worker异常对调用者可见，关闭全过程用deadline，超时登记仍存活线程/不完整输出。
6. 按影响构建独立依赖闭包，使用安装产物验证，核对colcon注册数量。运行Python功能、隔离ROS发送/独立接收、需要的MCAP/RRD/handoff；compileall/shell/diff只是补充，不能代替功能。
7. 启用前更新topic_contract、entrypoints/migration对应记录、README/本手册；一份配置一个权威源，每run冻结快照。新派生写独立analysis，旧run不改。
8. 离线→只读实机→60秒新run→30分钟稳定性逐级验收；设备活性、采集完整、算法准确、校准、transport/viewer分别登记verified/failed/environment_blocked/not_executed/unverified。未启用不写passed。

驱动与无线接入要先确认协议，不直接复制其他机器人默认接口。无线新增系统在RobotFactory注册明确robot_id，handler被动ROS订阅，command必须验证类型/参数/队列/退出/watchdog。扩展原tank系统优先配置已有六设备实际topic；新类型才写显式adapter。默认控制保持关闭，软件published不等于硬件执行ACK。

## 4. 不启动硬件的调试与验收

```bash
# 核查graph只读；无数据不写通过
ros2 node list
ros2 topic list -t
ros2 topic info /d405/camera/color/image_raw --verbose
ros2 topic echo /d405/camera/depth/camera_info --once
ros2 param get /d405/camera enable_color
ros2 launch agv_bridge tank_inspection.launch.py --show-args
# 软件fixture；该入口不启动正式设备。各组结果分别保留。
bash "$S/scripts/verify_offline.sh"
```

新增模块测试单独选受影响测试，不反复执行无关全量。ROS fixture使用独立ROS_DOMAIN_ID和LOCALHOST_ONLY=1，发布与订阅须匹配reliable/best_effort、volatile/transient_local及depth。reader正确不等于DDS送达；原速率数量不一致就报告，不降低rate后宣布原速率通过。时间戳分别统计总数、有效、零/负/重复/回退；动态全无效不能“单调通过”，tf_static例外有明文契约。

MCAP检查必须实际 create→write→close→read→CDR→topic/type/count/stamp/value，不只查包能否import。环境先source正确Humble/aarch64 overlay；既有离线sysroot补插件，不升级rosbag2 core。已修复初始插件缺包，旧失败保留，最新roundtrip/RRD/multibag fixture通过；历史SSD仍另有阻断。

## 5. 相机、自检和没有画面的定位顺序

1. 看ROS数据：唯一owner、topic类型、当前进程的ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY、USB速度/设备序列。D405 SDK serial=352122271190；USB descriptor 254723075213不能作为驱动serial_no。相机已运行不再launch第二owner。
2. 核对启用Color与848×480×30实际profile；上位机RGB订阅color/image_raw，Depth仍独立保存。ROS16UC1毫米乘0.001为米，预览JPEG不是真实深度。
3. 看桥回包中depth_camera received/accepted/invalid/repeated/regressed及reason。连接101只说明WebSocket握手成功；no_valid_message、stale、invalid_header、regressed_header分别定位。保留历史错误计数。
4. 看端口与会话：9001被已有桥占用时Underlying Transport Error；先确认所有者，不能全局pkill。HTTP8080只是页面服务，视频走WebSocket9001。
5. 看浏览器收到/解码/source_encoding/frame_id/stamp；页面离线HTML＋console.js需一起复制。上位机地址应使用板端实际可达IP，不能用上位机自己的localhost。source与桥进程环境改变后仅重启本次拥有的实例。

```bash
ss -ltnp | rg ':9001|:8080'
ip -br address
ros2 param get /d405/camera depth_module.global_time_enabled
ros2 param get /d405/camera enable_color
ros2 topic info /d405/camera/color/image_raw --verbose
```

本次曾确认hardware_clock 32位微秒回绕导致header退到桥的历史最大时间之前，桥正确拒绝。用户RGB预览用已有SDK global_time=true恢复；后续约78分钟区间累计计数未新增回退，metadata差值支持跨一次回绕，但没有逐帧边界捕获。生产默认配置未改，正式长采集时钟验收仍需执行。详细命令、PID/starttime登记和安全停止在 collection_and_replay.md 的D405章节。不要在桥里填host时间或放宽单调性以恢复画面。

## 6. D405融合巡迹使用和后续开发

完整安装/抽取/标注/训练/三组评价/ROS/overlay/Rerun命令见 [独立框架README](../../d405_seam_tracking/README.md)。使用Infra1默认；RGB与Depth按真实factory外参配准。新命名空间 `/vision/d405_seam`，result=JSON String、path_3d=米制相机Path、diagnostics=状态/drop/耗时、overlay=显式请求且有订阅者才绘制，save_snapshot=Trigger。不要把旧SeamGeometry中的Point32像素解释为米，也不要改旧消息。

最新实验用局部平面/二次罐壁背景，不是原始深度阈值；视觉/深度候选独立，冲突降级，深度失效保留可靠二维；预测不算测量，几何凸起未确认身份。工作距离和现场有效深度需先检查，再判断LOST是否合理，不调整ROI“消除失败”。当前现场所有帧LOST的真实原因是有效深度低/视觉不连续，不是已验证焊缝效果。

下一模块建议按已有入口增量实现：先补真实dataset+人工标注→evaluate三组→按会话训练→浮点/ONNX对比→匹配S100P nash-m工具链小编译→量化/几何差异→有效焊缝实时负载。已有OpenCL核是实GPU，native BPU adapter编译完成但无焊缝HBM/编译器，状态implemented_unverified；10Hz/P95<100ms是有效场景待实测目标。暂不引入DFormer、工业跨相机/MID360空间融合/整罐BEV/运动闭环。

3Hz显示问题的后续修复与实际操作见 [显示帧率修复与复测](collection_and_replay.md#显示帧率修复与复测)。当前preview独立安装保留旧入口；RGB实测14.94FPS，融合overlay实测平均11.43FPS，完整回调P95 91.8ms。两秒窗口最低9.77FPS、全部420帧LOST，不能将平均速度计为有效焊缝/严格窗口/物理无线验收。新增diagnostics区分pipeline、完整callback与overlay耗时，上位机区分收到与实际解码，排查时先确认这些计数各自的含义。

## 7. 采集、停止、回放、RRD、交付

可复制全流程在 [collection_and_replay.md](collection_and_replay.md)，保持原参数与默认行为。只有确认SSD/preflight/唯一owner及配置后才正式collect，使用新run_id。stop按PID和启动时间只停登记进程，封存与SHA分别验。快照/ROI/provenance需显式tracker启用，未启用是disabled/not_executed。

历史 `/media/mcap_data/runs` 只读；新派生 `/analysis/run/analysis_id`；交付 `/deliveries/run/profile`。验证器不改旧报告以获得通过。工业视频必须按decoder/frame_map/sidecar证据对应，编码packet数不等帧数，不以95%比例或第n帧平移配时间。多bag按媒体时间合并；RRD strict重新核对选中源实际SHA，提交前再核对，失败保留partial；dry-run实际情况看hash_verified，不把viewer可读当ROS E2E。handoff按当前run与RRD manifest解析名称，不写死批次。

当前媒体路径不存在，findmnt指向系统eMMC，lsblk没发现原SSD；不要建立假runs、格式化、修改fstab或强remount。用户接入真实卷或修复宿主bind后，只读核查原FIELD-VEHICLE-20260928_115534；不存在应登记并选择实际封存run。准确识别步骤见 collection_and_replay.md，不猜/dev/sdX。

## 8. 证据、清理和回退

派生manifest登记source_run_id/源与配置/模型/代码SHA/时间/坐标/标定/输出/complete；未知保持unknown。性能用相同输入/开关/环境比较P50/P95、wall/FPS、CPU/RSS/drop，包含传输/同步成本；短时空场景不能代替有效焊缝负载或30分钟稳定性。

仅清理本次可重建且无引用的临时fixture/cache/失败残留，先查import/shell/launch/install规则/测试/文档/动态加载/软链/人工用途；兼容入口、唯一研究实现、标定/reference、历史数据及证据、当前/回退install保留。RealSense third_party/librealsense/build/release是必要运行依赖，不能按名称build删除。

回退在新终端只source原环境和旧install，或使用scripts/rollback.sh已有入口；无线可指定FIELD_WIRELESS_INSTALL为原install_wireless。D405新实验停独立节点/9002桥即可；9001原RGB和传感器owner保留。无需Git reset，也不要求恢复用户已有修改。
