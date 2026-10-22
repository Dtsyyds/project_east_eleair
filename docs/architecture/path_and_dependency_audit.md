# M0 路径与非 Git 依赖审计

日期：2026-10-22；工作目录 `/media/mcap_data/project_east_eleair_refactor`；基准提交 `b34a3c29c62500915c302b340cb4d51880b00980`。原工程 `/media/mcap_data/east_eleair_plus` 全程只读。下列“存在”“相同”指文件核对；“建议”“应”是 M1 及后续措施，尚未实施。

## 1. 方法与审计边界

对受 Git 管理源码使用 `rg` 搜索固定路径、目录层级推导、Python 搜索路径、构建/安装前缀、配置与资源加载；逐项核对各包 CMakeLists.txt、setup.py、launch Python。按 package.xml 解析真实包名与依赖，不以历史 entrypoint 清单替代当前文件。

文件对比以 SHA-256 为准。非 Git 依赖在原/新目录中按动态库、归档、模型后缀、标定资源扫描；业务源码与配置另外检查 `model_path`、`calibration_file`、SDK 根目录。忽略 build/install/log/output/venv 等运行树的可复用依赖统计，另外确认这些旧产物存在但不迁移。模型补充扫描覆盖旧构建/输出目录，排除 `.git` 和虚拟环境；发现的 `.bin` 均是 CMake 编译器探针，未发现模型权重。未解包离线归档、未检查工程外模型缓存，也未打开设备。

## 2. 路径发现与整改清单

严重度：阻断=现有代码或直接目录搬迁在缺失路径处会失败（静态结论）；风险=需要参数化、重新安装验证或保留兼容；信息=已有可保留的定位方式。没有实际执行构建或设备启动来确认这些失败。

| 编号/严重度 | 证据定位（M0 行号） | 已核实事实 | 迁移处理与验证 |
|---|---|---|---|
| P01 阻断 | `tank_inspection_system/scripts/build.sh:10-19,25,44-66` | 读取 offline 清单后逐个 read_bytes()，新目录 33 个归档全部缺失；所有业务 --base-paths 写死旧层级。 | 先以外置只读归档仓库+显式路径映射恢复输入；按批改 base-paths，输出只能到新 build/install/log。不能靠复制旧 install 绕过。 |
| P02 风险 | `tank_inspection_system/scripts/env.sh:5-13,37-58,62,74,82` | 从脚本上一级/再上一级推根；FIELD_PHYSIC_ROOT、VISION_ROOT、.venv_rerun、SDK、offline/sysroot、RealSense build/release 默认均依赖旧布局。TANK_OFFLINE_PREFIX 无条件赋值，不接受外部同名覆盖。 | 移动 env.sh 必须同时重做根定位与角色路径；统一输出策略，显式区分第三方只读资源与可写 sysroot。已有变量要保持兼容或同步改消费者。 |
| P03 风险 | `tank_inspection_system/scripts/install_core.py:4-19` | 算法分发固定 tank_vision_inspection，staging 固定 tank_inspection_system/build_core；build.sh 的 TANK_BUILD_ROOT 不控制这里。 | packages 下分发路径与 wheel 暂存路径同改；业务 wheel 从当前源码重建；保证 install_core 与 tank_vision_core 安装的 tank_vision 来源一致。 |
| P04 阻断 | `system_websocket/run.sh:5-18` | 无条件 source physic_bev/install_lifecycle/setup.bash（新目录不存在），桥源码固定 physic_bev/src/system_websocket。构建 stamp 只检查 agv_bridge/src 和 include，未覆盖 agv_protocol、CMake、配置等输入。 | 改为明确的新 overlay；通信包路径同改；使重建判定覆盖实际输入或交给显式构建入口，不能用旧 stamp 证明构建完成。 |
| P05 阻断 | `tank_vision_inspection/ros2_ws/src/tank_vision_core/CMakeLists.txt:11-14` | PACKAGE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../../../tank_vision 当前确实指向 tank_vision_inspection/tank_vision。单独移动适配包后会改指 ros2_ws/src/tank_vision。 | 建议从 repo 根明确指向 packages/tank_vision_inspection/tank_vision，或提供显式 CMake 源码参数并在 configure 时验证；保留 ROS 包名和 Python 导入名。安装后检查 tank_vision.__file__ 和源码哈希，不保留两份算法。 |
| P06 阻断 | `tank_vision_inspection/ros2_ws/src/tank_vision_ros/setup.py:17-22` | 4 个安装资源来自 ../../../configs：stable seam YAML、guided anchor YAML、width reference JSON、geometry reference JSON。目前 4 个均存在；迁 ROS 包或外部 configs 都会破坏相对关系。 | 迁到 config/vision 后同步 data_files 收集路径，安装目标继续 share/tank_vision_ros/config；launch 和节点已通过 ament share 找资源，可保留。还须处理这些 YAML 内的 SDK 固定路径。 |
| P07 阻断 | `physic_bev/src/FAST-LIVO2/CMakeLists.txt:156-157` | 直接链接 ${CMAKE_SOURCE_DIR}/../../install/vikit_common/lib/libvikit_common.so 与 vikit_ros/lib/libvikit_ros.so。统一到 functions/localization 后该相对地址不会落在工作区 install。 | 改用 vikit 导出目标/库变量；需要先核对其真实导出配置，不能只替换成另一个绝对路径；clean configure/build + readelf 检查最终链接。 |
| P08 风险 | `physic_bev/src/FAST-LIVO2/CMakeLists.txt:44`、`src/LIVMapper.cpp:311-315,331-332,545`、`include/common_lib.h:39`；`rpg_vikit/vikit_ros/CMakeLists.txt:10` | ROOT_DIR 在编译时嵌入源码目录。FAST-LIVO2 用它写 Log/PCD、Log/image、结果并执行 scripts/colmap_output.sh。 | 源码位置变化须重编；后续将只读资源与可写输出参数化，不能把 installed 节点能启动等同于源码目录不再被依赖。 |
| P09 风险 | `physic_bev/src/livox_ros_driver2/CMakeLists.txt:94,256`；两个 `launch/*MID360_launch.py:14,18` | SDK find_library 默认 /usr/local/lib，支持显式 CMake 覆盖；launch 有 /home/livox/livox_test.lvx 默认，config 通过 __file__/../config 查找。 | Livox SDK 从 third_party/Livox-SDK2 在新 build 重建并显式传库/头文件；保留 launch/config 同级安装布局；离线文件默认参数化，不能把未使用默认值记为已回放。 |
| P10 风险 | `tank_vision_inspection/configs/s100p_stable_seam_4096x3072.yaml:17`、`s100p_guided_seam_4096x3072.yaml:17`、`s100p_guided_anchor_seam_4096x3072.yaml:18`；`camera_collect.py:6` | SDK 固定 /home/sunrise/east_eleair/MindVisionlinuxSDK；VendorCameraSource._sdk_root 使用 config.sdk_root 优先于 MINDVISION_SDK_ROOT，因此只设置环境变量不能修掉这些配置。 | 改为显式当前 SDK 参数/空配置+环境变量，并用匹配的绑定和原生库；相机独占和图像有效性另行硬件验证。 |
| P11 风险 | `physic_bev/src/mindvision_camera_driver/mindvision_camera_driver/node.py:43-89` | SDK 发现先 configured、环境、EAST_ELEAIR_ROOT/固定 SDK 子目录、安装前缀祖先，最后旧/opt 和 /usr/local 根。loader 要求 binding 与 lib/<arch>/libMVSDK.so 同时存在。 | SDK 整理到 third_party 后须更新发现位置或显式传 MINDVISION_SDK_ROOT；不得仅依靠全局 /usr/lib/libMVSDK.so。ld 路径与 driver diagnostics 必须记录选中绝对库路径和版本。 |
| P12 风险 | `physic_bev/scripts/start_vehicle_collection.sh:6,106-145,238-251,531-569`；`start_mapping_eddy_collection.sh:11`；`run_d405_lifecycle_validation.sh:28-37` | 旧兄弟 VISION_ROOT、install_s100p_humble、install/install_lifecycle、src/inspection_tools 的 PYTHONPATH、src/inspection_bringup 配置及冻结源码路径。部分受 FIELD_* 覆盖，部分直接拼接，不可仅改环境。 | 工具搬到 tools/data 前先定义新源码、资源、安装根；本批先改 Eddy freeze-file；后续统一通过 ament share 获取已安装配置，采集冻结代码仍应显式追到权威源码。 |
| P13 风险 | `physic_bev/scripts/field_visual_preflight.py:17,50,129`；`build_role_handoff.py:27,369,755-773`；`generate_vehicle_collection_summary.py:111` | 预检从脚本位置导入 inspection_bringup/launch 和相机源码；handoff 依赖 PROJECT_ROOT/src 及旧未提交仓库说明；summary 记录旧 install_s100p_humble。 | 工具迁移与包迁移一起改路径解析/溯源文字；脚本同目录 sys.path 用法可以随整组保留，但跨源码树注入要去掉或明确化。 |
| P14 风险 | 各 package.xml 对照 CMake/setup/import | vikit_common 未声明 build_type，被发现为 catkin；vikit_py/setup.py 仍用 catkin_pkg，而 CMake 的 ament_python_install_package 被注释；realsense2_ros_mqtt_bridge/package.xml 只声明测试依赖；fast_livo/vikit 的 Sophus、OpenCV 等 CMake 依赖不完整反映在 manifest。 | 保留包名；迁移时补齐构建类型与必要依赖并验证导出/导入。上表只列明确例子，不宣称已穷尽全部 Python/系统依赖。 |
| P15 风险 | `system_websocket/src/run_system.py:17,45-48,64,77,97`；`operations.py:23,78-79,276`；`config/system.yaml:25` | 应用读 system_websocket 配置、根 websocket_console、旧 tank_inspection_system/.runtime、旧 agv_bridge config；按文件路径动态载入 emat_serial_once.py、measure_thickness.py，saved_waveform 也为旧相对路径。 | apps 应用、config、packages 下 EMAT 库及 ROS 包迁移需同步；桥配置用 share，EMAT 先确定 Python 包边界再改导入；运行输出落独立 runtime。不能只迁 apps/system_websocket。 |
| P16 风险 | `physic_bev/src/inspection_bringup/launch/field_validation_common.py:92-93` | 用 __file__.parents 层级猜 vision 与 physic Git 根；从已安装 share/launch 运行时，这些路径也不是源码仓库身份。 | 改为显式仓库根/构建注入的身份元数据；当前顶层已有真实 HEAD，不能继续宣称 src 仓库无 HEAD。 |
| P17 风险 | `tank_inspection_system/scripts/check_static.py`、`check_installation.py`、`verify_offline.sh`、`prepare_rollback.py`、`rollback.sh`；`tank_inspection_system/tests/ros_native_eddy_e2e.py:29` | 检查脚本列举旧源码路径且向旧 reports 写新结果；rollback 需要旧 backups、core_env_baseline、旧 install；eddy e2e 用 install_cpp 下固定二进制，不能指向新 overlay。 | 迁移前将活跃检查与历史证据分开；e2e 用新包前缀，报告输出到新位置；rollback 以新提交+新环境重建策略替代对旧 venv/install 的依赖。 |
| P18 风险 | `/media/mcap_data`、`/dev/eddy`、`/opt/ros/humble`、`/opt/tros/humble`、`/usr/local`、`/proc`、`/sys` 等 | 包含数据盘、设备、系统 underlay 和探测路径，不都是工程源码路径。例：validate_vehicle_collection.py:551 还有 /media/mcap_data/runs 的保护条件。 | 保留有意义的系统/设备默认值，提供部署参数并测试覆盖。不要全局替换 /media/mcap_data；保护条件随 FIELD_DATA_ROOT 可配置性一起审查。 |
| P19 信息 | 各包内 config、resource、launch、msg、include 与安装规则 | 大多数安装使用包内相对路径和 ament share/prefix；RealSense 多机与 description launch 的 sys.path 指向自身目录。 | 整体移动包时一起保留；确认 setup.py glob 包含 helper Python、config/URDF/mesh/plugin XML；静态无跨包命中不代表安装验证通过。 |

### tank_vision_core 的安装策略必须先确定

当前存在两条安装路线：`tank_vision_core` 用 ament 从包外目录安装 `tank_vision`；`install_core.py` 则从同一分发源码构建 wheel 安装进 venv，且当前 `build.sh` 的 vision `--packages-select` 未包含 tank_vision_core。迁移后必须保留一份权威算法源码，明确哪个环境使用 wheel、哪个使用 ROS 安装，避免导入搜索顺序掩盖旧版本。后续验收记录 `importlib.util.find_spec('tank_vision').origin`、实际文件哈希和 sys.path，不能只看 import 成功。

Python 配置加载不会自动替换所有 YAML 内路径；`model_path`/`calibration_file`、参考 JSON 以及相机 SDK 路径需在加载入口和配置权威路径之间明确解析规则。当前 ROS launch 使用 ament share 的部分可保留，源码工具使用 VISION_ROOT/configs 的部分需随 config/vision 同步修改。

## 3. 可共享只读依赖与重建产物

| 类别 | 实际原工程/系统状态 | 新目录状态 | 迁移策略 |
|---|---|---|---|
| MindVision 厂商 SDK | 旧版 5 个架构库，新版 7 个架构库；包含 mvsdk.py、头文件、工具；新版发布 tar.gz 实物存在。 | 仅 Git 中头文件/绑定/demo 等；全部厂商 .so 与 tools 被忽略，缺失。 | SDK 原始内容可共享只读，选择 arm64 时仍需完整对应根目录；不得向原 SDK 写运行缓存/相机参数。表中给出原库 SHA。M0 未建立链接或复制。 |
| 系统厂商库 | /usr/lib/libMVSDK.so 与旧版 arm64 哈希相同；/usr/local/lib 下 Livox shared/static、RealSense 2.58.4 存在。 | 机器共享系统安装，不是新 checkout 自带。 | 可作为有版本记录的只读系统依赖候选；存在性不代表 2.1.0.50、Livox 符号绑定修复、ABI 或功能已验证。 |
| 32 deb + websockets wheel | 原 offline 清单 33/33 哈希匹配，总 19,976,610 bytes。 | 全部缺失。 | 归档可置外部只读缓存/third_party 离线区；从校验归档向新 sysroot 解包。不能复制旧 sysroot 充当新构建验证。build.sh 和 env.sh 尚无完整外置缓存支持。 |
| D405 offline 39 wheels | 215,258,030 bytes；包括 torch、torchvision、onnxruntime 等依赖及 tank_vision_inspection 旧业务 wheel。 | 全部缺失。 | 第三方 wheels 可共享只读并记录 SHA/架构；未验证解析闭包。业务 wheel 从迁移后源码重建，不把旧 wheel 当源码权威。 |
| 其他归档 | Linux.selection.tar 与新版 MindVision 发布 tar.gz 在原目录。 | 均缺失。 | 只读候选；本轮仅哈希/大小检查，未解包或验证内容，Linux.selection.tar 用途未确认。 |
| 模型 | 源码/配置检查未发现实际 .onnx/.rknn/.pt/.pth/.engine/.tflite/.ckpt 模型；广域 .bin 候选为编译器探针。默认及平台 model_path 均有 null。 | 未发现模型权重。 | 模型后端缺少可验收权重；如后续启用需补充文件、版本、hash、输入输出契约和对应平台 runtime。归档内部与工程外缓存未检查。 |
| 相机参数/标定 | mvdat 见下表；rough_extrinsics 与 FAST-LIVO2 config、视觉参考 JSON 已在 Git。 | Git 资源相同；vision/Camera 下两个非 Git 副本缺失，但 physic_bev 中有逐字相同副本。 | 允许只读共享或将权威配置纳入 config；mvdat 按 SDK 相机参数文件保存，不能当作已验证内参。rough 外参明确非生产有效；不得修改已有封存 run 的 frozen_configs。 |
| 旧 build/install/log/venv/.runtime | 原工程多套 .runtime、install_lifecycle、install_s100p_humble、core_env、core_env_baseline、.venv_rerun、D405 .venv 等存在。 | 未带入对应运行树。 | 全部在新目录重建；不复制、不 source、不拿旧库或脚本 shebang 作为新结果。Livox/Sophus/librealsense 的旧 build 也属于重建产物。 |
| 历史报告、采集与缓存 | reports/refactor_v1 包含旧快照/结果/哈希/旧路径；原 output 和数据文件不是构建依赖。 | 部分历史报告已在 Git，采集大文件不随 Git。 | 历史结果只读保留；新验收必须另写，不能覆写历史 hashes 或假设历史 passed 适用于当前。 |

### 3.1 MindVision 核心动态库逐项清单

以下每行均已核对：原工程存在、非 Git；新目录同路径缺失。SHA-256 为本次对原文件重新计算。库架构名称为厂商目录名，未执行 `dlopen` 或 ABI 测试。除核心库外，厂商 tools 目录包含 GUI/Qt 支撑库，下文另列统计。

| 原工程相对路径 | bytes | SHA-256 |
|---|---|---|
| `MindVisionSDK_v2.1.0.50/sdk/lib/arm/libMVSDK.so` | 15472254 | `73ed54589b60684e93f6563d70103954a31d9e798022541964bc78bd0138d81b` |
| `MindVisionSDK_v2.1.0.50/sdk/lib/arm64/libMVSDK.so` | 24894352 | `f53bde08472b4fe307e7b55a5a7206b5064780182e4ad72ad2447a4b42a2a281` |
| `MindVisionSDK_v2.1.0.50/sdk/lib/arm_musl/libMVSDK.so` | 17346692 | `57b8ee0ca2d9ecf0b044ed11275ba949fae0c91bfb045b0b0a981ae963094f3e` |
| `MindVisionSDK_v2.1.0.50/sdk/lib/arm_softfp/libMVSDK.so` | 16804771 | `61bb4ae4761d8a4bf6669388ed8bc69cd7101bcaffd8c8bc5c56e70441ed0677` |
| `MindVisionSDK_v2.1.0.50/sdk/lib/ite/libMVSDK.so` | 24629408 | `2613909a0440fadbdca3687bfdc926c35b4c7dac1a58dc413d51645dd59a7050` |
| `MindVisionSDK_v2.1.0.50/sdk/lib/x64/libMVSDK.so` | 45307072 | `bbf09608afc1c14da7b34fb28e44e95fd9ff93885fb927e3477c9ecb5265fd38` |
| `MindVisionSDK_v2.1.0.50/sdk/lib/x86/libMVSDK.so` | 21632556 | `77a02dd17e1adcc6881bf51d7d5766ddcf16a8a68864820be60b894f6b989872` |
| `MindVisionlinuxSDK/lib/arm/libMVSDK.so` | 12047402 | `3ff22735d980f63af3339e79b2904592006e8fd58a304ce72f4861c938197b23` |
| `MindVisionlinuxSDK/lib/arm64/libMVSDK.so` | 20213224 | `bd5ac933fa3772d0cab855584f105740ef70960d96f3b82d0a3d12411d7e392f` |
| `MindVisionlinuxSDK/lib/arm_softfp/libMVSDK.so` | 13973571 | `6a0e1a24e1f031a80d1a8be825993c426de78aab8f75038c4fe1635d339964dd` |
| `MindVisionlinuxSDK/lib/x64/libMVSDK.so` | 25590960 | `0396a8572f18be560b0c795d65841701a5fba0917614aa5f34436914f00307f7` |
| `MindVisionlinuxSDK/lib/x86/libMVSDK.so` | 18390880 | `85d1b042ec7af3aa53c4c63cd8e0fa6299494f1018aeae33a3e3a620eee38aea` |

系统 `/usr/lib/libMVSDK.so` SHA-256：`bd5ac933fa3772d0cab855584f105740ef70960d96f3b82d0a3d12411d7e392f`，与旧 `MindVisionlinuxSDK/lib/arm64/libMVSDK.so` 相同；新版 arm64 为 `f53bde08472b4fe307e7b55a5a7206b5064780182e4ad72ad2447a4b42a2a281`。不能将这两版混用为同一依赖。

| 其他动态库分组（含 SONAME 链接条目） | 条目数 | 新目录 | 处置 |
|---|---|---|---|
| `MindVisionSDK_v2.1.0.50/sdk/tools/` | 56 | 同路径全部缺失 | tools 原始依赖可共享只读；sysroot 从 deb 重建 |
| `MindVisionlinuxSDK/tools/` | 56 | 同路径全部缺失 | tools 原始依赖可共享只读；sysroot 从 deb 重建 |
| `physic_bev/src/rpg_vikit/vikit_common/lib/` | 1 | 同路径缺失 | 源码目录内遗留编译库，须重新构建；不可归入厂商只读 SDK |
| `tank_inspection_system/offline/sysroot/` | 38 | 同路径全部缺失 | tools 原始依赖可共享只读；sysroot 从 deb 重建 |

此统计不包含旧 build/install/venv 中的库；不将 sysroot 的链接条目当作不同独立依赖包。GUI tools 不是当前 ROS 核心采集的已确认必需输入，本轮未验证其运行。

特别项：原 `physic_bev/src/rpg_vikit/vikit_common/lib/libvikit_common.so` 为 430,480 bytes，SHA-256 `b97bde92e74cb55f2036d0cb95dbd166405cb7ef954588e404ece7d9eaa3319f`；虽然位于源码树中，仍属于本地编译产物，新目录不应复制它。

`vikit_common/CMakeLists.txt:196-204` 的非 ROS 分支（当前 `USE_ROS=FALSE`）将 `EXECUTABLE_OUTPUT_PATH` 和 `LIBRARY_OUTPUT_PATH` 指向源码 `bin/`、`lib/`。后续 clean build 前应改为构建树内输出，再用 install 规则安装，避免再次污染源码目录。

### 3.2 离线清单中 33 个归档

清单：`tank_inspection_system/offline/dependency_manifest.json`，平台标记 `aarch64 Ubuntu22.04 ROS2 Humble`。下列路径相对该 offline 目录；原工程均存在、大小匹配清单且 SHA-256 匹配，新目录全部缺失。以下为文件完整性检查，不是安装/兼容测试。

| 相对路径 | bytes | 清单 SHA-256（实测匹配） |
|---|---|---|
| `debs/libboost-atomic1.74-dev_1.74.0-14ubuntu3_arm64.deb` | 221508 | `d1815f60fc8a3e644f49b37446d1a6234e88bd059fb9501f0f143bea82757836` |
| `debs/libboost-atomic1.74.0_1.74.0-14ubuntu3_arm64.deb` | 224136 | `09f9d0427df7e39803fa3c5b87ca9b9ee2b8e138e82e2cb634b303c0f6992cda` |
| `debs/libboost-chrono1.74-dev_1.74.0-14ubuntu3_arm64.deb` | 238664 | `b1024f6739b0e9ecb1f7cf055b7fb36bfaa78af29a063bee97e67a6012a35910` |
| `debs/libboost-chrono1.74.0_1.74.0-14ubuntu3_arm64.deb` | 231766 | `2f04cb0a929cbe7f03c1468e07e6f6fcd7d52d32fcafa6fe6b5f03748d47e7d5` |
| `debs/libboost-date-time1.74-dev_1.74.0-14ubuntu3_arm64.deb` | 225934 | `91ac225d39ee27af903598335bf15b44eddad2e1151d16fa80ef7cd8b810c755` |
| `debs/libboost-date-time1.74.0_1.74.0-14ubuntu3_arm64.deb` | 220654 | `5ed79b72fc247359bc331d1445e96c95054a50f650e506145d452c2650e02ac7` |
| `debs/libboost-serialization1.74-dev_1.74.0-14ubuntu3_arm64.deb` | 374296 | `d5c5a334648fb4b3170b3804449d27b34b13876a863529189806e99b499ccc61` |
| `debs/libboost-serialization1.74.0_1.74.0-14ubuntu3_arm64.deb` | 318666 | `6a19f6130426a429cc2570a23e7b005c9d344a9bff8710180e892751173b5070` |
| `debs/libboost-system-dev_1.74.0.3ubuntu7_arm64.deb` | 3390 | `0a315bfeab41a554709ae1d555be1ed681618cc3832465f216af55c7b768803c` |
| `debs/libboost-system1.74-dev_1.74.0-14ubuntu3_arm64.deb` | 218152 | `fe866c64ee5c826690bc15c4f1d757ddb3e34a87588b6416aaa4c76ef2537b84` |
| `debs/libboost-system1.74.0_1.74.0-14ubuntu3_arm64.deb` | 220832 | `b6f1761dd35d41759aabba85cc8606f0399cf25c7aeea3ed69cd8db594976ac7` |
| `debs/libboost-thread-dev_1.74.0.3ubuntu7_arm64.deb` | 3264 | `2d356033d952a41e2483c008e23ed8f949663626b2a8c39b7566d5b13570fc13` |
| `debs/libboost-thread1.74-dev_1.74.0-14ubuntu3_arm64.deb` | 274448 | `baaf372aaeae988c0c9a7320ae31f0b8b9f09cd0bba023aaefe671c2ffce8765` |
| `debs/libboost-thread1.74.0_1.74.0-14ubuntu3_arm64.deb` | 260112 | `06543f1e187154b51b687778bae0a87705e71fe326b732a9f122f4808715b12d` |
| `debs/libboost1.74-dev_1.74.0-14ubuntu3_arm64.deb` | 9608758 | `69a520b3da35320c5443957cf96deb4f444898989523c9428f5494a8a5b5bb37` |
| `debs/libjsoncpp-dev_1.9.5-3_arm64.deb` | 22912 | `8ee852b0ac0a7c7409282dc1fa5de10cb21a83932b0a6c9940e2e8147463e422` |
| `debs/libjsoncpp25_1.9.5-3_arm64.deb` | 76612 | `013cf07cb9210e11abbcd07c02f0279fd6e3112a03ee6750afc98fab0e8b1ffa` |
| `debs/liblz4-1_1.9.3-2build2_arm64.deb` | 57640 | `3e2274893e0963c758f0d18ba80d033736909373247a7deb211e9ff590a09133` |
| `debs/liblz4-dev_1.9.3-2build2_arm64.deb` | 82106 | `8cdcc5b9493f328d59bf8180c90197a0e859a89643af9e23b9df196c96e28de8` |
| `debs/libopencv-core-dev_4.5.4+dfsg-9ubuntu4_arm64.deb` | 1729726 | `3569619eb66eb75177a3c2274b51a2d86543f8219fe21323697829009f48e008` |
| `debs/libopencv-core4.5d_4.5.4+dfsg-9ubuntu4_arm64.deb` | 934172 | `c74e3d57b9ba37675dccda19f808a387372f94933bf3f7cd32c2d0dc26ad6be8` |
| `debs/libopencv-imgcodecs-dev_4.5.4+dfsg-9ubuntu4_arm64.deb` | 193708 | `f52dd7e526b3f890982523220949f7c80a68ee956343a7b78bfcdd38ec0b0035` |
| `debs/libopencv-imgcodecs4.5d_4.5.4+dfsg-9ubuntu4_arm64.deb` | 118020 | `b797e4889828bfe4177a9e5019c81d579fb2b6c928d8d274737fee1f6825caa5` |
| `debs/libopencv-imgproc-dev_4.5.4+dfsg-9ubuntu4_arm64.deb` | 1329084 | `7303acb30deb3b924d92f508e17b4191a792b27a9a86b8ccfa3b545065a06122` |
| `debs/libopencv-imgproc4.5d_4.5.4+dfsg-9ubuntu4_arm64.deb` | 1015804 | `411eb38d382a21353ac22b0b640660b41b379bb2ed04431192babcd8e18ff8c3` |
| `debs/libwebsocketpp-dev_0.8.2-4_arm64.deb` | 118502 | `229a50ac36550c364519159f62b0f5c392705d8f360e31ee17f0b0549b8ff722` |
| `debs/libzstd-dev_1.4.8+dfsg-3build1_arm64.deb` | 391710 | `5382229c585552619e1c75210ef07ee5d0e4230351bf388552e6511d053a19c7` |
| `debs/libzstd1_1.4.8+dfsg-3build1_arm64.deb` | 311026 | `0102eb80384c2432c4be819663341a1c4e511d05f13ec1467643696531ec6491` |
| `debs/ros-humble-mcap-vendor_0.15.17-1jammy.20260909.220900_arm64.deb` | 185312 | `39f7e20a6b5f39e39e84cffc2332c347cb5a873c63b6d3052e6c512a9139cf5f` |
| `debs/ros-humble-rosbag2-storage-mcap_0.15.17-1jammy.20260909.221142_arm64.deb` | 98506 | `9230f98f5bdc972762a7ecd378c9191f08354dca22ea6fdab2a89a58ad23f38d` |
| `wheels/websockets-15.0.1-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl` | 182312 | `0fdfe3e2a29e4db3659dbd5bbf04560cea53dd9610273917799f1cde46aa725e` |
| `debs/ros-humble-image-transport_3.1.13-1jammy.20260908.042813_arm64.deb` | 375258 | `966e523f9050f880d9e8cb4b8e9fa6d477e11dcca8475c1423c3be53ad4b04ce` |
| `debs/ros-humble-diagnostic-updater_4.0.7-1jammy.20260908.025324_arm64.deb` | 109620 | `770cdd58c06ffba767a35f266d5889a7424d2681f3102f8c09bfbf019606fa05` |

### 3.3 D405 额外 wheel 与其他归档

以下均在原工程存在、未被 Git 跟踪，新目录全部缺失；SHA 为本次实测，不表示 wheel 已安装、依赖已解出或源码来源已验证。`d405_seam_tracking/offline/tank_vision_inspection-0.1.0-py3-none-any.whl` 是业务产物，后续从当前源码重建。

| 原工程相对路径 | bytes | SHA-256 |
|---|---|---|
| `Linux.selection.tar` | 127164416 | `a4bc9dfd14ce7004a11f4f21c1eefe0b375555ea37ea65a18eebd06f535a25bd` |
| `MindVisionSDK_v2.1.0.50/linuxSDK_V2.1.0.50_20261009.tar.gz` | 88610593 | `d77699427623f39152f962b50d2c1e9477cbab891a4853f06ea6dcfdfb823f07` |
| `d405_seam_tracking/offline/MarkupSafe-2.0.1-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl` | 26830 | `2d7d807855b419fc2ed3e631034685db6079889a1f01d5d9dac950f764da3dad` |
| `d405_seam_tracking/offline/Pillow-9.0.1-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl` | 4224154 | `528a2a692c65dd5cafc130de286030af251d2ee0483a5bf50c9348aefe834e8a` |
| `d405_seam_tracking/offline/anyio-4.15.1-py3-none-any.whl` | 132079 | `6152fdbbf9a77fdec97731721bebf7c4c44f7c29b424b0065826173efc7ed101` |
| `d405_seam_tracking/offline/click-8.5.0-py3-none-any.whl` | 125251 | `255bc9599cf7748b4b1a446ccc735421bd08a2ae529a8b88597d3de5664ee360` |
| `d405_seam_tracking/offline/coloredlogs-15.0.1-py2.py3-none-any.whl` | 46018 | `612ee75c546f53e92e70049c9dbfcc18c935a2b9a53b66085ce9ef6a6e5c0934` |
| `d405_seam_tracking/offline/exceptiongroup-1.3.0-py3-none-any.whl` | 16674 | `4d111e6e0c13d0644cad6ddaa7ed0261a0b36971f6d23e7ec9b4b9097da78a10` |
| `d405_seam_tracking/offline/filelock-4.0.10-py3-none-any.whl` | 111193 | `698048d158dc3c9e18cc5e88b3efdd4bd0e77d76686b22adb4511cc4b2c2f0db` |
| `d405_seam_tracking/offline/flatbuffers-25.12.19-py2.py3-none-any.whl` | 26661 | `7634f50c427838bb021c2d66a3d1168e9d199b0607e6329399f04846d42e20b4` |
| `d405_seam_tracking/offline/fsspec-2026.9.0-py3-none-any.whl` | 221738 | `8dd6e646e99ea382bd85f97a45e6b526a442d79423a7dc673f1e2756d05fcb5f` |
| `d405_seam_tracking/offline/h11-0.16.0-py3-none-any.whl` | 37515 | `63cf8bbe7522de3bf65932fda1d9c2772064ffb3dae62d55932da54b31cb6c86` |
| `d405_seam_tracking/offline/hf_xet-1.6.0-cp38-abi3-manylinux_2_28_aarch64.whl` | 4262538 | `0e6e21fa3cdfcdcd76748564bf593870a5e013f47d97cf10aed63aa222cff5b7` |
| `d405_seam_tracking/offline/httpcore2-2.13.1-py3-none-any.whl` | 83423 | `e1e05d4f25f7d7d496bfb96748f6f4b67657b03da069b3a68c36069f3db73d0a` |
| `d405_seam_tracking/offline/httpx2-2.13.1-py3-none-any.whl` | 95597 | `6dff50fabc270ee5fd25d845d0b078ed20564579744d6d962850975996d2f9a4` |
| `d405_seam_tracking/offline/huggingface_hub-2.1.1-py3-none-any.whl` | 838033 | `d76fa1d8e6a59e01f012b1d88da604c099c7a0f2e26b1cc7d708f0a4283bd2a2` |
| `d405_seam_tracking/offline/humanfriendly-10.0-py2.py3-none-any.whl` | 86794 | `1697e1a8a8f550fd43c2865cd84542fc175a61dcb779b6fee18cf6b6ccba1477` |
| `d405_seam_tracking/offline/idna-3.20-py3-none-any.whl` | 69583 | `ab7ae7122974553370f0bdb919e1a960b2cd1bc1ef0276416d896db81c14582c` |
| `d405_seam_tracking/offline/jinja2-3.1.6-py3-none-any.whl` | 134899 | `85ece4451f492d0c13c5dd7c13a64681a86afae63a5f347908daf103ce6d2f67` |
| `d405_seam_tracking/offline/mpmath-1.3.0-py3-none-any.whl` | 536198 | `a0b2b9fe80bbcd81a6647ff13108738cfb482d481d826cc0e02f5b35e5c88d2c` |
| `d405_seam_tracking/offline/networkx-3.4.2-py3-none-any.whl` | 1723263 | `df5d4365b724cf81b8c6a7312509d0c22386097011ad1abe274afd5e9d3bbc5f` |
| `d405_seam_tracking/offline/numpy-1.26.4-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl` | 14219016 | `d209d8969599b27ad20994c8e41936ee0964e6da07478d6c35016bc386b66ad4` |
| `d405_seam_tracking/offline/onnx-1.17.0-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl` | 15908420 | `d545335cb49d4d8c47cc803d3a805deb7ad5d9094dc67657d66e568610a36d7d` |
| `d405_seam_tracking/offline/onnxruntime-1.20.1-cp310-cp310-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl` | 11954010 | `7b2908b50101a19e99c4d4e97ebb9905561daf61829403061c1adc1b588bc0de` |
| `d405_seam_tracking/offline/opencv_python-4.11.0.86-cp37-abi3-manylinux_2_17_aarch64.manylinux2014_aarch64.whl` | 42230439 | `1b92ae2c8852208817e6776ba1ea0d6b1e0a1b5431e971a2a0ddd2a8cc398202` |
| `d405_seam_tracking/offline/packaging-25.0-py3-none-any.whl` | 66469 | `29572ef2b1f17581046b3a2227d5c611fb25ec70ca1ba8554b24b0e69331a484` |
| `d405_seam_tracking/offline/protobuf-7.36.2-cp310-abi3-manylinux2014_aarch64.whl` | 344219 | `e11e1f0180583a2af89db6a2ecd9e8dc40aa6d2988ca175bfd0e6d12ea72d74e` |
| `d405_seam_tracking/offline/pyyaml-6.0.3-cp310-cp310-manylinux2014_aarch64.manylinux_2_17_aarch64.manylinux_2_28_aarch64.whl` | 740646 | `b30236e45cf30d2b8e7b3e85881719e98507abed1011bf463a8fa23e9c3e98a8` |
| `d405_seam_tracking/offline/safetensors-0.8.0-cp310-abi3-manylinux_2_17_aarch64.manylinux2014_aarch64.whl` | 502844 | `7a46e5ff292c356d6991e60942ba7f79817682d3a2cef0702136448cb9c4d235` |
| `d405_seam_tracking/offline/segmentation_models_pytorch-0.5.0-py3-none-any.whl` | 154789 | `c34e09047771aa4dd8878b4f899e8125700cd1f8f7db16e58c37204154151a05` |
| `d405_seam_tracking/offline/setuptools-75.3.0-py3-none-any.whl` | 1251070 | `f2504966861356aa38616760c0f66568e535562374995367b4e69c7143cf6bcd` |
| `d405_seam_tracking/offline/sympy-1.13.1-py3-none-any.whl` | 6189177 | `db36cdc64bf61b9b24578b6f7bab1ecdd2452cf008f34faa33776680c26d66f8` |
| `d405_seam_tracking/offline/tank_vision_inspection-0.1.0-py3-none-any.whl` | 123210 | `1d29080c2d12f374bdf7fe847cf2df9a010e12e337b36d764a091927d2256fd4` |
| `d405_seam_tracking/offline/timm-1.0.15-py3-none-any.whl` | 2361373 | `5a3dc460c24e322ecc7fd1f3e3eb112423ddee320cb059cc1956fbc9731748ef` |
| `d405_seam_tracking/offline/tomli-1.2.2-py3-none-any.whl` | 12118 | `f04066f68f5554911363063a30b108d2b5a5b1a010aa8b6132af78489fe3aade` |
| `d405_seam_tracking/offline/torch-2.5.1-cp310-cp310-manylinux2014_aarch64.whl` | 91919522 | `34bfa1a852e5714cbfa17f27c49d8ce35e1b7af5608c4bc6e81392c352dbc601` |
| `d405_seam_tracking/offline/torchvision-0.20.1-cp310-cp310-manylinux2014_aarch64.whl` | 14265343 | `75f8a4d51a593c4bab6c9bf7d75bdd88691b00a53b07656678bc55a3a753dd73` |
| `d405_seam_tracking/offline/tqdm-4.70.1-py3-none-any.whl` | 80199 | `c293e525e6fef9c20e8728fd4612df02a0aa31bb5fe91ecd93e123b1b7bffa73` |
| `d405_seam_tracking/offline/truststore-0.10.4-py3-none-any.whl` | 18660 | `adaeaecf1cbb5f4de3b1959b42d41f6fab57b2b1666adb59e89cb0b53361d981` |
| `d405_seam_tracking/offline/typing_extensions-4.16.0-py3-none-any.whl` | 45571 | `481caa481374e813c1b176ada14e97f1f67a4539ce9cfeb3f350d78d6370c2e8` |
| `d405_seam_tracking/offline/wheel-0.45.1-py3-none-any.whl` | 72494 | `708e7481cc80179af0e556bbf0cc00b8444c7321e2700b8d8580231d13017248` |

### 3.4 相机参数、标定和参考资源

本次不以 SDK 参数文件替代相机内参标定；没有评估文件参数是否对应当前硬件。已纳入 Git 的资源在 5,685 文件 SHA 对比中全部相同。物理内容存在不代表生产精度或校准过程有效。

| 原工程相对路径 | Git | 新目录 | SHA-256 |
|---|---|---|---|
| `Camera/Data/046041920527.mvdat` | 是 | 同路径存在且相同 | `361c471d2e988affceef3060c90e944a4c7bea7f7324dd1614a51619a49d269a` |
| `Camera/Data/046081820650.mvdat` | 是 | 同路径存在且相同 | `03b9c9c18569c21f1c486bba9e1b8db91aa9035060c15412733aabcf472530de` |
| `MindVisionSDK_v2.1.0.50/sdk/demo/python_demo/Camera/Data/046041920527.mvdat` | 是 | 同路径存在且相同 | `361c471d2e988affceef3060c90e944a4c7bea7f7324dd1614a51619a49d269a` |
| `physic_bev/Camera/Data/043110120081.mvdat` | 是 | 同路径存在且相同 | `d82d36532818855620ee56c1d991c5bbee39d0304753019f8922648535fc66e9` |
| `physic_bev/Camera/Data/046081820650.mvdat` | 是 | 同路径存在且相同 | `03b9c9c18569c21f1c486bba9e1b8db91aa9035060c15412733aabcf472530de` |
| `physic_bev/src/inspection_bringup/config/rough_extrinsics.yaml` | 是 | 同路径存在且相同 | `85d24efb35ea10d833838adf697d76c4ccf869a92d1feb8238d03cb1b0b79440` |
| `tank_vision_inspection/Camera/Data/043110120081.mvdat` | 否 | 同路径缺失 | `d82d36532818855620ee56c1d991c5bbee39d0304753019f8922648535fc66e9` |
| `tank_vision_inspection/Camera/Data/046081820650.mvdat` | 否 | 同路径缺失 | `03b9c9c18569c21f1c486bba9e1b8db91aa9035060c15412733aabcf472530de` |

`tank_vision_inspection/Camera/Data/` 下缺失的 046081820650 与 043110120081 分别与 `physic_bev/Camera/Data/` 同名文件哈希一致，未造成独有文件内容丢失。此结论只覆盖这两个文件；SDK 默认 CWD/相机参数读写行为未做设备验证。

其他必须保留并验证加载位置的已跟踪配置：

- `tank_vision_inspection/configs/seam_width_reference_s100p_v1.json`、`seam_geometry_reference_s100p_live_v1.json` 与各 s100p YAML；setup.py 跨目录安装 4 个文件见 P06。
- `physic_bev/src/FAST-LIVO2/config/{livo,mid360,d405,log}.yaml` 和 Livox `config/MID360_config.json`；文件存在并与原工程相同，尚未评估外参/设备网络有效性。
- `physic_bev/src/inspection_bringup/config/rough_extrinsics.yaml`：`calibration_id=mid360_d405_rough_20260721_02`，`production_valid=false`；带人工测量/不确定度为空等限制。其引用的 `正视图.png`、`侧视.png`、`俯视.png` 在新旧工程中递归查找均未找到（排除 Git 元数据和虚拟环境），缺少现场证据文件；配置中记录的图片 hash 因而无法实物复核。最终外参精度未验收。
- 默认视觉配置的 `camera.calibration_file: null`、`calibration_version: null`；不能给出已标定结论。RealSense 工厂内参来自硬件，本轮未读取。
- `physic_bev/src/mindvision_camera_driver/mindvision_camera_driver/node.py:448-455` 创建 CameraInfo 时明确保持标定数组为零；这是当前源码行为，不能因 `.mvdat` 文件存在就认为 ROS CameraInfo 已提供有效内参。

## 4. 各构建/安装/launch 文件静态审查索引

覆盖非 third_party/、非 MindVision vendor 的全部 CMakeLists.txt、setup.py 与 launch 目录 Python，共 63 个文件。下面“包内定位/依赖查找”表示未命中跨源码目录的特征规则，并结合安装声明进行静态审查；不代表 configure、安装、launch 求值或功能已通过。标准 BUILD_INTERFACE、INSTALL_INTERFACE、包内 glob 本身不属于待修固定路径。包内 config、resource、URDF、meshes、plugin XML 仍须随包保留。

| 文件 | 静态定位/需验证事项 |
|---|---|
| `d405_seam_tracking/native/CMakeLists.txt` | 包内定位/依赖查找；未见跨源码目录特征 |
| `d405_seam_tracking/ros2_ws/src/d405_seam_ros/launch/tracking.launch.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `d405_seam_tracking/ros2_ws/src/d405_seam_ros/setup.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/FAST-LIVO2/CMakeLists.txt` | 44: `add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")`；156: `${CMAKE_SOURCE_DIR}/../../install/vikit_common/lib/libvikit_common.so`；157: `${CMAKE_SOURCE_DIR}/../../install/vikit_ros/lib/libvikit_ros.so` |
| `physic_bev/src/FAST-LIVO2/launch/mapping.launch.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/eddy_driver/CMakeLists.txt` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/eddy_driver/launch/eddy_driver.launch.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/fisheye_camera_driver/launch/fisheye_camera.launch.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/fisheye_camera_driver/setup.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/inspection_bringup/CMakeLists.txt` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/inspection_bringup/launch/baseline_capture.launch.py` | 20: `DEFAULT_MCAP_ROOT = Path("/media/mcap_data/mcap")` |
| `physic_bev/src/inspection_bringup/launch/baseline_replay.launch.py` | 11: `DEFAULT_MCAP_ROOT = Path("/media/mcap_data/mcap")` |
| `physic_bev/src/inspection_bringup/launch/eddy_sync_capture.launch.py` | 20: `DEFAULT_MCAP_ROOT = Path("/media/mcap_data/mcap")` |
| `physic_bev/src/inspection_bringup/launch/eddy_sync_replay.launch.py` | 11: `DEFAULT_MCAP_ROOT = Path("/media/mcap_data/mcap")` |
| `physic_bev/src/inspection_bringup/launch/field_validation_common.py` | 92: `"tank_vision_inspection": _git_evidence(Path(__file__).resolve().parents[4] / "tank_vision_inspection"),`；93: `"physic_bev_src": _git_evidence(Path(__file__).resolve().parents[1].parents[1]),` |
| `physic_bev/src/inspection_bringup/launch/hardware_pipeline.launch.py` | 33: `return Path(os.environ.get("FIELD_DATA_ROOT", "/media/mcap_data")) / "mcap"` |
| `physic_bev/src/inspection_bringup/launch/mock_pipeline.launch.py` | 13: `STORAGE_MOUNT = Path(os.environ.get("FIELD_DATA_ROOT", "/media/mcap_data"))` |
| `physic_bev/src/inspection_bringup/launch/record_only.launch.py` | 13: `STORAGE_MOUNT = Path(os.environ.get("FIELD_DATA_ROOT", "/media/mcap_data"))` |
| `physic_bev/src/inspection_bringup/launch/rough_calibration_view.launch.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/inspection_bringup/launch/visual_field_validation.launch.py` | 24: `sys.path.insert(0, str(Path(__file__).resolve().parent))`；177: `DeclareLaunchArgument("data_root", default_value="/media/mcap_data"),` |
| `physic_bev/src/inspection_interfaces/CMakeLists.txt` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/inspection_livox_adapter/CMakeLists.txt` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/inspection_mock_sensors/CMakeLists.txt` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/inspection_sync/CMakeLists.txt` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/inspection_tools/setup.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/livox_ros_driver2/CMakeLists.txt` | 94: `find_library(LIVOX_LIDAR_SDK_LIBRARY  liblivox_lidar_sdk_static.a    /usr/local/lib)`；256: `find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_shared.so /usr/local/lib REQUIRED)` |
| `physic_bev/src/livox_ros_driver2/launch/msg_MID360_launch.py` | 14: `lvx_file_path = '/home/livox/livox_test.lvx'`；18: `cur_config_path = cur_path + '../config'` |
| `physic_bev/src/livox_ros_driver2/launch/rviz_MID360_launch.py` | 14: `lvx_file_path = '/home/livox/livox_test.lvx'`；18: `cur_config_path = cur_path + '../config'` |
| `physic_bev/src/mindvision_camera_driver/launch/mindvision_camera.launch.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/mindvision_camera_driver/setup.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/realsense-ros/realsense2_camera/CMakeLists.txt` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/realsense-ros/realsense2_camera/launch/rs_intra_process_demo_launch.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/realsense-ros/realsense2_camera/launch/rs_launch.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/realsense-ros/realsense2_camera/launch/rs_multi_camera_launch.py` | 35: `sys.path.append(str(pathlib.Path(__file__).parent.absolute()))` |
| `physic_bev/src/realsense-ros/realsense2_camera/launch/rs_multi_camera_launch_sync.py` | 37: `sys.path.append(str(pathlib.Path(__file__).parent.absolute()))` |
| `physic_bev/src/realsense-ros/realsense2_camera_msgs/CMakeLists.txt` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/realsense-ros/realsense2_description/CMakeLists.txt` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/realsense-ros/realsense2_description/launch/launch_utils.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/realsense-ros/realsense2_description/launch/view_model.launch.py` | 22: `sys.path.append(os.path.dirname(os.path.realpath(__file__)))` |
| `physic_bev/src/realsense-ros/realsense2_rgbd_plugin/CMakeLists.txt` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/realsense-ros/realsense2_ros_mqtt_bridge/launch/rs_launch.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/realsense-ros/realsense2_ros_mqtt_bridge/setup.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/rpg_vikit/vikit_common/CMakeLists.txt` | 包内定位/依赖查找；未见跨源码目录特征；manifest 与 colcon 识别差异（P14） |
| `physic_bev/src/rpg_vikit/vikit_py/CMakeLists.txt` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/rpg_vikit/vikit_py/setup.py` | 包内定位/依赖查找；未见跨源码目录特征；catkin_pkg 遗留（P14） |
| `physic_bev/src/rpg_vikit/vikit_ros/CMakeLists.txt` | 10: `add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")` |
| `physic_bev/src/system_websocket/agv_bridge/CMakeLists.txt` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/system_websocket/agv_bridge/launch/tank_inspection.launch.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/system_websocket/agv_protocol/CMakeLists.txt` | 21: `/usr/local/include` |
| `physic_bev/src/top_pkg/CMakeLists.txt` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/top_pkg/launch/bringup.launch.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `physic_bev/src/top_pkg/launch/bringup_sensor.launch.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `tank_vision_inspection/ros2_ws/src/inspection_rerun/launch/rerun_live.launch.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `tank_vision_inspection/ros2_ws/src/inspection_rerun/launch/rerun_replay.launch.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `tank_vision_inspection/ros2_ws/src/inspection_rerun/setup.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `tank_vision_inspection/ros2_ws/src/tank_vision_core/CMakeLists.txt` | 13: `PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../tank_vision"` |
| `tank_vision_inspection/ros2_ws/src/tank_vision_interfaces/CMakeLists.txt` | 包内定位/依赖查找；未见跨源码目录特征 |
| `tank_vision_inspection/ros2_ws/src/tank_vision_ros/launch/stable_seam_direct.launch.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `tank_vision_inspection/ros2_ws/src/tank_vision_ros/launch/stable_seam_direct_guided_anchor.launch.py` | 14: `default_snapshots = "/media/mcap_data/vision/stable_seam_failure_snapshots"`；39: `default_value="/media/mcap_data/vision/stable_seam_live_sessions",` |
| `tank_vision_inspection/ros2_ws/src/tank_vision_ros/launch/stable_seam_topic.launch.py` | 17: `DeclareLaunchArgument("roi_validation_output_root", default_value="/media/mcap_data/runs/roi_validation"),`；21: `DeclareLaunchArgument("geometry_provenance_output_root", default_value="/media/mcap_data/runs/geometry_provenance"),` |
| `tank_vision_inspection/ros2_ws/src/tank_vision_ros/launch/vision.launch.py` | 包内定位/依赖查找；未见跨源码目录特征 |
| `tank_vision_inspection/ros2_ws/src/tank_vision_ros/setup.py` | 18: `"../../../configs/s100p_stable_seam_4096x3072.yaml",`；19: `"../../../configs/s100p_guided_anchor_seam_4096x3072.yaml",`；20: `"../../../configs/seam_width_reference_s100p_v1.json",`；21: `"../../../configs/seam_geometry_reference_s100p_live_v1.json",` |
| `tank_vision_inspection/setup.py` | 包内定位/依赖查找；未见跨源码目录特征 |

## 5. M1 后续验证门槛与剩余缺失

第一批精确文件、同期引用和命令见 [迁移映射](migration_map.md#m1-第一批迁移方案)。后续算法/驱动/应用批次须另外覆盖：

| 方向 | 应执行验证 | 当前状态 |
|---|---|---|
| 算法安装 | wheel 和 tank_vision_core 各自干净安装；检查 import origin、逐文件哈希；避免两份旧算法被 PYTHONPATH 混入 | 未执行 |
| 配置与资源 | ROS launch --show-args；安装 share 中逐项存在性与 SHA；在仓库外 CWD 运行 CLI；不同配置根覆盖；只读源码安装运行 | 未执行 |
| 全量构建 | 修复 vikit 元数据、fast_livo 链接、SDK 前缀后，在新工作区 clean configure/build；完整记录已启用/忽略包 | 未执行 |
| SDK | 只读 SDK 目录 hash/架构/dlopen、实际绑定路径/version；Livox 新编译符号绑定；RealSense 库 SONAME/ABI | 本轮仅存在性/哈希；未执行加载与硬件验证 |
| 现场依赖 | 补模型权重（如启用模型后端）、有效标定、设备序列号/网络/挂载条件与必要回放 fixture | 缺权重/有效标定验收；未读取设备 |
| 数据功能 | 新建临时采集→冻结配置→MCAP/视频→校验/封存→回放；run 数据目录与代码源目录分离 | 未执行 |
| 通信应用 | 更新 app/config/EMAT/bridge/console 路径；复用已有 system_websocket、websocket_console 测试，fixture 到临时目录；再做设备端联调 | 未执行 |
| 可迁移性 | 新前缀构建后检查 CMakeCache、cmake 导出、RPATH、shebang、Python origin；移到不同目录/仓库外 CWD 验证 | 未执行 |

只读共享原始 SDK、deb/wheel、标定输入，不代表允许复用旧 build/install/venv。所有新验证保留真实退出码、命令、输入哈希、安装前缀和跳过原因；未运行项目保持“未执行”。

## 全量迁移后的状态（2026-10-22）

以上 M0 内容保留其当时语义；不再作为当前路径清单。原源码已全量迁移，最新去向见 migration_map.md / migration_files.json，验证见 migration_completion.md。
已修复旧 install/build/venv 引用、统一源码扫描与入口、视觉跨目录安装、业务与 ROS 适配边界、配置加载与资源位置。33 清单归档与两套 SDK 已准备，校验见 third_party/dependency_inventory.json。
额外发现 M0 Git 清单以外的 5 个 outputs 源文件被原视觉目录 output*/ 忽略规则排除，已从原工程只读恢复、校验并纳入源码；见 migration_supplemental_files.json。
模型/粗标定证据缺口未伪造解决。历史文档/封存数据字段中仍保留旧路径以保持来源可审计；当前执行命令以 README 与 deploy/README 为准。
