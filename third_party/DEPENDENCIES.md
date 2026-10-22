# 本地依赖记录

原工程只读来源 `/media/mcap_data/east_eleair_plus`；所有运行入口使用新目录或显式配置，不回退该来源。
`dependency_inventory.json` 逐文件记录 311 个分发文件（723,804,201 字节）的源路径、目标、SHA256、软链接及 ELF e_machine/class；复制后核对内容，52 项还与 M0 表内 SHA 一致，系统 33 归档与原 dependency_manifest.json 完全一致。
不表示已经加载动态库、解包全部依赖、构建或通过设备 ABI 测试。

| 内容 | 保存/安装策略 |
|---|---|
| system 32 deb + 1 websockets 15.0.1 wheel | third_party/offline/system，保留原 SHA 清单；ARM64 deb 仅可解包到本 runtime/sysroot |
| D405 39 wheels | third_party/offline/d405；保持已有版本；tank_vision_inspection-0.1.0 wheel 只归档，不作为业务安装输入 |
| MindVision legacy | 原 MindVisionlinuxSDK；确切版本未从文件名验证，不冒称与新版相同 |
| MindVision 2.1.0.50 | 源/绑定在 third_party/mindvision/2.1.0.50，原始 tar 在 third_party/offline/mindvision；运行绑定和库从同一版本根选择 |
| Livox-SDK2、Sophus、librealsense | 保留已有非 ROS 源码及许可证/补丁；重新 cmake 构建到 runtime/native，不复制原 lib/build |
| Linux.selection.tar | M0 SHA a4bc9dfd14ce7004a11f4f21c1eefe0b375555ea37ea65a18eebd06f535a25bd；只读列目录显示包含同版 SDK tar、MVDCP2 tar、安装/运行脚本。未整体复制或执行，其 active SDK 原分发已单独保存 |

SDK lib 包含 ARM/ARM64/x86/x86-64 等，文件架构逐项见 JSON。默认主机按 uname 选择 arm64 或 x64，unsupported 平台应报错。不要把 SDK 两版混装到系统；未改 /usr/lib、udev 或服务。
许可证沿原目录保留；第三方版本由已有 package.xml/CMake/源码和基线提交固定，本轮没有更新第三方版本。

缺口：Rerun 0.36.2 的独立环境及其完整 wheel 集不在已核验分发中；应补充同版归档及 SHA 后在新 rerun_venv 安装。x86 不可用 ARM64 wheels/debs，须提供对应版本分发或现有受控依赖。系统 PyYAML 5.4.1 不满足项目元数据；新 metadata_venv 已从本地安装原固定 6.0.3，系统未修改，正式 build 使用同版。

模型默认 null，没有可验收的 ONNX/RKNN/BPU/训练权重；恢复时提供版本、hash、输入输出契约。rough_extrinsics 的正视图.png、侧视.png、俯视.png 在新旧工程均缺失，哈希保留原 YAML，须按记录恢复原证据或重新标定，production_valid 仍 false。
原大型 output/采集数据未复制，读取历史视觉证据通过 FIELD_VISION_EVIDENCE_ROOT 明确指定；旧源码快照复现通过 INSPECTION_BASELINE_FIXTURES 明确指定。旧 build/install/log/venv/offline sysroot 一概未搬入。
