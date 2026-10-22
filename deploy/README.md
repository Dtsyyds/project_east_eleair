# 部署入口

所有主入口可从任意当前目录通过绝对路径调用，`--help` 不启动设备。先阅读 README 与下一轮验证计划。

- env.sh：source 设置本工作树与平台路径；拒绝继承原工程路径的环境；已有系统 ROS underlay 可读使用，不安装或修改系统。
- build.sh：--plan 只打印命令；默认构建 profile，--packages-select 实际按依赖闭包 packages-up-to 选择；--skip-native 使用明确准备的依赖；--offline 校验原清单并在新 sysroot 解包 ARM64 deb。所有 build/install/log/venv/native/sysroot 在新 `.runtime/<platform>`。
- run.sh / apps/web_console/run.sh：启动原硬件、operations、bridge 和网页 supervisor；普通执行有设备副作用，本轮仅执行 --help。
- collect/stop/replay/validate/handoff：转发 inspection CLI；run_id、name:=value 等保持原能力，子命令帮助可查看入口语法。
- preflight.sh：仅检查依赖/工具/资源，新 ROS overlay 缺失返回 2；不是硬件预检通过证明。
- verify_offline.sh：迁移后的原离线/ROS mock 回归工作流，下一轮干净构建和证据准备后运行；输出独立验证目录，不修改历史报告。
- rollback/restore.sh COMMIT NEW_DIRECTORY：创建独立 detached worktree 后按对应版本重建；不 reset 当前工作、不复用旧 install。
- offline.py --check-only：校验 33 个原清单归档，不解包。

pip 安装步骤去掉 PYTHONPATH，避免旧 setuptools 在源码已可见时省略 editable .pth 导致干净环境不可导入。部署仍使用项目固定源码；不安装旧业务 wheel。
Rerun 使用独立 `.runtime/<platform>/rerun_venv`（FIELD_RRD_PYTHON 可明确覆盖），保持 0.36.2 与其 NumPy2 需求隔离，不污染 D405 的 NumPy1.26.4。tools/vision/setup_rerun_s100p.sh 仅从本地 wheel 安装；当前清单缺对应归档，须补齐原版本，不能假报可用。
兼容脚本列表见 docs/architecture/compatibility_manifest.json。旧 --wireless-only 转译到新 build --packages-select agv_bridge。
