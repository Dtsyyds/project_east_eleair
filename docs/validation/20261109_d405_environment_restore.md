# 2026-11-09 巡迹 Python 环境恢复

报错原因是配置指定的 `.runtime/s100p/d405_prototype_venv/bin/python` 不存在。
已从 `/media/mcap_data/east_eleair_plus/d405_seam_tracking/offline` 的本地 wheel
重建独立环境，使用系统 ROS 依赖。复现命令：

```bash
bash deploy/restore_d405_environment.sh /media/mcap_data/east_eleair_plus/d405_seam_tracking/offline
```

恢复版本为 torch 2.5.1、torchvision 0.20.1、segmentation-models-pytorch 0.5.0、
NumPy 1.26.4、OpenCV 4.11.0.86、PyYAML 6.0.3。未修改系统 Python 包。
验证了这些依赖与 rclpy、d405_seam_ros.node 的导入，及巡迹入口 `--help`。
模型结构的零输入前向计算通过；这仅验证依赖，不代表训练权重或识别效果通过。

## 尚缺的原始模型

当前 `config/d405/round1_a.yaml` 要求：

```text
.runtime/s100p/analysis/d405_unet_round1_20261102/models/fold_1/checkpoint.pt
SHA256: 15dbdde88dc26eba86adda212a146786b624a23bd57929f0bfb1a0152ca4327f
```

本机未找到该原始模型，需要从旧备份恢复。没有生成替代权重或放宽校验。
Python 环境恢复后，真实视觉巡迹仍受此缺失文件阻止，底盘未因此启用。
启动检查现在在创建实验目录和子进程前检查权重存在性与 SHA256，直接返回缺失或校验失败原因。
原服务重新启动后加载该检查。

`test_vision.py` 与 `test_d405_a_runtime.py` 的适用测试共 16 项通过，覆盖缺失权重、
错误权重以及有效测试夹具的启动路径。依赖旧 `visual_config.json` 证据的
`test_a_config_preserves_every_geometry_parameter` 未运行（原证据目录缺失），不计入通过数。
恢复脚本通过 bash 语法检查。没有进行实机巡迹或底盘控制验收。
