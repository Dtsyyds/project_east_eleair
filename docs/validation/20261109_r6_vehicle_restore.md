# 2026-11-09 恢复 R6 磁吸附整车模型

用户指出当前上位机仍显示旧车模。核对本地 2026-11-07 会话
`01a1b643-2537-70e2-b2b9-1295bedfd68a`，确认当晚曾按用户要求完成 R6 磁吸附整车模型替换。
本次检查时，两版 `vehicle_model.js` 却都引用旧 chassis、四轮、雷达、夹具共 7 个部件。
此前四窗恢复采用了较早的车模版本，未包含随后完成的 R6 替换。本次恢复这项更新。

## 找到的正确文件

- 完整整车网格：`/home/sunrise/robot_models/1005_asm_D405_sidewall_R6_full_assembly_mm.stl`。
- SHA256：`4368e981e4eb7deb86fd23eed4aa7d546ce3790caa92eed9486690ffea74fd0d`，与同目录 `.sha256` 一致。
- 坐标参考：`/home/sunrise/robot_models/R6_full_assembly_coordinate_reference.json`。
- 原生工程包：`/home/sunrise/robot_models/robot_R6_20261107.zip`。
- ZIP 内顶层总装：`project/1005_asm_D405_sidewall_R6.SLDASM`；说明中保留 Mag_Robot 来源记录。

这是一份车体三维显示模型，与本次新训练的 D405 焊缝识别 checkpoint 是两个不同资源。
此前已经更新的焊缝识别权重配置继续保留。

## 恢复内容

从上述历史记录恢复已验证的 `tools/export_r6_vehicle_preview.py` 和 `config/lidar/r6_vehicle_preview.json`，
复制真实坐标参考至 `config/lidar/r6_full_assembly_coordinate_reference.json`。
使用现有 `config/visualization/vehicle_alignment_20261107.json` 中保存的历史静止 IMU 观测重新导出。
没有执行历史会话中的整段命令，只恢复并检查了所需转换代码。

原始 654,318 个三角面压缩为 11,544 个，网格约 282 KiB，整车统一保留装配位置，
不再另叠加旧轮组或旧雷达网格。两版网页资源一致，model_id 为 `r6_full_assembly_20261107`，
标签为“R6 磁吸附整车”。模型及渲染器资源版本统一更新为 `20261109-r6-full-restored`。

雷达几何中心在显示系中为 `(0.060048, -0.001200, 0.219573)` 米。
STL 毫米转米，轴向沿用历史 CAD 假设，再依据静止 IMU 作最小调平；CAD 与 IMU 残差约 2.42°。
该显示关系仍不是实测外参，仅用于 livox_frame 预览；不修改 TF、原始点云、IMU 或录制数据。

## 验证与证据

`tests/web_console/test_lidar_preview.py`：5 passed（30.10 秒）。
覆盖两版实际 Firefox 页面加载、R6 模型身份、显示开关、视角、尺度/坐标换算、网格索引，
以及不同源 frame 禁止车模叠加、点云过期/重连、四窗布局与窄屏行为。
转换脚本 Python 编译检查通过；环境没有 Node.js，JavaScript 由上述真实浏览器测试执行验证。

独立 Firefox 实际画布预览及模型状态位于：
`.runtime/s100p/analysis/r6_vehicle_restore_20261109/`。

- `perspective.png`、`top.png`：当前渲染器的 R6 预览。
- `browser_verification.json`：R6 model_id、11,544 个渲染三角面及显示估计状态。
- `before/`：恢复前两版页面的模型、渲染器、HTML 和使用说明。
- `recovered_exporter.patch`：从历史记录找回的转换实现。

当前整机网页服务未运行；下一次正常启动后加载新资源，已打开的浏览器使用 Ctrl+F5 刷新。
本次没有启动硬件服务，也没有完成现场实时点云与车模配准验收。

## 重新生成

```bash
python3 tools/export_r6_vehicle_preview.py \
  --stl /home/sunrise/robot_models/1005_asm_D405_sidewall_R6_full_assembly_mm.stl \
  --reference config/lidar/r6_full_assembly_coordinate_reference.json \
  --settings config/lidar/r6_vehicle_preview.json \
  --observations config/visualization/vehicle_alignment_20261107.json \
  --output apps/web_console/vehicle_model.js
cp apps/web_console/vehicle_model.js apps/web_console_chassis_tracking/vehicle_model.js
```
