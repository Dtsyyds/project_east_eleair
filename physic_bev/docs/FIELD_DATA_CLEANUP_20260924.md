# 现场存储盘清理记录（2026-09-24）

清理目标：为后续实际下场采集释放空间，同时保留已通过的正式批次和状态不明的旧式历史证据。

清理规则：只删除 `/media/mcap_data/runs` 下存在 `complete.json`，且不同时满足以下三个条件的批次：

- `complete == true`
- `finalized == true`
- `validation == "passed"`

删除前统计：23 个批次，共 48,071,617,521 字节（约 44.77 GiB）。

## 删除目标

```text
FIELD-VEHICLE-20260922_232714
FIELD-VEHICLE-20260922_234713
FIELD-VEHICLE-20260923_102724
FIELD-W5-D405-DI1-15FPS-10MIN-20260923_113354
FIELD-W5-D405-DI1-15FPS-20260923_1110
FIELD-W5-D405-DI1-15FPS-NOPOLL-10MIN-20260923_112750
FIELD-W5-D405-DI1-640-15FPS-20260923_1116
FIELD-W5-D405-I1-15FPS-10MIN-20260923_114040
FIELD-W5-D405-I1-15FPS-FIXED-SMOKE-20260923_131503
FIELD-W5-D405-I1-15FPS-IDR-SMOKE-20260923_130743
FIELD-W5-D405-I1-15FPS-SEG30-SMOKE-20260923_130158
FIELD-W5-EDDY-MEAN-SMOKE-20260923_1535
FIELD-W5-S1-ALL-20260923_105340
FIELD-W5-S1-CLEAN-20260923_1016
FIELD-W5-S1-CLEAN-20260923_1019
FIELD-W5-S1-CLEAN-20260923_104008
FIELD-W5-S1-FORMAL-20260922_2242
FIELD-W5-S1-SMOKE-20260922_2221
FIELD-W5-S1-SMOKE2-20260922_2228
FIELD-W6-COMPACT-60S-20260924_1006
FIELD-W6-COMPACT-FORMAL-20260924_1013
FIELD-W6-COMPACT-ROI-FORMAL-20260924_1023
FIELD-W6-MAPPING-EDDY-60S-20260923_2237
```

## 明确保留

以下已通过批次不在本次清理范围：

```text
FIELD-VEHICLE-20260923_103315
FIELD-W5-D405-DI1-15FPS-NOPOLL-20260923_1121
FIELD-W5-D405-I1-15FPS-30MIN-20260923_132327
FIELD-W5-D405-I1-15FPS-FINAL-SMOKE-20260923_131954
FIELD-W5-EDDY-MEAN-SMOKE2-20260923_1540
FIELD-W6-COMPACT-ROI-FORMAL-20260924_1027
FIELD-W6-COMPACT-ROI-10MIN-20260924_1036
```

W1–W4 批次多数没有 `complete.json`，无法仅凭目录名确定证据价值，因此本轮保留。根目录下的 `configs_frozen`、`reports`、`tuning_ui_live` 和 `tuning_ui_failure_snapshots` 同样保留。

## 执行结果

- 23 个目标先逐项移入外接盘回收站，再按各自的精确 trash URI 永久删除；
- 用户原有的其他回收站条目未被清空或修改；
- 删除后不存在带 `complete.json` 但未完成/未通过的批次；
- 7 个明确通过的批次均仍存在；
- 文件系统可用空间从约 851 GiB 增加到约 896 GiB，使用率从 10% 降到 5%。

上述 23 个目录已永久删除，不能从本机回收站恢复。

## 第二轮清理：只保留最新 W6 10 分钟批次

经用户明确确认，随后删除 `/media/mcap_data/runs` 下除以下目录以外的所有内容：

```text
FIELD-W6-COMPACT-ROI-10MIN-20260924_1036
```

第二轮永久删除 56 个顶层目录，共 44,760,999,362 字节（约 41.69 GiB），其中包括 W1–W5 历史证据、其他已通过批次、旧调试报告和调参会话。

最终核对：

- `/media/mcap_data/runs` 仅剩上述 10 分钟批次；
- 保留批次大小约 943 MiB；
- `complete=true`、`finalized=true`、`validation=passed`、`sha256_valid=true`；
- 对 `sha256sums.txt` 重新执行全量校验，退出码为 0；
- runs 对应的回收站条目为空；
- 存储盘约 937 GiB 可用，使用率 1%；
- 没有活动采集、回放或 rosbag writer 进程。

两轮清理合计释放约 86.46 GiB。除最新 W6 10 分钟批次外，其余历史数据已永久删除，无法从本机回收站恢复。

## 工作区历史采集检查

随后检查 `/home/sunrise/east_eleair`：

- 删除 `/home/sunrise/east_eleair/physic_bev/bags/physic_bev`；
- 该目录包含旧的 `physic_bev_0.mcap` 和 `metadata.yaml`，共 1,702,073,866 字节（约 1.59 GiB）；
- 该 MCAP 没有运行时消费者，只在 README/架构文档中作为历史录制命令示例路径出现；
- 删除后工作区不再存在项目自产的 `.mcap` 或 `.db3` 采集文件；
- 外接盘保留的最新 W6 10 分钟批次仍为完整、已封存、验证通过且 SHA 有效。

以下内容经检查后保留：

- `tank_vision_inspection/output`：人工标注、焊缝真值、失败样本、相机验证和算法回归数据；
- `physic_bev/output/field_w6_mapping_eddy_collection`：小体积汇总报告和紧凑轨迹副本；
- SDK、第三方依赖、源码、构建/安装目录和配置。

工作区旧 MCAP 已永久删除，不能从本机回收站恢复。
