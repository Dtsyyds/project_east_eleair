# FIELD-W1 现场测试矩阵

每个 case 独立 run；先记录 `run_id`，再开始计时。所有数字来自
`run_manifest.json`、`live_session/live_summary.json`、`frame_metrics.jsonl`、
`preflight_report.json` 和 MCAP validation report。失败不得临时降低 geometry gate。

| ID | 条件 | 时长 | 预期 |
|---|---|---:|---|
| A01 | 中央静止 | 60 s | TRACKING |
| A02 | 中央偏左但仍在允许区 | 60 s | TRACKING |
| A03 | 中央偏右但仍在允许区 | 60 s | TRACKING |
| A04 | 左倾 | 60 s | TRACKING |
| A05 | 右倾 | 60 s | TRACKING |
| A06 | 缓慢左右移动 | 120 s | TRACKING，detected ≥90% |
| A07 | 环境光轻微变化 | 120 s | TRACKING |
| A08 | 部分模糊或短暂遮挡 | 60 s | DEGRADED 后恢复 |
| N01 | 无焊缝 | 60 s | LOST，false positive=0 |
| S01 | 稳定性 | 30 min | 无崩溃、无持续 RSS 增长 |

## 每项必填字段

```text
run_id:
现场条件:
相机距离/角度:
曝光模式/请求曝光:
实际曝光、analog_gain、analog_gain_raw、gain_db 可用性:
输入 Hz / 输出 Hz:
detected 率 / TRACKING 率:
最长 LOST 时间:
joint inlier 分布:
residual 分布:
processing P50/P95/max:
快照数量 / metadata complete 率:
MCAP 路径:
validate 状态 / finalize 状态:
CameraInfo valid（应为 false，除非重新标定）:
人工备注:
```

## 像素级门槛

- camera ≥10 Hz；tracker ≥8 Hz；processing P95 ≤100 ms；
- A01 detected ≥95%；A06 detected ≥90%；进入 TRACKING ≤3 个处理帧；
- 中线人工误差 P95 ≤25 px，左右边界 P95 ≤30 px；crossing=0；
- N01 false positive=0；快照 metadata complete=100%；
- MCAP 必需话题存在且 timestamp 单调；S01 无崩溃和持续 RSS 增长。

## 证据等级

`synthetic`、`mock`、`field_validation_candidate`、`real_hardware_pending`、
`production_calibrated` 必须在备注中区分。没有真实相机、MID-360 或 DDS 时不得填写
真实硬件结果。
