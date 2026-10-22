# 真实数据集规范

本目录只定义规范，不包含或伪造真实罐体图像。清单使用 UTF-8 JSONL（每行一个样本）或 JSON 数组，字段约束见 `schema/manifest.schema.json`。

目录用途：`raw/` 原始图像，`annotations/` JSON/YAML 标注，`splits/` 分组切分文件，`calibration/` 以标定版本命名的 YAML/JSON。大文件默认由 `.gitignore` 排除，占位文件和规范保留。

第一版语义标签只有 `weld`、`rust_corrosion`、`scratch`、`coating_damage`、`surface_anomaly`、`ignore`。无法从 RGB 证实的内部裂纹、凹坑深度或材料内部状态必须使用保守的 `surface_anomaly` 或 `ignore`。

切分以 `(session_id, tank_id, scan_id)` 为不可拆分组，禁止随机拆分相邻视频帧。校验命令：

```bash
python3 scripts/validate_dataset.py dataset/manifest.jsonl --dataset-root dataset
python3 scripts/create_grouped_split.py dataset/manifest.jsonl dataset/splits/grouped.json
```
