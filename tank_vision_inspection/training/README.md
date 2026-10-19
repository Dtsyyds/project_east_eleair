# 模型训练准备（未训练）

当前没有真实标注数据、模型权重或真实性能结论。本目录只约定两个独立模型：

- 模型 A：焊缝分割，输出 mask；后续继续使用 `mask -> skeleton -> centerline -> RANSAC -> lateral_error/heading_error`。
- 模型 B：表面缺陷分割，输出外部可见类别或保守 `surface_anomaly`，不推断内部裂纹或深度。

S100P 的第一候选可以评估轻量分割网络（例如 YOLOv8n-seg），但必须依次完成 ONNX 导出、官方算子兼容检查、Nash-m 模型转换、真实校准集 INT8 量化、黄金测试集精度比较和板端速度测试。不同 RDK 板卡的模型文件不能混用。

`export_onnx.py` 只调用用户明确提供并审查过的 exporter；`validate_onnx.py` 只检查 ONNX Runtime 能否加载及输入/输出元数据，不猜测模型解码方式。
