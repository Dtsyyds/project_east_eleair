# S1-B2d 真实焊缝失败诊断（2026-09-07）

## 结论

使用显式加载的 `configs/s100p_realtime_1920x1200_896.yaml` 离线复现代表图片，结果仍为 `detected=false`、`confidence=0`、`seam_geometry_ms=0`。首个失败门控是 `ClassicalSeamDetector._component_mask()` 的 minimum-response 条件：处理后响应 P99.5 为 `1.0`，配置要求 `>= 4.0`，因此在阈值分割前直接返回 `None`。生产代码随后在 `classical_detector.py:156-160` 返回 `lost_reason=no_valid_component`，几何阶段从未进入。

这是“响应不足/光学质量不足”分类，尚不能从这张证据唯一断言肉眼所见一定不是焊缝，也不能归因到 RANSAC、残差、连续性或最终置信度。图像质量是明确的次级证据：灰度均值 `8.9267`、标准差 `0.4541`、P1/P5/P50/P95/P99=`8/8/9/9/10`，欠曝比例 `0.999102`、过曝比例 `0`、Laplacian 方差 `2.5718`、对比度标准差 `0.4541`，触发 `severe_underexposure`、`low_contrast`、`severe_blur`。

## 代码处理顺序和门控归因

实际顺序是：质量评估 → 可选去畸变 → ROI/缩放 → 通道、CLAHE、滤波、局部对比度/形态学梯度响应 → minimum-response → Otsu/自适应阈值 → close/open → 连通域面积和方向跨度 → 中心点 → RANSAC → 置信度因素 → lateral/heading 与中心线 → `confidence >= min_confidence`。

代码证据：

- `classical_detector.py:139-160`：空图、去畸变/ROI 异常、响应和 `_component_mask` 失败；本配置不启用去畸变、ROI 为全图。
- `classical_detector.py:82-84`：`percentile(response, response_percentile) < minimum_response` 直接返回；本图为 `1.0 < 4.0`。
- `classical_detector.py:85-133`：阈值、形态学、连通域面积/跨度及候选选择。由于首个门控失败，这些步骤在生产路径没有执行；诊断图中的 raw/cleaned mask 和连通域是“绕过首个门控后的只读延续”，不是生产输出。
- `classical_detector.py:161-173` 与 `geometry.py:19-62`：中心点不足 `degree+1`、采样跨度或内点数不足会导致 RANSAC 失败；本图 RANSAC 未执行。
- `confidence.py:20-61`：中心合理区域、内点率、残差、连续性、对比度、覆盖率均为置信度因素；中心合理区域、拟合残差、连续性不是当前实现中的硬拒绝门。
- `classical_detector.py:225-233`：最终硬门为 `confidence >= min_confidence`；本图未到达。
- `realtime_pipeline.py:78-100`：质量结果不会在 detector 前直接拒绝；只在已有 seam 结果后缩放置信度，且只有已有 `detected=true` 时才可能降为质量门控失败。因此质量不能解释本例的 `seam_geometry_ms=0`，但解释了为什么图像不具备可靠测量条件。

各可能门控的机器可读状态见 `output/s1_b2d_positive_failure_diagnosis_20260907/diagnosis.json` 的 `gate_catalog`。

## 复现和配置一致性

BMP 文件由 OpenCV 以 `IMREAD_UNCHANGED` 解码为 `1920×1200×3`、`uint8`、BGR；配置显式加载成功，无默认配置回退。全图 ROI、无去畸变；896 处理尺寸为 `896×560`，`scale_x=scale_y=0.466666667`。诊断同时调用生产 detector 的 `_response` 和 `_component_mask`，并验证连通域重建与生产返回一致。

离线一帧的 `lost_reason` 是 `no_valid_component`；ROS 保存状态是连续丢帧后的 `lost_threshold_reached:no_valid_component`。两者的 `detected=false`、`confidence=0`、空中心线和 NaN 几何字段语义一致。validator 也记录 `seam_detected_ratio=0` 及相同的质量警告。

## 阶段图和图像证据

指定的 15 个阶段图全部写入输出目录，所有处理尺度图都标注 `processed coordinates` 及实际尺寸；最终 overlay 标注的是恢复到原图坐标的结果。input preview 显示画面几乎全黑；combined response 主要为低幅随机散斑，没有稳定贯穿结构。896 灰度 raw/cleaned 掩膜比例分别为 `0.060961/0.098611`，但这只是 minimum-response 失败后的诊断延续。清理后有 805 个前景连通域，生产可用数量为 0；不存在 RANSAC 候选中心线。

## 方向诊断

只读 Canny + `HoughLinesP` 没有得到足够线段（0 条），方向为 **uncertain**，不是 vertical、horizontal 或 diagonal。因而不能支持“焊缝主要横向、vertical 配置不匹配”的强结论，也没有改成 horizontal 或实现双方向检测。若后续有更清晰的真实样本显示横向贯穿结构，应在下一阶段显式设计纵缝/环缝方向策略。

## 有限敏感性分析

只运行了用户允许的变体；未扫描阈值、未搜索参数、未修改生产配置。

| 变体 | detected/confidence | cleaned mask | 生产可用连通域 | 结构性 area/span 候选* | 中心线点 | P99.5 | seam_detect / geometry ms |
|---|---:|---:|---:|---:|---:|---:|---:|
| A 896 gray vertical | false / 0 | 0.098611 | 0 | 0 | 0 | 1 | 约 1.77 / 0 |
| B 1024 gray vertical | false / 0 | 0.162053 | 0 | 0 | 0 | 1 | 约 2.43 / 0 |
| C 896 gray horizontal | false / 0 | 0.097202 | 0 | 1 | 0 | 1 | 约 1.82 / 0 |
| D 896 blue vertical | false / 0 | 0.999924 | 0 | 1 | 0 | 2 | 约 3.72 / 0 |
| D 896 green vertical | false / 0 | 0.196821 | 0 | 0 | 0 | 1 | 约 1.66 / 0 |
| D 896 red vertical | false / 0 | 0.997598 | 0 | 1 | 0 | 2 | 约 3.26 / 0 |

`*`结构性候选表示仅按 area/span 观察、忽略已经失败的 minimum-response；它们不代表生产路径通过。1024 只改变处理尺度，仍同样失败。颜色变化显著改变掩膜比例，但 P99.5 仍低于 4，不能归因成简单 BGR/RGB、坐标横纵或缩放恢复错误。

## 人工可见性边界和下一步

当前证据支持：

1. 有充分证据证明当前输入的响应不足且光学质量不足；
2. 没有充分证据证明焊缝不存在；
3. 没有证据证明中心线存在后被 RANSAC/几何拒绝；
4. 没有发现配置加载、默认配置回退、BGR/RGB、坐标或缩放恢复错误；
5. 传统特征在这张真实外观上没有稳定响应。

下一步先采集多张有标注真实样本并改善照明、距离、焦距和视场，尤其避免欠曝/低对比/失焦；若传统响应仍不稳定，进入真实数据采集和 S2 模型检测，不根据单张图直接调生产阈值。若新样本明确显示横向焊缝，再单独设计方向策略。

## 证据保护

四份输入 SHA256 在开始和结束完全相同：

```text
representative_input.bmp       f27937284057951085bee580a2484d0d68184de7889b4efc3222ba09285a61c8
representative_seam_state.json  c43349a7602a615958e125b4683829f6f9efed7122f9981ada42d0e34a8b49cd
validator.json                  4629a1c9194c768edf562b28fda48b4be052c883f53db1eeb813d84a36846ba5
s100p_realtime_1920x1200_896.yaml c2246ba0f3e054fb430d8eade97108817af2f39cb025e3b9c8893fd3f117ebfc
```

## 变更和验证

新增：`scripts/diagnose_real_seam_failure.py`、`tests/test_diagnose_real_seam_failure.py`、本报告，以及输出目录中的诊断结果。未修改 detector、ROS 消息、相机驱动、BEV/Fusion/MCAP 或生产配置。专项测试：`3 passed in 6.46s`；纯 Python 测试（排除 ros2/camera/s100p/rk3588 标记）：`80 passed, 4 deselected in 11.09s`；`compileall` 通过；`git diff --check` 通过。本工作树在任务开始前已有其他用户改动，本阶段未覆盖或清理。
