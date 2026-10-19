# S1-P1 稳定实时焊缝巡检生产候选

## 使用范围与结论边界

本实现是一套独立、可回退的新后端，不替换现有 classical detector、`vision_node` 或 `SeamState`。它只面向 4096×3072 全视场中、画面中下部且焊缝接近中心、帧间位置/方向/宽度连续的工况。默认不搜索顶部，也不承诺任意位置、任意方向焊缝检测。

冻结 V2 单图可计算像素误差；其他真实图片没有对应 V2 标注，只能报告检测/拒绝、几何合法性和叠加图，不能声称现场精度。当前没有相机内外参与尺度标定，所有几何量均为像素或角度，禁止解释为毫米精度。

软件实现和离线 aarch64 CPU 基准已完成；MindVision 实机连续采集、DDS 端到端频率和现场长期稳定性仍需硬件验证。因此交付状态为：`software implementation complete, hardware realtime validation pending`。

## 冻结人工参考

生成命令：

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection
python3 scripts/build_seam_width_reference.py
```

生成器在写出参考前验证 manifest 的 `status=frozen`、`human_overlay_approved=true`、`json_overlay_consistent=true`，并校验原图、V2 JSON 和 V2 overlay 的 SHA256。它只读取冻结 V2 左右人工点：先在共同 y 区间线性插值，再对宽度执行保留端点的三点运行中位数轻度稳健平滑。该处理压制单点波动但保留整体宽度趋势。

运行时只加载 `configs/seam_width_reference_s100p_v1.json`，不会读取人工标注。参考文件含 64 个样本，每个样本保存 `y_normalized`、`center_x_normalized`、`width_normalized` 和参考分辨率下的宽度像素值。

## ROI 与分辨率规则

固定 ROI 归一化范围为：

- x: `[0.30, 0.60)`
- y: `[0.40, 0.98)`
- 4096×3072 原图像素范围：`x=[1229,2458)`、`y=[1229,3011)`，即 `(x,y,w,h)=(1229,1229,1229,1782)`

该范围覆盖冻结标注下半部主要焊缝，左右各保留数倍参考宽度的搜索空间，并排除顶部无关纹理。人工检查图为 `output/s1_p1_stable_seam_tracker/reference_validation/roi_preview.png`。

默认仅接受 4096×3072。输入尺寸不同时，只有在确认是同一全视场等比例缩放并显式设置 `camera.same_fov_scaled=true` 后才缩放归一化参考。长宽比不同会拒绝。MindVision resolution index 3 是不同 ROI 视场，不能视作全视场简单缩放；没有视场映射时必须保持 `same_fov_scaled=false`，算法会返回带清晰原因的 LOST。

## 算法流程

算法仅依赖 NumPy、OpenCV 和 YAML，不依赖 ROS 或 MindVision SDK：固定 ROI 裁剪；等比例缩小到最大宽度 896；灰度与 5×5 Gaussian；多水平扫描带；有符号水平 Sobel；每侧 Top-12 局部边缘；向量化左右配对；Top-8 配对状态的空间动态规划；中心二次多项式与宽度一次模型的 Huber IRLS 联合内点拟合；原图坐标恢复；时间门控与 EMA。

左边界要求从暗区进入亮带的正水平梯度，右边界要求离开亮带的负水平梯度。每个配对的以下分量都归一化到 `[0,1]` 后加权：左右边缘强度、双边平衡、内外对比、人工参考宽度一致性、弱中心先验、时间中心一致性和时间宽度一致性。初始化中心先验由人工参考中心 0.75 与 ROI 中心 0.25 混合，它只参与连续评分，不是硬位置限制。

宽度先通过 `0.55*w_ref <= w <= 1.80*w_ref` 合法性门控，不再使用“图像宽度 6%”。门内连续得分为：

```text
score_width = exp(-abs(log(candidate_width / reference_width)))
```

因此相同倍数的变宽和变窄受到对称惩罚，门内不会硬切换。TRACKING/DEGRADED 时，上一帧中心线是主要位置预测，候选中心限制在预测线左右 2.5 个参考宽度内；LOST 时始终回到完整固定 ROI 捕获，不无限扩窗。

DP 从 ROI 上部向下连接配对，转移项包括中心、宽度、左边界、右边界变化和缺失带代价；所有位置变化除以当前位置参考宽度。最大连续缺失为 3 带，并限制单步中心、左右边界及宽度比例变化。输出覆盖率、有效跨度、最大缺失段、左右突跳计数、接受/拒绝扫描带数和路径置信度。

几何模型为：

```text
left(y)   = center(y) - width(y)/2
right(y)  = center(y) + width(y)/2
center(y) = 二次多项式
width(y)  = 一次多项式
```

中心和宽度分别稳健拟合后取联合内点重拟合，并联合计算中心/半宽残差。发布前在拟合跨度内检查宽度为正、左中右顺序、非交叉、连续且全部位于 ROI 内。

## 状态机与无证据行为

- `LOST(0)`：没有可发布的当前检测；在完整固定 ROI 重新捕获。
- `ACQUIRING(1)`：当前帧有几何证据，但尚未达到连续 3 帧确认。
- `TRACKING(2)`：连续证据达到门槛并使用时间预测。
- `DEGRADED(3)`：短期失败，仅可明确发布 `predicted_only=true`、`detected=false` 的预测结果。

当前测量在覆盖率低于 0.60、有效跨度低于 0.65、连续缺失超过 3、联合内点率低于 0.68、联合拟合残差超过 14 px、路径置信度低于 0.42、几何越界/交叉，或 lateral、width、heading 突跳时拒绝。最多保持 2 帧 DEGRADED，连续失败达到 3 帧即 LOST 并清除模型。只有当前帧通过全部几何证据检查时才以 `alpha=0.50` 更新中心系数、宽度、lateral 和 heading；该值由合成小幅运动回归在滞后误差与平滑间折中确定。预测置信度每失败帧乘 0.5，绝不伪装成当前检测。

## 配置值来源

`configs/s100p_stable_seam_4096x3072.yaml` 是唯一生产配置；阈值均显式存在，不在算法内隐藏。ROI 来自冻结 V2 下半部覆盖和横向留白。896 是此前 S1-B2B 已验证的 ROI 实时处理宽度。扫描步长 8、带半高 4、Sobel 3、Gaussian 5、剖面 Gaussian 9 在保留边缘符号和纵向采样密度的同时控制计算量。每侧边缘 Top-12 保留候选多样性；进入 DP 的 Top-8 是 profile 后的状态空间优化。宽度 0.55–1.80 是宽松合法门，实际排序仍使用连续宽度分数。

几何门槛 0.60/0.65/3/0.68/14 px 来自冻结单图、合成扰动和无标注拒绝样本的保守分离：冻结单图联合内点率约 0.735、残差约 11.69 px；无标注错误路径的内点率最高约 0.376。14 px 为联合中心/半宽残差留出轻微仿射与噪声余量。时间门控（0.8 个宽度 lateral、40% 宽度、8° heading）覆盖题设的小幅帧间运动，同时拒绝突跳。所有数值都可通过 YAML 调整并应在新增真实标注后重新验证。

其余配置值及来源如下，生产调参不得绕过 YAML：

| 组 | 最终值 | 来源/作用 |
|---|---|---|
| camera | 4096×3072、index 0、BGR8、auto AE、100–50000 us、gain raw 2–16、warmup 60、timeout 1000 ms | 参考图与 MindVision 全视场设置；运行时仍按 SDK 单色/彩色能力自动选择 MONO8/BGR8 |
| edge | percentile 88、最小响应 0.8、最小间隔 5 px、横向边距 max(2 px, 2.5% ROI width) | 百分位适应亮度，响应下限拒绝近似平坦帧，间隔/边距去除重复峰和裁剪边伪边缘 |
| pairing sampling | interior margin 2 px、exterior 0.25×候选宽度且至少 3 px | 避开 Sobel 边缘本身并在两侧取稳定背景对比 |
| pairing score | 左/右边缘各 0.10、平衡 0.12、对比 0.18、参考宽度 0.24、中心先验 0.10、时间中心 0.09、时间宽度 0.07 | 权重和为 1；人工宽度与双边图像证据优先，中心只是弱先验，时间项防帧间跳变 |
| score normalization | 对比 offset 0.50/gray scale 80；参考中心权重 0.75；中心衰减 2.5×宽度；时间中心衰减 1.0×宽度；无历史时间分 0.50 | 将灰度差和位置差归一化，初始化时保持中性而不制造硬限制 |
| DP cost | center/width/left/right 权重 0.42/0.24/0.12/0.12，skip 0.38 | 中心连续性为主、宽度次之，两侧共同约束且允许付费跳过反光带 |
| DP gates | missing 3，center jump 0.80×宽度，boundary jump 1.10×宽度，宽度 log-ratio 0.55 | 允许短缺失和透视变化，但阻止局部反光把整条路径拉走 |
| robust fit | center degree 2、width degree 1、IRLS 8 次、center Huber 20 px、width Huber 2×center | 适配冻结标注的轻微曲率和宽度趋势；宽度容忍较宽但最终使用联合内点 |
| geometry | 至少 20 点、coverage 0.60、span 0.65、missing 3、inlier 0.68、residual 14 px、突跳最多 3 且门槛 0.65×宽度、path confidence 0.42 | 由冻结 V2、合成扰动和无标注拒绝样本分离确定，证据不足宁可 LOST |
| tracking | acquire 3、degraded 2、lost 3、EMA 0.50、lateral 0.80×宽度、width 40%、heading 8°、search 2.5×宽度、预测置信衰减 0.50 | 对应题设帧间缓变和两帧短失败；第三帧失败清空模型 |
| output | 32 个曲线采样点、debug/raw 默认 false | 足够表达二次中线且限制消息体；生产默认无显示和全分辨率图像发布 |

## 结果坐标和单位

图像原点在左上角，x 向右为正、y 向下为正。三条曲线的所有点均为输入原图像素坐标；ROS `Point32.z=0`。`lateral_error_px` 在 `reference_y=0.90*image_height` 处定义为 `center_x - image_width/2`，向右为正。`heading_error_deg=atan(dx/dy)`，相对图像竖直向下方向，曲线随 y 增大向右偏为正。宽度、残差和 lateral 单位是 px；heading 单位是 degree。没有任何毫米输出。

## 离线运行与显示

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection
python3 scripts/run_stable_seam_tracker.py output/s1_real_seam_images/seam_center_lit_quality_01.bmp --no-display --save-overlay --save-json
python3 scripts/run_stable_seam_tracker.py /path/to/images --display --max-frames 100
python3 scripts/run_stable_seam_tracker.py /path/to/video.mp4 --video --no-display
python3 scripts/run_stable_seam_tracker.py --mindvision --no-display --max-frames 300
```

只有 `--display` 会调用 OpenCV 窗口 API，显示画面限制在 1280×960；q/ESC 退出。默认 `--no-display`。MindVision 默认 resolution index 0、全视场 4096×3072、彩色 BGR8/单色 MONO8 自动选择、自动曝光 100–50000 us、raw gain 2–16、预热 60 帧。SDK raw buffer 在独立复制后释放，退出时释放 aligned buffer 和 camera handle；超时与采集错误单独统计。

## ROS 2 接口

- `/vision/front/seam_geometry`：新 `tank_vision_interfaces/msg/SeamGeometry`。
- `/vision/front/seam_state`：继续发布兼容 `SeamState`，旧消息定义不变。
- `/vision/front/debug_image`：仅 `publish_debug_image=true` 且确有订阅者时生成。
- topic 输入：`/camera/front/image_raw`，BEST_EFFORT、KEEP_LAST、depth=1；与现有 `/camera/front/camera_info` 使用同一相机链路。

topic 模式回调只转换并替换 latest-frame slot，worker 独立运行算法；处理中到达的新帧覆盖旧帧，不积压无界队列，并在消息中统计 input/processed/dropped。关闭节点时安全 join worker。生产节点不调用 `imshow`。

`mindvision_direct` 由检测节点独占相机，不启动 `mindvision_camera_node`，全分辨率图像不会经过 DDS，且默认不发布原图；这是最高性能推荐模式。`ros_topic` 用于连接已有相机发布器，代价是全分辨率 DDS/`cv_bridge` 复制。

## 验证结果与限制

权威机器可读结果位于 `output/s1_p1_stable_seam_tracker/validation_summary.json` 和 `benchmark.json`。冻结 V2 单图、合成回归、无标注真实图和性能统计以这些 JSON 为准；`final_overlay.png` 与 `synthetic_sequence_montage.jpg` 供人工检查。

本次冻结单图 detected=true，coverage/effective span 均为 1.0，无缺失带或左右突跳。误差 P50/P90/P95/max（px）分别为：左边界 5.49/12.41/12.86/14.65，右边界 6.38/16.24/19.34/25.54，中线 5.60/12.53/13.54/17.64，宽度 6.74/11.15/14.11/22.66。联合内点率 0.735，残差 11.69 px。

12 帧合成回归的 detected rate 为 83.33%；未检测的两帧正是人为制造的无证据严重失焦帧，均正确标记 DEGRADED/predicted-only。false jump 为 0，lateral 残差抖动 11.09 px，heading 残差抖动 0.31°，中线误差 P95 25.50 px；下一帧恢复 detection，3 帧恢复 TRACKING。该结果仅为合成回归，不是真实现场精度。

其余 6 张真实全视场图均因几何证据不足被保守输出 LOST；所有输出均无越界或交叉。这是无标注行为检查，不可解释为检测精度或漏检率结论。

关闭显示、debug 和保存，预热 30 帧后测量 300 帧：总耗时 mean/P50/P95/max 为 79.87/78.98/88.07/104.99 ms，约 12.52 Hz；preprocess 3.38 ms、candidate 27.73 ms、pairing 13.00 ms、DP 29.34 ms、fitting 5.88 ms、temporal 0.020 ms、overlay 0 ms（均值）。300 帧峰值 RSS 未增长。该数据来自当前 aarch64 主机的离线同帧算法基准；未包含相机和 ROS/DDS，MindVision 实机输出频率仍待验证。

性能测试关闭显示、debug 和保存，预热 30 次、正式 300 次。当前 aarch64 主机结果满足平均与 P95 小于 100 ms，并达到 10 Hz 以上的算法输出；这不是 MindVision 实机或 ROS 端到端频率证明。已知限制包括固定 FOV/ROI、依赖亮带双边有符号梯度、强反光/曝光变化/焊缝移出 ROI 时会保守 LOST，以及缺少相机标定和现场多帧人工真值。

## 三终端启动

终端 1（最高性能 direct，debug 默认关闭）：

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash
export ROS_DOMAIN_ID=83
export ROS_LOCALHOST_ONLY=1
ros2 launch tank_vision_ros stable_seam_direct.launch.py
```

终端 2（先查看一条完整消息，再持续查看频率）：

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash
export ROS_DOMAIN_ID=83
export ROS_LOCALHOST_ONLY=1
ros2 topic echo /vision/front/seam_geometry --once
ros2 topic hz /vision/front/seam_geometry
```

终端 3（可选实时 debug；动态开启后 viewer 订阅才触发 overlay）：

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash
export ROS_DOMAIN_ID=83
export ROS_LOCALHOST_ONLY=1
ros2 param set /stable_seam_node publish_debug_image true
ros2 run tank_vision_ros seam_debug_viewer
```
