# 数据、标注、坐标与结果契约

`index.json` schema=d405_dataset_v1。samples 中每条含 sample（相对于索引目录、禁止逃逸）、sha256（实际NPZ）、session、split、annotation。source_run_id 未知写 unavailable；sources 登记本次明确指定源 MCAP/metadata 的实际路径和 SHA。禁止以伪彩、RRD、reference 或 tracker 输出代替无损深度/人工真值。目录由使用时创建，不写源码或 wheel，不修改已封存run。

NPZ 使用 allow_pickle=false：image 为原尺寸 uint8 mono8或RGB8；depth 为原 uint16 或float；raw_valid 是原始有效 boolean mask；metadata 为JSON字符串。缺深度用空数组。原始值保持，几何另外显式乘 depth_scale_m；米制float比例1，当前RealSense ROS16UC1毫米比例0.001。缺比例报错，不用dtype猜单位。

metadata 保留 sequence、两个源时间戳的十进制纳秒字符串、独立接收时间（未知null）、image/depth各自Calibration、depth_to_image、depth_scale_m与source。Calibration包括width/height、row-major K、D、distortion、frame_id、source；支持none、OpenCV plumb_bob/rational_polynomial、equidistant。缺K/不支持的畸变/分辨率不符拒绝测量。RealSense ROS原Extrinsics.rotation为column-major，适配后统一row-major；translation_m为米，明确from_frame/to_frame与factory来源。

像素中心用整数，(0,0)是首像素中心；原图裁剪缩放转换为 `(p-[x0,y0]+0.5)*[sx,sy]-0.5`，逆转换保留。首版测量在原图坐标，不对深度插值，训练视觉area resize；mask nearest resize。未对齐流按实际标定反投影→R/t→目标畸变投影→最近像素取最近Z；遮挡取近点，不补洞。仅在K/D/尺寸全部一致且真实R=I/t=0时直通，不能仅凭同分辨率。深度边界与视觉边界口径不同。

最小人工标注：

```json
{
  "status": "approved",
  "origin": "human",
  "author": "operator",
  "no_seam": false,
  "left_xy": [[74, 40], [76, 100]],
  "right_xy": [[86, 40], [88, 100]]
}
```

无焊缝写 `no_seam=true`，不需要边界。左右分别严格递增y、原图内，共同y区间线性插值生成mask，左右不可交叉；区间之外mask=0但未标区域不等于实际无焊缝。GUI现有格式的人工原始点经显式approve/author后导入，核验输入图像和SHA。synthetic标签只能在明确synthetic fixture/smoke中接受，绝不伪称human。session只属于train/val/test/unassigned之一；同一session跨split拒绝。按会话的val/test与训练不重叠；单会话不足以证明独立泛化。

多个批准样本合并索引时，应在共同根目录下引用相对NPZ路径，例如：

```json
{
  "schema": "d405_dataset_v1",
  "samples": [
    {"sample": "session_a/raw.npz", "sha256": "实际64位SHA", "session": "session_a", "split": "train", "annotation": {"status": "approved", "origin": "human", "author": "operator", "no_seam": true}}
  ],
  "sources": []
}
```

以上SHA是格式说明，不能直接当有效清单。CLI读每个源样本都会实际验SHA。原run、config/model/code、时间范围及标定状态应在分析manifest登记；没有对应信息就保持unknown/unavailable，禁止从自己的深度预测生成毫米真值。

`d405_seam_result_v1`：source原metadata；mode；coordinate_2d=original_image_pixel_centers、unit_2d=pixels；points_2d_y_left_right按升y排列 `[y,left_x,right_x]`；visual/depth分别保留原候选rows、拟合points、coverage/confidence/reasons/coefficients。valid_2d、metric_valid、seam_identity_confirmed、predicted_only、identity_field_validated是独立字段。seam_identity_confirmed只是实验两证据确认，identity_field_validated始终false直至独立现场验收。

points_3d_center与二维采样一一对应，未测得点为null，points_3d_valid为mask；sampled_depth_pixels_xy登记实际读取的最近深度像素，point_3d_to_point_2d_index登记对应关系。单位metres，frame_id_3d只允许真实图像optical frame，body_coordinate_valid=false。标准ROS Path仅包含有效点且按y排列，点数可能少于二维；不能从Path点数恢复缺失mask，应同时读取结果JSON。预测或无效时空Path清除旧轨迹。

sync_delta_ns保留真实两源差值；超容差深度失效、可靠视觉可降二维。源回退/重复/长间隔/session或标定/尺寸变化reset，不重写header。online同步每侧6帧、latest1对，过期/覆盖/未配对分别统计；正式offline索引每条结果必须写盘。DDS未收到的消息数无法凭内部计数补出，须独立subscriber验收。

结果每帧包含thresholds、观测/余量、noise/拟合残差、相对有符号高度、可信覆盖、降级原因和实际执行backend/阶段墙钟。background不够/退化失败，不返回零残差冒充成功。mm评价只有独立真实mm标签时才可实现，目前输出unavailable。IoU/Dice为视觉概率阈值口径；每层边界/中线/宽度误差P50/P95/max按原像素；失败全进入有效率分母，预测不计测量，候选检出与身份确认分开。实时统计窗口300，离线精确分位数明确注明精确算法。

事件：仅显式service请求时复制原图/深度/诊断。raw.npz/result.json/diagnostics.npz完成后写metadata.json包含实际SHA与complete；根manifest汇总限频/drop/worker状态。关闭统一deadline复用已安装AsyncLifecycle；超时/worker异常保持incomplete，不强杀线程。丢弃统计不是已保存证据。

轻量ROS结果可被既有MCAP录制，但须实际确认topic库存和新run frozen配置；本项目未改全车默认采集。离线rerun对NPZ和每帧结果时间/序号验证，失败保留partial、不提交正式RRD；这不替代原全车strict builder/role handoff验收。
