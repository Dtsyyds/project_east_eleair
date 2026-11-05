# D405 焊缝偏差话题

实现依据根目录 `WELD_TRACKING_TOPIC_INTERFACE.md`。消息类型为
`duct_robot_msgs/msg/WeldTrackingError`，话题为 `/weld_tracking/error`。
发布和订阅 QoS 均为 KeepLast(1)、BestEffort、Volatile。

默认适配器读取完整 A 的最终左右边界，取中点后在固定局部区间恢复二次曲线的中心及切线。
它不改变 A 权重、概率图、候选选择、拟合结果、55% 覆盖门槛或实时算法配置，不使用 C、
PARTIAL_2D 或历史预测。丢失、局部支持不足、坐标异常、计算失败均发布 `valid=false`，
两个误差及 confidence 均为零。A 是否把板边误认成焊缝仍是 A 的识别限制，接口封装不会解决该问题。

## 固定图像参考

独立文件 `config/d405/weld_tracking_error.yaml`，启动时读取一次。
接口原文未给出具体参考值，本次先采用以下软件参考，尚未确认车体/探头工作中心：

| 项目 | 当前值 |
|---|---|
| 处理图像 | 原始 RGB 640×480，不使用网络输入尺寸或缩略图 |
| 参考行 | y=360 |
| 参考中线 | x=320，竖直向上 |
| 方向区间 | y=336～384 |
| 局部支持 | 方向区间内至少 6 个实测点；不要求完整覆盖区间、跨过参考行或满足最大行间隔 |
| 局部曲线检查 | 二次曲线最大残差不超过 5 px，仅影响偏差有效性 |
| 方向约定 | 图像向上为前进方向，图像左侧为前进方向左侧 |

局部曲线只使用 A 最终保留行。当前 A 的左右界本身为二次曲线，局部恢复不会改动这些边界。
参考行处 `lateral=(center_x-320)/320`；局部切线以向上为正向，
`heading=atan(dx/dy)-atan(reference_dx_dy)`。向前左偏角度为正，右偏为负。
两项均为图像量，不是米制误差或壁面角度。允许用区间内至少 6 个点拟合后在固定参考行求值，
因此只有单侧点时包含局部外推；不使用上一帧点，不将 PARTIAL 算作完整检出。
尺寸变化时发布无效值，不悄悄缩放约定。有效消息的共同 confidence 继承 A 的完整输出置信度，
局部有效性另行检查；它不是人工标注准确率或经标定的概率。

header 原样使用输入图像的采集时间戳和光学 frame_id。frame_seq 在本次进程内每个取出处理的输入递增。
每个完成处理的输入仅发布一条消息，处理异常也清零。沿用已有 latest-frame 丢帧策略，
尚未进入识别的被覆盖帧不伪造输出；不新增定时重发。
诊断话题保留 `tracking_error_reference`、`tracking_error_reasons` 和发布计数。

## 在 D405 画面检查与调整参考线

启动视觉巡迹并收到节点诊断后，网页在 D405 预览上显示黄色虚线参考中线、
参考点十字、参考行及浅黄色方向拟合区，箭头指向参考前进方向。
画面下方显示原图参考点、向左偏角及拟合区间。数据直接来自运行节点的
`diagnostics.tracking_error_reference`，并按预览尺寸映射；不会在网页写死 320/360。
焊缝丢失时仍显示参考线，巡迹停止、画面过期或断连时清除。
“隐藏叠加”同时隐藏焊缝线和参考标注。

调整 `config/d405/weld_tracking_error.yaml` 后，点击“停止视觉巡迹”，再点击
“启动视觉巡迹”，检查画面下方回传的生效数值；无需重启相机。运行中编辑文件不会立即生效。

- `reference_x`：参考点横坐标，增大向右、减小向左。例如 320 改成 340，参考线右移 20 像素。
- `reference_y`：横向偏差测量行，增大向下；须保持在 `heading_y_min` 与 `heading_y_max` 之间。
- `reference_dx_dy`：参考线斜率。竖直为 0；向前左偏角为 θ 时填写 `tan(θ)`，
  例如左偏 5° 为约 0.08749，右偏 5° 为约 -0.08749。线绕参考点旋转。
- `heading_y_min` / `heading_y_max`：方向拟合区域的上下边界，须在原图内。

参考线方程为 `x(y)=reference_x+reference_dx_dy*(y-reference_y)`。
修改会同时改变后续 `/weld_tracking/error` 的计算基准；它不是仅供显示的辅助线。

## 192.168.127 网段

板端地址为 **192.168.127.10（eth1）**。标注入口迁至 **http://192.168.127.10:8767/**，
仅监听这个地址，原标注及来源索引保留。

`config/communication/fastdds_d405_127.xml` 限定 DDS UDP 接口为 192.168.127.10 和本机回环，
保留原大帧 SHM 配置。`system_websocket/run_d405.sh` 下次启动使用此配置。
若已有巡迹节点，不要另启重复节点。

仅在需要启动被动 A 订阅器时执行下列命令；它本身不启动相机、底盘或录制：

```bash
bash tools/vision/run_d405_error_127.sh
```

PC 接入 192.168.127.0/24，安装/编译同一 `duct_robot_msgs` 包，source 对应 ROS 工作空间，
使用 ROS_DOMAIN_ID=77、ROS_LOCALHOST_ONLY=0。若 PC 限定网卡，请使用 PC 自己的 192.168.127.x 地址，
不要照搬板端 192.168.127.10 白名单。

```bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic echo /weld_tracking/error duct_robot_msgs/msg/WeldTrackingError \
  --qos-history keep_last --qos-depth 1 --qos-reliability best_effort --qos-durability volatile
```

## 验证

- 消息包及 D405 ROS 适配包已编译，`ros2 interface show` 可读取新类型。
- `test_tracking_error.py` 和 `test_d405_a_runtime.py` 共 30 项通过，覆盖符号、局部切线、
  支持范围、非有限值、左右顺序、丢失、PARTIAL、预测，以及 A 原参数与帧独立行为。
- `tools/vision/validate_weld_tracking_error.py` 在 localhost、隔离域 188 上运行实际 ROS
  发布订阅，模型替换为显式测试夹具，10 个输入逐一验证类型、QoS、采集头、帧号、正负号、
  失效清零和异常后恢复。不读取相机，不运行模型推理，不发送底盘命令。
- Fast DDS 发现接口不保证返回 history/depth，测试在实际创建的发布器上核验这两项，
  在订阅侧核验 reliability/durability。
- 报告：`.runtime/s100p/analysis/weld_tracking_error_20261103/ros_contract_verified/report.json`。
- 未进行跨 PC 联调或硬件精度验收；尚需确认实际探头中心与图像前进方向。

## 显式候选模式与只读接收（2026-11-03）

`bash system_websocket/run_d405.sh --candidate` 使用独立通信配置
`config/communication/d405_candidate.yaml`，上位机点击“启动视觉巡迹”后才启动算法。
不带此开关仍为原 A。两者均使用原 `config/d405/round1_a.yaml`，默认三相机配置不变。
如果系统已经运行，先结束原启动实例，不能并起第二个设备所有者。
已有驱动运行时也可在先停止旧巡迹节点后，用
`bash tools/vision/run_d405_error_127.sh --runtime-mode candidate` 启动被动订阅器；
该命令不打开相机。去掉 `--runtime-mode candidate` 恢复被动 A 入口。

候选模式直接复用 8772 回放的 `center_2d.decode` 和 `CPUInference`，
FP32 channels-last、两线程、每个处理帧一次网络。A 有效时原样返回，只有
`curve_outliers_or_discontinuous` 触发 5 px 中线筛选，保留原连续段和 53/96 门槛。
CENTER_2D 不提供左右界或三维几何；中线在上述方向区间内至少有 6 个实测点并通过拟合检查时，
可发布有效偏差，不再要求覆盖完整参考区。显示中线仍不等于 `/weld_tracking/error.valid=true`。

`receiver_hold.ErrorHold` 实现只读接收状态，不发布任何运动命令：

- A 或 CENTER_2D 的有效新帧清零计数并保存实测误差。
- 无效新帧 1～4 暂持原误差及原采集时间；第 5 帧丢线并清除历史。
- 无历史不暂持；重复/乱序帧不计新帧；消息丢序或发布者变化不跨越暂持。
- 消息超时 0.5 秒或 `/weld_tracking/source_alive=false` 立即失效，不等待 5 帧。
  发布端以最后收到的唯一 RGB 判定源超时 0.5 秒。实际检测仍有调度延迟。
- 暂持仅存在接收端，绝不把旧值重发成新的有效 `WeldTrackingError`。

`tools/vision/observe_weld_error.py` 是 RDK 只读观察端和人工阶段日志页，
不是底盘消费进程。当前仓库未发现真实底盘订阅实现，不能宣称已在底盘部署暂持逻辑。
本次真机记录位于 `.runtime/s100p/analysis/d405_candidate_live_20261103/`。

## 丢线候选选择与继续跟踪（2026-11-04）

`--candidate` 现在增加第三层恢复流程：A 和原 CENTER_2D 均失败时，
从当前帧概率图生成最多三条独立疑似中线。原 A/CENTER_2D 成功时保持原结果。
候选只复用本帧的概率图，不增加网络推理，也不把上一帧线作为本帧检测结果。

候选生成单独采用 0.30 概率门槛，复用扫描行动态规划，按概率、覆盖率及拟合残差排序。
提取一条后抑制其条带，再查找不同的路径；至少需要 20% 扫描行支持，不足三条就少发，
无证据则发空列表。候选得分仅用于排序，不是经过标定的正确率。

网页在匹配的 D405 图像上用紫/青/橙虚线及编号显示候选。点击“选择候选 1/2/3”后，
请求带上画面对应的会话、候选集合及候选编号。节点确认后进入 ACQUIRING，
用后续新图像中的候选关联选中的目标；另一路更高分不会直接覆盖选择。
“恢复自动选线”清除选择，下一帧恢复原 A/CENTER_2D 优先流程。

选中后，关联使用共同实测行上的横向距离，90 分位距离不超过 24 px，
最佳与次佳关联至少相差 8 px；几何支持仍须覆盖至少 55% 扫描行且中心拟合最大残差不超过 5 px。
符合条件才输出 `CENTER_2D`，并标记 `measurement_kind=SELECTED_CENTER_2D`。
`/weld_tracking/error` 仍需独立通过原参考区域检查，不能因人工选择而强制 valid=true。
所选目标失效期间发布无效偏差，不发布旧点；超过 0.5 秒、源超时、尺寸/标定变化或时间回退
会要求重新选择，不自动切换到另一条焊缝。所有输出仍为图像量，不直接发送底盘运动命令。

### ROS 话题协议

新增话题均使用 `std_msgs/msg/String` 承载 JSON，Reliable / Volatile。
候选发布器 KeepLast(1)，选择请求和确认 KeepLast(10)。

| 话题 | 方向 | 内容 |
|---|---|---|
| `/weld_tracking/candidates` | 视觉 → 上位机 | 每个处理帧的候选列表和选择状态；有效跟踪时列表为空 |
| `/weld_tracking/select_candidate` | 上位机 → 视觉 | 选择候选或清除选择 |
| `/weld_tracking/selection_status` | 视觉 → 上位机 | 对指定 request_id 的 completed / failed 确认 |

候选包包含 `schema=weld_recovery_candidates_v1`、`session_id`、`candidate_set_id`、
`frame_seq`、原图 `stamp_ns` / `frame_id`、`image_width` / `image_height`、
`coordinate_space=original_image_pixel_centers`、`candidates` 及 `selection`。
每条候选包含字符串 `candidate_id`、`rank`、`score`、`confidence`、`coverage`、
`residual_px`、`measurement_supported`、`points_2d_y_center`（每点为 `[y,x]`）。
`measurement_supported` 仅表示候选几何门槛通过，不代表偏差有效，也不代表已选中。

选择请求示例（必须替换为刚收到的实际会话及集合编号）：

```json
{"request_id":"unique-choice-1","session_id":"VISION-...","action":"select","candidate_set_id":"<当前集合编号>","candidate_id":"2"}
```

清除选择：

```json
{"request_id":"unique-clear-1","session_id":"VISION-...","action":"clear"}
```

确认格式为 `request_id`、`session_id`、`status`、`selection`；失败时另有 `error`。
completed 表示选择请求已执行，不代表已恢复有效测量；继续观察 selection.state 和误差话题 valid。
候选集合缓存最多 32 帧 / 5 秒，启动实例变化、源超时或已恢复有效输出后旧集合失效。
不允许只按“当前第二条”选择：编号仅在对应集合内有效。
同一 request_id 重发不会重复执行，若同一编号携带不同参数会被拒绝。
WebSocket 操作名为 `vision.select_candidate` / `vision.clear_selection`，选择参数字段
与上述一致但不包含 action/request_id，后两者由操作服务生成。

此扩展需更新 `inspection_operations` 与 `d405_seam_ros` 安装包后重新启动系统，
并刷新网页。默认 A 模式没有候选选择能力，`--candidate` 采集配置会记录新增话题。

验证入口：`packages/d405_seam_tracking/tests/test_recovery.py`、
`tests/web_console/test_vision_controls.py`、`tools/vision/validate_weld_recovery.py`。
ROS 验证使用 localhost 域 189 和合成概率图，未启动硬件；识别准确率仍需实际图像验证。
