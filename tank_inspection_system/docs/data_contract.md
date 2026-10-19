# 数据契约

历史 runs/<run_id> 保持原路径、内容、封存和验收。新派生分析位于 analysis/<run_id>/<analysis_id>/{configs,metrics,outputs,logs}，输出路径不得覆盖已有产物。deliveries/<run_id>/<profile> 供角色交付；calibrations 和 datasets 只定义目录规则，本任务没有创造虚假标定或数据集。

complete、closed、sha256_valid、collection acceptance、transport、algorithm、calibration、replay、viewer 是不同字段。check_results 的 passed/failed/disabled/not_applicable/not_executed 是新验收权威状态；旧 checks bool 为兼容保留，不能把 disabled 的旧 true 计入通过。采集通过可以同时算法 not_executed。

新 frames.jsonl 每行保留原 ros_header_stamp_ns 并增加 capture/enqueue/写入状态。rawvideo_fixed_fps_unique_pts_v2 的实际 PTS 对应入队 PTS；H264 通过 NAL VCL 内容身份和 decoder pkt_pos 对应，映射计算发生在关闭阶段，不增加实时 SHA 每帧运算。编码 packet 数量不等于解码帧数。frame_map 记录视频和 journal SHA、source_index、实际 decoded_index、时间戳与未映射源记录。

缺首帧/中帧必须有唯一证据才建立映射；重复内容歧义、坏帧、旧 run 无 map、跨段回退均拒绝严格对齐。不存在以 95% 比例或顺序平移修复。旧数据只允许外置分析，无法确定缺失位置写 unavailable。

RRD strict 对实际选中 MCAP/metadata/视频/journal/map/冻结配置及接口源文件重新 SHA，与 run manifest 比较；缺失/逃逸/无效封存失败，提交前再次 SHA。输出包含 source_run_id、当前 SHA、配置 SHA、builder SHA、时间范围、message frame_id、标定 unverified、输出/CLI验证结果。dry-run 当前做实际哈希，不产生正式 RRD；失败保留 partial。

ROI 保留 roi_raw.bin + frames.jsonl 原接口；快照 complete=true 必须在所有文件完成后并能校验 SHA。异常 writer 状态 incomplete，绝不安全关闭虚假声明。完整离线输出通过有界阻塞实现，消费者须同时排空；实时 latest 丢弃旧结果明确计数。

关闭期限覆盖任务排空、writer flush/fsync 和收尾元数据写盘。发生阻塞时调用方报告 FAILED 与 worker_alive/metadata_worker_alive；不能声称安全关闭。Python I/O 线程不能强杀，仍在执行的 I/O 可能延迟完成，随后重算并补写 incomplete；磁盘仍不可用时无法保证立即落盘，调用方异常和既有 incomplete 记录是有效证据。正常状态文件中的线程存活字段是写入时快照，不是后续查询。
