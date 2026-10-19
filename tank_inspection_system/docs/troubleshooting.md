# 排障

- preflight 返回2 / rosbag2报 storage mcap load failed：检查 get_registered_readers/writers；本机确实缺少 mcap。恢复与当前 Humble 环境匹配的插件后重新运行 MCAP 两项失败和真实多 bag / sensor-only ROS E2E，不能降低标准。
- DDS UDP socket/getifaddrs 在受限环境报错：本次共享内存实际传递 Eddy 各5条及快照 E2E 成功，但不证明跨机网络/高负载。若受阻应记录 environment_blocked，不伪造 topic 数量。
- close 超时：查看 writer_status 或 recorder manifest 的 worker_alive、metadata_worker_alive、unfinished、failed/abandoned/unwritten；保留 incomplete，不能强杀 Python线程后宣布成功。关闭状态写盘也共用 deadline，磁盘阻塞时先以调用方异常/内存 FAILED 和既有 incomplete 标记为准；I/O 恢复后工作线程重算并补写 incomplete。文件里的线程存活字段是写入时快照，应结合当前生命周期状态判断。
- 旧 MKV 无 frame_map：不能以 packet count 或95%数量通过。若没有 raw/PTS唯一证据，记录不可严格对齐，报告写analysis而非补写旧sidecar。
- strict RRD 哈希失败：检查选中 source 和封存清单，保留 partial。禁止更新旧清单来获得通过。
- 零测试：查看 colcon 注册与 test-result summary，使用 explicit base-paths；总控已 COLCON_IGNORE。
- stable 默认配置 guided_search 缺失：已修复 optional 参数映射，安装后重新 build，不调整算法参数。

hardware_revalidation_status=not_executed；算法现场验收待定。fixture速率不能替代本机真实6Hz门槛。


当前末次审计 `/media/mcap_data/runs` 不可见，source_data_unchanged=false 的含义是未完成复核，不是本任务删除了370个文件。恢复媒体可见性后用 check_source_preservation.py 写新报告；不要覆盖 historical_stat_before.json 或历史验收。

涡流 E2E fixture 曾出现一次 FusionIndex 字段断言失败，原记录保留在 final_metadata_workflow/eddy.txt。测试现在使用唯一进程节点名、核对本次 endpoint，并在输入前等候发现稳定；数量和字段门槛、帧间节奏未降低。两个后续独立执行均各产生 raw/mean 5 条有效输出。先前 fixture_rate_hz=4 是未测量常量，已改为 header 间隔 5Hz 与实测墙钟约 3.16Hz 分别登记，不作为原速率回放或硬件性能证据。发现阶段竞态是排查推断，旧失败没有足够字段记录确认具体缺失流；真实高负载/DDS 的字段和数量仍需短测。
