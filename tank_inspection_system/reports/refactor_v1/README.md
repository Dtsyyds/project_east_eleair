# Refactor V1 证据索引

读取 final_report.json，再按 evidence_index 查看逐问题复现、回归、安装与修改清单。初始登记370个历史文件，但末次媒体目录不可见，无法完成size/mtime复核。没有改写媒体或历史报告；source_data_preservation.json 明确environment_blocked，不能把不可见文件当已删除或完整性通过。

主回归分组：core51通过，data68通过，video consumers7通过，RRD22通过2环境失败。colcon C++9通过，physic Python36通过；Rerun用原冻结SDK执行38项，2项MCAP插件失败。分组存在重复，不能简单累加。tank_vision_ros/interface colcon自身0tests不计通过，实际安装版快照/Eddy E2E单独记录。

真实Rerun CLI与ROS CDR的注入reader fixture不是MCAP验收；旧MCAP失败保留。正常业务90条记录SHA完全一致，33帧trace开关业务一致；benchmark峰值RSS包含有限诊断记录，长期实时6000帧的RSS另有bounded_memory.json。

backups 仅编辑过的文件。before/after源码SHA、初始dirty/untracked、原修改保护、独立build日志和rollback安装证据均保留。早期失败/跳过/0tests日志未删除，没有用新通过改写历史报告。涡流一次字段断言失败保留在 final_metadata_workflow/eddy.txt；后续 fixture 明确匹配本次进程的唯一节点并仅在输入前等候发现稳定，未放宽数量/字段或降低帧间节奏。报告区分5Hz header间隔和实测墙钟约3.16Hz；此前4Hz常量不是实测速率，不能用作性能证据。硬件命令在 ../../docs/collection_and_replay.md，本任务未启动真实设备。
