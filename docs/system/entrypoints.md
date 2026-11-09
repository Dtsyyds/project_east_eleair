# 入口与安装契约

机器可读清单 manifests/entrypoints.json 由源码 AST/文本扫描生成，保留参数表达式、默认值、launch arguments、imports、consumers、console_scripts 与 package dependencies；动态值不冒充已经解析的运行值。reports/refactor_v1/entrypoint_inventory.json 是本次冻结副本。

|总控|唯一现有入口|参数|
|---|---|---|
|collect.sh|physic_bev/scripts/start_vehicle_collection.sh|name:=value，与原默认一致|
|stop.sh|physic_bev/scripts/stop_vehicle_collection.sh|RUN_ID|
|replay.sh|physic_bev/scripts/replay_vehicle_collection.sh|RUN_ID MODE RATE OFFSET DURATION|
|validate.sh|physic_bev/scripts/validate_vehicle_collection.py|RUN_DIR --report-dir 分析目录|
|handoff.sh|physic_bev/scripts/build_role_handoff.py|--run-dir / --profile / --manifest-only 等原参数|
|preflight.sh|只读软件检测|--output 新报告路径|

包装层 exec 调用，逐参数透传，退出码/TERM 信号 fixture 可验证；--help 不启动硬件。源 CLI、原 launch/ROS包名/topic/消息格式没有重命名。

新安装：tank_vision 在 core_env 的 wheel；inspection_tools/mindvision 在 install_physic；inspection_interfaces/eddy_driver/inspection_sync 在 install_cpp；tank_vision_interfaces/tank_vision_ros/inspection_rerun 在 install_vision。其余原驱动与 bringup 来自现有 physic install underlay，未重建/覆盖。package 来源与 SHA 对应见 installation_correspondence.json。
