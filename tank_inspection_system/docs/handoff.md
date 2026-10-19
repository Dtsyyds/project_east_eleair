# 角色交付

唯一 profiles 配置为 physic_bev/config/data_handoff_profiles.yaml；RRD placeholder 从当前 run 的 rrd_manifest.output_path 或 run manifest 解析，未指定时按当前 run_id 推导，拒绝路径逃逸。只读历史 run，不固定任何旧批次名称。

```bash
S=/home/sunrise/east_eleair/tank_inspection_system
RUN_ID=你的新采集ID
bash "$S/scripts/handoff.sh" --run-dir "/media/mcap_data/runs/$RUN_ID" --profile front_camera --output-root "/media/mcap_data/deliveries/$RUN_ID" --manifest-only
bash "$S/scripts/handoff.sh" --run-dir "/media/mcap_data/runs/$RUN_ID" --profile front_camera --output-root "/media/mcap_data/deliveries/$RUN_ID" --destination "/media/mcap_data/deliveries/$RUN_ID/front_camera" --materialize --copy-method python
bash "$S/scripts/handoff.sh" --run-dir "/media/mcap_data/runs/$RUN_ID" --profile front_camera --output-root "/media/mcap_data/deliveries/$RUN_ID" --destination "/media/mcap_data/deliveries/$RUN_ID/front_camera" --verify-only
```

角色包括 common、fastlivo2、front_camera、d405、eddy、rerun_viewer、system_integration。以当前 profiles 为准，fixture 覆盖当前 ID、物化、SHA 和缺失检测。外置 RRD 使用新增兼容参数 `--rrd-manifest "$A/outputs/rrd_manifest.json" --profile rerun_viewer`。manifest 必须位于当前 run 的 analysis 根内，ID/complete/strict/hash_verified 与实际 RRD SHA 均核验；来自 analysis 的文件 origin=analysis_rrd，不回写源 run。可以按同样的 manifest-only/materialize/verify-only 流程交付。

返回 result_manifest 模板保留原字段；source_run_id/config/source SHA/code状态/时间范围/坐标/标定未知值必须明确。RRD 可读不能代替 MCAP/MKV，mean 不能代替 raw waveform。不要复制板端 install 作为 PC 接口依赖，接收方从消息源码构建。
