std_msgs/Header header

# 每个完整 ADC 帧内 20 个采样点的 8 通道均值，单位与 /eddy/data 一致（mV）。
# 此精简消息是车辆 MCAP 采集唯一记录的 Eddy 数据；不包含原始 160 点采样。
float64[8] channel_mean
