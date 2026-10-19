std_msgs/Header header

# 驱动侧帧序号，每收到一个完整 ADC 帧加 1。
uint32 sequence

# 当前帧结构。
uint16 samples_per_channel
uint16 channel_count

# 展平顺序：
# sample0_ch0 ... sample0_ch7, sample1_ch0 ... sample19_ch7，单位 mV。
int16[160] data

# 本帧中 20 个采样点内各通道的均值。
float64[8] channel_mean

# 设备确认的最新激励设置。
uint32 excitation_amplitude
uint32 excitation_frequency

# 有界驱动 FIFO 丢弃的完整帧总数。
uint64 dropped_frames
