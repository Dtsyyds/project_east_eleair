std_msgs/Header header

bool connected
uint32 current_amplitude
uint32 current_frequency

bool has_status
uint8 last_status_cmd
uint32 last_status_value

# 最近一次 0xE0 错误响应的解码结果；last_status_is_error 为 false 时均为 0。
bool last_status_is_error
uint8 error_command
uint8 error_stage
uint8 error_hal_status
uint8 error_detail

uint64 received_frames
uint64 dropped_frames
uint32 queue_depth
