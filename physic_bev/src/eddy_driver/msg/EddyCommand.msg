# STM32 协议定义的命令。
uint8 CMD_SET_AMPLITUDE=1
uint8 CMD_SET_FREQUENCY=2
uint8 CMD_QUERY_AMPLITUDE=17
uint8 CMD_QUERY_FREQUENCY=18

uint8 command
# STM32 协议中的 data 字段为 uint32 小端序。
uint32 value
